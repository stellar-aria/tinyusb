/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Koji Kitayama
 * Portions copyrighted (c) 2021 Roland Winistoerfer
 * Copyright (c) 2024 Sapphire Koser
 * Copyright (c) 2024 Katherine Whitlock
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

/*
 * This implementation is based on the RZ.A1L TRM, v6 (R01UH0437EJ0600).
 *
 * Important implementation notes:
 *  - rhport 0 and 1 both share clock configuration.
 *  - Users must set RUSB1_CLOCK_SOURCE to select where the USB modules will be clocked from.
 *      0: Use the USB_X1 external crystal oscillator @ 48MHz
 *      1: Use the EXTAL connection @ 12MHz.
 *  - RUSB1_WAIT_CYCLES must also be defined such that the number of peripheral bus 1 cycles exceeds 67ns.
 */

#include "tusb_option.h"

#if CFG_TUD_ENABLED && defined(TUP_USBIP_RUSB1)

  #include "device/dcd.h"
  #include "osal/osal.h"

  #if TU_CHECK_MCU(OPT_MCU_RZA1X)
    #include "rusb1_rza1.h"
  #else
    #error "Unsupported MCU"
  #endif

  #include "rusb1_type.h"
  #include "dcd_rusb1.h"

  #if !defined(RUSB1_CLOCK_SOURCE)
    #error "Must define RUSB1_CLOCK_SOURCE to 0 (if the board has the 48MHz USB_X1 connected) or 1 (if the board has the 12MHz EXTAL connected)"
  #endif

  #if !defined(RUSB1_WAIT_CYCLES)
    #error "Must define RUSB1_WAIT_CYCLES to configure the number of peripheral 1 bus wait cycles on register access. Must result in a total wait time > 67ns"
  #endif

//--------------------------------------------------------------------+
// REGISTER ACCESS MACROS
//--------------------------------------------------------------------+
#define REG_VAL(NAME, VAL) ((VAL) << (NAME##_SHIFT))
#define REG_READ_FIELD(var, NAME) (((var) & NAME) >> (NAME##_SHIFT))
#define REG_WRITE_FIELD(reg, NAME, val) (((val << NAME##_SHIFT) & NAME) | ((~NAME) & (reg)))
#define REG_RMW_FIELD(reg, NAME, val) reg = (((val << NAME##_SHIFT) & NAME) | ((~NAME) & (reg)))

TU_VERIFY_STATIC(USB_PIPEnCTR_1_5_PID == USB_PIPEnCTR_6_8_PID);
TU_VERIFY_STATIC(USB_PIPEnCTR_1_5_PID == USB_PIPEnCTR_9_PID);
TU_VERIFY_STATIC(USB_PIPEnCTR_1_5_PID == USB_PIPEnCTR_A_F_PID);
TU_VERIFY_STATIC(USB_PIPEnCTR_1_5_PID_SHIFT == USB_PIPEnCTR_6_8_PID_SHIFT);
TU_VERIFY_STATIC(USB_PIPEnCTR_1_5_PID_SHIFT == USB_PIPEnCTR_9_PID_SHIFT);
TU_VERIFY_STATIC(USB_PIPEnCTR_1_5_PID_SHIFT == USB_PIPEnCTR_A_F_PID_SHIFT);

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+
enum {
  #if TU_CHECK_MCU(OPT_MCU_RZA1X)
  PIPE_COUNT = 16,
  #else
    #error "Unsupported MCU"
  #endif
};

typedef struct {
  void *buf;          /* the start address of a transfer data buffer */
  uint16_t length;    /* the number of bytes in the buffer */
  uint16_t remaining; /* the number of bytes remaining in the buffer */

  rusb1_pipe_config_t config; /* Pipe configuration */
  uint8_t xfer;               /* TUSB_XFER_* type */

  uint8_t ep; /* an assigned endpoint address */
  uint8_t ff; /* `buf` is TU_FUFO or POD */
} pipe_state_t;

typedef struct
{
  pipe_state_t pipe[PIPE_COUNT];
  uint8_t ep[2][16]; /* a lookup table for a pipe index from an endpoint address */
  // Track whether sof has been manually enabled
  bool sof_enabled;

  // packet memory usage tracking
#if CFG_TUSB_DEBUG
  uint32_t used_packet_blocks[RUSB1_PACKET_BUFFER_SIZE_BLOCKS / 32];
#endif
} dcd_data_t;

// Device state for each supported peripheral
static dcd_data_t _dcd[RUSB1_RHPORT_COUNT];

TU_ATTR_ALWAYS_INLINE inline static dcd_data_t * dcd_for_rhport(uint8_t rhport) {
  // TODO: support disabling ports and compressing the DCD list
  return &_dcd[rhport];
}

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+

static void delay_ms(uint32_t ms) {
  osal_task_delay(ms);
}

static volatile uint16_t *get_pipectr(struct st_usb20 *rusb, unsigned num) {
  if (num) {
    return &(&(rusb->PIPE1CTR))[num - 1];
  } else {
    return &(rusb->DCPCTR);
  }
}

static volatile struct st_usb20_from_pipe1tre *get_pipetre(struct st_usb20 *rusb, unsigned num) {
  volatile struct st_usb20_from_pipe1tre *tre = NULL;
  if ((1 <= num) && (num <= 5)) {
    tre = &((volatile struct st_usb20_from_pipe1tre *) (&rusb->PIPE1TRE))[num - 1];
  }
  return tre;
}

static volatile uint16_t *ep_addr_to_pipectr(uint8_t rhport, unsigned ep_addr) {
  struct st_usb20 *rusb = RUSB1_REG(rhport);
  const unsigned epn = tu_edpt_number(ep_addr);

  if (epn) {
    const unsigned dir = tu_edpt_dir(ep_addr);
    const unsigned num = dcd_for_rhport(rhport)->ep[dir][epn];
    return get_pipectr(rusb, num);
  } else {
    return get_pipectr(rusb, 0);
  }
}

static uint16_t edpt0_max_packet_size(struct st_usb20 *rusb) {
  return REG_READ_FIELD(rusb->DCPMAXP, USB_DCPMAXP_MXPS);
}

static uint16_t edpt_max_packet_size(struct st_usb20 *rusb, unsigned num) {
  rusb->PIPESEL = num;
  return REG_READ_FIELD(rusb->PIPEMAXP, USB_PIPEMAXP_MXPS);
}

//--------------------------------------------------------------------+
// Pipe FIFO
//--------------------------------------------------------------------+

static inline void rusb_cfifo(struct st_usb20 *rusb, rusb1_fifo_t *fifo) {
  fifo->data = &rusb->CFIFO.UINT32;
  fifo->sel = &rusb->CFIFOSEL;
  fifo->ctr = &rusb->CFIFOCTR;
}

static inline void rusb_fifo0(struct st_usb20 *rusb, rusb1_fifo_t *fifo) {
  fifo->data = &rusb->D0FIFO.UINT32;
  fifo->sel = &rusb->D0FIFOSEL;
  fifo->ctr = &rusb->D0FIFOCTR;
}

static inline void fifo_wait_for_ready(rusb1_fifo_t *fifo, unsigned num) {
  while ((REG_READ_FIELD(*fifo->sel, USB_DnFIFOSEL_CURPIPE)) != num) {}
  while (!REG_READ_FIELD(*fifo->ctr, USB_CFIFOCTR_FRDY)) {}
}

static void fifo_set_mbw(rusb1_fifo_t *fifo, uint32_t mbw) {
  uint32_t value = *fifo->sel;
  value = REG_WRITE_FIELD(value, USB_DnFIFOSEL_MBW, mbw);
  *fifo->sel = value;
}

// Write data buffer --> hw fifo
//
// fifo_sel is the FIFO control register to use
// fifo is the FIFO register to be written to
// buf is the data to write
// len is the amount of data copy, in bytes
static bool sw_to_hw_fifo(rusb1_fifo_t *fifo, uint8_t *buf, unsigned len) {
  fifo_set_mbw(fifo, RUSB1_FIFOSEL_MBW_32BIT);
  while (len >= 4) {
    *fifo->data = tu_unaligned_read32(buf);
    len -= 4;
    buf += 4;
  }

  if (len >= 2) {
    fifo_set_mbw(fifo, RUSB1_FIFOSEL_MBW_16BIT);
    *fifo->data = tu_unaligned_read16(buf);
    buf += 2;
    len -= 2;
  }

  if (len >= 1) {
    fifo_set_mbw(fifo, RUSB1_FIFOSEL_MBW_8BIT);
    *fifo->data = *buf;
    buf += 1;
    len -= 1;
  }

  TU_ASSERT(len == 0);

  return true;
}

// Read data buffer <-- hw fifo
//
// fifo is the FIFO register to read
// buf is the output buffer
// len is the number of bytes to read
//
// Assumes the fifo is in 8-bit mode
// XXX: try in 32-bit mode instead, see what perf is like
static bool hw_to_sw_fifo(rusb1_fifo_t *fifo, uint8_t *buf, unsigned len) {
  volatile uint8_t *data = (volatile uint8_t*) fifo->data;
  while (len != 0) {
    *buf = *data;
    buf++;
    len--;
  }

  return true;
}

// Write data sw fifo --> hw fifo
static bool sw_to_hw_fifo_ff(rusb1_fifo_t *hw_fifo, tu_fifo_t *sw_fifo, uint16_t total_len) {
  tu_fifo_buffer_info_t info;
  tu_fifo_get_read_info(sw_fifo, &info);

  uint16_t count = tu_min16(total_len, info.len_lin);
  if (!sw_to_hw_fifo(hw_fifo, info.ptr_lin, count)) {
    return false;
  }

  uint16_t rem = total_len - count;
  if (rem) {
    rem = tu_min16(rem, info.len_wrap);
    if (!sw_to_hw_fifo(hw_fifo, info.ptr_wrap, rem)) {
      return false;
    }
    count += rem;
  }

  return true;

  tu_fifo_advance_read_pointer(sw_fifo, count);
}

// Read data sw fifo <-- hw fifo
static bool hw_to_sw_fifo_ff(rusb1_fifo_t *hw_fifo, tu_fifo_t *sw_fifo, uint16_t total_len) {
  tu_fifo_buffer_info_t info;
  tu_fifo_get_write_info(sw_fifo, &info);

  uint16_t count = tu_min16(total_len, info.len_lin);
  if (!hw_to_sw_fifo(hw_fifo, info.ptr_lin, count)) {
    return false;
  }

  uint16_t rem = total_len - count;
  if (rem) {
    rem = tu_min16(rem, info.len_wrap);
    if (!hw_to_sw_fifo(hw_fifo, info.ptr_wrap, rem)) {
      return false;
    }
    count += rem;
  }

  tu_fifo_advance_write_pointer(sw_fifo, count);
}

//--------------------------------------------------------------------+
// Pipe Transfer
//--------------------------------------------------------------------+

static bool pipe0_xfer_in(dcd_data_t *dcd, struct st_usb20 *rusb) {
  rusb1_fifo_t fifo;
  pipe_state_t *pipe = &dcd->pipe[0];
  const unsigned rem = pipe->remaining;

  if (!rem) {
    pipe->buf = NULL;
    return true;
  }

  rusb_cfifo(rusb, &fifo);

  const uint16_t mps = edpt0_max_packet_size(rusb);
  const uint16_t len = tu_min16(mps, rem);
  void *buf = pipe->buf;

  if (len) {
    if (pipe->ff) {
      if (!sw_to_hw_fifo_ff(&fifo, (tu_fifo_t *) buf, len)) {
        return false;
      };
    } else {
      if (!sw_to_hw_fifo(&fifo, buf, len)) {
        return false;
      }
      pipe->buf = (uint8_t *) buf + len;
    }
  }

  if (len < mps) {
    rusb->CFIFOCTR = USB_CFIFOCTR_BVAL;
  }

  pipe->remaining = rem - len;
  return false;
}

static bool pipe0_xfer_out(dcd_data_t *dcd, struct st_usb20 *rusb) {
  pipe_state_t *pipe = &dcd->pipe[0];
  const unsigned rem = pipe->remaining;

  const uint16_t mps = edpt0_max_packet_size(rusb);
  const uint16_t vld = REG_READ_FIELD(rusb->CFIFOCTR, USB_CFIFOCTR_DTLN);
  const uint16_t len = tu_min16(tu_min16(rem, mps), vld);
  void *buf = pipe->buf;

  if (len) {
    rusb1_fifo_t fifo;
    rusb_cfifo(rusb, &fifo);

    if (pipe->ff) {
      if (!hw_to_sw_fifo_ff(&fifo, (tu_fifo_t *) buf, len)) {
        return false;
      };
    } else {
      if (!hw_to_sw_fifo(&fifo, buf, len)) {
        return false;
      };
      pipe->buf = (uint8_t *) buf + len;
    }
  }

  if (len < mps) {
    rusb->CFIFOCTR = USB_CFIFOCTR_BCLR;
  }

  pipe->remaining = rem - len;
  if ((len < mps) || (rem == len)) {
    pipe->buf = NULL;
    return true;
  }

  return false;
}

static bool pipe_xfer_in(dcd_data_t * dcd, struct st_usb20 *rusb, unsigned num) {
  rusb1_fifo_t fifo;
  pipe_state_t *pipe = &dcd->pipe[num];
  const unsigned rem = pipe->remaining;

  if (!rem) {
    pipe->buf = NULL;
    return true;
  }

  rusb_fifo0(rusb, &fifo);

  *fifo.sel = 0 | REG_VAL(USB_DnFIFOSEL_CURPIPE, num) | REG_VAL(USB_DnFIFOSEL_MBW, RUSB1_FIFOSEL_MBW_32BIT) | (TU_BYTE_ORDER == TU_BIG_ENDIAN ? USB_DnFIFOSEL_BIGEND : 0);

  const uint16_t mps = edpt_max_packet_size(rusb, num);
  fifo_wait_for_ready(&fifo, num);
  const uint16_t len = tu_min16(rem, mps);
  void *buf = pipe->buf;

  if (len) {
    if (pipe->ff) {
      if (!sw_to_hw_fifo_ff(&fifo, (tu_fifo_t *) buf, len)) {
        return false;
      }
    } else {
      if (!sw_to_hw_fifo(&fifo, buf, len)) {
        return false;
      }
      pipe->buf = (uint8_t *) buf + len;
    }
  }

  if (len < mps) {
    *fifo.ctr = USB_CFIFOCTR_BVAL;
  }

  *fifo.sel = 0;
  while (REG_READ_FIELD(*fifo.sel, USB_DnFIFOSEL_CURPIPE)) {} /* if CURPIPE bits changes, check written value */

  pipe->remaining = rem - len;

  return false;
}

static bool pipe_xfer_out(dcd_data_t * dcd, struct st_usb20 *rusb, unsigned num) {
  pipe_state_t *pipe = &dcd->pipe[num];
  const uint16_t rem = pipe->remaining;

  rusb1_fifo_t fifo;
  rusb_fifo0(rusb, &fifo);

  *fifo.sel = REG_VAL(USB_DnFIFOSEL_CURPIPE, num);
  const uint16_t mps = edpt_max_packet_size(rusb, num);
  fifo_wait_for_ready(&fifo, num);

  fifo_set_mbw(&fifo, RUSB1_FIFOSEL_MBW_8BIT);

  const uint16_t vld = REG_READ_FIELD(*fifo.ctr, USB_DnFIFOCTR_DTLN);
  const uint16_t len = tu_min16(tu_min16(rem, mps), vld);
  void *buf = pipe->buf;

  if (len) {
    if (pipe->ff) {
      if (!hw_to_sw_fifo_ff(&fifo, (tu_fifo_t *) buf, len)) {
        return false;
      }
    } else {
      if (!hw_to_sw_fifo(&fifo, buf, len)) {
        return false;
      }
      pipe->buf = (uint8_t *) buf + len;
    }
  }

  if (len < mps) {
    *fifo.ctr = USB_DnFIFOCTR_BCLR;
  }

  *fifo.sel = 0;
  while (REG_READ_FIELD(*fifo.sel, USB_DnFIFOSEL_CURPIPE)) {} /* if CURPIPE bits changes, check written value */

  pipe->remaining = rem - len;
  if ((len < mps) || (rem == len)) {
    pipe->buf = NULL;
    return NULL != buf;
  }

  return false;
}

static void process_setup_packet(uint8_t rhport) {
  struct st_usb20 *rusb = RUSB1_REG(rhport);
  if (0 == (rusb->INTSTS0 & USB_INTSTS0_VALID)) return;

  rusb->CFIFOCTR = USB_CFIFOCTR_BCLR;
  uint16_t setup_packet[4] = {
      tu_htole16(rusb->USBREQ),
      tu_htole16(rusb->USBVAL),
      tu_htole16(rusb->USBINDX),
      tu_htole16(rusb->USBLENG)};

  rusb->INTSTS0 = ~((uint16_t) USB_INTSTS0_VALID);
  dcd_event_setup_received(rhport, (const uint8_t *) &setup_packet[0], true);
}

static void process_status_completion(uint8_t rhport) {
  struct st_usb20 *rusb = RUSB1_REG(rhport);
  uint8_t ep_addr;
  /* Check the data stage direction */
  if (rusb->CFIFOSEL & USB_CFIFOSEL_ISEL_) {
    /* IN transfer. */
    ep_addr = tu_edpt_addr(0, TUSB_DIR_IN);
  } else {
    /* OUT transfer. */
    ep_addr = tu_edpt_addr(0, TUSB_DIR_OUT);
  }

  dcd_event_xfer_complete(rhport, ep_addr, 0, XFER_RESULT_SUCCESS, true);
}

static bool process_pipe0_xfer(dcd_data_t *dcd, struct st_usb20 *rusb, int buffer_type, uint8_t ep_addr, void *buffer, uint16_t total_bytes) {
  /* configure fifo direction and access unit settings */
  if (ep_addr) {
    /* IN, 4 bytes */
    rusb->CFIFOSEL = USB_CFIFOSEL_ISEL_ | REG_VAL(USB_CFIFOSEL_MBW, RUSB1_FIFOSEL_MBW_32BIT) | (TU_BYTE_ORDER == TU_BIG_ENDIAN ? USB_CFIFOSEL_BIGEND : 0);
    while (!(rusb->CFIFOSEL & USB_CFIFOSEL_ISEL_)) {}
  } else {
    /* OUT, a byte */
    rusb->CFIFOSEL = REG_VAL(USB_CFIFOSEL_MBW, RUSB1_FIFOSEL_MBW_32BIT);
    while (rusb->CFIFOSEL & USB_CFIFOSEL_ISEL_) {}
  }

  pipe_state_t *pipe = &dcd->pipe[0];
  pipe->ff = buffer_type;
  pipe->length = total_bytes;
  pipe->remaining = total_bytes;

  if (total_bytes) {
    pipe->buf = buffer;
    if (ep_addr) {
      /* IN */
      TU_ASSERT(REG_READ_FIELD(rusb->DCPCTR, USB_DCPCTR_BSTS) == 1);
      TU_ASSERT(REG_READ_FIELD(rusb->USBREQ, USB_USBREQ_BMREQUESTTYPE) & 0x80);
      pipe0_xfer_in(dcd, rusb);
    }
    rusb->DCPCTR = REG_VAL(USB_DCPCTR_PID, RUSB1_DCPCTR_PID_BUF);
  } else {
    /* ZLP */
    pipe->buf = NULL;
    /* This combination of bits will send the ACK */
    rusb->DCPCTR = REG_VAL(USB_DCPCTR_PID, RUSB1_DCPCTR_PID_BUF) | USB_DCPCTR_CCPL;
  }

  return true;
}

static bool process_pipe_xfer(dcd_data_t *dcd, struct st_usb20 *rusb, int buffer_type, uint8_t ep_addr, void *buffer, uint16_t total_bytes) {
  const unsigned epn = tu_edpt_number(ep_addr);
  const unsigned dir = tu_edpt_dir(ep_addr);
  const unsigned num = dcd->ep[dir][epn];

  TU_ASSERT(num);

  rusb1_fifo_t fifo;
  pipe_state_t *pipe = &dcd->pipe[num];
  pipe->ff = buffer_type;
  pipe->buf = buffer;
  pipe->length = total_bytes;
  pipe->remaining = total_bytes;

  rusb_fifo0(rusb, &fifo);

  if (dir) {
    /* IN */
    if (total_bytes) {
      pipe_xfer_in(dcd, rusb, num);
    } else {
      /* ZLP */
      *fifo.sel = num;
      fifo_wait_for_ready(&fifo, num);
      /* Immediately mark the buffer as "ready", causing the peripheral to ACK */
      *fifo.ctr = USB_DnFIFOCTR_BVAL;
      *fifo.sel = 0;
      while (REG_READ_FIELD(*fifo.sel, USB_DnFIFOSEL_CURPIPE) != 0) {}
    }
  } else {
    // OUT
    volatile struct st_usb20_from_pipe1tre *pt = get_pipetre(rusb, num);

    if (pt) {
      const uint16_t mps = edpt_max_packet_size(rusb, num);
      volatile uint16_t *ctr = get_pipectr(rusb, num);

      // If the pipe isn't in NAK mode, set it to NAK mode so we can modify its configuration
      if (REG_READ_FIELD(*ctr, USB_PIPEnCTR_1_5_PID) != RUSB1_PIPE_CTR_PID_NAK) {
        *ctr = REG_VAL(USB_PIPEnCTR_1_5_PID, RUSB1_PIPE_CTR_PID_NAK);
      }

      // Clear the transaction counter and then reset it to the expected number of packets
      pt->PIPE1TRE = USB_PIPEnTRE_TRCLR;
      pt->PIPE1TRN = (total_bytes + mps - 1) / mps;
      pt->PIPE1TRE = USB_PIPEnTRE_TRENB;

      // re-enable the buffer state dependent NAK responses
      *ctr = REG_VAL(USB_PIPEnCTR_1_5_PID, RUSB1_PIPE_CTR_PID_BUF);
    }
  }

  //  TU_LOG2("X %x %d %d\r\n", ep_addr, total_bytes, buffer_type);
  return true;
}

static bool process_edpt_xfer(dcd_data_t * dcd, struct st_usb20 *rusb, int buffer_type, uint8_t ep_addr, void *buffer, uint16_t total_bytes) {
  const unsigned epn = tu_edpt_number(ep_addr);
  if (0 == epn) {
    return process_pipe0_xfer(dcd, rusb, buffer_type, ep_addr, buffer, total_bytes);
  } else {
    return process_pipe_xfer(dcd, rusb, buffer_type, ep_addr, buffer, total_bytes);
  }
}

static void process_pipe0_bemp(uint8_t rhport) {
  struct st_usb20 *rusb = RUSB1_REG(rhport);
  bool completed = pipe0_xfer_in(dcd_for_rhport(rhport), rusb);
  if (completed) {
    pipe_state_t *pipe = &dcd_for_rhport(rhport)->pipe[0];
    dcd_event_xfer_complete(rhport, tu_edpt_addr(0, TUSB_DIR_IN),
                            pipe->length, XFER_RESULT_SUCCESS, true);
  }
}

static void process_pipe_brdy(uint8_t rhport, unsigned num) {
  dcd_data_t * dcd = dcd_for_rhport(rhport);
  struct st_usb20 *rusb = RUSB1_REG(rhport);
  pipe_state_t *pipe = &dcd_for_rhport(rhport)->pipe[num];
  const unsigned dir = tu_edpt_dir(pipe->ep);
  bool completed;

  if (dir) {
    /* IN */
    completed = pipe_xfer_in(dcd, rusb, num);
  } else {
    // OUT
    if (num) {
      completed = pipe_xfer_out(dcd, rusb, num);
    } else {
      completed = pipe0_xfer_out(dcd, rusb);
    }
  }
  if (completed) {
    dcd_event_xfer_complete(rhport, pipe->ep,
                            pipe->length - pipe->remaining,
                            XFER_RESULT_SUCCESS, true);
    //  TU_LOG1("C %d %d\r\n", num, pipe->length - pipe->remaining);
  }
}

static void process_bus_reset(uint8_t rhport) {
  struct st_usb20 *rusb = RUSB1_REG(rhport);

  rusb->BRDYENB = 1;
  rusb->CFIFOCTR = USB_DnFIFOCTR_BCLR;

  rusb->D0FIFOSEL = 0;
  while ((REG_READ_FIELD(rusb->D0FIFOSEL, USB_DnFIFOSEL_CURPIPE)) != 0) {}

  rusb->D1FIFOSEL = 0;
  while ((REG_READ_FIELD(rusb->D1FIFOSEL, USB_DnFIFOSEL_CURPIPE)) != 0) {}

  volatile uint16_t *ctr = (volatile uint16_t *) ((uintptr_t) (&rusb->PIPE1CTR));
  volatile uint16_t *tre = (volatile uint16_t *) ((uintptr_t) (&rusb->PIPE1TRE));

  // Clear the various pipes
  for (int i = 1; i <= 15; ++i) {
    rusb->PIPESEL = i;
    rusb->PIPECFG = 0;
    *ctr = USB_PIPEnCTR_1_5_ACLRM;
    *ctr = 0;
    ++ctr;
    // pipes 1-5 and 9-15 (9, a-f) have adjacent TRE registers. Pipes 6-8 do not
    // have TRE registers to clear. The order doesn't match exactly, but these
    // registers can be modified independently of the CTR registers.
    if (i <= 5 || 9 <= i) {
      *tre = USB_PIPEnTRE_TRCLR;
      // Add 2 to skip the TRN registers
      tre += 2;
    }
  }

  TU_VERIFY_STATIC(USB_PIPEnCTR_1_5_ACLRM == USB_PIPEnCTR_6_8_ACLRM);
  TU_VERIFY_STATIC(USB_PIPEnCTR_1_5_ACLRM == USB_PIPEnCTR_9_ACLRM);
  TU_VERIFY_STATIC(USB_PIPEnCTR_1_5_ACLRM == USB_PIPEnCTR_A_F_ACLRM);

  dcd_data_t * dcd = dcd_for_rhport(rhport);

  tu_varclr(dcd);

#if CFG_TUSB_DEBUG
  // The first 4 blocks are always used by EP0
  dcd->used_packet_blocks[0] |= 0b1111;
#endif
  // This is automatically set on peripheral reset, make sure our internal tracking matches it.
  dcd->pipe[0].config.buffer_size = 4;

  TU_LOG3("Bus reset, RHST = %u\r\n", REG_READ_FIELD(rusb->DVSTCTR0, USB_DVSTCTR0_RHST));
  tusb_speed_t speed;
  switch (REG_READ_FIELD(rusb->DVSTCTR0, USB_DVSTCTR0_RHST)) {
    case RUSB1_DVSTCTR0_RHST_LS:
      speed = TUSB_SPEED_LOW;
      break;

    case RUSB1_DVSTCTR0_RHST_FS:
      speed = TUSB_SPEED_FULL;
      break;

    case RUSB1_DVSTCTR0_RHST_HS:
      speed = TUSB_SPEED_HIGH;
      break;

    default:
      TU_ASSERT(false, );
  }

  dcd_event_bus_reset(rhport, speed, true);
}

static void process_set_address(uint8_t rhport) {
  struct st_usb20 *rusb = RUSB1_REG(rhport);
  const uint16_t addr = REG_READ_FIELD(rusb->USBADDR, USB_USBADDR_USBADDR);
  if (!addr) return;

  const tusb_control_request_t setup_packet = {
  #if defined(__CCRX__)
    .bmRequestType = {0}, /* Note: CCRX needs the braces over this struct member */
  #else
    .bmRequestType = 0,
  #endif
    .bRequest = TUSB_REQ_SET_ADDRESS,
    .wValue = addr,
    .wIndex = 0,
    .wLength = 0,
  };

  dcd_event_setup_received(rhport, (const uint8_t *) &setup_packet, true);
}

/*------------------------------------------------------------------*/
/* Device API
 *------------------------------------------------------------------*/

bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rh_init;
  // We always need access to the USB0 registers, as reset and oscillator
  // control is done through that port.
  struct st_usb20 *rusb0 = RUSB1_REG(0);
  struct st_usb20 *rusb = RUSB1_REG(rhport);

  // We disable SOF for now until needed later on.
  // Since TinyUSB doesn't use SOF for now, and this interrupt often (1ms interval)
  dcd_for_rhport(rhport)->sof_enabled = false;

  // Disable clock to both modules, since we're about to modify the clock configuration bits
  bool needs_reenable_ip0 = !!REG_READ_FIELD(rusb0->SUSPMODE, USB_SUSPMODE_SUSPM);
  REG_RMW_FIELD(rusb->SUSPMODE, USB_SUSPMODE_SUSPM, 0);

  // Configure the module clock inputs
  REG_RMW_FIELD(rusb0->SYSCFG0, USB_SYSCFG_UCKSEL, RUSB1_CLOCK_SOURCE);
  REG_RMW_FIELD(rusb0->SYSCFG0, USB_SYSCFG_UPLLE, 1);

  // Wait for the clocks to stabilize
  delay_ms(10);

  // Set CPU busy wait cycles
  rusb->BUSWAIT = RUSB1_WAIT_CYCLES;

  // Reenable module clocks
  REG_RMW_FIELD(rusb->SUSPMODE, USB_SUSPMODE_SUSPM, 1);
  if (needs_reenable_ip0) {
    REG_RMW_FIELD(rusb0->SUSPMODE, USB_SUSPMODE_SUSPM, 1);
  }

  // If HS operation is desired, enable it
  REG_RMW_FIELD(rusb->SYSCFG0, USB_SYSCFG_HSE, (TUD_OPT_HIGH_SPEED ? 1 : 0));

  // Select USB function mode
  REG_RMW_FIELD(rusb->SYSCFG0, USB_SYSCFG_DCFM, 0);

  // Configure pulls
  REG_RMW_FIELD(rusb->SYSCFG0, USB_SYSCFG_DRPD, 0); // D+/D- pulldowns
  REG_RMW_FIELD(rusb->SYSCFG0, USB_SYSCFG_DPRPU, 0);// D+ pullup

  // Enable the module
  REG_RMW_FIELD(rusb->SYSCFG0, USB_SYSCFG_USBE, 1);

  // Wait for peripheral to boot
  delay_ms(10);

  /* Setup default control pipe */
  REG_RMW_FIELD(rusb->DCPMAXP, USB_DCPMAXP_MXPS, 64);

  // Clear any pending interrupts and enable all the interrupts we want on the control pipe
  rusb->INTSTS0 = 0;
  rusb->INTENB0 = 0
    | USB_INTENB0_VBSE  // VBus interrupt
    | USB_INTENB0_BRDYE // Buffer Ready
    | USB_INTENB0_BEMPE // Buffer Empty
    | USB_INTENB0_DVSE  // Device State change
    | USB_INTENB0_CTRE  // Control Transfer Stage Transition
    | USB_INTENB0_RSME; // Resume
  rusb->BEMPENB = 1;
  rusb->BRDYENB = 1;

  // If VBUS (detect) pin is not used, application need to call tud_connect() manually after tud_init()
  if (REG_READ_FIELD(rusb->INTSTS0, USB_INTSTS0_VBSTS)) {
    dcd_connect(rhport);
  }

  return true;
}

void dcd_int_enable(uint8_t rhport) {
  // USB interrupts are on 73 and 74
  uint32_t const base_interrupt = 73;
  uint32_t const interrupt = base_interrupt + rhport;

  // Interrupt Set Enable register enables the interrupts that have a bit set in the written value
  volatile uint32_t *addr = (volatile uint32_t *) &INTC.ICDISER0;
  uint32_t mask = 1u << (interrupt & 0x1f);

  *(addr + (interrupt >> 5)) = mask;
}

void dcd_int_disable(uint8_t rhport) {
  // USB interrupts are on 73 and 74
  uint32_t const base_interrupt = 73;
  uint32_t const interrupt = base_interrupt + rhport;

  // Interrupt Clear Enable register disables the interrupts that have a bit set in the written value
  volatile uint32_t *addr = (volatile uint32_t *) &INTC.ICDICER0;
  uint32_t mask = 1u << (interrupt & 0x1f);

  *(addr + (interrupt >> 5)) = mask;
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr) {
  (void) rhport;
  (void) dev_addr;
}

void dcd_remote_wakeup(uint8_t rhport) {
  struct st_usb20 *rusb = RUSB1_REG(rhport);
  REG_RMW_FIELD(rusb->DVSTCTR0, USB_DVSTCTR0_WKUP, 1);
}

void dcd_connect(uint8_t rhport) {
  struct st_usb20 *rusb = RUSB1_REG(rhport);

  REG_RMW_FIELD(rusb->SYSCFG0, USB_SYSCFG_DPRPU, 1);
}

void dcd_disconnect(uint8_t rhport) {
  struct st_usb20 *rusb = RUSB1_REG(rhport);
  REG_RMW_FIELD(rusb->SYSCFG0, USB_SYSCFG_DPRPU, 0);
}

void dcd_sof_enable(uint8_t rhport, bool en) {
  struct st_usb20 *rusb = RUSB1_REG(rhport);
  dcd_for_rhport(rhport)->sof_enabled = en;
  REG_RMW_FIELD(rusb->INTENB0, USB_INTENB0_SOFE, en ? 1 : 0);
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+
bool rusb1_configure_pipe(uint8_t rhport, uint8_t ep, tusb_dir_t ep_dir, uint8_t pipe, rusb1_pipe_config_t const * pipe_cfg) {
  const unsigned buffer_offset = pipe_cfg->buffer_offset;
  const unsigned blocks_used = pipe_cfg->buffer_size * (pipe_cfg->flags.double_buffer ? 2 : 1);

  // Pipe 0 is autoconfigured by peripheral reset, there are only
  TU_ASSERT(1 <= pipe && pipe < PIPE_COUNT);
  // Only pipes 1-5 and 9-15 are allowed to use continous transfer mode.
  TU_ASSERT(!pipe_cfg->flags.continuous || pipe <= 5 || pipe >= 9);
  // Only pipes 1-5 and 9-15 are allowed to use double-buffer mode.
  TU_ASSERT(!pipe_cfg->flags.double_buffer || pipe <= 5 || pipe >= 9);

  // All pipes must be <= 2048 bytes in length
  TU_ASSERT(pipe_cfg->buffer_size <= 2048 / RUSB1_PACKET_BUFFER_BLOCK_SIZE_BYTES);

  switch(pipe) {
  case 6:
  case 7:
  case 8:
    // The interrupt pipes (6-8) only support 1-block buffers
    TU_ASSERT(pipe_cfg->buffer_size == 1);
    break;
  default:
    break;
  }

  dcd_data_t * dcd = dcd_for_rhport(rhport);
  pipe_state_t * state = &dcd->pipe[pipe];

  // Check we haven't used this pipe already
  TU_ASSERT(state->ep == 0);

#if CFG_TUSB_DEBUG
  // Check the blocks being assigned are not used by another endpoint already
  const unsigned last_block = buffer_offset + blocks_used;
  for (int block = buffer_offset; block < last_block; ++block) {
    uint32_t mask = 1 << (block & 0x1f);
    unsigned block_block = block >> 5;
    TU_ASSERT(!(dcd->used_packet_blocks[block_block] & mask));
    dcd->used_packet_blocks[block_block] |= mask;
  }
#endif

  state->config = *pipe_cfg;
  state->ep = tu_edpt_addr(ep, ep_dir);
  dcd->ep[ep_dir][ep] = pipe;

  return true;
}

bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const *ep_desc) {
  dcd_data_t * dcd = dcd_for_rhport(rhport);
  struct st_usb20 *rusb = RUSB1_REG(rhport);
  const unsigned ep_addr = ep_desc->bEndpointAddress;
  const unsigned epn = tu_edpt_number(ep_addr);
  const unsigned dir = tu_edpt_dir(ep_addr);
  const unsigned xfer = ep_desc->bmAttributes.xfer;
  const unsigned mps = tu_edpt_packet_size(ep_desc);
  unsigned pipe = dcd->ep[dir][epn];

  TU_LOG2("dcd_edpt_open: EP 0x%02X (ep=%d, dir=%d) -> pipe=%d\r\n", ep_addr, epn, dir, pipe);

  // If pipe not configured yet, auto-allocate one
  if (pipe == 0) {
    TU_LOG2("  Pipe not pre-configured, auto-allocating...\r\n");

    // Find a free pipe suitable for this transfer type
    // Pipes 6-8 for interrupt, Pipes 1-5 for bulk/iso
    unsigned start_pipe, end_pipe;
    if (xfer == TUSB_XFER_INTERRUPT) {
      start_pipe = 6;
      end_pipe = 8;
    } else {
      start_pipe = 1;
      end_pipe = 5;
    }

    // Find first free pipe in range
    for (unsigned p = start_pipe; p <= end_pipe; p++) {
      if (dcd->pipe[p].ep == 0) {
        pipe = p;
        TU_LOG2("  Auto-allocated PIPE %u for EP 0x%02X (xfer=%d)\r\n", pipe, ep_addr, xfer);

        // Configure the pipe
        rusb1_pipe_config_t cfg;
        // Blocks 0-3 are reserved for EP0, so start at block 4

        if (xfer == TUSB_XFER_ISOCHRONOUS) {
          // Isochronous endpoints need larger buffers for audio streaming
          // Calculate buffer size based on endpoint max packet size
          // buffer_size field: 0=64B, 1=64B, 2=128B, 3=256B, 4=512B, 5=1024B
          unsigned buffer_size_field = 0;
          if (mps > 512) buffer_size_field = 5;       // 1024 bytes
          else if (mps > 256) buffer_size_field = 4;  // 512 bytes
          else if (mps > 128) buffer_size_field = 3;  // 256 bytes
          else if (mps > 64) buffer_size_field = 2;   // 128 bytes
          else buffer_size_field = 1;                 // 64 bytes

          cfg.buffer_offset = 4 + (p * 4);  // Larger spacing for iso pipes
          cfg.buffer_size = buffer_size_field;
          cfg.flags.double_buffer = 1;  // Enable double buffering for iso
          cfg.flags.continuous = 1;     // Enable continuous mode for iso
          TU_LOG2("  ISO endpoint: mps=%u, buffer_size_field=%u, offset=%u\r\n",
                  mps, buffer_size_field, cfg.buffer_offset);
        } else {
          // Bulk/interrupt endpoints
          // Calculate buffer size based on endpoint max packet size
          // buffer_size field: 0=64B, 1=64B, 2=128B, 3=256B, 4=512B, 5=1024B
          unsigned buffer_size_field = 1;  // Default 64 bytes
          if (mps > 512) buffer_size_field = 5;       // 1024 bytes
          else if (mps > 256) buffer_size_field = 4;  // 512 bytes
          else if (mps > 128) buffer_size_field = 3;  // 256 bytes
          else if (mps > 64) buffer_size_field = 2;   // 128 bytes

          cfg.buffer_offset = 4 + (p * 10);  // Space pipes further apart for larger buffers
          cfg.buffer_size = buffer_size_field;
          cfg.flags.double_buffer = (xfer == TUSB_XFER_BULK) ? 1 : 0;
          cfg.flags.continuous = (xfer == TUSB_XFER_BULK) ? 1 : 0;
          TU_LOG2("  Bulk/INT endpoint: mps=%u, buffer_size_field=%u, offset=%u\r\n",
                  mps, buffer_size_field, cfg.buffer_offset);
        }

        if (!rusb1_configure_pipe(rhport, epn, dir, pipe, &cfg)) {
          TU_LOG2("  Failed to auto-configure pipe\r\n");
          return false;
        }
        break;
      }
    }
  }

  TU_ASSERT(0 < pipe && pipe < PIPE_COUNT);

  pipe_state_t * pipe_state = &dcd->pipe[pipe];

  // Check to make sure the pipe has a configuration
  TU_ASSERT(pipe_state->ep == ep_addr);

  pipe_state->xfer = xfer;

  // Check that the endpoint and pipe configuration is valid
#if CFG_TUSB_DEBUG
  #if TUD_OPT_HIGH_SPEED
    if ((pipe == 1 || pipe == 2) && xfer == TUSB_XFER_ISOCHRONOUS) {
      TU_ASSERT(1 <= mps && mps <= 1024);
    } else if (pipe <= 5 || pipe >= 9) {
      TU_ASSERT(xfer == TUSB_XFER_BULK);
      TU_ASSERT(mps == 512);
    } else if (6 <= pipe && pipe <= 8) {
      TU_ASSERT(xfer == TUSB_XFER_INTERRUPT);
      TU_ASSERT(1 <= mps && mps <= 64);
    }
  #else
    if ((pipe == 1 || pipe == 2) && xfer == TUSB_XFER_ISOCHRONOUS) {
      TU_ASSERT(1 <= mps && mps <= 1024);
    } else if (pipe <= 5 || pipe >= 9) {
      TU_ASSERT(xfer == TUSB_XFER_BULK);
      TU_ASSERT(mps == 8 || mps == 16 || mps == 32 || mps == 64);
    } else if (6 <= pipe && pipe <= 8) {
      TU_ASSERT(xfer == TUSB_XFER_INTERRUPT);
      TU_ASSERT(1 <= mps && mps <= 64);
    }
  #endif

  // Decode buffer_size field to actual byte count
  // buffer_size encoding: 0,1=64B, 2=128B, 3=256B, 4=512B, 5=1024B
  unsigned pipe_buffer_size_bytes;
  if (pipe_state->config.buffer_size <= 1) {
    pipe_buffer_size_bytes = 64;
  } else if (pipe_state->config.buffer_size == 2) {
    pipe_buffer_size_bytes = 128;
  } else if (pipe_state->config.buffer_size == 3) {
    pipe_buffer_size_bytes = 256;
  } else if (pipe_state->config.buffer_size == 4) {
    pipe_buffer_size_bytes = 512;
  } else { // 5
    pipe_buffer_size_bytes = 1024;
  }
  TU_ASSERT(pipe_buffer_size_bytes >= mps);
#endif

  /* setup pipe */
  dcd_int_disable(rhport);

  rusb->PIPESEL = pipe;
  rusb->PIPEBUF =
    REG_VAL(USB_PIPEBUF_BUFNMB, pipe_state->config.buffer_offset) |
    REG_VAL(USB_PIPEBUF_BUFSIZE, pipe_state->config.buffer_size);
  rusb->PIPEMAXP = mps;
  volatile uint16_t *ctr = get_pipectr(rusb, pipe);
  // Disable auto-buffer clear and reset the data toggle
  *ctr = USB_PIPEnCTR_1_5_ACLRM | USB_PIPEnCTR_1_5_SQCLR;
  // Set the Response PID mode to NAK (deny all transactions on this pipe)
  *ctr = 0;

  // Configure the pipe to respond to this endpoint/direction pair
  unsigned cfg = (dir << 4) | epn;

  switch (xfer) {
  case TUSB_XFER_BULK:
    // BULK type
    cfg |= REG_VAL(USB_PIPECFG_TYPE, 0b01);
    // Disable pipe after transfer
    cfg |= REG_VAL(USB_PIPECFG_SHTNAK, 0b1);
    // Use double-buffer mode
    cfg |= REG_VAL(USB_PIPECFG_DBLB, 0b1);
    TU_ASSERT(pipe_state->config.flags.double_buffer);
    break;
  case TUSB_XFER_INTERRUPT:
    // INTERRUPT mode
    cfg |= REG_VAL(USB_PIPECFG_TYPE, 0b10);
    TU_ASSERT(!pipe_state->config.flags.double_buffer);
    break;
  case TUSB_XFER_ISOCHRONOUS:
    // Isochronous mode
    cfg |= REG_VAL(USB_PIPECFG_TYPE, 0b11);
    // Use double-buffer mode
    cfg |= REG_VAL(USB_PIPECFG_DBLB, 0b1);
    TU_ASSERT(pipe_state->config.flags.double_buffer);
    break;
  case TUSB_XFER_CONTROL:
    // Only pipe 0 can be the control pipe, so we sholdn't get here.
  default:
    TU_ASSERT(!"Unsupported transfer mode");
  }

  rusb->PIPECFG = cfg;

  // Enable the "buffer ready" interrupt for this pipe
  rusb->BRDYSTS = 0x3FFu ^ TU_BIT(pipe);
  rusb->BRDYENB |= TU_BIT(pipe);

  // dir == TUSB_DIR_IN
  if (dir || (xfer != TUSB_XFER_BULK)) {
    // The next response on this endpoint/pipe depends on the buffer state
    // (BUF setting)
    *ctr = REG_VAL(USB_PIPEnCTR_1_5_PID, RUSB1_PIPE_CTR_PID_BUF);
  }

  // TU_LOG1("O %d %x %x\r\n", rusb->PIPESEL, rusb->PIPECFG, rusb->PIPEMAXP);
  dcd_int_enable(rhport);

  return true;
}

void dcd_edpt_close_all(uint8_t rhport) {
  dcd_data_t * dcd = dcd_for_rhport(rhport);
  unsigned i = TU_ARRAY_SIZE(dcd->pipe);
  dcd_int_disable(rhport);
  while (--i) { /* Close all pipes except 0 */
    const unsigned ep_addr = dcd->pipe[i].ep;
    if (!ep_addr) continue;
    dcd_edpt_close(rhport, ep_addr);
  }
  dcd_int_enable(rhport);
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr) {
  dcd_data_t * dcd = dcd_for_rhport(rhport);
  struct st_usb20 *rusb = RUSB1_REG(rhport);
  const unsigned epn = tu_edpt_number(ep_addr);
  const unsigned dir = tu_edpt_dir(ep_addr);
  const unsigned num = dcd->ep[dir][epn];

  rusb->BRDYENB &= ~TU_BIT(num);
  volatile uint16_t *ctr = get_pipectr(rusb, num);
  *ctr = 0;
  rusb->PIPESEL = num;
  rusb->PIPECFG = 0;
  dcd->pipe[num].ep = 0;
  dcd->ep[dir][epn] = 0;
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes) {
  struct st_usb20 *rusb = RUSB1_REG(rhport);

  dcd_int_disable(rhport);
  bool r = process_edpt_xfer(dcd_for_rhport(rhport), rusb, 0, ep_addr, buffer, total_bytes);
  dcd_int_enable(rhport);

  return r;
}

bool dcd_edpt_xfer_fifo(uint8_t rhport, uint8_t ep_addr, tu_fifo_t *ff, uint16_t total_bytes) {
  // USB buffers always work in bytes so to avoid unnecessary divisions we demand item_size = 1
  TU_ASSERT(ff->item_size == 1);
  struct st_usb20 *rusb = RUSB1_REG(rhport);

  dcd_int_disable(rhport);
  bool r = process_edpt_xfer(dcd_for_rhport(rhport), rusb, 1, ep_addr, ff, total_bytes);
  dcd_int_enable(rhport);

  return r;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr) {
  volatile uint16_t *ctr = ep_addr_to_pipectr(rhport, ep_addr);
  if (!ctr) return;
  dcd_int_disable(rhport);
  // Set STALL condition for the relevant pipe
  //
  // - To make a transition from NAK (00) to STALL, set 10
  // - To make a transition from BUF (01) to STALL, set 11
  const uint32_t pid = REG_READ_FIELD(*ctr, USB_PIPEnCTR_1_5_PID);
  *ctr = pid | REG_VAL(USB_PIPEnCTR_1_5_PID, RUSB1_PIPE_CTR_PID_STALL10);
  *ctr = REG_VAL(USB_PIPEnCTR_1_5_PID, RUSB1_PIPE_CTR_PID_STALL11);
  dcd_int_enable(rhport);
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr) {
  dcd_data_t * dcd = dcd_for_rhport(rhport);
  unsigned ep_dir = tu_edpt_dir(ep_addr);
  unsigned ep_num = tu_edpt_number(ep_addr);
  unsigned pipe = dcd->ep[ep_dir][ep_num];
  pipe_state_t * pipe_state = &dcd->pipe[pipe];
  struct st_usb20 *rusb = RUSB1_REG(rhport);
  volatile uint16_t *ctr = ep_addr_to_pipectr(rhport, ep_addr);
  if (!ctr) return;

  dcd_int_disable(rhport);
  // Clear the toggle bit and transition PID to NAK mode
  const uint32_t pid = REG_READ_FIELD(*ctr, USB_PIPEnCTR_1_5_PID);

  if (ep_dir == TUSB_DIR_IN || pipe_state->xfer != TUSB_XFER_BULK) {
    // - To make a transition from STALL to BUF, set 00 (NAK) and then 01 (BUF)
    *ctr = USB_PIPEnCTR_1_5_SQCLR;
    *ctr = REG_VAL(USB_PIPEnCTR_1_5_PID, RUSB1_PIPE_CTR_PID_BUF);
  } else {
    // - To make a transition from STALL (11) to NAK, set 10 and then 00
    *ctr = USB_PIPEnCTR_1_5_SQCLR | REG_VAL(USB_PIPEnCTR_1_5_PID, RUSB1_PIPE_CTR_PID_BUF);
    *ctr = REG_VAL(USB_PIPEnCTR_1_5_PID, RUSB1_PIPE_CTR_PID_NAK);
  }

  dcd_int_enable(rhport);
}

//--------------------------------------------------------------------+
// ISR
//--------------------------------------------------------------------+

  #if defined(__CCRX__)
TU_ATTR_ALWAYS_INLINE static inline unsigned __builtin_ctz(unsigned int value) {
  unsigned int count = 0;
  while ((value & 1) == 0) {
    value >>= 1;
    count++;
  }
  return count;
}
  #endif

void dcd_int_handler(uint8_t rhport) {
  struct st_usb20 *rusb = RUSB1_REG(rhport);
  dcd_data_t * dcd = dcd_for_rhport(rhport);

  uint16_t is0 = rusb->INTSTS0;

  /* clear active bits except VALID (don't write 0 to already cleared bits according to the HW manual) */
  rusb->INTSTS0 = ~((USB_INTSTS0_CTRT | USB_INTSTS0_DVST | USB_INTSTS0_SOFR |
                     USB_INTSTS0_RESM | USB_INTSTS0_VBINT) & is0) |
                  USB_INTSTS0_VALID;

  // VBUS changes
  if (is0 & USB_INTSTS0_VBINT) {
    if (REG_READ_FIELD(rusb->INTSTS0, USB_INTSTS0_VBSTS)) {
      dcd_connect(rhport);
    } else {
      dcd_disconnect(rhport);
    }
  }

  // Resumed
  if (is0 & USB_INTSTS0_RESM) {
    dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);
    if (!dcd->sof_enabled) {
      REG_RMW_FIELD(rusb->INTENB0, USB_INTENB0_SOFE, 0);
    }
  }

  // SOF received
  if ((is0 & USB_INTSTS0_SOFR) && REG_READ_FIELD(rusb->INTENB0, USB_INTENB0_SOFE)) {
    // USBD will exit suspended mode when SOF event is received
    const uint32_t frame = REG_READ_FIELD(rusb->FRMNUM, USB_FRMNUM_FRNM);
    dcd_event_sof(rhport, frame, true);
    if (!dcd->sof_enabled) {
      REG_RMW_FIELD(rusb->INTENB0, USB_INTENB0_SOFE, 0);
    }
  }

  // Device state changes
  if (is0 & USB_INTSTS0_DVST) {
    switch (REG_READ_FIELD(is0, USB_INTSTS0_DVSQ)) {
      case RUSB1_INTSTS0_DVSQ_DEFAULT:
        process_bus_reset(rhport);
        break;

      case RUSB1_INTSTS0_DVSQ_ADDRESS:
        process_set_address(rhport);
        break;

      case RUSB1_INTSTS0_DVSQ_SUSP0:
      case RUSB1_INTSTS0_DVSQ_SUSP1:
      case RUSB1_INTSTS0_DVSQ_SUSP2:
      case RUSB1_INTSTS0_DVSQ_SUSP3:
        dcd_event_bus_signal(rhport, DCD_EVENT_SUSPEND, true);
        if (!dcd->sof_enabled) {
          REG_RMW_FIELD(rusb->INTENB0, USB_INTENB0_SOFE, 0);
        }

      default:
        break;
    }
  }

  // Control transfer stage changes
  if (is0 & USB_INTSTS0_CTRT) {
    unsigned control_stage = REG_READ_FIELD(is0, USB_INTSTS0_CTSQ);
    TU_LOG3("Control stage %d\r\n", control_stage);
    if (control_stage == RUSB1_INTSTS0_CTSQ_IDLE) {
      // Control has gone idle, report completion
      process_status_completion(rhport);
    } else {
      // A setup packet has been received.
      process_setup_packet(rhport);
    }
  }

  // Buffer empty
  if (is0 & USB_INTSTS0_BEMP) {
    const uint16_t s = rusb->BEMPSTS;
    rusb->BEMPSTS = 0;
    if (s & 1) {
      process_pipe0_bemp(rhport);
    }
  }

  // Buffer ready
  if (is0 & USB_INTSTS0_BRDY) {
    const unsigned m = rusb->BRDYENB;
    unsigned s = rusb->BRDYSTS & m;
    /* clear active bits (don't write 0 to already cleared bits according to the HW manual) */
    rusb->BRDYSTS = ~s;
    while (s) {
      const unsigned num = __builtin_ctz(s);
      process_pipe_brdy(rhport, num);
      s &= ~TU_BIT(num);
    }
  }
}

#endif
