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
  uint16_t mps;       /* max packet size, cached to avoid PIPESEL+PIPEMAXP on every BRDY */

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
  if ((1 <= num) && (num <= 5)) {
    // Pipes 1-5 are contiguous in memory — use array indexing.
    return &((volatile struct st_usb20_from_pipe1tre *) (&rusb->PIPE1TRE))[num - 1];
  }
  // Pipes 9-F also have TRE/TRN pairs, but they are NOT contiguous with pipes 1-5
  // (the hardware layout is B,C,D,E,F,9,A after pipe 5).  Pipes 6-8 have no TRE/TRN.
  switch (num) {
    case  9: return (volatile struct st_usb20_from_pipe1tre *) &rusb->PIPE9TRE;
    case 10: return (volatile struct st_usb20_from_pipe1tre *) &rusb->PIPEATRE;
    case 11: return (volatile struct st_usb20_from_pipe1tre *) &rusb->PIPEBTRE;
    case 12: return (volatile struct st_usb20_from_pipe1tre *) &rusb->PIPECTRE;
    case 13: return (volatile struct st_usb20_from_pipe1tre *) &rusb->PIPEDTRE;
    case 14: return (volatile struct st_usb20_from_pipe1tre *) &rusb->PIPEETRE;
    case 15: return (volatile struct st_usb20_from_pipe1tre *) &rusb->PIPEFTRE;
    default: return NULL; // pipes 6-8 have no TRE/TRN registers
  }
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

static inline void rusb_fifo1(struct st_usb20 *rusb, rusb1_fifo_t *fifo) {
  fifo->data = &rusb->D1FIFO.UINT32;
  fifo->sel = &rusb->D1FIFOSEL;
  fifo->ctr = &rusb->D1FIFOCTR;
}

// Route ISO pipes (1-2) to D0FIFO and bulk/interrupt pipes (3+) to D1FIFO.
// This eliminates FIFO port contention: a bulk CDC transfer can no longer
// block an isochronous audio transfer from accessing the hardware FIFO.
static inline void rusb_fifo_for_pipe(struct st_usb20 *rusb, rusb1_fifo_t *fifo, unsigned pipe_num) {
  if (pipe_num <= 2) {
    rusb_fifo0(rusb, fifo);
  } else {
    rusb_fifo1(rusb, fifo);
  }
}

// Wait for the FIFO port to reflect the requested pipe (CURPIPE) and for
// the FIFO buffer to become ready (FRDY).
//
// Returns true on success, false on failure. Callers must deselect the FIFO
// and abort the transfer when false is returned;
//
static inline bool fifo_is_ready(rusb1_fifo_t *fifo, unsigned num) {
  if ((REG_READ_FIELD(*fifo->sel, USB_DnFIFOSEL_CURPIPE)) != num) {
    return false;
  }
  if (!REG_READ_FIELD(*fifo->ctr, USB_CFIFOCTR_FRDY)) {
    return false;
  }
  return true;
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
  // MBW=32 is already set by the CURPIPE select write — no initial fifo_set_mbw needed.
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

// Select the widest MBW that is valid for ALL fragments of a single FIFO read.
// MBW must not change once reading begins (TRM §28.3.8).  When a packet spans
// the ring-buffer wrap in hw_to_sw_fifo_ff, both the linear and wrap fragments
// must use the same width; choosing from the minimum alignment of all fragments
// prevents an illegal mid-packet MBW switch.
// Pass wrap=0 when there is only one fragment.
static inline uint32_t fifo_mbw_for_frags(uint16_t lin, uint16_t wrap) {
  if ((lin % 4 == 0) && (wrap == 0 || wrap % 4 == 0)) return RUSB1_FIFOSEL_MBW_32BIT;
  if ((lin % 2 == 0) && (wrap == 0 || wrap % 2 == 0)) return RUSB1_FIFOSEL_MBW_16BIT;
  return RUSB1_FIFOSEL_MBW_8BIT;
}

// Raw FIFO drain: reads `len` bytes into `buf` at the given MBW.
// MBW must already be set in the SEL register before the first call and must
// not change between fragments of the same packet (TRM §28.3.8).
static inline void hw_fifo_drain(volatile uint32_t *data, uint8_t *buf, unsigned len, uint32_t mbw) {
  if (mbw == RUSB1_FIFOSEL_MBW_32BIT) {
    while (len >= 4) { *((uint32_t *)buf) = *data; buf += 4; len -= 4; }
  } else if (mbw == RUSB1_FIFOSEL_MBW_16BIT) {
    volatile uint16_t *d16 = (volatile uint16_t *)data;
    while (len >= 2) { *((uint16_t *)buf) = *d16; buf += 2; len -= 2; }
  } else {
    volatile uint8_t *d8 = (volatile uint8_t *)data;
    while (len != 0) { *buf++ = *d8; len--; }
  }
}

// Read data buffer <-- hw fifo
//
// On entry MBW in the SEL register is already RUSB1_FIFOSEL_MBW_32BIT (set by
// the CURPIPE select write in pipe_xfer_out / process_pipe0_xfer).  We only
// call fifo_set_mbw when a narrower width is required, saving one RMW P1-bus
// access per ISO BRDY for the 4-byte-aligned (16-bit stereo) common case.
//
// RZA1 hardware quirk: writing D1FIFOSEL (even to change only MBW with CURPIPE
// unchanged) re-triggers the FIFO data-port switching state machine.  If the
// first data read happens before the port settles the hardware returns 0xFF.
// For payloads shorter than one 32-bit word we avoid the MBW change entirely:
// read a single MBW=32 word (already configured) and unpack the valid bytes.
// TRM §28.3.8 specifies that the hardware pads short FIFO reads to the full
// MBW width, so the first `len` bytes are always valid regardless of padding.
static bool hw_to_sw_fifo(rusb1_fifo_t *fifo, uint8_t *buf, unsigned len) {
  if (len < 4) {
    // Sub-word payload: keep MBW=32, read one word, unpack valid bytes.
    uint32_t word = *fifo->data;
    if (len >= 1) *buf++ = (uint8_t)(word);
    if (len >= 2) *buf++ = (uint8_t)(word >> 8);
    if (len >= 3) *buf++ = (uint8_t)(word >> 16);
    return true;
  }
  uint32_t mbw = fifo_mbw_for_frags((uint16_t)len, 0);
  if (mbw != RUSB1_FIFOSEL_MBW_32BIT) {
    fifo_set_mbw(fifo, mbw);
  }
  hw_fifo_drain(fifo->data, buf, len, mbw);
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

  tu_fifo_advance_read_pointer(sw_fifo, count);
  return true;
}

// Read data sw fifo <-- hw fifo
//
// MBW is chosen ONCE from the combined alignment of the linear and wrap
// fragments, then set before the first byte is read (TRM §28.3.8 compliance).
// For HS UAC2 audio both cases are typically one fifo_set_mbw call at most:
//   16-bit stereo (4 B/frame): 4-aligned → MBW=32 always, skip fifo_set_mbw.
//   24-bit stereo (6 B/frame): 2-aligned → MBW=16, one call regardless of split.
static bool hw_to_sw_fifo_ff(rusb1_fifo_t *hw_fifo, tu_fifo_t *sw_fifo, uint16_t total_len) {
  tu_fifo_buffer_info_t info;
  tu_fifo_get_write_info(sw_fifo, &info);

  uint16_t lin  = tu_min16(total_len, info.len_lin);
  uint16_t wrap = total_len - lin;

  // Set MBW once before the first byte is read.
  // MBW=32 is already present from the CURPIPE select write; only switch if narrower.
  uint32_t mbw = fifo_mbw_for_frags(lin, wrap);
  if (mbw != RUSB1_FIFOSEL_MBW_32BIT) {
    fifo_set_mbw(hw_fifo, mbw);
  }

  hw_fifo_drain(hw_fifo->data, info.ptr_lin, lin, mbw);
  if (wrap) {
    hw_fifo_drain(hw_fifo->data, info.ptr_wrap, wrap, mbw);
  }

  tu_fifo_advance_write_pointer(sw_fifo, lin + wrap);
  return true;
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

  rusb_fifo_for_pipe(rusb, &fifo, num);

  *fifo.sel = 0 | REG_VAL(USB_DnFIFOSEL_CURPIPE, num) | REG_VAL(USB_DnFIFOSEL_MBW, RUSB1_FIFOSEL_MBW_32BIT) | (TU_BYTE_ORDER == TU_BIG_ENDIAN ? USB_DnFIFOSEL_BIGEND : 0);

  const uint16_t mps = pipe->mps; // cached — avoids PIPESEL write + PIPEMAXP read every BRDY
  if (!fifo_is_ready(&fifo, num)) {
    // FIFO not ready — leave CURPIPE as-is (the next FIFO access will
    // overwrite it directly, matching the Renesas USBx/SDK pattern of
    // never deselecting DnFIFO between operations).
    return false;
  }
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

  // Leave CURPIPE pointing at this pipe — no deselect.  The next FIFO
  // access (possibly for a different pipe) will overwrite CURPIPE directly.
  // This eliminates the deselect→reselect cycle that caused cascading FRDY
  // timeouts when multiple bulk pipes shared D1FIFO.

  pipe->remaining = rem - len;

  return false;
}

static bool pipe_xfer_out(dcd_data_t * dcd, struct st_usb20 *rusb, unsigned num) {
  pipe_state_t *pipe = &dcd->pipe[num];

  rusb1_fifo_t fifo;
  rusb_fifo_for_pipe(rusb, &fifo, num);

  // Select the pipe; MBW is initially set to 32-bit here so that CURPIPE+MBW
  // are written simultaneously per the TRM requirement for changing CURPIPE.
  // hw_to_sw_fifo() overrides MBW to 8-bit before the first data read —
  // changing MBW before reading starts is compliant per TRM section 28.3.8.
  *fifo.sel = REG_VAL(USB_DnFIFOSEL_CURPIPE, num) | REG_VAL(USB_DnFIFOSEL_MBW, RUSB1_FIFOSEL_MBW_32BIT)
            | (TU_BYTE_ORDER == TU_BIG_ENDIAN ? USB_DnFIFOSEL_BIGEND : 0);
  const uint16_t mps = pipe->mps; // cached — avoids PIPESEL write + PIPEMAXP read every BRDY
  if (!fifo_is_ready(&fifo, num)) {
    // FIFO not ready — leave CURPIPE as-is (see pipe_xfer_in comment).
    return false;
  }

  // Guard: if no transfer is active (spurious BRDY after completion),
  // don't read data.
  //
  // For ISO OUT: we MUST still issue BCLR to release the hardware buffer back to
  // the SIE.  ISO pipes are double-buffered and receive every microframe
  // unconditionally.  If BCLR is not issued, the buffer stays "occupied" from
  // the SIE's perspective; after two consecutive missed BRDYs both double-buffer
  // banks fill up and the SIE silently drops every subsequent audio frame,
  // causing audible artefacts whenever the class driver is even briefly late
  // re-arming between transfers.
  //
  // For bulk/interrupt OUT: keep the buffer occupied (no BCLR) so the pipe NAKs
  // until the class driver explicitly re-arms via dcd_edpt_xfer.
  if (!pipe->buf) {
    if (pipe->xfer == TUSB_XFER_ISOCHRONOUS) {
      *fifo.ctr = USB_DnFIFOCTR_BCLR;
    }
    return false;
  }

  const uint16_t rem = pipe->remaining;
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

  // Always BCLR after reading.  In single-buffer mode the SIE considers the
  // buffer occupied until the CPU explicitly clears it.  Without BCLR after a
  // full-size (== mps) packet the hardware NAKs every subsequent packet and
  // BRDY never fires again.  (With double-buffering the alternate buffer
  // masked this, but single-buffer mode requires it unconditionally.)
  *fifo.ctr = USB_DnFIFOCTR_BCLR;

  // Leave CURPIPE pointing at this pipe — no deselect (see pipe_xfer_in).

  pipe->remaining = rem - len;
  if ((len < mps) || (rem == len)) {
    pipe->buf = NULL;
    return true;
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
    // Wait for the ISEL direction bit to take effect.  CFIFOSEL register echoes
    // back the new value within a few P1-bus cycles; 1000 iterations @ 33MHz ≈ 30µs
    // which is far more than enough for a register echo.  The bound prevents an
    // infinite stall if the peripheral is in a bad state during enumeration.
    unsigned isel_tries = 1000;
    while (!(rusb->CFIFOSEL & USB_CFIFOSEL_ISEL_) && --isel_tries) {}
  } else {
    /* OUT, a byte */
    rusb->CFIFOSEL = REG_VAL(USB_CFIFOSEL_MBW, RUSB1_FIFOSEL_MBW_32BIT);
    unsigned isel_tries = 1000;
    while ((rusb->CFIFOSEL & USB_CFIFOSEL_ISEL_) && --isel_tries) {}
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

  TU_LOG2("process_pipe_xfer: EP %02X, pipe %d, xfer type %d, bytes %d\r\n", ep_addr, num, pipe->xfer, total_bytes);

  rusb_fifo_for_pipe(rusb, &fifo, num);

  if (dir) {
    /* IN */
    volatile uint16_t *ctr = get_pipectr(rusb, num);

    if (total_bytes) {
      const unsigned rem_before = pipe->remaining;
      pipe_xfer_in(dcd, rusb, num);

      // If pipe_xfer_in failed to write any data (e.g. FRDY timeout because
      // both double-buffer slots are full), do NOT set PID=BUF.  Arming the
      // pipe with an empty FIFO creates a ghost transfer: the SIE NAKs all
      // host IN tokens (no data to send), BRDY never fires (no full→empty
      // transition), and the pipe is stuck forever.
      //
      // Instead, reset the pipe state and return false so the class driver
      // can retry later.
      if (pipe->remaining == rem_before && rem_before > 0) {
        pipe->buf = NULL;
        pipe->remaining = 0;
        pipe->length = 0;
        return false;
      }
    } else {
      /* ZLP */
      *fifo.sel = num;
      if (!fifo_is_ready(&fifo, num)) {
        // FIFO not ready for ZLP — leave CURPIPE as-is (see pipe_xfer_in).
        return false;
      }
      /* Immediately mark the buffer as "ready", causing the peripheral to ACK */
      *fifo.ctr = USB_DnFIFOCTR_BVAL;
      // Leave CURPIPE pointing at this pipe — no deselect.
    }

    // Re-arm the pipe to BUF for bulk and interrupt IN endpoints.
    //
    // Bulk IN pipes are opened with SHTNAK=1: after the SIE sends the last
    // (short) packet the hardware reverts PID to NAK automatically.  If the
    // main loop is slow to re-submit (e.g. while draining audio), the pipe
    // stays NAK and the next dcd_edpt_xfer() writes data to the FIFO but the
    // SIE never transmits it — ep.busy is stuck at 1 forever.
    //
    // Fix: explicitly set PID=BUF here, after we have written data into the
    // FIFO.  Writing BUF when the pipe is already BUF is harmless.
    //
    // ISO IN pipes use continuous BUF mode managed by dcd_edpt_iso_activate;
    // they must NOT be touched here — changing their PID mid-stream disrupts
    // the isochronous schedule.
    if (pipe->xfer != TUSB_XFER_ISOCHRONOUS) {
      *ctr = REG_VAL(USB_PIPEnCTR_1_5_PID, RUSB1_PIPE_CTR_PID_BUF);
    }
  } else {
    // OUT
    volatile uint16_t *ctr = get_pipectr(rusb, num);

    // Isochronous transfers don't use the transaction counter (TRE/TRN)
    // They are time-based and don't use ACK/NAK handshaking
    if (pipe->xfer == TUSB_XFER_ISOCHRONOUS) {
      // ISO OUT: the pipe stays in BUF mode continuously (set in dcd_edpt_iso_activate).
      // No hardware reconfiguration is needed between transfers — the pipe keeps
      // receiving every microframe regardless.  The buffer pointers above are all
      // that need updating.
      (void)ctr; // PID is already BUF — nothing to do
    } else {
      // Bulk/Interrupt OUT: do NOT use the transaction counter (TRE/TRN).
      //
      // The USBx reference driver never enables TRE — it relies solely on
      // SHTNAK (auto-NAK after a short packet) and manual PID=BUF re-arm.
      //
      // Using TRN=1 with double-buffering is problematic: at High-Speed the
      // host can send back-to-back packets faster than the SIE decrements
      // TRN, causing the second packet to be accepted into the alternate
      // buffer.  When the firmware drains only the first buffer and re-arms
      // with a fresh TRN=1, the stale second buffer triggers a spurious
      // BRDY or is silently lost, eventually stalling the pipe.
      //
      // Instead: simply ensure PID=BUF so the pipe accepts packets.  BRDY
      // fires on each received packet; pipe_xfer_out drains the buffer and
      // reports completion when a short packet arrives (len < mps) or the
      // requested byte count is reached.

      // Make sure TRE is disabled (clear TRENB) so the hardware transaction
      // counter doesn't interfere.
      volatile struct st_usb20_from_pipe1tre *pt = get_pipetre(rusb, num);
      if (pt) {
        pt->PIPE1TRE = USB_PIPEnTRE_TRCLR;
      }

      // Set PID=BUF to accept incoming packets
      *ctr = REG_VAL(USB_PIPEnCTR_1_5_PID, RUSB1_PIPE_CTR_PID_BUF);
    }
  }

  TU_LOG2("X %x %d %d\r\n", ep_addr, total_bytes, buffer_type);
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

  TU_LOG2("process_pipe_brdy: pipe %d, EP %02X, dir %d\r\n", num, pipe->ep, dir);

  if (dir) {
    /* IN */
    completed = pipe_xfer_in(dcd, rusb, num);
  } else {
    // OUT
    if (num) {
      // For bulk/interrupt OUT pipes: set PID=NAK BEFORE reading the FIFO.
      //
      // All bulk OUT pipes are single-buffer (see endpoint-open comment).
      // By setting NAK while the buffer is still full, the host can't land
      // a new packet in the window between BCLR and re-arm.
      //
      // ISO OUT stays in continuous BUF mode — no NAK.
      bool needs_nak = (pipe->xfer != TUSB_XFER_ISOCHRONOUS);
      volatile uint16_t *ctr = NULL;
      if (needs_nak) {
        ctr = get_pipectr(rusb, num);
        *ctr = REG_VAL(USB_PIPEnCTR_1_5_PID, RUSB1_PIPE_CTR_PID_NAK);
      }

      completed = pipe_xfer_out(dcd, rusb, num);

      // Re-arm to BUF if the transfer needs more data.
      if (!completed && needs_nak && pipe->buf) {
        *ctr = REG_VAL(USB_PIPEnCTR_1_5_PID, RUSB1_PIPE_CTR_PID_BUF);
      }
    } else {
      completed = pipe0_xfer_out(dcd, rusb);
    }
  }

  TU_LOG2("BRDY pipe %d completed=%d, transferred=%d\r\n", num, completed, pipe->length - pipe->remaining);

  if (completed) {
    uint16_t xferred = pipe->length - pipe->remaining;
    TU_LOG2("Calling dcd_event_xfer_complete: EP %02X, %d bytes\r\n", pipe->ep, xferred);
    dcd_event_xfer_complete(rhport, pipe->ep, xferred, XFER_RESULT_SUCCESS, true);
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
  // EP0 uses 4 blocks = 256 bytes, BUFSIZE = 4-1 = 3
  dcd->pipe[0].config.buffer_size = 3;

  TU_LOG2("Bus reset, RHST = %u\r\n", REG_READ_FIELD(rusb->DVSTCTR0, USB_DVSTCTR0_RHST));
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

  // NOTE: Interrupt enables are set in dcd_connect() after D+ pullup is enabled,
  // as the RZA1L peripheral requires VBUS detection before these registers become writable.

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

  // Enable D+ pullup to signal device presence to host
  REG_RMW_FIELD(rusb->SYSCFG0, USB_SYSCFG_DPRPU, 1);

  // NOTE: On RZA1L, interrupt enable registers (INTENB0, BEMPENB, BRDYENB) only
  // become writable after VBUS is detected and the peripheral enters an active state.
  // This happens asynchronously after enabling the D+ pullup.
  // Applications must enable interrupts manually.
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
  // BUFSIZE register encoding: buffer_bytes = 64 * (BUFSIZE + 1)
  // Number of 64-byte blocks per buffer = (BUFSIZE + 1)
  const unsigned blocks_used = (pipe_cfg->buffer_size + 1) * (pipe_cfg->flags.double_buffer ? 2 : 1);

  // Pipe 0 is autoconfigured by peripheral reset, there are only
  TU_ASSERT(1 <= pipe && pipe < PIPE_COUNT);
  // Only pipes 1-5 and 9-15 are allowed to use continous transfer mode.
  TU_ASSERT(!pipe_cfg->flags.continuous || pipe <= 5 || pipe >= 9);
  // Only pipes 1-5 and 9-15 are allowed to use double-buffer mode.
  TU_ASSERT(!pipe_cfg->flags.double_buffer || pipe <= 5 || pipe >= 9);

  // All pipes must be <= 2048 bytes in length (BUFSIZE max = (2048/64)-1 = 31)
  TU_ASSERT(pipe_cfg->buffer_size <= (2048 / RUSB1_PACKET_BUFFER_BLOCK_SIZE_BYTES) - 1);

  switch(pipe) {
  case 6:
  case 7:
  case 8:
    // The interrupt pipes (6-8) only support 1-block buffers (BUFSIZE=0 → 64B)
    TU_ASSERT(pipe_cfg->buffer_size == 0);
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

//--------------------------------------------------------------------+
// ISO Alloc/Activate - pre-allocate pipe once, activate per alt setting
//--------------------------------------------------------------------+

bool dcd_edpt_iso_alloc(uint8_t rhport, uint8_t ep_addr, uint16_t largest_packet_size) {
  dcd_data_t *dcd = dcd_for_rhport(rhport);
  struct st_usb20 *rusb = RUSB1_REG(rhport);
  const unsigned epn = tu_edpt_number(ep_addr);
  const unsigned dir = tu_edpt_dir(ep_addr);

  // Find a free ISO pipe (only pipes 1-2)
  unsigned pipe = 0;
  for (unsigned p = 1; p <= 2; p++) {
    if (dcd->pipe[p].ep == 0) {
      pipe = p;
      break;
    }
  }
  TU_ASSERT(pipe != 0);

  // Calculate PIPEBUF BUFSIZE field.
  // Hardware encoding: buffer_bytes = 64 * (BUFSIZE + 1)
  // BUFSIZE = (ceil_to_64(mps) / 64) - 1
  unsigned buffer_size_field = ((largest_packet_size + 63) / 64) - 1;
  if (buffer_size_field > 31) buffer_size_field = 31; // 5-bit field max

  // Compute a non-overlapping buffer offset: find the first free block after all
  // currently configured pipes.  A fixed stride (e.g. 4 + pipe*4) silently overlaps
  // when buffer_size >= 3 (>128-byte frames, e.g. 44100 Hz stereo 16-bit).
  unsigned next_offset = 4; // blocks 0-3 reserved for EP0
  for (unsigned p = 1; p < (unsigned) PIPE_COUNT; p++) {
    const pipe_state_t *ps = &dcd->pipe[p];
    if (ps->ep != 0) {
      unsigned end = ps->config.buffer_offset +
                     (ps->config.buffer_size + 1) * (ps->config.flags.double_buffer ? 2 : 1);
      if (end > next_offset) next_offset = end;
    }
  }

  // BUFNMB must be aligned to (BUFSIZE+1) block boundary per hardware requirement.
  const unsigned alignment = buffer_size_field + 1;
  next_offset = (next_offset + alignment - 1) & ~(alignment - 1);

  rusb1_pipe_config_t cfg;
  cfg.buffer_offset = next_offset;
  cfg.buffer_size = buffer_size_field;
  cfg.flags.double_buffer = 1;
  cfg.flags.continuous = 1;

  TU_LOG2("dcd_edpt_iso_alloc: EP 0x%02X -> pipe %u, mps=%u, buf_field=%u\r\n",
          ep_addr, pipe, largest_packet_size, buffer_size_field);

  TU_ASSERT(rusb1_configure_pipe(rhport, epn, dir, pipe, &cfg));
  dcd->pipe[pipe].mps = largest_packet_size; // will be refined per alt-setting in iso_activate

  // Now write all the hardware registers that rusb1_configure_pipe doesn't touch.
  // rusb1_configure_pipe only sets up software state (pipe_state, ep mapping).
  // We must configure the actual RUSB1 pipe hardware here.

  dcd_int_disable(rhport);

  rusb->PIPESEL = pipe;

  // PIPEBUF: allocate FIFO buffer in USB controller RAM
  rusb->PIPEBUF =
    REG_VAL(USB_PIPEBUF_BUFNMB, cfg.buffer_offset) |
    REG_VAL(USB_PIPEBUF_BUFSIZE, buffer_size_field);

  // PIPEMAXP: set to largest packet size (will be updated per alt in iso_activate)
  rusb->PIPEMAXP = largest_packet_size;

  // PIPECFG: configure pipe type, endpoint number, direction, double-buffer
  {
    unsigned pipecfg = (dir << 4) | epn;
    // Isochronous type = 0b11
    pipecfg |= REG_VAL(USB_PIPECFG_TYPE, 0b11);
    // Enable double-buffer mode
    pipecfg |= REG_VAL(USB_PIPECFG_DBLB, 0b1);
    rusb->PIPECFG = pipecfg;
  }

  // PIPEPERI: isochronous interval
  {
    uint16_t pipeperi = 0; // interval=0 (2^0 = 1 microframe)
    if (dir == TUSB_DIR_IN) {
      pipeperi |= USB_PIPEPERI_IFIS;
    }
    rusb->PIPEPERI = pipeperi;
  }

  // Reset pipe: clear FIFO and data toggle
  volatile uint16_t *ctr = get_pipectr(rusb, pipe);
  *ctr = USB_PIPEnCTR_1_5_ACLRM | USB_PIPEnCTR_1_5_SQCLR;
  *ctr = 0;

  dcd_int_enable(rhport);

  return true;
}

bool dcd_edpt_iso_activate(uint8_t rhport, tusb_desc_endpoint_t const *desc_ep) {
  dcd_data_t *dcd = dcd_for_rhport(rhport);
  struct st_usb20 *rusb = RUSB1_REG(rhport);
  const unsigned ep_addr = desc_ep->bEndpointAddress;
  const unsigned epn = tu_edpt_number(ep_addr);
  const unsigned dir = tu_edpt_dir(ep_addr);
  const unsigned mps = tu_edpt_packet_size(desc_ep);
  const unsigned pipe = dcd->ep[dir][epn];

  TU_ASSERT(pipe == 1 || pipe == 2);

  pipe_state_t *pipe_state = &dcd->pipe[pipe];
  pipe_state->xfer = TUSB_XFER_ISOCHRONOUS;
  pipe_state->mps  = (uint16_t)mps; // update cached MPS for this alt-setting

  dcd_int_disable(rhport);

  rusb->PIPESEL = pipe;
  // Update max packet size (buffer layout stays the same)
  rusb->PIPEMAXP = mps;

  // Update PIPEPERI from the descriptor — interval and IFIS may change per alt-setting.
  {
    uint8_t interval = desc_ep->bInterval;
    if (interval > 0) interval -= 1;  // bInterval is 1-based; IITV is 0-based (2^IITV microframes)
    if (interval > 7) interval = 7;   // IITV is 3 bits
    uint16_t pipeperi = interval;
    if (dir == TUSB_DIR_IN) {
      pipeperi |= USB_PIPEPERI_IFIS;  // Flush IN buffer at end of each transfer
    }
    rusb->PIPEPERI = pipeperi;
  }

  // Reset the pipe: clear FIFO and data toggle
  volatile uint16_t *ctr = get_pipectr(rusb, pipe);
  *ctr = USB_PIPEnCTR_1_5_ACLRM | USB_PIPEnCTR_1_5_SQCLR;
  *ctr = 0;

  // Clear any pending BRDY status for this pipe and enable interrupt
  rusb->BRDYSTS = 0x3FFu ^ TU_BIT(pipe);
  rusb->BRDYENB |= TU_BIT(pipe);

  // Set PID to BUF (accept transactions)
  *ctr = REG_VAL(USB_PIPEnCTR_1_5_PID, RUSB1_PIPE_CTR_PID_BUF);

  dcd_int_enable(rhport);

  TU_LOG2("dcd_edpt_iso_activate: EP 0x%02X pipe %u mps=%u\r\n", ep_addr, pipe, mps);

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
    // Pipes 1-2: ISO (reserved for audio streaming)
    // Pipes 3-5, 9-15: BULK
    // Pipes 6-8: Interrupt
    unsigned start_pipe, end_pipe;
    if (xfer == TUSB_XFER_INTERRUPT) {
      start_pipe = 6;
      end_pipe = 8;
    } else if (xfer == TUSB_XFER_ISOCHRONOUS) {
      // Only pipes 1 and 2 support isochronous transfers
      start_pipe = 1;
      end_pipe = 2;
    } else {
      // Bulk endpoints use pipes 3-5 and 9-15
      // Pipes 1-2 are reserved for isochronous (speaker OUT + mic IN)
      start_pipe = 3;
      end_pipe = PIPE_COUNT - 1;
    }

    // Find first free pipe in range (skip interrupt-only pipes 6-8 for bulk)
    for (unsigned p = start_pipe; p <= end_pipe; p++) {
      if (xfer == TUSB_XFER_BULK && p >= 6 && p <= 8) continue;
      if (dcd->pipe[p].ep == 0) {
        pipe = p;
        TU_LOG2("  Auto-allocated PIPE %u for EP 0x%02X (xfer=%d)\r\n", pipe, ep_addr, xfer);

        // Configure the pipe
        rusb1_pipe_config_t cfg;
        // Blocks 0-3 are reserved for EP0, so start at block 4

        if (xfer == TUSB_XFER_ISOCHRONOUS) {
          // Isochronous endpoints need larger buffers for audio streaming
          // BUFSIZE = (ceil_to_64(mps) / 64) - 1; buffer_bytes = 64*(BUFSIZE+1)
          unsigned buffer_size_field = ((mps + 63) / 64) - 1;
          if (buffer_size_field > 31) buffer_size_field = 31;

          // Dynamic watermark: start after the highest occupied block.
          {
            unsigned next_offset = 4;
            for (unsigned q = 1; q < (unsigned) PIPE_COUNT; q++) {
              const pipe_state_t *ps = &dcd->pipe[q];
              if (ps->ep != 0) {
                unsigned end = ps->config.buffer_offset +
                               (ps->config.buffer_size + 1) * (ps->config.flags.double_buffer ? 2 : 1);
                if (end > next_offset) next_offset = end;
              }
            }
            // BUFNMB must be aligned to (BUFSIZE+1) block boundary per hardware requirement.
            const unsigned iso_align = buffer_size_field + 1;
            next_offset = (next_offset + iso_align - 1) & ~(iso_align - 1);
            cfg.buffer_offset = next_offset;
          }
          cfg.buffer_size = buffer_size_field;
          cfg.flags.double_buffer = 1;  // Enable double buffering for iso
          cfg.flags.continuous = 1;     // Enable continuous mode for iso
          TU_LOG2("  ISO endpoint: mps=%u, BUFSIZE=%u (%uB), offset=%u\r\n",
                  mps, buffer_size_field, 64*(buffer_size_field+1), cfg.buffer_offset);
        } else {
          // Bulk/interrupt endpoints
          // BUFSIZE = (ceil_to_64(mps) / 64) - 1; buffer_bytes = 64*(BUFSIZE+1)
          unsigned buffer_size_field = ((mps + 63) / 64) - 1;
          if (buffer_size_field > 31) buffer_size_field = 31;

          // Dynamic watermark: start after the highest occupied block.
          {
            unsigned next_offset = 4;
            for (unsigned q = 1; q < (unsigned) PIPE_COUNT; q++) {
              const pipe_state_t *ps = &dcd->pipe[q];
              if (ps->ep != 0) {
                unsigned end = ps->config.buffer_offset +
                               (ps->config.buffer_size + 1) * (ps->config.flags.double_buffer ? 2 : 1);
                if (end > next_offset) next_offset = end;
              }
            }
            // BUFNMB must be aligned to (BUFSIZE+1) block boundary per hardware requirement.
            const unsigned bulk_align = buffer_size_field + 1;
            next_offset = (next_offset + bulk_align - 1) & ~(bulk_align - 1);
            cfg.buffer_offset = next_offset;
          }
          cfg.buffer_size = buffer_size_field;
          // Single-buffer for all bulk/interrupt OUT endpoints.
          //
          // Double-buffering for bulk OUT causes a race: if a BRDY fires for
          // the alternate bank while pipe->buf==NULL (between transfer
          // completion and the class driver re-arming via tud_task), the guard
          // returns without issuing BCLR.  The unread alternate bank then
          // prevents the SIE from accepting further packets, silently dropping
          // the tail of multi-packet messages (e.g. the last 259 bytes of a
          // 771-byte UpdateDisplay frame, corrupting the OLED bottom quarter).
          //
          // Bulk IN endpoints use double-buffering: the SIE can transmit from
          // bank A while the CPU fills bank B, with no buf==NULL race.
          cfg.flags.double_buffer = (dir == TUSB_DIR_IN && xfer == TUSB_XFER_BULK) ? 1 : 0;
          cfg.flags.continuous = 0;
          TU_LOG2("  Bulk/INT endpoint: mps=%u, BUFSIZE=%u (%uB), offset=%u, dblb=%u\r\n",
                  mps, buffer_size_field, 64*(buffer_size_field+1), cfg.buffer_offset, cfg.flags.double_buffer);
        }

        if (!rusb1_configure_pipe(rhport, epn, dir, pipe, &cfg)) {
          TU_LOG2("  Failed to auto-configure pipe\r\n");
          return false;
        }

        // Configure PIPEPERI for isochronous endpoints (pipe still selected from rusb1_configure_pipe)
        if (xfer == TUSB_XFER_ISOCHRONOUS) {
          // IITV: Isochronous IN Transaction Interval (for high-speed: 2^(bInterval-1) microframes)
          // IFIS: Isochronous IN Buffer Flush Select (1 = flush buffer at end of transfer)
          uint8_t interval = ep_desc->bInterval;
          if (interval > 0) interval -= 1;  // Convert to 0-based for IITV
          if (interval > 7) interval = 7;   // IITV is 3 bits max
          uint16_t pipeperi = interval;     // IITV in bits 0-2
          if (dir == TUSB_DIR_IN) {
            pipeperi |= USB_PIPEPERI_IFIS;  // Enable buffer flush for IN endpoints
          }
          rusb->PIPEPERI = pipeperi;
          TU_LOG2("  ISO PIPEPERI: interval=%u, PIPEPERI=0x%04X\r\n", ep_desc->bInterval, pipeperi);
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
  pipe_state->mps  = (uint16_t)mps; // cache to avoid PIPESEL+PIPEMAXP on every BRDY

  // Check that the endpoint and pipe configuration is valid
  // Pipe allocation: 1-2 for ISO, 3-5/9+ for BULK, 6-8 for INTERRUPT
#if CFG_TUSB_DEBUG
  #if TUD_OPT_HIGH_SPEED
    if (pipe == 1 || pipe == 2) {
      // Pipes 1-2 reserved for isochronous
      TU_ASSERT(xfer == TUSB_XFER_ISOCHRONOUS);
      TU_ASSERT(1 <= mps && mps <= 1024);
    } else if ((3 <= pipe && pipe <= 5) || pipe >= 9) {
      // Pipes 3-5, 9+ for bulk
      TU_ASSERT(xfer == TUSB_XFER_BULK);
      TU_ASSERT(mps == 512);
    } else if (6 <= pipe && pipe <= 8) {
      TU_ASSERT(xfer == TUSB_XFER_INTERRUPT);
      TU_ASSERT(1 <= mps && mps <= 64);
    }
  #else
    if (pipe == 1 || pipe == 2) {
      // Pipes 1-2 reserved for isochronous
      TU_ASSERT(xfer == TUSB_XFER_ISOCHRONOUS);
      TU_ASSERT(1 <= mps && mps <= 1024);
    } else if ((3 <= pipe && pipe <= 5) || pipe >= 9) {
      // Pipes 3-5, 9+ for bulk
      TU_ASSERT(xfer == TUSB_XFER_BULK);
      TU_ASSERT(mps == 8 || mps == 16 || mps == 32 || mps == 64);
    } else if (6 <= pipe && pipe <= 8) {
      TU_ASSERT(xfer == TUSB_XFER_INTERRUPT);
      TU_ASSERT(1 <= mps && mps <= 64);
    }
  #endif

  // Decode BUFSIZE register value to actual byte count
  // Hardware encoding: buffer_bytes = 64 * (BUFSIZE + 1)
  unsigned pipe_buffer_size_bytes = 64 * (pipe_state->config.buffer_size + 1);
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
    // Auto-NAK on short packet receipt (OUT only; harmless for IN)
    cfg |= REG_VAL(USB_PIPECFG_SHTNAK, 0b1);
    // Double-buffering for bulk IN only — bulk OUT must be single-buffer
    // to avoid the buf==NULL BRDY race (see endpoint-open comment above).
    if (pipe_state->config.flags.double_buffer) {
      cfg |= REG_VAL(USB_PIPECFG_DBLB, 0b1);
    }
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
    // Note: CNTMD (continuous mode) is NOT used - it changes BRDY behavior
    // in ways that may not work with TinyUSB's transfer model
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

  TU_LOG1("O %d %x %x\r\n", rusb->PIPESEL, rusb->PIPECFG, rusb->PIPEMAXP);
  dcd_int_enable(rhport);

  return true;
}

static void rusb1_edpt_close(uint8_t rhport, uint8_t ep_addr) {
  dcd_data_t * dcd = dcd_for_rhport(rhport);
  struct st_usb20 *rusb = RUSB1_REG(rhport);
  const unsigned epn = tu_edpt_number(ep_addr);
  const unsigned dir = tu_edpt_dir(ep_addr);
  const unsigned num = dcd->ep[dir][epn];

  if (num == 0) return; // Already closed or never opened

#if CFG_TUSB_DEBUG
  // Free packet buffer blocks that were allocated for this pipe
  pipe_state_t* pipe_state = &dcd->pipe[num];
  if (pipe_state->config.buffer_size > 0) {
    const unsigned buffer_offset = pipe_state->config.buffer_offset;
    // Must match allocation formula from rusb1_configure_pipe
    const unsigned blocks_used = (pipe_state->config.buffer_size + 1) * (pipe_state->config.flags.double_buffer ? 2 : 1);
    const unsigned last_block = buffer_offset + blocks_used;

    for (int block = buffer_offset; block < last_block; ++block) {
      uint32_t mask = 1 << (block & 0x1f);
      unsigned block_block = block >> 5;
      dcd->used_packet_blocks[block_block] &= ~mask;
    }
  }
#endif

  rusb->BRDYENB &= ~TU_BIT(num);
  volatile uint16_t *ctr = get_pipectr(rusb, num);
  *ctr = 0;
  rusb->PIPESEL = num;
  rusb->PIPECFG = 0;
  dcd->pipe[num].ep = 0;
  dcd->ep[dir][epn] = 0;
}

void dcd_edpt_close_all(uint8_t rhport) {
  dcd_data_t * dcd = dcd_for_rhport(rhport);
  unsigned i = TU_ARRAY_SIZE(dcd->pipe);
  dcd_int_disable(rhport);
  while (--i) { /* Close all pipes except 0 */
    const unsigned ep_addr = dcd->pipe[i].ep;
    if (!ep_addr) continue;
    rusb1_edpt_close(rhport, ep_addr);
  }
  dcd_int_enable(rhport);
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
    // USBD will exit suspended mode when SOF event is received.
    // Also check for CRC errors and FIFO overruns here — FRMNUM is only read
    // when we are already paying the cost of a SOF handler entry (1ms / 125µs),
    // avoiding an unconditional register fetch on every BRDY ISR.
    const uint16_t frmnum = rusb->FRMNUM;
    if (frmnum & (USB_FRMNUM_CRCE | USB_FRMNUM_OVRN)) {
      TU_LOG1("USB ERR: FRMNUM=0x%04X%s%s\r\n", frmnum,
              (frmnum & USB_FRMNUM_CRCE) ? " CRCE" : "",
              (frmnum & USB_FRMNUM_OVRN) ? " OVRN" : "");
      // Clear sticky error flags (write 0 to clear)
      rusb->FRMNUM = (uint16_t)~(USB_FRMNUM_CRCE | USB_FRMNUM_OVRN);
    }
    const uint32_t frame = REG_READ_FIELD(frmnum, USB_FRMNUM_FRNM);
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
    TU_LOG2("Control stage %d\r\n", control_stage);
    if (control_stage == RUSB1_INTSTS0_CTSQ_IDLE) {
      // Control has gone idle, report completion
      process_status_completion(rhport);
    } else {
      // A setup packet has been received.
      process_setup_packet(rhport);
    }
  }

  // Buffer empty (fires only for pipe 0 control IN — infrequently).
  if (is0 & USB_INTSTS0_BEMP) {
    // Read BEMPENB-masked status so we only act on enabled pipes, and only
    // clear the bits we actually read (per TRM: don't write 0 to bits we
    // didn't observe — they may have been set by hardware since the read).
    const uint16_t m_bemp = rusb->BEMPENB;
    const uint16_t s_bemp = rusb->BEMPSTS & m_bemp;
    rusb->BEMPSTS = (uint16_t)~s_bemp; // RC-W0: clear only what we saw
    if (s_bemp & 1) {
      process_pipe0_bemp(rhport);
    }
  }

  // Buffer ready — hot path at bInterval=1 HS (up to 16 000 BRDY events/sec
  // across 2 ISO pipes + CDC + MIDI).  ISO audio pipes are on D0FIFO (pipes
  // 1-2) while CDC/MIDI are on D1FIFO (pipes 3+), so no FIFO port contention
  // exists between groups.  We process both groups in a single masked pass;
  // __builtin_ctz naturally picks the lowest-numbered pipe first, which means
  // ISO pipe 1 → ISO pipe 2 → bulk/interrupt pipes, matching hardware priority.
  if (is0 & USB_INTSTS0_BRDY) {
    const unsigned m = rusb->BRDYENB;
    unsigned s = rusb->BRDYSTS & m;
    /* clear active bits (don't write 0 to already cleared bits according to the HW manual) */
    rusb->BRDYSTS = (uint16_t)~s;
    while (s) {
      const unsigned num = __builtin_ctz(s);
      process_pipe_brdy(rhport, num);
      s &= ~TU_BIT(num);
    }
  }
}

#endif
