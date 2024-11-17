/*
 * The MIT License (MIT)
 *
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

#ifndef _RUSB1_TYPE_H_
#define _RUSB1_TYPE_H_

#define RUSB1_FIFOSEL_MBW_8BIT  (0U)
#define RUSB1_FIFOSEL_MBW_16BIT (1U)
#define RUSB1_FIFOSEL_MBW_32BIT (2U)

#define RUSB1_PIPE_CTR_PID_NAK     (0b00U)
#define RUSB1_PIPE_CTR_PID_BUF     (0b01U)
#define RUSB1_PIPE_CTR_PID_STALL10 (0b10U)
#define RUSB1_PIPE_CTR_PID_STALL11 (0b11U)

#define RUSB1_DCPCTR_PID_NAK     (0b00U)
#define RUSB1_DCPCTR_PID_BUF     (0b01U)
#define RUSB1_DCPCTR_PID_STALL10 (0b10U)
#define RUSB1_DCPCTR_PID_STALL11 (0b11U)

#define RUSB1_DVSTCTR0_RHST_LS   (0b001U)
#define RUSB1_DVSTCTR0_RHST_FS   (0b010U)
#define RUSB1_DVSTCTR0_RHST_HS   (0b011U)

#define RUSB1_INTSTS0_DVSQ_POWERED    (0b000U)
#define RUSB1_INTSTS0_DVSQ_DEFAULT    (0b001U)
#define RUSB1_INTSTS0_DVSQ_ADDRESS    (0b010U)
#define RUSB1_INTSTS0_DVSQ_CONFIGURED (0b011U)
#define RUSB1_INTSTS0_DVSQ_SUSP0      (0b100U)
#define RUSB1_INTSTS0_DVSQ_SUSP1      (0b101U)
#define RUSB1_INTSTS0_DVSQ_SUSP2      (0b110U)
#define RUSB1_INTSTS0_DVSQ_SUSP3      (0b111U)

#define RUSB1_INTSTS0_CTSQ_IDLE         (0b000)
#define RUSB1_INTSTS0_CTSQ_READ_DATA    (0b001)
#define RUSB1_INTSTS0_CTSQ_READ_STATUS  (0b010)
#define RUSB1_INTSTS0_CTSQ_WRITE_DATA   (0b011)
#define RUSB1_INTSTS0_CTSQ_WRITE_STATUS (0b100)
#define RUSB1_INTSTS0_CTSQ_WRITE_ZLP    (0b101)
#define RUSB1_INTSTS0_CTSQ_SEQ_ERR      (0b110)

#define RUSB1_PACKET_BUFFER_SIZE_BYTES       8192
#define RUSB1_PACKET_BUFFER_BLOCK_SIZE_BYTES   64
#define RUSB1_PACKET_BUFFER_SIZE_BLOCKS      (RUSB1_PACKET_BUFFER_SIZE_BYTES / RUSB1_PACKET_BUFFER_BLOCK_SIZE_BYTES)

typedef struct {
  volatile uint32_t * data;
  volatile uint16_t * sel;
  volatile uint16_t * ctr;
} rusb1_fifo_t;

#endif
