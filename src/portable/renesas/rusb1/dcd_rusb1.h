/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Sapphire Koser
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

#ifndef _RUSB1_DCD_RUSB1_H_
#define _RUSB1_DCD_RUSB1_H_

#include "device/dcd.h"

TU_ATTR_PACKED_BEGIN

typedef struct rusb1_pipe_config {
  // Start offset, in 64-byte blocks, of for the packet buffer in the packet memory.
  uint8_t buffer_offset;

  // Size, in blocks, of the packet memory.
  uint8_t buffer_size;

  struct TU_ATTR_PACKED {
    uint8_t double_buffer : 1;
    uint8_t continuous : 1;
  } flags;
} rusb1_pipe_config_t;

TU_ATTR_PACKED_END

// Configure a pipe for use with a specific endpoint.
bool rusb1_configure_pipe(uint8_t rhport, uint8_t ep, tusb_dir_t ep_dir, uint8_t pipe, rusb1_pipe_config_t const * desc_ep);

#endif
