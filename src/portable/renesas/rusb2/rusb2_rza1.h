/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Katherine Whitlock (@stellar_aria)
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

#ifndef _RUSB2_RZA1_H_
#define _RUSB2_RZA1_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "iodefine_cfg.h"

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

typedef struct {
  uint32_t reg_base;
  int32_t irqnum;
}rusb2_controller_t;

#define RUSB2_CONTROLLER_COUNT 2

#define rusb2_is_highspeed_rhport(_p)  (true)
#define rusb2_is_highspeed_reg(_reg)   (true)

extern rusb2_controller_t rusb2_controller[];
#define RUSB2_REG(_p)      ((rusb2_reg_t*) rusb2_controller[_p].reg_base)

//--------------------------------------------------------------------+
// RUSB2 API
//--------------------------------------------------------------------+

TU_ATTR_ALWAYS_INLINE static inline void rusb2_module_start(uint8_t rhport, bool start) {
  uint32_t const mask = 1U << (11+rhport);
  if (start) {
    R_MSTP->MSTPCRB &= ~mask;
  }else {
    R_MSTP->MSTPCRB |= mask;
  }
}

TU_ATTR_ALWAYS_INLINE static inline void rusb2_int_enable(uint8_t rhport) {
  NVIC_EnableIRQ(rusb2_controller[rhport].irqnum);
}

TU_ATTR_ALWAYS_INLINE static inline void rusb2_int_disable(uint8_t rhport) {
  NVIC_DisableIRQ(rusb2_controller[rhport].irqnum);
}

// MCU specific PHY init
TU_ATTR_ALWAYS_INLINE static inline void rusb2_phy_init(void) {
}



#ifdef __cplusplus
}
#endif

#endif /* _RUSB2_RZA1_H_ */
