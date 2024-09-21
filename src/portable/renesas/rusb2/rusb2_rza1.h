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
//#include "r_intc.h"

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

typedef struct {
  uint32_t reg_base;
  int32_t irqnum;
}rusb2_controller_t;

#define RUSB2_CONTROLLER_COUNT USB20_COUNT

#define rusb2_is_highspeed_rhport(_p)  (true)
#define rusb2_is_highspeed_reg(_reg)   (true)

extern rusb2_controller_t rusb2_controller[];
#define RUSB2_REG(_p)      ((rusb2_reg_t*) rusb2_controller[_p].reg_base)

//--------------------------------------------------------------------+
// RUSB2 API
//--------------------------------------------------------------------+

TU_ATTR_ALWAYS_INLINE static inline void rusb2_module_start(uint8_t rhport, bool start) {
  //TODO: Needs replacing
  (void) rhport;
  (void) start;

  // Initialize the user-interrupt controller (INTC)
  // R_INTC_Init()

  // Start up the PLL stuff

}

TU_ATTR_ALWAYS_INLINE static inline void rusb2_int_enable(uint8_t rhport) {
  //R_INTC_Enable(rusb2_controller[rhport].irqnum);

  uint32_t const interrupt = rusb2_controller[rhport].irqnum;
  // Interrupt Set Enable register enables the interrupts that have a bit set in the written value
  volatile uint32_t* addr = &INTC.ICDISER0;

  /* Create mask data */
  uint32_t mask = 1u << (interrupt & 0x1f);

  /* Write ICDISERn   */
  *(addr + (interrupt >> 5)) = mask;
}

TU_ATTR_ALWAYS_INLINE static inline void rusb2_int_disable(uint8_t rhport) {
  //R_INTC_Disable(rusb2_controller[rhport].irqnum);

  uint32_t const interrupt = rusb2_controller[rhport].irqnum;
  // Interrupt Clear Enable register disables the interrupts that have a bit set in the written value
  volatile uint32_t* addr = &INTC.ICDICER0;

  /* Create mask data */
  uint32_t mask = 1u << (interrupt & 0x1f);

  /* Write ICDICERn   */
  *(addr + (interrupt >> 5)) = mask;
}

// MCU specific PHY init
TU_ATTR_ALWAYS_INLINE static inline void rusb2_phy_init(void) {
  // TODO: might need special init stuff
}



#ifdef __cplusplus
}
#endif

#endif /* _RUSB2_RZA1_H_ */
