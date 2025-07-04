/**
 ****************************************************************************************
 *
 * @file Xtal_TRIM_platform.h
 *
 * @brief Platform specifics API for automated XTAL trim calculation algorithm.
 *
 * Copyright (C) 2012-2025 Renesas Electronics Corporation and/or its affiliates.
 * All rights reserved. Confidential Information.
 *
 * This software ("Software") is supplied by Renesas Electronics Corporation and/or its
 * affiliates ("Renesas"). Renesas grants you a personal, non-exclusive, non-transferable,
 * revocable, non-sub-licensable right and license to use the Software, solely if used in
 * or together with Renesas products. You may make copies of this Software, provided this
 * copyright notice and disclaimer ("Notice") is included in all such copies. Renesas
 * reserves the right to change or discontinue the Software at any time without notice.
 *
 * THE SOFTWARE IS PROVIDED "AS IS". RENESAS DISCLAIMS ALL WARRANTIES OF ANY KIND,
 * WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. TO THE
 * MAXIMUM EXTENT PERMITTED UNDER LAW, IN NO EVENT SHALL RENESAS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE, EVEN IF RENESAS HAS BEEN ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGES. USE OF THIS SOFTWARE MAY BE SUBJECT TO TERMS AND CONDITIONS CONTAINED IN
 * AN ADDITIONAL AGREEMENT BETWEEN YOU AND RENESAS. IN CASE OF CONFLICT BETWEEN THE TERMS
 * OF THIS NOTICE AND ANY SUCH ADDITIONAL LICENSE AGREEMENT, THE TERMS OF THE AGREEMENT
 * SHALL TAKE PRECEDENCE. BY CONTINUING TO USE THIS SOFTWARE, YOU AGREE TO THE TERMS OF
 * THIS NOTICE.IF YOU DO NOT AGREE TO THESE TERMS, YOU ARE NOT PERMITTED TO USE THIS
 * SOFTWARE.
 *
 ****************************************************************************************
 */

#ifndef XTAL_TRIM_PLATFORM_H_
#define XTAL_TRIM_PLATFORM_H_

// Include Header files based on the IC platform
#ifdef __DA14531__          // DA14531

#include "rwip.h"
#include "gpio.h"

#else                       // DA14585

#include "rwip.h"
#include "gpio.h"
#endif

// XTAL_16M specific
#ifdef __DA14531__

// DA14531 uses a XTAL32M with a fixed divider{2}.
#define XTAL16M                         8000000             // 500 msec  TRIM = 1200 (16M*0.5=8M)
#define TRIM_MIN_16M                    10                  // minimum TRIM value (limits 4 - 12 pF)
#define TRIM_MAX_16M                    250                 // maximum TRIM value
#define PPM_1_16M                       10                  // 1.25 ppm (8M) - 1.04 ppm (9.6M)
#define PPM_2_16M                       20                  // 2.5 ppm  (8M) - 2.08 ppm (9.632M)
#define START_TRIM_VALUE_16M            110                 // Initial value used

#else

// XTAL_16M specific
#define XTAL16M                         8000000             // 500 msec  TRIM = 1200 (16M*0.5=8M)
#define TRIM_MIN_16M                    300                 // minimum TRIM value (limits 4 - 12 pF)
#define TRIM_MAX_16M                    2000                // maximum TRIM value
#define PPM_1_16M                       10                  // 1.25 ppm (8M) - 1.04 ppm (9.6M)
#define PPM_2_16M                       20                  // 2.5 ppm  (8M) - 2.08 ppm (9.632M)
#define START_TRIM_VALUE_16M            1150                // Initial value used

#endif

// XTAL_32M specific 69x
#define XTAL32M                         9600000             // 300 msec  TRIM = 252 (ideal 32M * 0.3 = 9.6M)
#define TRIM_MIN_32M                    10                  // minimum TRIM value (limits 4 - 12 pF)
#define TRIM_MAX_32M                    700                 // maximum TRIM value
#define PPM_1_32M                       10                  // 1.25 ppm (8M) - 1.04 ppm (9.6M)
#define PPM_2_32M                       20                  // 2.5 ppm  (8M) - 2.08 ppm (9.632M)
#define START_TRIM_VALUE_32M            110                 // Initial value used


/*
 *  Function declaration, for functions used by XTAL_Trim.c
 */
int xtal_pltfrm_get_port_pin_reg(uint8_t port_number, uint32_t *port_reg, uint16_t *pin_bit);
int xtal_pltfrm_set_port_in_pu(uint8_t port_number);
int xtal_pltfrm_get_val_from_reg(volatile uint32_t *Trim_Value);
int xtal_pltfrm_set_val_to_reg(uint32_t Trim_Value);
uint32_t xtal_pltfrm_measure_pulse(uint32_t datareg, uint16_t shift_bit);

#endif /* XTAL_TRIM_PLATFORM_H_ */
