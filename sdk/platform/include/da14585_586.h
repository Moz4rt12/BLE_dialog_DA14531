/**
 ****************************************************************************************
 *
 * @file da14585_586.h
 *
 * @brief DA14585/DA14586 register definitions header file.
 *
 * Copyright (C) 2016-2025 Renesas Electronics Corporation and/or its affiliates.
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

#ifndef _DA14585_586_H_
#define _DA14585_586_H_

#include <stdint.h>

/*===========================================================================*/ 
/*========================== Start of header part ===========================*/ 
/*===========================================================================*/ 
/*
* =============================================================================================
* ---------- Interrupt ReQuest number definitions ---------------------------------------------
* =============================================================================================
*/

/*
* --------------------------------------------------------------------------------------------
* The Cortex-M0 Processor Exceptions Numbers and the Device Specific Interrupt Numbers. -
* --------------------------------------------------------------------------------------------
*/
typedef enum IRQn
{
/****** Cortex-M0 Processor Exceptions Numbers ***********************************************/
NonMaskableInt_IRQn  = -14, /* 2 Non Maskable Interrupt */
HardFault_IRQn       = -13, /* 3 Cortex-M0 Hard Fault Interrupt */
SVCall_IRQn          = -5,  /* 11 Cortex-M0 SV Call Interrupt */
PendSV_IRQn          = -2,  /* 14 Cortex-M0 Pend SV Interrupt */
SysTick_IRQn         = -1,  /* 15 Cortex-M0 System Tick Interrupt */

/****** DA14585 Specific Interrupt Numbers ****************************************************/
BLE_WAKEUP_LP_IRQn   =  0, /* Wakeup from Low Power interrupt from BLE.                       */
BLE_GEN_IRQn         =  1, /* BLE Interrupt request. Sources:                                 */
                           /*   - finetgtim_irq  : Fine Target Timer interrupt generated when */
                           /*                      Fine Target timer expired. Timer           */
                           /*                      resolution is 625us base time reference.   */
                           /*   - grosstgtim_irq : Gross Target Timer interrupt generated     */
                           /*                      when Gross Target timer expired. Timer     */
                           /*                      resolution is 16 times 625us base time     */
                           /*                      reference.                                 */
                           /*   - cscnt_irq      : 625us base time reference interrupt,       */
                           /*                      available in active modes.                 */
                           /*   - slp_irq        : End of Sleep mode interrupt.               */
                           /*   - error_irq      : Error interrupt, generated when undesired  */
                           /*                      behavior or bad programming occurs in the  */
                           /*                      BLE Core.                                  */
                           /*   - rx_irq         : Receipt interrupt at the end of each       */
                           /*                      received packets.                          */
                           /*   - event_irq      : End of Advertising / Scanning / Connection */
                           /*                      events interrupt.                          */
                           /*   - crypt_irq      : Encryption / Decryption interrupt,         */
                           /*                      generated either when AES and/or CCM       */
                           /*                      processing is finished.                    */
                           /*   - sw_irq         : SW triggered interrupt, generated on SW    */
                           /*                      request.                                   */
UART_IRQn            =  2, /* UART interrupt.                                                 */
UART2_IRQn           =  3, /* UART2 interrupt.                                                */
I2C_IRQn             =  4, /* I2C interrupt.                                                  */
SPI_IRQn             =  5, /* SPI interrupt.                                                  */
ADC_IRQn             =  6, /* Analog-Digital Converter interrupt.                             */
KEYBRD_IRQn          =  7, /* Keyboard interrupt.                                             */
BLE_RF_DIAG_IRQn     =  8, /* Baseband or Radio Diagnostics Interrupt. Required for signaling */
                           /* Radio or Baseband internal events.                              */
                           /* 2 signals per Radio and 2 per BB                                */
RFCAL_IRQn           =  9, /* Radio Calibration interrupt.                                    */
GPIO0_IRQn           = 10, /* GPIO interrupt through debounce.                                */
GPIO1_IRQn           = 11, /* GPIO interrupt through debounce.                                */
GPIO2_IRQn           = 12, /* GPIO interrupt through debounce.                                */
GPIO3_IRQn           = 13, /* GPIO interrupt through debounce.                                */
GPIO4_IRQn           = 14, /* GPIO interrupt through debounce.                                */
SWTIM_IRQn           = 15, /* Software Timer (Timer0) interrupt.                              */
WKUP_QUADEC_IRQn     = 16, /* Combines the Wake up Capture Timer interrupt,                   */
                           /* the GPIO interrupt and the QuadDecoder interrupt                */
PCM_IRQn             = 17, /* PCM Interrupt request.                                          */
SRC_IN_IRQn          = 18, /* Sample Rate Converter Input Interrupt request.                  */
SRC_OUT_IRQn         = 19, /* Sample Rate Converter Output Interrupt request.                 */
DMA_IRQn             = 20, /* DMA Interrupt request.                                          */
RESERVED21_IRQn      = 21, /* SoftWare interrupt request.                                     */
RESERVED22_IRQn      = 22, /* SoftWare interrupt request.                                     */
RESERVED23_IRQn      = 23, /* SoftWare interrupt request                                      */
} IRQn_Type;


/*--------------------------------------------------------------------------------------------*/
/* Configuration of the Cortex-M0 Processor and Core Peripherals.                             */
/*--------------------------------------------------------------------------------------------*/
/* Note: The next 2 defines are used in "core_cm0.h" for the CMSIS Core SysTick function.     */
#define __NVIC_PRIO_BITS       (2) /* Number of Bits used for Priority Levels                 */
#define __Vendor_SysTickConfig (0) /* Set to 1 if different SysTick Config is used            */

/*--GPIO PID functions------------------------------------------------------------------------*/
#define FUNC_GPIO          ( 0)
#define FUNC_UART_RX       ( 1)
#define FUNC_UART_TX       ( 2)
#define FUNC_UART2_RX      ( 3)
#define FUNC_UART2_TX      ( 4)
#define FUNC_SPI_DI        ( 5)
#define FUNC_SPI_DO        ( 6)
#define FUNC_SPI_CLK       ( 7)
#define FUNC_SPI_EN        ( 8)
#define FUNC_I2C_SCL       ( 9)
#define FUNC_I2C_SDA       (10)
#define FUNC_UART_IRDA_RX  (11)
#define FUNC_UART_IRDA_TX  (12)
#define FUNC_UART2_IRDA_RX (13)
#define FUNC_UART2_IRDA_TX (14)
#define FUNC_ADC           (15)
#define FUNC_PWM0          (16)
#define FUNC_PWM1          (17)
#define FUNC_BLE_DIAG      (18)
#define FUNC_UART_CTSN     (19)
#define FUNC_UART_RTSN     (20)
#define FUNC_UART2_CTSN    (21)
#define FUNC_UART2_RTSN    (22)
#define FUNC_PWM2          (23)
#define FUNC_PWM3          (24)
#define FUNC_PWM4          (25)
#define FUNC_PCM_DI        (26)
#define FUNC_PCM_DO        (27)
#define FUNC_PCM_FSC       (28)
#define FUNC_PCM_CLK       (29)
#define FUNC_PDM_DATA      (30)
#define FUNC_PDM_CLK       (31)
/*--GPIO Direction settings------------------------------------------------------------------*/
#define DIR_INPUT          0x000
#define DIR_PULLUP         0x100
#define DIR_PULLDOWN       0x200
#define DIR_OUTPUT         0x300


/*===========================================================================*/ 
/*=========================== End of header part ============================*/ 
/*===========================================================================*/ 



/*===========================================================================*/ 
/*==================== Start of automatic generated code ====================*/ 
/*===========================================================================*/ 



/*===========================================================================*/
/* memory map ANAMISC */
/*===========================================================================*/



#define CLK_REF_SEL_REG                      (0x50001600) /* Select clock for oscillator calibration */
#define CLK_REF_SEL_REG_RESET                (0x00000000)

/*=============================*/
struct __CLK_REF_SEL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_REF_CLK_SEL                      : 2;
    volatile uint16_t BITFLD_REF_CAL_START                    : 1;
};
#define REF_CLK_SEL                          (0x0003)
#define REF_CAL_START                        (0x0004)



#define CLK_REF_CNT_REG                      (0x50001602) /* Count value for oscillator calibration */
#define CLK_REF_CNT_REG_RESET                (0x00000000)

/*=============================*/
struct __CLK_REF_CNT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_REF_CNT_VAL                      : 16;
};
#define REF_CNT_VAL                          (0xFFFF)



#define CLK_REF_VAL_L_REG                    (0x50001604) /* XTAL16M reference cycles, lower 16 bits */
#define CLK_REF_VAL_L_REG_RESET              (0x00000000)

/*=============================*/
struct __CLK_REF_VAL_L_REG
/*=============================*/
{
    volatile uint16_t BITFLD_XTAL_CNT_VAL                     : 16;
};
#define XTAL_CNT_VAL                         (0xFFFF)



#define CLK_REF_VAL_H_REG                    (0x50001606) /* XTAL16M reference cycles, upper 16 bits */
#define CLK_REF_VAL_H_REG_RESET              (0x00000000)

/*=============================*/
struct __CLK_REF_VAL_H_REG
/*=============================*/
{
    volatile uint16_t BITFLD_XTAL_CNT_VAL                     : 16;
};
#define XTAL_CNT_VAL                         (0xFFFF)



/*===========================================================================*/
/* memory map APU */
/*===========================================================================*/



#define SRC1_CTRL_REG                        (0x50004000) /* SRC1 control register */
#define SRC1_CTRL_REG_RESET                  (0x00000000)

/*=============================*/
struct __SRC1_CTRL_REG
/*=============================*/
{
    volatile uint32_t BITFLD_SRC_EN                           : 1;
    volatile uint32_t BITFLD_SRC_IN_AMODE                     : 1;
    volatile uint32_t BITFLD_SRC_IN_CAL_BYPASS                : 1;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_SRC_IN_DS                        : 2;
    volatile uint32_t BITFLD_SRC_IN_OK                        : 1;
    volatile uint32_t BITFLD_SRC_DITHER_DISABLE               : 1;
    volatile uint32_t                                         : 2;
    volatile uint32_t                                         : 3;
    volatile uint32_t BITFLD_SRC_OUT_AMODE                    : 1;
    volatile uint32_t BITFLD_SRC_OUT_CAL_BYPASS               : 1;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_SRC_OUT_US                       : 2;
    volatile uint32_t BITFLD_SRC_OUT_OK                       : 1;
    volatile uint32_t BITFLD_SRC_RESYNC                       : 1;
    volatile uint32_t BITFLD_SRC_IN_OVFLOW                    : 1;
    volatile uint32_t BITFLD_SRC_IN_UNFLOW                    : 1;
    volatile uint32_t BITFLD_SRC_OUT_OVFLOW                   : 1;
    volatile uint32_t BITFLD_SRC_OUT_UNFLOW                   : 1;
    volatile uint32_t BITFLD_SRC_IN_FLOWCLR                   : 1;
    volatile uint32_t BITFLD_SRC_OUT_FLOWCLR                  : 1;
    volatile uint32_t BITFLD_SRC_PDM_DI_DEL                   : 2;
    volatile uint32_t BITFLD_SRC_PDM_MODE                     : 2;
    volatile uint32_t BITFLD_SRC_PDM_DO_DEL                   : 2;
};
#define SRC_EN                               (0x0001)
#define SRC_IN_AMODE                         (0x0002)
#define SRC_IN_CAL_BYPASS                    (0x0004)
#define SRC_IN_DS                            (0x0030)
#define SRC_IN_OK                            (0x0040)
#define SRC_DITHER_DISABLE                   (0x0080)
#define SRC_OUT_AMODE                        (0x2000)
#define SRC_OUT_CAL_BYPASS                   (0x4000)
#define SRC_OUT_US                           (0x30000)
#define SRC_OUT_OK                           (0x40000)
#define SRC_RESYNC                           (0x80000)
#define SRC_IN_OVFLOW                        (0x100000)
#define SRC_IN_UNFLOW                        (0x200000)
#define SRC_OUT_OVFLOW                       (0x400000)
#define SRC_OUT_UNFLOW                       (0x800000)
#define SRC_IN_FLOWCLR                       (0x1000000)
#define SRC_OUT_FLOWCLR                      (0x2000000)
#define SRC_PDM_DI_DEL                       (0xC000000)
#define SRC_PDM_MODE                         (0x30000000)
#define SRC_PDM_DO_DEL                       (0xC0000000)



#define SRC1_IN_FS_REG                       (0x50004004) /* SRC1 Sample input rate */
#define SRC1_IN_FS_REG_RESET                 (0x00000000)

/*=============================*/
struct __SRC1_IN_FS_REG
/*=============================*/
{
    volatile uint32_t BITFLD_SRC_IN_FS                        : 24;
};
#define SRC_IN_FS                            (0xFFFFFF)



#define SRC1_OUT_FS_REG                      (0x50004008) /* SRC1 Sample output rate */
#define SRC1_OUT_FS_REG_RESET                (0x00000000)

/*=============================*/
struct __SRC1_OUT_FS_REG
/*=============================*/
{
    volatile uint32_t BITFLD_SRC_OUT_FS                       : 24;
};
#define SRC_OUT_FS                           (0xFFFFFF)



#define SRC1_IN1_REG                         (0x5000400C) /* SRC1 data in 1 */
#define SRC1_IN1_REG_RESET                   (0x00000000)

/*=============================*/
struct __SRC1_IN1_REG
/*=============================*/
{
    volatile uint32_t                                         : 8;
    volatile uint32_t BITFLD_SRC_IN                           : 24;
};
#define SRC_IN                               (0xFFFFFF00)



#define SRC1_IN2_REG                         (0x50004010) /* SRC1 data in 2 */
#define SRC1_IN2_REG_RESET                   (0x00000000)

/*=============================*/
struct __SRC1_IN2_REG
/*=============================*/
{
    volatile uint32_t                                         : 8;
    volatile uint32_t BITFLD_SRC_IN                           : 24;
};
#define SRC_IN                               (0xFFFFFF00)



#define SRC1_OUT1_REG                        (0x50004014) /* SRC1 data out 1 */
#define SRC1_OUT1_REG_RESET                  (0x00000000)

/*=============================*/
struct __SRC1_OUT1_REG
/*=============================*/
{
    volatile uint32_t                                         : 8;
    volatile uint32_t BITFLD_SRC_OUT                          : 24;
};
#define SRC_OUT                              (0xFFFFFF00)



#define SRC1_OUT2_REG                        (0x50004018) /* SRC1 data out 2 */
#define SRC1_OUT2_REG_RESET                  (0x00000000)

/*=============================*/
struct __SRC1_OUT2_REG
/*=============================*/
{
    volatile uint32_t                                         : 8;
    volatile uint32_t BITFLD_SRC_OUT                          : 24;
};
#define SRC_OUT                              (0xFFFFFF00)



#define APU_MUX_REG                          (0x5000401C) /* APU mux register */
#define APU_MUX_REG_RESET                    (0x00000000)

/*=============================*/
struct __APU_MUX_REG
/*=============================*/
{
    volatile uint32_t BITFLD_SRC1_MUX_IN                      : 3;
    volatile uint32_t BITFLD_PCM1_MUX_IN                      : 3;
    volatile uint32_t BITFLD_PDM1_MUX_IN                      : 1;
};
#define SRC1_MUX_IN                          (0x0007)
#define PCM1_MUX_IN                          (0x0038)
#define PDM1_MUX_IN                          (0x0040)



#define COEF10_SET1_REG                      (0x50004020) /* SRC coefficient 1,0 set 1 */
#define COEF10_SET1_REG_RESET                (0x79A99278)

/*=============================*/
struct __COEF10_SET1_REG
/*=============================*/
{
    volatile uint32_t BITFLD_SRC_COEF0                        : 16;
    volatile uint32_t BITFLD_SRC_COEF1                        : 16;
};
#define SRC_COEF0                            (0xFFFF)
#define SRC_COEF1                            (0xFFFF0000)



#define COEF32_SET1_REG                      (0x50004024) /* SRC coefficient 3,2 set 1 */
#define COEF32_SET1_REG_RESET                (0x6D568B41)

/*=============================*/
struct __COEF32_SET1_REG
/*=============================*/
{
    volatile uint32_t BITFLD_SRC_COEF2                        : 16;
    volatile uint32_t BITFLD_SRC_COEF3                        : 16;
};
#define SRC_COEF2                            (0xFFFF)
#define SRC_COEF3                            (0xFFFF0000)



#define COEF54_SET1_REG                      (0x50004028) /* SRC coefficient 5,4 set 1 */
#define COEF54_SET1_REG_RESET                (0x9BC5BE15)

/*=============================*/
struct __COEF54_SET1_REG
/*=============================*/
{
    volatile uint32_t BITFLD_SRC_COEF4                        : 16;
    volatile uint32_t BITFLD_SRC_COEF5                        : 16;
};
#define SRC_COEF4                            (0xFFFF)
#define SRC_COEF5                            (0xFFFF0000)



#define COEF76_SET1_REG                      (0x5000402C) /* SRC coefficient 7,6 set 1 */
#define COEF76_SET1_REG_RESET                (0x8C287E1A)

/*=============================*/
struct __COEF76_SET1_REG
/*=============================*/
{
    volatile uint32_t BITFLD_SRC_COEF6                        : 16;
    volatile uint32_t BITFLD_SRC_COEF7                        : 16;
};
#define SRC_COEF6                            (0xFFFF)
#define SRC_COEF7                            (0xFFFF0000)



#define COEF98_SET1_REG                      (0x50004030) /* SRC coefficient 9,8 set 1 */
#define COEF98_SET1_REG_RESET                (0x92D775E6)

/*=============================*/
struct __COEF98_SET1_REG
/*=============================*/
{
    volatile uint32_t BITFLD_SRC_COEF8                        : 16;
    volatile uint32_t BITFLD_SRC_COEF9                        : 16;
};
#define SRC_COEF8                            (0xFFFF)
#define SRC_COEF9                            (0xFFFF0000)



#define COEF0A_SET1_REG                      (0x50004034) /* SRC coefficient 10 set 1 */
#define COEF0A_SET1_REG_RESET                (0x000041F2)

/*=============================*/
struct __COEF0A_SET1_REG
/*=============================*/
{
    volatile uint32_t BITFLD_SRC_COEF10                       : 16;
};
#define SRC_COEF10                           (0xFFFF)



#define PCM1_CTRL_REG                        (0x50004100) /* PCM1 Control register */
#define PCM1_CTRL_REG_RESET                  (0x00000000)

/*=============================*/
struct __PCM1_CTRL_REG
/*=============================*/
{
    volatile uint32_t BITFLD_PCM_EN                           : 1;
    volatile uint32_t BITFLD_PCM_MASTER                       : 1;
    volatile uint32_t BITFLD_PCM_FSCLEN                       : 4;
    volatile uint32_t BITFLD_PCM_FSCDEL                       : 1;
    volatile uint32_t BITFLD_PCM_PPOD                         : 1;
    volatile uint32_t BITFLD_PCM_CLKINV                       : 1;
    volatile uint32_t BITFLD_PCM_FSCINV                       : 1;
    volatile uint32_t BITFLD_PCM_CLK_BIT                      : 1;
    volatile uint32_t BITFLD_PCM_CH_DEL                       : 5;
    volatile uint32_t BITFLD_PCM_FSC_EDGE                     : 1;
    volatile uint32_t                                         : 3;
    volatile uint32_t BITFLD_PCM_FSC_DIV                      : 12;
};
#define PCM_EN                               (0x0001)
#define PCM_MASTER                           (0x0002)
#define PCM_FSCLEN                           (0x003C)
#define PCM_FSCDEL                           (0x0040)
#define PCM_PPOD                             (0x0080)
#define PCM_CLKINV                           (0x0100)
#define PCM_FSCINV                           (0x0200)
#define PCM_CLK_BIT                          (0x0400)
#define PCM_CH_DEL                           (0xF800)
#define PCM_FSC_EDGE                         (0x10000)
#define PCM_FSC_DIV                          (0xFFF00000)



#define PCM1_IN1_REG                         (0x50004104) /* PCM1 data in 1 */
#define PCM1_IN1_REG_RESET                   (0x00000000)

/*=============================*/
struct __PCM1_IN1_REG
/*=============================*/
{
    volatile uint32_t BITFLD_PCM_IN                           : 32;
};
#define PCM_IN                               (0xFFFFFFFF)



#define PCM1_IN2_REG                         (0x50004108) /* PCM1 data in 2 */
#define PCM1_IN2_REG_RESET                   (0x00000000)

/*=============================*/
struct __PCM1_IN2_REG
/*=============================*/
{
    volatile uint32_t BITFLD_PCM_IN                           : 32;
};
#define PCM_IN                               (0xFFFFFFFF)



#define PCM1_OUT1_REG                        (0x5000410C) /* PCM1 data out 1 */
#define PCM1_OUT1_REG_RESET                  (0xFFFFFFFF)

/*=============================*/
struct __PCM1_OUT1_REG
/*=============================*/
{
    volatile uint32_t BITFLD_PCM_OUT                          : 32;
};
#define PCM_OUT                              (0xFFFFFFFF)



#define PCM1_OUT2_REG                        (0x50004110) /* PCM1 data out 2 */
#define PCM1_OUT2_REG_RESET                  (0xFFFFFFFF)

/*=============================*/
struct __PCM1_OUT2_REG
/*=============================*/
{
    volatile uint32_t BITFLD_PCM_OUT                          : 32;
};
#define PCM_OUT                              (0xFFFFFFFF)



/*===========================================================================*/
/* memory map BLE */
/*===========================================================================*/



#define BLE_RWBLECNTL_REG                    (0x40000000) /* BLE Control register */
#define BLE_RWBLECNTL_REG_RESET              (0x00000000)

/*=============================*/
struct __BLE_RWBLECNTL_REG
/*=============================*/
{
    volatile uint32_t BITFLD_SYNCERR                          : 3;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_RXWINSZDEF                       : 4;
    volatile uint32_t BITFLD_RWBLE_EN                         : 1;
    volatile uint32_t BITFLD_ADVERTFILT_EN                    : 1;
    volatile uint32_t                                         : 2;
    volatile uint32_t                                         : 2;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_HOP_REMAP_DSB                    : 1;
    volatile uint32_t BITFLD_CRC_DSB                          : 1;
    volatile uint32_t BITFLD_WHIT_DSB                         : 1;
    volatile uint32_t BITFLD_CRYPT_DSB                        : 1;
    volatile uint32_t BITFLD_NESN_DSB                         : 1;
    volatile uint32_t BITFLD_SN_DSB                           : 1;
    volatile uint32_t BITFLD_MD_DSB                           : 1;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_SCAN_ABORT                       : 1;
    volatile uint32_t BITFLD_ADVERT_ABORT                     : 1;
    volatile uint32_t BITFLD_RFTEST_ABORT                     : 1;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_SWINT_REQ                        : 1;
    volatile uint32_t BITFLD_REG_SOFT_RST                     : 1;
    volatile uint32_t BITFLD_MASTER_TGSOFT_RST                : 1;
    volatile uint32_t BITFLD_MASTER_SOFT_RST                  : 1;
};
#define SYNCERR                              (0x0007)
#define RXWINSZDEF                           (0x00F0)
#define RWBLE_EN                             (0x0100)
#define ADVERTFILT_EN                        (0x0200)
#define HOP_REMAP_DSB                        (0x10000)
#define CRC_DSB                              (0x20000)
#define WHIT_DSB                             (0x40000)
#define CRYPT_DSB                            (0x80000)
#define NESN_DSB                             (0x100000)
#define SN_DSB                               (0x200000)
#define MD_DSB                               (0x400000)
#define SCAN_ABORT                           (0x1000000)
#define ADVERT_ABORT                         (0x2000000)
#define RFTEST_ABORT                         (0x4000000)
#define SWINT_REQ                            (0x10000000)
#define REG_SOFT_RST                         (0x20000000)
#define MASTER_TGSOFT_RST                    (0x40000000)
#define MASTER_SOFT_RST                      (0x80000000)



#define BLE_VERSION_REG                      (0x40000004) /* Version register */
#define BLE_VERSION_REG_RESET                (0x07010000)

/*=============================*/
struct __BLE_VERSION_REG
/*=============================*/
{
    volatile uint32_t BITFLD_BUILD                            : 8;
    volatile uint32_t BITFLD_UPG                              : 8;
    volatile uint32_t BITFLD_REL                              : 8;
    volatile uint32_t BITFLD_TYP                              : 8;
};
#define BUILD                                (0x00FF)
#define UPG                                  (0xFF00)
#define REL                                  (0xFF0000)
#define TYP                                  (0xFF000000)



#define BLE_RWBLECONF_REG                    (0x40000008) /* Configuration register */
#define BLE_RWBLECONF_REG_RESET              (0x1002001F)

/*=============================*/
struct __BLE_RWBLECONF_REG
/*=============================*/
{
    volatile uint32_t BITFLD_BUSWIDTH                         : 1;
    volatile uint32_t BITFLD_USECRYPT                         : 1;
    volatile uint32_t BITFLD_USEDBG                           : 1;
    volatile uint32_t BITFLD_COEX                             : 1;
    volatile uint32_t BITFLD_INTMODE                          : 1;
    volatile uint32_t BITFLD_DMMODE                           : 1;
    volatile uint32_t BITFLD_DECIPHER                         : 1;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_CLK_SEL                          : 6;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_RFIF                             : 7;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_ADD_WIDTH                        : 6;
};
#define BUSWIDTH                             (0x0001)
#define USECRYPT                             (0x0002)
#define USEDBG                               (0x0004)
#define COEX                                 (0x0008)
#define INTMODE                              (0x0010)
#define DMMODE                               (0x0020)
#define DECIPHER                             (0x0040)
#define CLK_SEL                              (0x3F00)
#define RFIF                                 (0x7F0000)
#define ADD_WIDTH                            (0x3F000000)



#define BLE_INTCNTL_REG                      (0x4000000C) /* Interrupt controller register */
#define BLE_INTCNTL_REG_RESET                (0x0000811F)

/*=============================*/
struct __BLE_INTCNTL_REG
/*=============================*/
{
    volatile uint32_t BITFLD_CSCNTINTMSK                      : 1;
    volatile uint32_t BITFLD_RXINTMSK                         : 1;
    volatile uint32_t BITFLD_SLPINTMSK                        : 1;
    volatile uint32_t BITFLD_EVENTINTMSK                      : 1;
    volatile uint32_t BITFLD_CRYPTINTMSK                      : 1;
    volatile uint32_t BITFLD_ERRORINTMSK                      : 1;
    volatile uint32_t BITFLD_GROSSTGTIMINTMSK                 : 1;
    volatile uint32_t BITFLD_FINETGTIMINTMSK                  : 1;
    volatile uint32_t BITFLD_EVENTAPFAINTMSK                  : 1;
    volatile uint32_t BITFLD_SWINTMSK                         : 1;
    volatile uint32_t                                         : 5;
    volatile uint32_t BITFLD_CSCNTDEVMSK                      : 1;
};
#define CSCNTINTMSK                          (0x0001)
#define RXINTMSK                             (0x0002)
#define SLPINTMSK                            (0x0004)
#define EVENTINTMSK                          (0x0008)
#define CRYPTINTMSK                          (0x0010)
#define ERRORINTMSK                          (0x0020)
#define GROSSTGTIMINTMSK                     (0x0040)
#define FINETGTIMINTMSK                      (0x0080)
#define EVENTAPFAINTMSK                      (0x0100)
#define SWINTMSK                             (0x0200)
#define CSCNTDEVMSK                          (0x8000)



#define BLE_INTSTAT_REG                      (0x40000010) /* Interrupt status register */
#define BLE_INTSTAT_REG_RESET                (0x00000000)

/*=============================*/
struct __BLE_INTSTAT_REG
/*=============================*/
{
    volatile uint32_t BITFLD_CSCNTINTSTAT                     : 1;
    volatile uint32_t BITFLD_RXINTSTAT                        : 1;
    volatile uint32_t BITFLD_SLPINTSTAT                       : 1;
    volatile uint32_t BITFLD_EVENTINTSTAT                     : 1;
    volatile uint32_t BITFLD_CRYPTINTSTAT                     : 1;
    volatile uint32_t BITFLD_ERRORINTSTAT                     : 1;
    volatile uint32_t BITFLD_GROSSTGTIMINTSTAT                : 1;
    volatile uint32_t BITFLD_FINETGTIMINTSTAT                 : 1;
    volatile uint32_t BITFLD_EVENTAPFAINTSTAT                 : 1;
    volatile uint32_t BITFLD_SWINTSTAT                        : 1;
};
#define CSCNTINTSTAT                         (0x0001)
#define RXINTSTAT                            (0x0002)
#define SLPINTSTAT                           (0x0004)
#define EVENTINTSTAT                         (0x0008)
#define CRYPTINTSTAT                         (0x0010)
#define ERRORINTSTAT                         (0x0020)
#define GROSSTGTIMINTSTAT                    (0x0040)
#define FINETGTIMINTSTAT                     (0x0080)
#define EVENTAPFAINTSTAT                     (0x0100)
#define SWINTSTAT                            (0x0200)



#define BLE_INTRAWSTAT_REG                   (0x40000014) /* Interrupt raw status register */
#define BLE_INTRAWSTAT_REG_RESET             (0x00000000)

/*=============================*/
struct __BLE_INTRAWSTAT_REG
/*=============================*/
{
    volatile uint32_t BITFLD_CSCNTINTRAWSTAT                  : 1;
    volatile uint32_t BITFLD_RXINTRAWSTAT                     : 1;
    volatile uint32_t BITFLD_SLPINTRAWSTAT                    : 1;
    volatile uint32_t BITFLD_EVENTINTRAWSTAT                  : 1;
    volatile uint32_t BITFLD_CRYPTINTRAWSTAT                  : 1;
    volatile uint32_t BITFLD_ERRORINTRAWSTAT                  : 1;
    volatile uint32_t BITFLD_GROSSTGTIMINTRAWSTAT             : 1;
    volatile uint32_t BITFLD_FINETGTIMINTRAWSTAT              : 1;
    volatile uint32_t BITFLD_EVENTAPFAINTRAWSTAT              : 1;
    volatile uint32_t BITFLD_SWINTRAWSTAT                     : 1;
};
#define CSCNTINTRAWSTAT                      (0x0001)
#define RXINTRAWSTAT                         (0x0002)
#define SLPINTRAWSTAT                        (0x0004)
#define EVENTINTRAWSTAT                      (0x0008)
#define CRYPTINTRAWSTAT                      (0x0010)
#define ERRORINTRAWSTAT                      (0x0020)
#define GROSSTGTIMINTRAWSTAT                 (0x0040)
#define FINETGTIMINTRAWSTAT                  (0x0080)
#define EVENTAPFAINTRAWSTAT                  (0x0100)
#define SWINTRAWSTAT                         (0x0200)



#define BLE_INTACK_REG                       (0x40000018) /* Interrupt acknowledge register */
#define BLE_INTACK_REG_RESET                 (0x00000000)

/*=============================*/
struct __BLE_INTACK_REG
/*=============================*/
{
    volatile uint32_t BITFLD_CSCNTINTACK                      : 1;
    volatile uint32_t BITFLD_RXINTACK                         : 1;
    volatile uint32_t BITFLD_SLPINTACK                        : 1;
    volatile uint32_t BITFLD_EVENTINTACK                      : 1;
    volatile uint32_t BITFLD_CRYPTINTACK                      : 1;
    volatile uint32_t BITFLD_ERRORINTACK                      : 1;
    volatile uint32_t BITFLD_GROSSTGTIMINTACK                 : 1;
    volatile uint32_t BITFLD_FINETGTIMINTACK                  : 1;
    volatile uint32_t BITFLD_EVENTAPFAINTACK                  : 1;
    volatile uint32_t BITFLD_SWINTACK                         : 1;
};
#define CSCNTINTACK                          (0x0001)
#define RXINTACK                             (0x0002)
#define SLPINTACK                            (0x0004)
#define EVENTINTACK                          (0x0008)
#define CRYPTINTACK                          (0x0010)
#define ERRORINTACK                          (0x0020)
#define GROSSTGTIMINTACK                     (0x0040)
#define FINETGTIMINTACK                      (0x0080)
#define EVENTAPFAINTACK                      (0x0100)
#define SWINTACK                             (0x0200)



#define BLE_BASETIMECNT_REG                  (0x4000001C) /* Base time reference counter */
#define BLE_BASETIMECNT_REG_RESET            (0x00000000)

/*=============================*/
struct __BLE_BASETIMECNT_REG
/*=============================*/
{
    volatile uint32_t BITFLD_BASETIMECNT                      : 27;
};
#define BASETIMECNT                          (0x7FFFFFF)



#define BLE_FINETIMECNT_REG                  (0x40000020) /* Fine time reference counter */
#define BLE_FINETIMECNT_REG_RESET            (0x00000000)

/*=============================*/
struct __BLE_FINETIMECNT_REG
/*=============================*/
{
    volatile uint32_t BITFLD_FINECNT                          : 10;
};
#define FINECNT                              (0x03FF)



#define BLE_BDADDRL_REG                      (0x40000024) /* BLE device address LSB register */
#define BLE_BDADDRL_REG_RESET                (0x00000000)

/*=============================*/
struct __BLE_BDADDRL_REG
/*=============================*/
{
    volatile uint32_t BITFLD_BDADDRL                          : 32;
};
#define BDADDRL                              (0xFFFFFFFF)



#define BLE_BDADDRU_REG                      (0x40000028) /* BLE device address MSB register */
#define BLE_BDADDRU_REG_RESET                (0x00000000)

/*=============================*/
struct __BLE_BDADDRU_REG
/*=============================*/
{
    volatile uint32_t BITFLD_BDADDRU                          : 16;
    volatile uint32_t BITFLD_PRIV_NPUB                        : 1;
};
#define BDADDRU                              (0xFFFF)
#define PRIV_NPUB                            (0x10000)



#define BLE_CURRENTRXDESCPTR_REG             (0x4000002C) /* Rx Descriptor Pointer for the Receive Buffer Chained List */
#define BLE_CURRENTRXDESCPTR_REG_RESET       (0x00000000)

/*=============================*/
struct __BLE_CURRENTRXDESCPTR_REG
/*=============================*/
{
    volatile uint32_t BITFLD_CURRENTRXDESCPTR                 : 15;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_ETPTR                            : 16;
};
#define CURRENTRXDESCPTR                     (0x7FFF)
#define ETPTR                                (0xFFFF0000)



#define BLE_DEEPSLCNTL_REG                   (0x40000030) /* Deep-Sleep control register */
#define BLE_DEEPSLCNTL_REG_RESET             (0x00000000)

/*=============================*/
struct __BLE_DEEPSLCNTL_REG
/*=============================*/
{
    volatile uint32_t BITFLD_DEEP_SLEEP_IRQ_EN                : 2;
    volatile uint32_t BITFLD_DEEP_SLEEP_ON                    : 1;
    volatile uint32_t BITFLD_DEEP_SLEEP_CORR_EN               : 1;
    volatile uint32_t BITFLD_SOFT_WAKEUP_REQ                  : 1;
    volatile uint32_t                                         : 10;
    volatile uint32_t BITFLD_DEEP_SLEEP_STAT                  : 1;
    volatile uint32_t                                         : 15;
    volatile uint32_t BITFLD_EXTWKUPDSB                       : 1;
};
#define DEEP_SLEEP_IRQ_EN                    (0x0003)
#define DEEP_SLEEP_ON                        (0x0004)
#define DEEP_SLEEP_CORR_EN                   (0x0008)
#define SOFT_WAKEUP_REQ                      (0x0010)
#define DEEP_SLEEP_STAT                      (0x8000)
#define EXTWKUPDSB                           (0x80000000)



#define BLE_DEEPSLWKUP_REG                   (0x40000034) /* Time (measured in Low Power clock cycles) in Deep Sleep Mode before waking-up the device */
#define BLE_DEEPSLWKUP_REG_RESET             (0x00000000)

/*=============================*/
struct __BLE_DEEPSLWKUP_REG
/*=============================*/
{
    volatile uint32_t BITFLD_DEEPSLTIME                       : 32;
};
#define DEEPSLTIME                           (0xFFFFFFFF)



#define BLE_DEEPSLSTAT_REG                   (0x40000038) /* Duration of the last deep sleep phase register */
#define BLE_DEEPSLSTAT_REG_RESET             (0x00000000)

/*=============================*/
struct __BLE_DEEPSLSTAT_REG
/*=============================*/
{
    volatile uint32_t BITFLD_DEEPSLDUR                        : 32;
};
#define DEEPSLDUR                            (0xFFFFFFFF)



#define BLE_ENBPRESET_REG                    (0x4000003C) /* Time in low power oscillator cycles register */
#define BLE_ENBPRESET_REG_RESET              (0x00000000)

/*=============================*/
struct __BLE_ENBPRESET_REG
/*=============================*/
{
    volatile uint32_t BITFLD_TWIRQ_RESET                      : 10;
    volatile uint32_t BITFLD_TWIRQ_SET                        : 11;
    volatile uint32_t BITFLD_TWEXT                            : 11;
};
#define TWIRQ_RESET                          (0x03FF)
#define TWIRQ_SET                            (0x1FFC00)
#define TWEXT                                (0xFFE00000)



#define BLE_FINECNTCORR_REG                  (0x40000040) /* Phase correction value register */
#define BLE_FINECNTCORR_REG_RESET            (0x00000000)

/*=============================*/
struct __BLE_FINECNTCORR_REG
/*=============================*/
{
    volatile uint32_t BITFLD_FINECNTCORR                      : 10;
};
#define FINECNTCORR                          (0x03FF)



#define BLE_BASETIMECNTCORR_REG              (0x40000044) /* Base Time Counter */
#define BLE_BASETIMECNTCORR_REG_RESET        (0x00000000)

/*=============================*/
struct __BLE_BASETIMECNTCORR_REG
/*=============================*/
{
    volatile uint32_t BITFLD_BASETIMECNTCORR                  : 27;
};
#define BASETIMECNTCORR                      (0x7FFFFFF)



#define BLE_DIAGCNTL_REG                     (0x40000050) /* Diagnostics Register */
#define BLE_DIAGCNTL_REG_RESET               (0x00000000)

/*=============================*/
struct __BLE_DIAGCNTL_REG
/*=============================*/
{
    volatile uint32_t BITFLD_DIAG0                            : 6;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_DIAG0_EN                         : 1;
    volatile uint32_t BITFLD_DIAG1                            : 6;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_DIAG1_EN                         : 1;
    volatile uint32_t BITFLD_DIAG2                            : 6;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_DIAG2_EN                         : 1;
    volatile uint32_t BITFLD_DIAG3                            : 6;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_DIAG3_EN                         : 1;
};
#define DIAG0                                (0x003F)
#define DIAG0_EN                             (0x0080)
#define DIAG1                                (0x3F00)
#define DIAG1_EN                             (0x8000)
#define DIAG2                                (0x3F0000)
#define DIAG2_EN                             (0x800000)
#define DIAG3                                (0x3F000000)
#define DIAG3_EN                             (0x80000000)



#define BLE_DIAGSTAT_REG                     (0x40000054) /* Debug use only */
#define BLE_DIAGSTAT_REG_RESET               (0x00000000)

/*=============================*/
struct __BLE_DIAGSTAT_REG
/*=============================*/
{
    volatile uint32_t BITFLD_DIAG0STAT                        : 8;
    volatile uint32_t BITFLD_DIAG1STAT                        : 8;
    volatile uint32_t BITFLD_DIAG2STAT                        : 8;
    volatile uint32_t BITFLD_DIAG3STAT                        : 8;
};
#define DIAG0STAT                            (0x00FF)
#define DIAG1STAT                            (0xFF00)
#define DIAG2STAT                            (0xFF0000)
#define DIAG3STAT                            (0xFF000000)



#define BLE_DEBUGADDMAX_REG                  (0x40000058) /* Upper limit for the memory zone */
#define BLE_DEBUGADDMAX_REG_RESET            (0x00000000)

/*=============================*/
struct __BLE_DEBUGADDMAX_REG
/*=============================*/
{
    volatile uint32_t BITFLD_EM_ADDMAX                        : 16;
    volatile uint32_t BITFLD_REG_ADDMAX                       : 16;
};
#define EM_ADDMAX                            (0xFFFF)
#define REG_ADDMAX                           (0xFFFF0000)



#define BLE_DEBUGADDMIN_REG                  (0x4000005C) /* Lower limit for the memory zone */
#define BLE_DEBUGADDMIN_REG_RESET            (0x00000000)

/*=============================*/
struct __BLE_DEBUGADDMIN_REG
/*=============================*/
{
    volatile uint32_t BITFLD_EM_ADDMIN                        : 16;
    volatile uint32_t BITFLD_REG_ADDMIN                       : 16;
};
#define EM_ADDMIN                            (0xFFFF)
#define REG_ADDMIN                           (0xFFFF0000)



#define BLE_ERRORTYPESTAT_REG                (0x40000060) /* Error Type Status registers */
#define BLE_ERRORTYPESTAT_REG_RESET          (0x00000000)

/*=============================*/
struct __BLE_ERRORTYPESTAT_REG
/*=============================*/
{
    volatile uint32_t BITFLD_TXCRYPT_ERROR                    : 1;
    volatile uint32_t BITFLD_RXCRYPT_ERROR                    : 1;
    volatile uint32_t BITFLD_PKTCNTL_EMACC_ERROR              : 1;
    volatile uint32_t BITFLD_RADIO_EMACC_ERROR                : 1;
    volatile uint32_t BITFLD_EVT_SCHDL_EMACC_ERROR            : 1;
    volatile uint32_t BITFLD_EVT_SCHDL_ENTRY_ERROR            : 1;
    volatile uint32_t BITFLD_EVT_SCHDL_APFM_ERROR             : 1;
    volatile uint32_t BITFLD_EVT_CNTL_APFM_ERROR              : 1;
    volatile uint32_t BITFLD_WHITELIST_ERROR                  : 1;
    volatile uint32_t BITFLD_IFS_UNDERRUN                     : 1;
    volatile uint32_t BITFLD_ADV_UNDERRUN                     : 1;
    volatile uint32_t BITFLD_LLCHMAP_ERROR                    : 1;
    volatile uint32_t BITFLD_CSFORMAT_ERROR                   : 1;
    volatile uint32_t BITFLD_TXDESC_EMPTY_ERROR               : 1;
    volatile uint32_t BITFLD_RXDESC_EMPTY_ERROR               : 1;
    volatile uint32_t BITFLD_TXDATA_PTR_ERROR                 : 1;
    volatile uint32_t BITFLD_RXDATA_PTR_ERROR                 : 1;
    volatile uint32_t BITFLD_CONCEVTIRQ_ERROR                 : 1;
};
#define TXCRYPT_ERROR                        (0x0001)
#define RXCRYPT_ERROR                        (0x0002)
#define PKTCNTL_EMACC_ERROR                  (0x0004)
#define RADIO_EMACC_ERROR                    (0x0008)
#define EVT_SCHDL_EMACC_ERROR                (0x0010)
#define EVT_SCHDL_ENTRY_ERROR                (0x0020)
#define EVT_SCHDL_APFM_ERROR                 (0x0040)
#define EVT_CNTL_APFM_ERROR                  (0x0080)
#define WHITELIST_ERROR                      (0x0100)
#define IFS_UNDERRUN                         (0x0200)
#define ADV_UNDERRUN                         (0x0400)
#define LLCHMAP_ERROR                        (0x0800)
#define CSFORMAT_ERROR                       (0x1000)
#define TXDESC_EMPTY_ERROR                   (0x2000)
#define RXDESC_EMPTY_ERROR                   (0x4000)
#define TXDATA_PTR_ERROR                     (0x8000)
#define RXDATA_PTR_ERROR                     (0x10000)
#define CONCEVTIRQ_ERROR                     (0x20000)



#define BLE_SWPROFILING_REG                  (0x40000064) /* Software Profiling register */
#define BLE_SWPROFILING_REG_RESET            (0x00000000)

/*=============================*/
struct __BLE_SWPROFILING_REG
/*=============================*/
{
    volatile uint32_t BITFLD_SWPROFVAL                        : 32;
};
#define SWPROFVAL                            (0xFFFFFFFF)



#define BLE_RADIOCNTL0_REG                   (0x40000070) /* Radio interface control register */
#define BLE_RADIOCNTL0_REG_RESET             (0x00000002)
//register BLE_RADIOCNTL0_REG found for which all fields are reserved, omitting.



#define BLE_RADIOCNTL1_REG                   (0x40000074) /* Radio interface control register */
#define BLE_RADIOCNTL1_REG_RESET             (0x00000000)

/*=============================*/
struct __BLE_RADIOCNTL1_REG
/*=============================*/
{
    volatile uint32_t                                         : 16;
    volatile uint32_t BITFLD_XRFSEL                           : 5;
};
#define XRFSEL                               (0x1F0000)



#define BLE_RADIOCNTL2_REG                   (0x40000078) /* Radio interface control register */
#define BLE_RADIOCNTL2_REG_RESET             (0x00000000)
//register BLE_RADIOCNTL2_REG found for which all fields are reserved, omitting.



#define BLE_RADIOCNTL3_REG                   (0x4000007C) /* Radio interface control register */
#define BLE_RADIOCNTL3_REG_RESET             (0x00000040)
//register BLE_RADIOCNTL3_REG found for which all fields are reserved, omitting.



#define BLE_RADIOPWRUPDN_REG                 (0x40000080) /* RX/TX power up/down phase register */
#define BLE_RADIOPWRUPDN_REG_RESET           (0x00D203D2)

/*=============================*/
struct __BLE_RADIOPWRUPDN_REG
/*=============================*/
{
    volatile uint32_t BITFLD_TXPWRUP                          : 8;
    volatile uint32_t BITFLD_TXPWRDN                          : 4;
    volatile uint32_t                                         : 4;
    volatile uint32_t BITFLD_RXPWRUP                          : 8;
    volatile uint32_t BITFLD_RTRIP_DELAY                      : 7;
};
#define TXPWRUP                              (0x00FF)
#define TXPWRDN                              (0x0F00)
#define RXPWRUP                              (0xFF0000)
#define RTRIP_DELAY                          (0x7F000000)



#define BLE_ADVCHMAP_REG                     (0x40000090) /* Advertising Channel Map */
#define BLE_ADVCHMAP_REG_RESET               (0x00000007)

/*=============================*/
struct __BLE_ADVCHMAP_REG
/*=============================*/
{
    volatile uint32_t BITFLD_ADVCHMAP                         : 3;
};
#define ADVCHMAP                             (0x0007)



#define BLE_ADVTIM_REG                       (0x400000A0) /* Advertising Packet Interval */
#define BLE_ADVTIM_REG_RESET                 (0x00000000)

/*=============================*/
struct __BLE_ADVTIM_REG
/*=============================*/
{
    volatile uint32_t BITFLD_ADVINT                           : 14;
};
#define ADVINT                               (0x3FFF)



#define BLE_ACTSCANSTAT_REG                  (0x400000A4) /* Active scan register */
#define BLE_ACTSCANSTAT_REG_RESET            (0x00010001)

/*=============================*/
struct __BLE_ACTSCANSTAT_REG
/*=============================*/
{
    volatile uint32_t BITFLD_UPPERLIMIT                       : 9;
    volatile uint32_t                                         : 7;
    volatile uint32_t BITFLD_BACKOFF                          : 9;
};
#define UPPERLIMIT                           (0x01FF)
#define BACKOFF                              (0x1FF0000)



#define BLE_WLPUBADDPTR_REG                  (0x400000B0) /* Start address of public devices list */
#define BLE_WLPUBADDPTR_REG_RESET            (0x00000000)

/*=============================*/
struct __BLE_WLPUBADDPTR_REG
/*=============================*/
{
    volatile uint32_t BITFLD_WLPUBADDPTR                      : 16;
};
#define WLPUBADDPTR                          (0xFFFF)



#define BLE_WLPRIVADDPTR_REG                 (0x400000B4) /* Start address of private devices list */
#define BLE_WLPRIVADDPTR_REG_RESET           (0x00000000)

/*=============================*/
struct __BLE_WLPRIVADDPTR_REG
/*=============================*/
{
    volatile uint32_t BITFLD_WLPRIVADDPTR                     : 16;
};
#define WLPRIVADDPTR                         (0xFFFF)



#define BLE_WLNBDEV_REG                      (0x400000B8) /* Devices in white list */
#define BLE_WLNBDEV_REG_RESET                (0x00000000)

/*=============================*/
struct __BLE_WLNBDEV_REG
/*=============================*/
{
    volatile uint32_t BITFLD_NBPUBDEV                         : 8;
    volatile uint32_t BITFLD_NBPRIVDEV                        : 8;
};
#define NBPUBDEV                             (0x00FF)
#define NBPRIVDEV                            (0xFF00)



#define BLE_AESCNTL_REG                      (0x400000C0) /* Start AES register */
#define BLE_AESCNTL_REG_RESET                (0x00000000)

/*=============================*/
struct __BLE_AESCNTL_REG
/*=============================*/
{
    volatile uint32_t BITFLD_AES_START                        : 1;
    volatile uint32_t BITFLD_AES_MODE                         : 1;
};
#define AES_START                            (0x0001)
#define AES_MODE                             (0x0002)



#define BLE_AESKEY31_0_REG                   (0x400000C4) /* AES encryption key */
#define BLE_AESKEY31_0_REG_RESET             (0x00000000)

/*=============================*/
struct __BLE_AESKEY31_0_REG
/*=============================*/
{
    volatile uint32_t BITFLD_AESKEY31_0                       : 32;
};
#define AESKEY31_0                           (0xFFFFFFFF)



#define BLE_AESKEY63_32_REG                  (0x400000C8) /* AES encryption key */
#define BLE_AESKEY63_32_REG_RESET            (0x00000000)

/*=============================*/
struct __BLE_AESKEY63_32_REG
/*=============================*/
{
    volatile uint32_t BITFLD_AESKEY63_32                      : 32;
};
#define AESKEY63_32                          (0xFFFFFFFF)



#define BLE_AESKEY95_64_REG                  (0x400000CC) /* AES encryption key */
#define BLE_AESKEY95_64_REG_RESET            (0x00000000)

/*=============================*/
struct __BLE_AESKEY95_64_REG
/*=============================*/
{
    volatile uint32_t BITFLD_AESKEY95_64                      : 32;
};
#define AESKEY95_64                          (0xFFFFFFFF)



#define BLE_AESKEY127_96_REG                 (0x400000D0) /* AES encryption key */
#define BLE_AESKEY127_96_REG_RESET           (0x00000000)

/*=============================*/
struct __BLE_AESKEY127_96_REG
/*=============================*/
{
    volatile uint32_t BITFLD_AESKEY127_96                     : 32;
};
#define AESKEY127_96                         (0xFFFFFFFF)



#define BLE_AESPTR_REG                       (0x400000D4) /* Pointer to the block to encrypt/decrypt */
#define BLE_AESPTR_REG_RESET                 (0x00000000)

/*=============================*/
struct __BLE_AESPTR_REG
/*=============================*/
{
    volatile uint32_t BITFLD_AESPTR                           : 16;
};
#define AESPTR                               (0xFFFF)



#define BLE_TXMICVAL_REG                     (0x400000D8) /* AES / CCM plain MIC value */
#define BLE_TXMICVAL_REG_RESET               (0x00000000)

/*=============================*/
struct __BLE_TXMICVAL_REG
/*=============================*/
{
    volatile uint32_t BITFLD_TXMICVAL                         : 32;
};
#define TXMICVAL                             (0xFFFFFFFF)



#define BLE_RXMICVAL_REG                     (0x400000DC) /* AES / CCM plain MIC value */
#define BLE_RXMICVAL_REG_RESET               (0x00000000)

/*=============================*/
struct __BLE_RXMICVAL_REG
/*=============================*/
{
    volatile uint32_t BITFLD_RXMICVAL                         : 32;
};
#define RXMICVAL                             (0xFFFFFFFF)



#define BLE_RFTESTCNTL_REG                   (0x400000E0) /* RF Testing Register */
#define BLE_RFTESTCNTL_REG_RESET             (0x00000000)

/*=============================*/
struct __BLE_RFTESTCNTL_REG
/*=============================*/
{
    volatile uint32_t BITFLD_TXLENGTH                         : 9;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_TXPKTCNTEN                       : 1;
    volatile uint32_t BITFLD_TXPLDSRC                         : 1;
    volatile uint32_t BITFLD_PRBSTYPE                         : 1;
    volatile uint32_t BITFLD_TXLENGTHSRC                      : 1;
    volatile uint32_t BITFLD_INFINITETX                       : 1;
    volatile uint32_t                                         : 11;
    volatile uint32_t BITFLD_RXPKTCNTEN                       : 1;
    volatile uint32_t                                         : 3;
    volatile uint32_t BITFLD_INFINITERX                       : 1;
};
#define TXLENGTH                             (0x01FF)
#define TXPKTCNTEN                           (0x0800)
#define TXPLDSRC                             (0x1000)
#define PRBSTYPE                             (0x2000)
#define TXLENGTHSRC                          (0x4000)
#define INFINITETX                           (0x8000)
#define RXPKTCNTEN                           (0x8000000)
#define INFINITERX                           (0x80000000)



#define BLE_RFTESTTXSTAT_REG                 (0x400000E4) /* RF Testing Register */
#define BLE_RFTESTTXSTAT_REG_RESET           (0x00000000)

/*=============================*/
struct __BLE_RFTESTTXSTAT_REG
/*=============================*/
{
    volatile uint32_t BITFLD_TXPKTCNT                         : 32;
};
#define TXPKTCNT                             (0xFFFFFFFF)



#define BLE_RFTESTRXSTAT_REG                 (0x400000E8) /* RF Testing Register */
#define BLE_RFTESTRXSTAT_REG_RESET           (0x00000000)

/*=============================*/
struct __BLE_RFTESTRXSTAT_REG
/*=============================*/
{
    volatile uint32_t BITFLD_RXPKTCNT                         : 32;
};
#define RXPKTCNT                             (0xFFFFFFFF)



#define BLE_TIMGENCNTL_REG                   (0x400000F0) /* Timing Generator Register */
#define BLE_TIMGENCNTL_REG_RESET             (0x81FE0096)

/*=============================*/
struct __BLE_TIMGENCNTL_REG
/*=============================*/
{
    volatile uint32_t BITFLD_PREFETCH_TIME                    : 9;
    volatile uint32_t                                         : 7;
    volatile uint32_t BITFLD_PREFETCHABORT_TIME               : 10;
    volatile uint32_t                                         : 5;
    volatile uint32_t BITFLD_APFM_EN                          : 1;
};
#define PREFETCH_TIME                        (0x01FF)
#define PREFETCHABORT_TIME                   (0x3FF0000)
#define APFM_EN                              (0x80000000)



#define BLE_GROSSTIMTGT_REG                  (0x400000F4) /* Gross Timer Target value */
#define BLE_GROSSTIMTGT_REG_RESET            (0x00000000)

/*=============================*/
struct __BLE_GROSSTIMTGT_REG
/*=============================*/
{
    volatile uint32_t BITFLD_GROSSTARGET                      : 23;
};
#define GROSSTARGET                          (0x7FFFFF)



#define BLE_FINETIMTGT_REG                   (0x400000F8) /* Fine Timer Target value */
#define BLE_FINETIMTGT_REG_RESET             (0x00000000)

/*=============================*/
struct __BLE_FINETIMTGT_REG
/*=============================*/
{
    volatile uint32_t BITFLD_FINETARGET                       : 27;
};
#define FINETARGET                           (0x7FFFFFF)



#define BLE_SAMPLECLK_REG                    (0x400000FC) /* Samples the Base Time Counter */
#define BLE_SAMPLECLK_REG_RESET              (0x00000000)

/*=============================*/
struct __BLE_SAMPLECLK_REG
/*=============================*/
{
    volatile uint32_t BITFLD_SAMP                             : 1;
};
#define SAMP                                 (0x0001)



#define BLE_COEXIFCNTL0_REG                  (0x40000100) /* Coexistence interface Control 0 Register */
#define BLE_COEXIFCNTL0_REG_RESET            (0x00000010)

/*=============================*/
struct __BLE_COEXIFCNTL0_REG
/*=============================*/
{
    volatile uint32_t BITFLD_COEX_EN                          : 1;
    volatile uint32_t BITFLD_SYNCGEN_EN                       : 1;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_WLANRXMSK                        : 2;
    volatile uint32_t BITFLD_WLANTXMSK                        : 2;
    volatile uint32_t                                         : 8;
    volatile uint32_t BITFLD_WLCTXPRIOMODE                    : 2;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_WLCRXPRIOMODE                    : 2;
};
#define COEX_EN                              (0x0001)
#define SYNCGEN_EN                           (0x0002)
#define WLANRXMSK                            (0x0030)
#define WLANTXMSK                            (0x00C0)
#define WLCTXPRIOMODE                        (0x30000)
#define WLCRXPRIOMODE                        (0x300000)



#define BLE_COEXIFCNTL1_REG                  (0x40000104) /* Coexistence interface Control 1 Register */
#define BLE_COEXIFCNTL1_REG_RESET            (0x00000000)

/*=============================*/
struct __BLE_COEXIFCNTL1_REG
/*=============================*/
{
    volatile uint32_t BITFLD_WLCPDELAY                        : 7;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_WLCPDURATION                     : 7;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_WLCPTXTHR                        : 5;
    volatile uint32_t                                         : 3;
    volatile uint32_t BITFLD_WLCPRXTHR                        : 5;
};
#define WLCPDELAY                            (0x007F)
#define WLCPDURATION                         (0x7F00)
#define WLCPTXTHR                            (0x1F0000)
#define WLCPRXTHR                            (0x1F000000)



#define BLE_BLEMPRIO0_REG                    (0x40000108) /* Coexistence interface Priority 0 Register */
#define BLE_BLEMPRIO0_REG_RESET              (0x3489ADEF)

/*=============================*/
struct __BLE_BLEMPRIO0_REG
/*=============================*/
{
    volatile uint32_t BITFLD_BLEM0                            : 4;
    volatile uint32_t BITFLD_BLEM1                            : 4;
    volatile uint32_t BITFLD_BLEM2                            : 4;
    volatile uint32_t BITFLD_BLEM3                            : 4;
    volatile uint32_t BITFLD_BLEM4                            : 4;
    volatile uint32_t BITFLD_BLEM5                            : 4;
    volatile uint32_t BITFLD_BLEM6                            : 4;
    volatile uint32_t BITFLD_BLEM7                            : 4;
};
#define BLEM0                                (0x000F)
#define BLEM1                                (0x00F0)
#define BLEM2                                (0x0F00)
#define BLEM3                                (0xF000)
#define BLEM4                                (0xF0000)
#define BLEM5                                (0xF00000)
#define BLEM6                                (0xF000000)
#define BLEM7                                (0xF0000000)



#define BLE_BLEMPRIO1_REG                    (0x4000010C) /* Coexistence interface Priority 1 Register */
#define BLE_BLEMPRIO1_REG_RESET              (0x30000000)

/*=============================*/
struct __BLE_BLEMPRIO1_REG
/*=============================*/
{
    volatile uint32_t                                         : 28;
    volatile uint32_t BITFLD_BLEMDEFAULT                      : 4;
};
#define BLEMDEFAULT                          (0xF0000000)



#define BLE_CNTL2_REG                        (0x40000200) /* BLE Control Register 2 */
#define BLE_CNTL2_REG_RESET                  (0x00000004)

/*=============================*/
struct __BLE_CNTL2_REG
/*=============================*/
{
    volatile uint32_t BITFLD_EMACCERRSTAT                     : 1;
    volatile uint32_t BITFLD_EMACCERRACK                      : 1;
    volatile uint32_t BITFLD_EMACCERRMSK                      : 1;
    volatile uint32_t BITFLD_BLE_DIAG_OVR                     : 1;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_BLE_CLK_STAT                     : 1;
    volatile uint32_t BITFLD_MON_LP_CLK                       : 1;
    volatile uint32_t BITFLD_RADIO_PWRDN_ALLOW                : 1;
    volatile uint32_t BITFLD_BLE_CLK_SEL                      : 6;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_BLE_PTI_SOURCE_SEL               : 1;
    volatile uint32_t BITFLD_BB_ONLY                          : 1;
    volatile uint32_t BITFLD_SW_RPL_SPI                       : 1;
    volatile uint32_t BITFLD_WAKEUPLPSTAT                     : 1;
    volatile uint32_t BITFLD_BLE_RSSI_SEL                     : 1;
};
#define EMACCERRSTAT                         (0x0001)
#define EMACCERRACK                          (0x0002)
#define EMACCERRMSK                          (0x0004)
#define BLE_DIAG_OVR                         (0x0008)
#define BLE_CLK_STAT                         (0x0040)
#define MON_LP_CLK                           (0x0080)
#define RADIO_PWRDN_ALLOW                    (0x0100)
#define BLE_CLK_SEL                          (0x7E00)
#define BLE_PTI_SOURCE_SEL                   (0x20000)
#define BB_ONLY                              (0x40000)
#define SW_RPL_SPI                           (0x80000)
#define WAKEUPLPSTAT                         (0x100000)
#define BLE_RSSI_SEL                         (0x200000)



#define BLE_EM_BASE_REG                      (0x40000208) /* Exchange Memory Base Register */
#define BLE_EM_BASE_REG_RESET                (0x00000000)

/*=============================*/
struct __BLE_EM_BASE_REG
/*=============================*/
{
    volatile uint32_t                                         : 10;
    volatile uint32_t BITFLD_BLE_EM_BASE_16_10                : 7;
};
#define BLE_EM_BASE_16_10                    (0x1FC00)



#define BLE_DIAGCNTL2_REG                    (0x4000020C) /* Debug use only */
#define BLE_DIAGCNTL2_REG_RESET              (0x00000000)

/*=============================*/
struct __BLE_DIAGCNTL2_REG
/*=============================*/
{
    volatile uint32_t BITFLD_DIAG4                            : 6;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_DIAG4_EN                         : 1;
    volatile uint32_t BITFLD_DIAG5                            : 6;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_DIAG5_EN                         : 1;
    volatile uint32_t BITFLD_DIAG6                            : 6;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_DIAG6_EN                         : 1;
    volatile uint32_t BITFLD_DIAG7                            : 6;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_DIAG7_EN                         : 1;
};
#define DIAG4                                (0x003F)
#define DIAG4_EN                             (0x0080)
#define DIAG5                                (0x3F00)
#define DIAG5_EN                             (0x8000)
#define DIAG6                                (0x3F0000)
#define DIAG6_EN                             (0x800000)
#define DIAG7                                (0x3F000000)
#define DIAG7_EN                             (0x80000000)



#define BLE_DIAGCNTL3_REG                    (0x40000210) /* Debug use only */
#define BLE_DIAGCNTL3_REG_RESET              (0x00000000)

/*=============================*/
struct __BLE_DIAGCNTL3_REG
/*=============================*/
{
    volatile uint32_t BITFLD_DIAG0_BIT                        : 3;
    volatile uint32_t BITFLD_DIAG0_INV                        : 1;
    volatile uint32_t BITFLD_DIAG1_BIT                        : 3;
    volatile uint32_t BITFLD_DIAG1_INV                        : 1;
    volatile uint32_t BITFLD_DIAG2_BIT                        : 3;
    volatile uint32_t BITFLD_DIAG2_INV                        : 1;
    volatile uint32_t BITFLD_DIAG3_BIT                        : 3;
    volatile uint32_t BITFLD_DIAG3_INV                        : 1;
    volatile uint32_t BITFLD_DIAG4_BIT                        : 3;
    volatile uint32_t BITFLD_DIAG4_INV                        : 1;
    volatile uint32_t BITFLD_DIAG5_BIT                        : 3;
    volatile uint32_t BITFLD_DIAG5_INV                        : 1;
    volatile uint32_t BITFLD_DIAG6_BIT                        : 3;
    volatile uint32_t BITFLD_DIAG6_INV                        : 1;
    volatile uint32_t BITFLD_DIAG7_BIT                        : 3;
    volatile uint32_t BITFLD_DIAG7_INV                        : 1;
};
#define DIAG0_BIT                            (0x0007)
#define DIAG0_INV                            (0x0008)
#define DIAG1_BIT                            (0x0070)
#define DIAG1_INV                            (0x0080)
#define DIAG2_BIT                            (0x0700)
#define DIAG2_INV                            (0x0800)
#define DIAG3_BIT                            (0x7000)
#define DIAG3_INV                            (0x8000)
#define DIAG4_BIT                            (0x70000)
#define DIAG4_INV                            (0x80000)
#define DIAG5_BIT                            (0x700000)
#define DIAG5_INV                            (0x800000)
#define DIAG6_BIT                            (0x7000000)
#define DIAG6_INV                            (0x8000000)
#define DIAG7_BIT                            (0x70000000)
#define DIAG7_INV                            (0x80000000)



/*===========================================================================*/
/* memory map CRG */
/*===========================================================================*/



#define CLK_AMBA_REG                         (0x50000000) /* HCLK, PCLK, divider and clock gates */
#define CLK_AMBA_REG_RESET                   (0x00000000)

/*=============================*/
struct __CLK_AMBA_REG
/*=============================*/
{
    volatile uint16_t BITFLD_HCLK_DIV                         : 2;
    volatile uint16_t                                         : 2;
    volatile uint16_t BITFLD_PCLK_DIV                         : 2;
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_OTP_ENABLE                       : 1;
};
#define HCLK_DIV                             (0x0003)
#define PCLK_DIV                             (0x0030)
#define OTP_ENABLE                           (0x0080)



#define CLK_FREQ_TRIM_REG                    (0x50000002) /* Xtal frequency trimming register */
#define CLK_FREQ_TRIM_REG_RESET              (0x00000000)

/*=============================*/
struct __CLK_FREQ_TRIM_REG
/*=============================*/
{
    volatile uint16_t BITFLD_FINE_ADJ                         : 8;
    volatile uint16_t BITFLD_COARSE_ADJ                       : 3;
};
#define FINE_ADJ                             (0x00FF)
#define COARSE_ADJ                           (0x0700)



#define CLK_PER_REG                          (0x50000004) /* Peripheral divider register */
#define CLK_PER_REG_RESET                    (0x00000000)

/*=============================*/
struct __CLK_PER_REG
/*=============================*/
{
    volatile uint16_t BITFLD_TMR_DIV                          : 2;
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_TMR_ENABLE                       : 1;
    volatile uint16_t BITFLD_WAKEUPCT_ENABLE                  : 1;
    volatile uint16_t BITFLD_I2C_ENABLE                       : 1;
    volatile uint16_t BITFLD_UART2_ENABLE                     : 1;
    volatile uint16_t BITFLD_UART1_ENABLE                     : 1;
    volatile uint16_t BITFLD_SPI_DIV                          : 2;
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_SPI_ENABLE                       : 1;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_QUAD_ENABLE                      : 1;
};
#define TMR_DIV                              (0x0003)
#define TMR_ENABLE                           (0x0008)
#define WAKEUPCT_ENABLE                      (0x0010)
#define I2C_ENABLE                           (0x0020)
#define UART2_ENABLE                         (0x0040)
#define UART1_ENABLE                         (0x0080)
#define SPI_DIV                              (0x0300)
#define SPI_ENABLE                           (0x0800)
#define QUAD_ENABLE                          (0x8000)



#define CLK_RADIO_REG                        (0x50000008) /* Radio PLL control register */
#define CLK_RADIO_REG_RESET                  (0x00000040)

/*=============================*/
struct __CLK_RADIO_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RFCU_DIV                         : 2;
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_RFCU_ENABLE                      : 1;
    volatile uint16_t BITFLD_BLE_DIV                          : 2;
    volatile uint16_t BITFLD_BLE_LP_RESET                     : 1;
    volatile uint16_t BITFLD_BLE_ENABLE                       : 1;
};
#define RFCU_DIV                             (0x0003)
#define RFCU_ENABLE                          (0x0008)
#define BLE_DIV                              (0x0030)
#define BLE_LP_RESET                         (0x0040)
#define BLE_ENABLE                           (0x0080)



#define CLK_CTRL_REG                         (0x5000000A) /* Clock control register */
#define CLK_CTRL_REG_RESET                   (0x00000080)

/*=============================*/
struct __CLK_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SYS_CLK_SEL                      : 2;
    volatile uint16_t BITFLD_XTAL16M_DISABLE                  : 1;
    volatile uint16_t BITFLD_XTAL16M_SPIKE_FLT_DISABLE        : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_RUNNING_AT_32K                   : 1;
    volatile uint16_t BITFLD_RUNNING_AT_RC16M                 : 1;
    volatile uint16_t BITFLD_RUNNING_AT_XTAL16M               : 1;
};
#define SYS_CLK_SEL                          (0x0003)
#define XTAL16M_DISABLE                      (0x0004)
#define XTAL16M_SPIKE_FLT_DISABLE            (0x0008)
#define RUNNING_AT_32K                       (0x0020)
#define RUNNING_AT_RC16M                     (0x0040)
#define RUNNING_AT_XTAL16M                   (0x0080)



#define PMU_CTRL_REG                         (0x50000010) /* Power Management Unit control register */
#define PMU_CTRL_REG_RESET                   (0x00000006)

/*=============================*/
struct __PMU_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RESET_ON_WAKEUP                  : 1;
    volatile uint16_t BITFLD_PERIPH_SLEEP                     : 1;
    volatile uint16_t BITFLD_RADIO_SLEEP                      : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_OTP_COPY_DIV                     : 2;
    volatile uint16_t BITFLD_FORCE_BUCK                       : 1;
    volatile uint16_t BITFLD_FORCE_BOOST                      : 1;
    volatile uint16_t BITFLD_RETENTION_MODE                   : 4;
};
#define RESET_ON_WAKEUP                      (0x0001)
#define PERIPH_SLEEP                         (0x0002)
#define RADIO_SLEEP                          (0x0004)
#define OTP_COPY_DIV                         (0x0030)
#define FORCE_BUCK                           (0x0040)
#define FORCE_BOOST                          (0x0080)
#define RETENTION_MODE                       (0x0F00)



#define SYS_CTRL_REG                         (0x50000012) /* System Control register */
#define SYS_CTRL_REG_RESET                   (0x00000020)

/*=============================*/
struct __SYS_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_REMAP_ADR0                       : 2;
    volatile uint16_t BITFLD_DEV_PHASE                        : 1;
    volatile uint16_t BITFLD_CLK32_SOURCE                     : 1;
    volatile uint16_t BITFLD_OTP_COPY                         : 1;
    volatile uint16_t BITFLD_PAD_LATCH_EN                     : 1;
    volatile uint16_t BITFLD_OTPC_RESET_REQ                   : 1;
    volatile uint16_t BITFLD_DEBUGGER_ENABLE                  : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_TIMEOUT_DISABLE                  : 1;
    volatile uint16_t                                         : 5;
    volatile uint16_t BITFLD_SW_RESET                         : 1;
};
#define REMAP_ADR0                           (0x0003)
#define DEV_PHASE                            (0x0004)
#define CLK32_SOURCE                         (0x0008)
#define OTP_COPY                             (0x0010)
#define PAD_LATCH_EN                         (0x0020)
#define OTPC_RESET_REQ                       (0x0040)
#define DEBUGGER_ENABLE                      (0x0080)
#define TIMEOUT_DISABLE                      (0x0200)
#define SW_RESET                             (0x8000)



#define SYS_STAT_REG                         (0x50000014) /* System status register */
#define SYS_STAT_REG_RESET                   (0x00000055)

/*=============================*/
struct __SYS_STAT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RAD_IS_DOWN                      : 1;
    volatile uint16_t BITFLD_RAD_IS_UP                        : 1;
    volatile uint16_t BITFLD_PER_IS_DOWN                      : 1;
    volatile uint16_t BITFLD_PER_IS_UP                        : 1;
    volatile uint16_t BITFLD_DBG_IS_DOWN                      : 1;
    volatile uint16_t BITFLD_DBG_IS_UP                        : 1;
    volatile uint16_t BITFLD_XTAL16_TRIM_READY                : 1;
    volatile uint16_t BITFLD_XTAL16_SETTLED                   : 1;
};
#define RAD_IS_DOWN                          (0x0001)
#define RAD_IS_UP                            (0x0002)
#define PER_IS_DOWN                          (0x0004)
#define PER_IS_UP                            (0x0008)
#define DBG_IS_DOWN                          (0x0010)
#define DBG_IS_UP                            (0x0020)
#define XTAL16_TRIM_READY                    (0x0040)
#define XTAL16_SETTLED                       (0x0080)



#define TRIM_CTRL_REG                        (0x50000016) /* Control trimming of the XTAL16M */
#define TRIM_CTRL_REG_RESET                  (0x000000A2)

/*=============================*/
struct __TRIM_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SETTLE_TIME                      : 4;
    volatile uint16_t BITFLD_TRIM_TIME                        : 4;
};
#define SETTLE_TIME                          (0x000F)
#define TRIM_TIME                            (0x00F0)



#define CLK_32K_REG                          (0x50000020) /* 32 kHz oscillator register */
#define CLK_32K_REG_RESET                    (0x000007AE)

/*=============================*/
struct __CLK_32K_REG
/*=============================*/
{
    volatile uint16_t BITFLD_XTAL32K_ENABLE                   : 1;
    volatile uint16_t BITFLD_XTAL32K_RBIAS                    : 2;
    volatile uint16_t BITFLD_XTAL32K_CUR                      : 4;
    volatile uint16_t BITFLD_RC32K_ENABLE                     : 1;
    volatile uint16_t BITFLD_RC32K_TRIM                       : 4;
    volatile uint16_t BITFLD_XTAL32K_DISABLE_AMPREG           : 1;
};
#define XTAL32K_ENABLE                       (0x0001)
#define XTAL32K_RBIAS                        (0x0006)
#define XTAL32K_CUR                          (0x0078)
#define RC32K_ENABLE                         (0x0080)
#define RC32K_TRIM                           (0x0F00)
#define XTAL32K_DISABLE_AMPREG               (0x1000)



#define CLK_16M_REG                          (0x50000022) /* 16 MHz RC-oscillator register */
#define CLK_16M_REG_RESET                    (0x000000A0)

/*=============================*/
struct __CLK_16M_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RC16M_ENABLE                     : 1;
    volatile uint16_t BITFLD_RC16M_TRIM                       : 4;
    volatile uint16_t BITFLD_XTAL16_CUR_SET                   : 3;
    volatile uint16_t BITFLD_XTAL16_BIAS_SH_ENABLE            : 1;
    volatile uint16_t BITFLD_XTAL16_NOISE_FILT_ENABLE         : 1;
};
#define RC16M_ENABLE                         (0x0001)
#define RC16M_TRIM                           (0x001E)
#define XTAL16_CUR_SET                       (0x00E0)
#define XTAL16_BIAS_SH_ENABLE                (0x0100)
#define XTAL16_NOISE_FILT_ENABLE             (0x0200)



#define CLK_RCX20K_REG                       (0x50000024) /* 20 kHz RXC-oscillator control register */
#define CLK_RCX20K_REG_RESET                 (0x000005B0)

/*=============================*/
struct __CLK_RCX20K_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RCX20K_TRIM                      : 4;
    volatile uint16_t BITFLD_RCX20K_NTC                       : 4;
    volatile uint16_t BITFLD_RCX20K_BIAS                      : 2;
    volatile uint16_t BITFLD_RCX20K_LOWF                      : 1;
    volatile uint16_t BITFLD_RCX20K_ENABLE                    : 1;
    volatile uint16_t BITFLD_RCX20K_SELECT                    : 1;
};
#define RCX20K_TRIM                          (0x000F)
#define RCX20K_NTC                           (0x00F0)
#define RCX20K_BIAS                          (0x0300)
#define RCX20K_LOWF                          (0x0400)
#define RCX20K_ENABLE                        (0x0800)
#define RCX20K_SELECT                        (0x1000)



#define BANDGAP_REG                          (0x50000028) /* Bandgap trimming */
#define BANDGAP_REG_RESET                    (0x00000000)

/*=============================*/
struct __BANDGAP_REG
/*=============================*/
{
    volatile uint16_t BITFLD_BGR_TRIM                         : 5;
    volatile uint16_t BITFLD_BGR_ITRIM                        : 5;
    volatile uint16_t BITFLD_LDO_RET_TRIM                     : 4;
    volatile uint16_t BITFLD_BGR_LOWPOWER                     : 1;
};
#define BGR_TRIM                             (0x001F)
#define BGR_ITRIM                            (0x03E0)
#define LDO_RET_TRIM                         (0x3C00)
#define BGR_LOWPOWER                         (0x4000)



#define ANA_STATUS_REG                       (0x5000002A) /* Status bit of analog (power management) circuits */
#define ANA_STATUS_REG_RESET                 (0x00000090)

/*=============================*/
struct __ANA_STATUS_REG
/*=============================*/
{
    volatile uint16_t BITFLD_VBAT1V_AVAILABLE                 : 1;
    volatile uint16_t BITFLD_VBAT1V_OK                        : 1;
    volatile uint16_t BITFLD_VDCDC_OK                         : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_LDO_VDD_OK                       : 1;
    volatile uint16_t BITFLD_LDO_ANA_OK                       : 1;
    volatile uint16_t BITFLD_BOOST_VBAT_OK                    : 1;
    volatile uint16_t BITFLD_BANDGAP_OK                       : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_BOOST_SELECTED                   : 1;
};
#define VBAT1V_AVAILABLE                     (0x0001)
#define VBAT1V_OK                            (0x0002)
#define VDCDC_OK                             (0x0004)
#define LDO_VDD_OK                           (0x0010)
#define LDO_ANA_OK                           (0x0020)
#define BOOST_VBAT_OK                        (0x0040)
#define BANDGAP_OK                           (0x0080)
#define BOOST_SELECTED                       (0x0200)



#define POR_PIN_REG                          (0x50000040) /* Selects a GPIO pin for POR generation */
#define POR_PIN_REG_RESET                    (0x00000000)

/*=============================*/
struct __POR_PIN_REG
/*=============================*/
{
    volatile uint16_t BITFLD_POR_PIN_SELECT                   : 6;
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_POR_PIN_POLARITY                 : 1;
};
#define POR_PIN_SELECT                       (0x003F)
#define POR_PIN_POLARITY                     (0x0080)



#define POR_TIMER_REG                        (0x50000042) /* Time for POR to happen */
#define POR_TIMER_REG_RESET                  (0x00000018)

/*=============================*/
struct __POR_TIMER_REG
/*=============================*/
{
    volatile uint16_t BITFLD_POR_TIME                         : 7;
};
#define POR_TIME                             (0x007F)



#define PCM_DIV_REG                          (0x50001C40) /* PCM divider and enables */
#define PCM_DIV_REG_RESET                    (0x00000000)

/*=============================*/
struct __PCM_DIV_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PCM_DIV                          : 12;
    volatile uint16_t BITFLD_CLK_PCM_EN                       : 1;
    volatile uint16_t BITFLD_PCM_SRC_SEL                      : 1;
};
#define PCM_DIV                              (0x0FFF)
#define CLK_PCM_EN                           (0x1000)
#define PCM_SRC_SEL                          (0x2000)



#define PCM_FDIV_REG                         (0x50001C42) /* PCM fractional division register */
#define PCM_FDIV_REG_RESET                   (0x00000000)

/*=============================*/
struct __PCM_FDIV_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PCM_FDIV                         : 16;
};
#define PCM_FDIV                             (0xFFFF)



#define PDM_DIV_REG                          (0x50001C44) /* PDM divider and enables */
#define PDM_DIV_REG_RESET                    (0x00000000)

/*=============================*/
struct __PDM_DIV_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PDM_DIV                          : 8;
    volatile uint16_t BITFLD_CLK_PDM_EN                       : 1;
    volatile uint16_t BITFLD_PDM_MASTER_MODE                  : 1;
};
#define PDM_DIV                              (0x00FF)
#define CLK_PDM_EN                           (0x0100)
#define PDM_MASTER_MODE                      (0x0200)



#define SRC_DIV_REG                          (0x50001C46) /* SRC divider and enables */
#define SRC_DIV_REG_RESET                    (0x00000000)

/*=============================*/
struct __SRC_DIV_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRC_DIV                          : 8;
    volatile uint16_t BITFLD_CLK_SRC_EN                       : 1;
};
#define SRC_DIV                              (0x00FF)
#define CLK_SRC_EN                           (0x0100)



/*===========================================================================*/
/* memory map crg580_dcdc_nl01 */
/*===========================================================================*/
//reserved only



/*===========================================================================*/
/* memory map DMA */
/*===========================================================================*/



#define DMA0_A_STARTL_REG                    (0x50003600) /* Start address Low A of DMA channel 0 */
#define DMA0_A_STARTL_REG_RESET              (0x00000000)

/*=============================*/
struct __DMA0_A_STARTL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA0_A_STARTL                    : 16;
};
#define DMA0_A_STARTL                        (0xFFFF)



#define DMA0_A_STARTH_REG                    (0x50003602) /* Start address High A of DMA channel 0 */
#define DMA0_A_STARTH_REG_RESET              (0x00000000)

/*=============================*/
struct __DMA0_A_STARTH_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA0_A_STARTH                    : 16;
};
#define DMA0_A_STARTH                        (0xFFFF)



#define DMA0_B_STARTL_REG                    (0x50003604) /* Start address Low B of DMA channel 0 */
#define DMA0_B_STARTL_REG_RESET              (0x00000000)

/*=============================*/
struct __DMA0_B_STARTL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA0_B_STARTL                    : 16;
};
#define DMA0_B_STARTL                        (0xFFFF)



#define DMA0_B_STARTH_REG                    (0x50003606) /* Start address High B of DMA channel 0 */
#define DMA0_B_STARTH_REG_RESET              (0x00000000)

/*=============================*/
struct __DMA0_B_STARTH_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA0_B_STARTH                    : 16;
};
#define DMA0_B_STARTH                        (0xFFFF)



#define DMA0_INT_REG                         (0x50003608) /* DMA receive interrupt register channel 0 */
#define DMA0_INT_REG_RESET                   (0x00000000)

/*=============================*/
struct __DMA0_INT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA0_INT                         : 16;
};
#define DMA0_INT                             (0xFFFF)



#define DMA0_LEN_REG                         (0x5000360A) /* DMA receive length register channel 0 */
#define DMA0_LEN_REG_RESET                   (0x00000000)

/*=============================*/
struct __DMA0_LEN_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA0_LEN                         : 16;
};
#define DMA0_LEN                             (0xFFFF)



#define DMA0_CTRL_REG                        (0x5000360C) /* Control register for the DMA channel 0 */
#define DMA0_CTRL_REG_RESET                  (0x00000000)

/*=============================*/
struct __DMA0_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA_ON                           : 1;
    volatile uint16_t BITFLD_BW                               : 2;
    volatile uint16_t BITFLD_IRQ_ENABLE                       : 1;
    volatile uint16_t BITFLD_DREQ_MODE                        : 1;
    volatile uint16_t BITFLD_BINC                             : 1;
    volatile uint16_t BITFLD_AINC                             : 1;
    volatile uint16_t BITFLD_CIRCULAR                         : 1;
    volatile uint16_t BITFLD_DMA_PRIO                         : 3;
    volatile uint16_t BITFLD_DMA_IDLE                         : 1;
    volatile uint16_t BITFLD_DMA_INIT                         : 1;
    volatile uint16_t BITFLD_REQ_SENSE                        : 1;
};
#define DMA_ON                               (0x0001)
#define BW                                   (0x0006)
#define IRQ_ENABLE                           (0x0008)
#define DREQ_MODE                            (0x0010)
#define BINC                                 (0x0020)
#define AINC                                 (0x0040)
#define CIRCULAR                             (0x0080)
#define DMA_PRIO                             (0x0700)
#define DMA_IDLE                             (0x0800)
#define DMA_INIT                             (0x1000)
#define REQ_SENSE                            (0x2000)



#define DMA0_IDX_REG                         (0x5000360E) /* Index value of DMA channel 0 */
#define DMA0_IDX_REG_RESET                   (0x00000000)

/*=============================*/
struct __DMA0_IDX_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA0_IDX                         : 16;
};
#define DMA0_IDX                             (0xFFFF)



#define DMA1_A_STARTL_REG                    (0x50003610) /* Start address Low A of DMA channel 1 */
#define DMA1_A_STARTL_REG_RESET              (0x00000000)

/*=============================*/
struct __DMA1_A_STARTL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA1_A_STARTL                    : 16;
};
#define DMA1_A_STARTL                        (0xFFFF)



#define DMA1_A_STARTH_REG                    (0x50003612) /* Start address High A of DMA channel 1 */
#define DMA1_A_STARTH_REG_RESET              (0x00000000)

/*=============================*/
struct __DMA1_A_STARTH_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA1_A_STARTH                    : 16;
};
#define DMA1_A_STARTH                        (0xFFFF)



#define DMA1_B_STARTL_REG                    (0x50003614) /* Start address Low B of DMA channel 1 */
#define DMA1_B_STARTL_REG_RESET              (0x00000000)

/*=============================*/
struct __DMA1_B_STARTL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA1_B_STARTL                    : 16;
};
#define DMA1_B_STARTL                        (0xFFFF)



#define DMA1_B_STARTH_REG                    (0x50003616) /* Start address High B of DMA channel 1 */
#define DMA1_B_STARTH_REG_RESET              (0x00000000)

/*=============================*/
struct __DMA1_B_STARTH_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA1_B_STARTH                    : 16;
};
#define DMA1_B_STARTH                        (0xFFFF)



#define DMA1_INT_REG                         (0x50003618) /* DMA receive interrupt register channel 1 */
#define DMA1_INT_REG_RESET                   (0x00000000)

/*=============================*/
struct __DMA1_INT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA1_INT                         : 16;
};
#define DMA1_INT                             (0xFFFF)



#define DMA1_LEN_REG                         (0x5000361A) /* DMA receive length register channel 1 */
#define DMA1_LEN_REG_RESET                   (0x00000000)

/*=============================*/
struct __DMA1_LEN_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA1_LEN                         : 16;
};
#define DMA1_LEN                             (0xFFFF)



#define DMA1_CTRL_REG                        (0x5000361C) /* Control register for the DMA channel 1 */
#define DMA1_CTRL_REG_RESET                  (0x00000000)

/*=============================*/
struct __DMA1_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA_ON                           : 1;
    volatile uint16_t BITFLD_BW                               : 2;
    volatile uint16_t BITFLD_IRQ_ENABLE                       : 1;
    volatile uint16_t BITFLD_DREQ_MODE                        : 1;
    volatile uint16_t BITFLD_BINC                             : 1;
    volatile uint16_t BITFLD_AINC                             : 1;
    volatile uint16_t BITFLD_CIRCULAR                         : 1;
    volatile uint16_t BITFLD_DMA_PRIO                         : 3;
    volatile uint16_t BITFLD_DMA_IDLE                         : 1;
    volatile uint16_t BITFLD_DMA_INIT                         : 1;
    volatile uint16_t BITFLD_REQ_SENSE                        : 1;
};
#define DMA_ON                               (0x0001)
#define BW                                   (0x0006)
#define IRQ_ENABLE                           (0x0008)
#define DREQ_MODE                            (0x0010)
#define BINC                                 (0x0020)
#define AINC                                 (0x0040)
#define CIRCULAR                             (0x0080)
#define DMA_PRIO                             (0x0700)
#define DMA_IDLE                             (0x0800)
#define DMA_INIT                             (0x1000)
#define REQ_SENSE                            (0x2000)



#define DMA1_IDX_REG                         (0x5000361E) /* Index value of DMA channel 1 */
#define DMA1_IDX_REG_RESET                   (0x00000000)

/*=============================*/
struct __DMA1_IDX_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA1_IDX                         : 16;
};
#define DMA1_IDX                             (0xFFFF)



#define DMA2_A_STARTL_REG                    (0x50003620) /* Start address Low A of DMA channel 2 */
#define DMA2_A_STARTL_REG_RESET              (0x00000000)

/*=============================*/
struct __DMA2_A_STARTL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA2_A_STARTL                    : 16;
};
#define DMA2_A_STARTL                        (0xFFFF)



#define DMA2_A_STARTH_REG                    (0x50003622) /* Start address High A of DMA channel 2 */
#define DMA2_A_STARTH_REG_RESET              (0x00000000)

/*=============================*/
struct __DMA2_A_STARTH_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA2_A_STARTH                    : 16;
};
#define DMA2_A_STARTH                        (0xFFFF)



#define DMA2_B_STARTL_REG                    (0x50003624) /* Start address Low B of DMA channel 2 */
#define DMA2_B_STARTL_REG_RESET              (0x00000000)

/*=============================*/
struct __DMA2_B_STARTL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA2_B_STARTL                    : 16;
};
#define DMA2_B_STARTL                        (0xFFFF)



#define DMA2_B_STARTH_REG                    (0x50003626) /* Start address High B of DMA channel 2 */
#define DMA2_B_STARTH_REG_RESET              (0x00000000)

/*=============================*/
struct __DMA2_B_STARTH_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA2_B_STARTH                    : 16;
};
#define DMA2_B_STARTH                        (0xFFFF)



#define DMA2_INT_REG                         (0x50003628) /* DMA receive interrupt register channel 2 */
#define DMA2_INT_REG_RESET                   (0x00000000)

/*=============================*/
struct __DMA2_INT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA2_INT                         : 16;
};
#define DMA2_INT                             (0xFFFF)



#define DMA2_LEN_REG                         (0x5000362A) /* DMA receive length register channel 2 */
#define DMA2_LEN_REG_RESET                   (0x00000000)

/*=============================*/
struct __DMA2_LEN_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA2_LEN                         : 16;
};
#define DMA2_LEN                             (0xFFFF)



#define DMA2_CTRL_REG                        (0x5000362C) /* Control register for the DMA channel 2 */
#define DMA2_CTRL_REG_RESET                  (0x00000000)

/*=============================*/
struct __DMA2_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA_ON                           : 1;
    volatile uint16_t BITFLD_BW                               : 2;
    volatile uint16_t BITFLD_IRQ_ENABLE                       : 1;
    volatile uint16_t BITFLD_DREQ_MODE                        : 1;
    volatile uint16_t BITFLD_BINC                             : 1;
    volatile uint16_t BITFLD_AINC                             : 1;
    volatile uint16_t BITFLD_CIRCULAR                         : 1;
    volatile uint16_t BITFLD_DMA_PRIO                         : 3;
    volatile uint16_t BITFLD_DMA_IDLE                         : 1;
    volatile uint16_t BITFLD_DMA_INIT                         : 1;
    volatile uint16_t BITFLD_REQ_SENSE                        : 1;
};
#define DMA_ON                               (0x0001)
#define BW                                   (0x0006)
#define IRQ_ENABLE                           (0x0008)
#define DREQ_MODE                            (0x0010)
#define BINC                                 (0x0020)
#define AINC                                 (0x0040)
#define CIRCULAR                             (0x0080)
#define DMA_PRIO                             (0x0700)
#define DMA_IDLE                             (0x0800)
#define DMA_INIT                             (0x1000)
#define REQ_SENSE                            (0x2000)



#define DMA2_IDX_REG                         (0x5000362E) /* Index value of DMA channel 2 */
#define DMA2_IDX_REG_RESET                   (0x00000000)

/*=============================*/
struct __DMA2_IDX_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA2_IDX                         : 16;
};
#define DMA2_IDX                             (0xFFFF)



#define DMA3_A_STARTL_REG                    (0x50003630) /* Start address Low A of DMA channel 3 */
#define DMA3_A_STARTL_REG_RESET              (0x00000000)

/*=============================*/
struct __DMA3_A_STARTL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA3_A_STARTL                    : 16;
};
#define DMA3_A_STARTL                        (0xFFFF)



#define DMA3_A_STARTH_REG                    (0x50003632) /* Start address High A of DMA channel 3 */
#define DMA3_A_STARTH_REG_RESET              (0x00000000)

/*=============================*/
struct __DMA3_A_STARTH_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA3_A_STARTH                    : 16;
};
#define DMA3_A_STARTH                        (0xFFFF)



#define DMA3_B_STARTL_REG                    (0x50003634) /* Start address Low B of DMA channel 3 */
#define DMA3_B_STARTL_REG_RESET              (0x00000000)

/*=============================*/
struct __DMA3_B_STARTL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA3_B_STARTL                    : 16;
};
#define DMA3_B_STARTL                        (0xFFFF)



#define DMA3_B_STARTH_REG                    (0x50003636) /* Start address High B of DMA channel 3 */
#define DMA3_B_STARTH_REG_RESET              (0x00000000)

/*=============================*/
struct __DMA3_B_STARTH_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA3_B_STARTH                    : 16;
};
#define DMA3_B_STARTH                        (0xFFFF)



#define DMA3_INT_REG                         (0x50003638) /* DMA receive interrupt register channel 3 */
#define DMA3_INT_REG_RESET                   (0x00000000)

/*=============================*/
struct __DMA3_INT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA3_INT                         : 16;
};
#define DMA3_INT                             (0xFFFF)



#define DMA3_LEN_REG                         (0x5000363A) /* DMA receive length register channel 3 */
#define DMA3_LEN_REG_RESET                   (0x00000000)

/*=============================*/
struct __DMA3_LEN_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA3_LEN                         : 16;
};
#define DMA3_LEN                             (0xFFFF)



#define DMA3_CTRL_REG                        (0x5000363C) /* Control register for the DMA channel 3 */
#define DMA3_CTRL_REG_RESET                  (0x00000000)

/*=============================*/
struct __DMA3_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA_ON                           : 1;
    volatile uint16_t BITFLD_BW                               : 2;
    volatile uint16_t BITFLD_IRQ_ENABLE                       : 1;
    volatile uint16_t BITFLD_DREQ_MODE                        : 1;
    volatile uint16_t BITFLD_BINC                             : 1;
    volatile uint16_t BITFLD_AINC                             : 1;
    volatile uint16_t BITFLD_CIRCULAR                         : 1;
    volatile uint16_t BITFLD_DMA_PRIO                         : 3;
    volatile uint16_t BITFLD_DMA_IDLE                         : 1;
    volatile uint16_t BITFLD_DMA_INIT                         : 1;
    volatile uint16_t BITFLD_REQ_SENSE                        : 1;
};
#define DMA_ON                               (0x0001)
#define BW                                   (0x0006)
#define IRQ_ENABLE                           (0x0008)
#define DREQ_MODE                            (0x0010)
#define BINC                                 (0x0020)
#define AINC                                 (0x0040)
#define CIRCULAR                             (0x0080)
#define DMA_PRIO                             (0x0700)
#define DMA_IDLE                             (0x0800)
#define DMA_INIT                             (0x1000)
#define REQ_SENSE                            (0x2000)



#define DMA3_IDX_REG                         (0x5000363E) /* Index value of DMA channel 3 */
#define DMA3_IDX_REG_RESET                   (0x00000000)

/*=============================*/
struct __DMA3_IDX_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA3_IDX                         : 16;
};
#define DMA3_IDX                             (0xFFFF)



#define DMA_REQ_MUX_REG                      (0x50003680) /* DMA channel assignments */
#define DMA_REQ_MUX_REG_RESET                (0x0000FFFF)

/*=============================*/
struct __DMA_REQ_MUX_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA01_SEL                        : 4;
    volatile uint16_t BITFLD_DMA23_SEL                        : 4;
    volatile uint16_t                                         : 4;
};
#define DMA01_SEL                            (0x000F)
#define DMA23_SEL                            (0x00F0)



#define DMA_INT_STATUS_REG                   (0x50003682) /* DMA interrupt status register */
#define DMA_INT_STATUS_REG_RESET             (0x00000000)

/*=============================*/
struct __DMA_INT_STATUS_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA_IRQ_CH0                      : 1;
    volatile uint16_t BITFLD_DMA_IRQ_CH1                      : 1;
    volatile uint16_t BITFLD_DMA_IRQ_CH2                      : 1;
    volatile uint16_t BITFLD_DMA_IRQ_CH3                      : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t                                         : 1;
};
#define DMA_IRQ_CH0                          (0x0001)
#define DMA_IRQ_CH1                          (0x0002)
#define DMA_IRQ_CH2                          (0x0004)
#define DMA_IRQ_CH3                          (0x0008)



#define DMA_CLEAR_INT_REG                    (0x50003684) /* DMA clear interrupt register */
#define DMA_CLEAR_INT_REG_RESET              (0x00000000)

/*=============================*/
struct __DMA_CLEAR_INT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMA_RST_IRQ_CH0                  : 1;
    volatile uint16_t BITFLD_DMA_RST_IRQ_CH1                  : 1;
    volatile uint16_t BITFLD_DMA_RST_IRQ_CH2                  : 1;
    volatile uint16_t BITFLD_DMA_RST_IRQ_CH3                  : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t                                         : 1;
};
#define DMA_RST_IRQ_CH0                      (0x0001)
#define DMA_RST_IRQ_CH1                      (0x0002)
#define DMA_RST_IRQ_CH2                      (0x0004)
#define DMA_RST_IRQ_CH3                      (0x0008)



/*===========================================================================*/
/* memory map GPADC */
/*===========================================================================*/



#define GP_ADC_CTRL_REG                      (0x50001500) /* General Purpose ADC Control Register */
#define GP_ADC_CTRL_REG_RESET                (0x00000000)

/*=============================*/
struct __GP_ADC_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_GP_ADC_EN                        : 1;
    volatile uint16_t BITFLD_GP_ADC_START                     : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_GP_ADC_CLK_SEL                   : 1;
    volatile uint16_t BITFLD_GP_ADC_INT                       : 1;
    volatile uint16_t BITFLD_GP_ADC_MINT                      : 1;
    volatile uint16_t BITFLD_GP_ADC_SEL                       : 4;
    volatile uint16_t BITFLD_GP_ADC_SIGN                      : 1;
    volatile uint16_t BITFLD_GP_ADC_SE                        : 1;
    volatile uint16_t BITFLD_GP_ADC_MUTE                      : 1;
    volatile uint16_t BITFLD_GP_ADC_CHOP                      : 1;
    volatile uint16_t BITFLD_GP_ADC_LDO_EN                    : 1;
    volatile uint16_t BITFLD_GP_ADC_LDO_ZERO                  : 1;
};
#define GP_ADC_EN                            (0x0001)
#define GP_ADC_START                         (0x0002)
#define GP_ADC_CLK_SEL                       (0x0008)
#define GP_ADC_INT                           (0x0010)
#define GP_ADC_MINT                          (0x0020)
#define GP_ADC_SEL                           (0x03C0)
#define GP_ADC_SIGN                          (0x0400)
#define GP_ADC_SE                            (0x0800)
#define GP_ADC_MUTE                          (0x1000)
#define GP_ADC_CHOP                          (0x2000)
#define GP_ADC_LDO_EN                        (0x4000)
#define GP_ADC_LDO_ZERO                      (0x8000)



#define GP_ADC_CTRL2_REG                     (0x50001502) /* General Purpose ADC Second Control Register */
#define GP_ADC_CTRL2_REG_RESET               (0x00000000)

/*=============================*/
struct __GP_ADC_CTRL2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_GP_ADC_DELAY_EN                  : 1;
    volatile uint16_t BITFLD_GP_ADC_ATTN3X                    : 1;
    volatile uint16_t BITFLD_GP_ADC_IDYN                      : 1;
    volatile uint16_t BITFLD_GP_ADC_I20U                      : 1;
};
#define GP_ADC_DELAY_EN                      (0x0001)
#define GP_ADC_ATTN3X                        (0x0002)
#define GP_ADC_IDYN                          (0x0004)
#define GP_ADC_I20U                          (0x0008)



#define GP_ADC_OFFP_REG                      (0x50001504) /* General Purpose ADC Positive Offset Register */
#define GP_ADC_OFFP_REG_RESET                (0x00000200)

/*=============================*/
struct __GP_ADC_OFFP_REG
/*=============================*/
{
    volatile uint16_t BITFLD_GP_ADC_OFFP                      : 10;
};
#define GP_ADC_OFFP                          (0x03FF)



#define GP_ADC_OFFN_REG                      (0x50001506) /* General Purpose ADC Negative Offset Register */
#define GP_ADC_OFFN_REG_RESET                (0x00000200)

/*=============================*/
struct __GP_ADC_OFFN_REG
/*=============================*/
{
    volatile uint16_t BITFLD_GP_ADC_OFFN                      : 10;
};
#define GP_ADC_OFFN                          (0x03FF)



#define GP_ADC_CLEAR_INT_REG                 (0x50001508) /* General Purpose ADC Clear Interrupt Register */
#define GP_ADC_CLEAR_INT_REG_RESET           (0x00000000)

/*=============================*/
struct __GP_ADC_CLEAR_INT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_GP_ADC_CLR_INT                   : 16;
};
#define GP_ADC_CLR_INT                       (0xFFFF)



#define GP_ADC_RESULT_REG                    (0x5000150A) /* General Purpose ADC Result Register */
#define GP_ADC_RESULT_REG_RESET              (0x00000000)

/*=============================*/
struct __GP_ADC_RESULT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_GP_ADC_VAL                       : 10;
};
#define GP_ADC_VAL                           (0x03FF)



#define GP_ADC_DELAY_REG                     (0x5000150C) /* General Purpose ADC Delay Register */
#define GP_ADC_DELAY_REG_RESET               (0x00000000)

/*=============================*/
struct __GP_ADC_DELAY_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DEL_LDO_EN                       : 8;
};
#define DEL_LDO_EN                           (0x00FF)



#define GP_ADC_DELAY2_REG                    (0x5000150E) /* General Purpose ADC Second Delay Register */
#define GP_ADC_DELAY2_REG_RESET              (0x00008880)

/*=============================*/
struct __GP_ADC_DELAY2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DEL_ADC_EN                       : 8;
    volatile uint16_t BITFLD_DEL_ADC_START                    : 8;
};
#define DEL_ADC_EN                           (0x00FF)
#define DEL_ADC_START                        (0xFF00)



/*===========================================================================*/
/* memory map GPIO */
/*===========================================================================*/



#define P0_DATA_REG                          (0x50003000) /* P0 Data input / output register */
#define P0_DATA_REG_RESET                    (0x00000000)

/*=============================*/
struct __P0_DATA_REG
/*=============================*/
{
    volatile uint16_t BITFLD_P0_DATA                          : 8;
};
#define P0_DATA                              (0x00FF)



#define P0_SET_DATA_REG                      (0x50003002) /* P0 Set port pins register */
#define P0_SET_DATA_REG_RESET                (0x00000000)

/*=============================*/
struct __P0_SET_DATA_REG
/*=============================*/
{
    volatile uint16_t BITFLD_P0_SET                           : 8;
};
#define P0_SET                               (0x00FF)



#define P0_RESET_DATA_REG                    (0x50003004) /* P0 Reset port pins register */
#define P0_RESET_DATA_REG_RESET              (0x00000000)

/*=============================*/
struct __P0_RESET_DATA_REG
/*=============================*/
{
    volatile uint16_t BITFLD_P0_RESET                         : 8;
};
#define P0_RESET                             (0x00FF)



#define P00_MODE_REG                         (0x50003006) /* P00 Mode Register */
#define P00_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P00_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P01_MODE_REG                         (0x50003008) /* P01 Mode Register */
#define P01_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P01_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P02_MODE_REG                         (0x5000300A) /* P02 Mode Register */
#define P02_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P02_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P03_MODE_REG                         (0x5000300C) /* P03 Mode Register */
#define P03_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P03_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P04_MODE_REG                         (0x5000300E) /* P04 Mode Register */
#define P04_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P04_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P05_MODE_REG                         (0x50003010) /* P05 Mode Register */
#define P05_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P05_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P06_MODE_REG                         (0x50003012) /* P06 Mode Register */
#define P06_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P06_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P07_MODE_REG                         (0x50003014) /* P07 Mode Register */
#define P07_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P07_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P1_DATA_REG                          (0x50003020) /* P1 Data input / output register */
#define P1_DATA_REG_RESET                    (0x00000020)

/*=============================*/
struct __P1_DATA_REG
/*=============================*/
{
    volatile uint16_t BITFLD_P1_DATA                          : 8;
};
#define P1_DATA                              (0x00FF)



#define P1_SET_DATA_REG                      (0x50003022) /* P1 Set port pins register */
#define P1_SET_DATA_REG_RESET                (0x00000000)

/*=============================*/
struct __P1_SET_DATA_REG
/*=============================*/
{
    volatile uint16_t BITFLD_P1_SET                           : 8;
};
#define P1_SET                               (0x00FF)



#define P1_RESET_DATA_REG                    (0x50003024) /* P1 Reset port pins register */
#define P1_RESET_DATA_REG_RESET              (0x00000000)

/*=============================*/
struct __P1_RESET_DATA_REG
/*=============================*/
{
    volatile uint16_t BITFLD_P1_RESET                         : 8;
};
#define P1_RESET                             (0x00FF)



#define P10_MODE_REG                         (0x50003026) /* P10 Mode Register */
#define P10_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P10_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P11_MODE_REG                         (0x50003028) /* P11 Mode Register */
#define P11_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P11_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P12_MODE_REG                         (0x5000302A) /* P12 Mode Register */
#define P12_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P12_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P13_MODE_REG                         (0x5000302C) /* P13 Mode Register */
#define P13_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P13_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P14_MODE_REG                         (0x5000302E) /* P14 Mode Register */
#define P14_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P14_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P15_MODE_REG                         (0x50003030) /* P15 Mode Register */
#define P15_MODE_REG_RESET                   (0x00000100)

/*=============================*/
struct __P15_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P2_DATA_REG                          (0x50003040) /* P2 Data input / output register */
#define P2_DATA_REG_RESET                    (0x00000000)

/*=============================*/
struct __P2_DATA_REG
/*=============================*/
{
    volatile uint16_t BITFLD_P2_DATA                          : 10;
};
#define P2_DATA                              (0x03FF)



#define P2_SET_DATA_REG                      (0x50003042) /* P2 Set port pins register */
#define P2_SET_DATA_REG_RESET                (0x00000000)

/*=============================*/
struct __P2_SET_DATA_REG
/*=============================*/
{
    volatile uint16_t BITFLD_P2_SET                           : 10;
};
#define P2_SET                               (0x03FF)



#define P2_RESET_DATA_REG                    (0x50003044) /* P2 Reset port pins register */
#define P2_RESET_DATA_REG_RESET              (0x00000000)

/*=============================*/
struct __P2_RESET_DATA_REG
/*=============================*/
{
    volatile uint16_t BITFLD_P2_RESET                         : 10;
};
#define P2_RESET                             (0x03FF)



#define P20_MODE_REG                         (0x50003046) /* P20 Mode Register */
#define P20_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P20_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P21_MODE_REG                         (0x50003048) /* P21 Mode Register */
#define P21_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P21_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P22_MODE_REG                         (0x5000304A) /* P22 Mode Register */
#define P22_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P22_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P23_MODE_REG                         (0x5000304C) /* P23 Mode Register */
#define P23_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P23_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P24_MODE_REG                         (0x5000304E) /* P24 Mode Register */
#define P24_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P24_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P25_MODE_REG                         (0x50003050) /* P25 Mode Register */
#define P25_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P25_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P26_MODE_REG                         (0x50003052) /* P26 Mode Register */
#define P26_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P26_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P27_MODE_REG                         (0x50003054) /* P27 Mode Register */
#define P27_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P27_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P28_MODE_REG                         (0x50003056) /* P28 Mode Register */
#define P28_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P28_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P29_MODE_REG                         (0x50003058) /* P29 Mode Register */
#define P29_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P29_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P01_PADPWR_CTRL_REG                  (0x50003070) /* Ports 0 and 1 Output Power Control Register */
#define P01_PADPWR_CTRL_REG_RESET            (0x00000000)

/*=============================*/
struct __P01_PADPWR_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_P0_OUT_CTRL                      : 8;
    volatile uint16_t BITFLD_P1_OUT_CTRL                      : 6;
};
#define P0_OUT_CTRL                          (0x00FF)
#define P1_OUT_CTRL                          (0x3F00)



#define P2_PADPWR_CTRL_REG                   (0x50003072) /* Port 2 Output Power Control Register */
#define P2_PADPWR_CTRL_REG_RESET             (0x00000000)

/*=============================*/
struct __P2_PADPWR_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_P2_OUT_CTRL                      : 10;
};
#define P2_OUT_CTRL                          (0x03FF)



#define P3_PADPWR_CTRL_REG                   (0x50003074) /* Port 3 Output Power Control Register */
#define P3_PADPWR_CTRL_REG_RESET             (0x00000000)

/*=============================*/
struct __P3_PADPWR_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_P3_OUT_CTRL                      : 8;
};
#define P3_OUT_CTRL                          (0x00FF)



#define P3_DATA_REG                          (0x50003080) /* P3 Data input / output register */
#define P3_DATA_REG_RESET                    (0x00000000)

/*=============================*/
struct __P3_DATA_REG
/*=============================*/
{
    volatile uint16_t BITFLD_P3_DATA                          : 8;
};
#define P3_DATA                              (0x00FF)



#define P3_SET_DATA_REG                      (0x50003082) /* P3 Set port pins register */
#define P3_SET_DATA_REG_RESET                (0x00000000)

/*=============================*/
struct __P3_SET_DATA_REG
/*=============================*/
{
    volatile uint16_t BITFLD_P3_SET                           : 8;
};
#define P3_SET                               (0x00FF)



#define P3_RESET_DATA_REG                    (0x50003084) /* P3 Reset port pins register */
#define P3_RESET_DATA_REG_RESET              (0x00000000)

/*=============================*/
struct __P3_RESET_DATA_REG
/*=============================*/
{
    volatile uint16_t BITFLD_P3_RESET                         : 8;
};
#define P3_RESET                             (0x00FF)



#define P30_MODE_REG                         (0x50003086) /* P30 Mode Register */
#define P30_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P30_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P31_MODE_REG                         (0x50003088) /* P31 Mode Register */
#define P31_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P31_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P32_MODE_REG                         (0x5000308A) /* P32 Mode Register */
#define P32_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P32_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P33_MODE_REG                         (0x5000308C) /* P33 Mode Register */
#define P33_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P33_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P34_MODE_REG                         (0x5000308E) /* P34 Mode Register */
#define P34_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P34_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P35_MODE_REG                         (0x50003090) /* P35 Mode Register */
#define P35_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P35_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P36_MODE_REG                         (0x50003092) /* P36 Mode Register */
#define P36_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P36_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



#define P37_MODE_REG                         (0x50003094) /* P37 Mode Register */
#define P37_MODE_REG_RESET                   (0x00000200)

/*=============================*/
struct __P37_MODE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PID                              : 5;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PUPD                             : 2;
};
#define PID                                  (0x001F)
#define PUPD                                 (0x0300)



/*===========================================================================*/
/* memory map GPREG */
/*===========================================================================*/



#define SET_FREEZE_REG                       (0x50003300) /* Controls freezing of various timers/counters. */
#define SET_FREEZE_REG_RESET                 (0x00000000)

/*=============================*/
struct __SET_FREEZE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_FRZ_WKUPTIM                      : 1;
    volatile uint16_t BITFLD_FRZ_SWTIM                        : 1;
    volatile uint16_t BITFLD_FRZ_BLETIM                       : 1;
    volatile uint16_t BITFLD_FRZ_WDOG                         : 1;
    volatile uint16_t BITFLD_FRZ_DMA                          : 1;
};
#define FRZ_WKUPTIM                          (0x0001)
#define FRZ_SWTIM                            (0x0002)
#define FRZ_BLETIM                           (0x0004)
#define FRZ_WDOG                             (0x0008)
#define FRZ_DMA                              (0x0010)



#define RESET_FREEZE_REG                     (0x50003302) /* Controls unfreezing of various timers/counters. */
#define RESET_FREEZE_REG_RESET               (0x00000000)

/*=============================*/
struct __RESET_FREEZE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_FRZ_WKUPTIM                      : 1;
    volatile uint16_t BITFLD_FRZ_SWTIM                        : 1;
    volatile uint16_t BITFLD_FRZ_BLETIM                       : 1;
    volatile uint16_t BITFLD_FRZ_WDOG                         : 1;
    volatile uint16_t BITFLD_FRZ_DMA                          : 1;
};
#define FRZ_WKUPTIM                          (0x0001)
#define FRZ_SWTIM                            (0x0002)
#define FRZ_BLETIM                           (0x0004)
#define FRZ_WDOG                             (0x0008)
#define FRZ_DMA                              (0x0010)



#define DEBUG_REG                            (0x50003304) /* Various debug information register. */
#define DEBUG_REG_RESET                      (0x00000001)

/*=============================*/
struct __DEBUG_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DEBUGS_FREEZE_EN                 : 1;
};
#define DEBUGS_FREEZE_EN                     (0x0001)



#define GP_STATUS_REG                        (0x50003306) /* General purpose system status register. */
#define GP_STATUS_REG_RESET                  (0x00000000)

/*=============================*/
struct __GP_STATUS_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CAL_PHASE                        : 1;
    volatile uint16_t                                         : 1;
};
#define CAL_PHASE                            (0x0001)



#define GP_CONTROL_REG                       (0x50003308) /* General purpose system control register. */
#define GP_CONTROL_REG_RESET                 (0x00000000)

/*=============================*/
struct __GP_CONTROL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_BLE_WAKEUP_REQ                   : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_BLE_WAKEUP_LP_IRQ                : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_CPU_DMA_BUS_PRIO                 : 1;
};
#define BLE_WAKEUP_REQ                       (0x0001)
#define BLE_WAKEUP_LP_IRQ                    (0x0004)
#define CPU_DMA_BUS_PRIO                     (0x0010)



#define BLE_FINECNT_SAMP_REG                 (0x5000330A) /* BLE FINECNT sampled value while in deep sleep state. */
#define BLE_FINECNT_SAMP_REG_RESET           (0x00000000)

/*=============================*/
struct __BLE_FINECNT_SAMP_REG
/*=============================*/
{
    volatile uint16_t BITFLD_BLE_FINECNT_SAMP                 : 10;
};
#define BLE_FINECNT_SAMP                     (0x03FF)



/*===========================================================================*/
/* memory map I2C */
/*===========================================================================*/



#define I2C_CON_REG                          (0x50001300) /* I2C Control Register */
#define I2C_CON_REG_RESET                    (0x0000007D)

/*=============================*/
struct __I2C_CON_REG
/*=============================*/
{
    volatile uint16_t BITFLD_I2C_MASTER_MODE                  : 1;
    volatile uint16_t BITFLD_I2C_SPEED                        : 2;
    volatile uint16_t BITFLD_I2C_10BITADDR_SLAVE              : 1;
    volatile uint16_t BITFLD_I2C_10BITADDR_MASTER             : 1;
    volatile uint16_t BITFLD_I2C_RESTART_EN                   : 1;
    volatile uint16_t BITFLD_I2C_SLAVE_DISABLE                : 1;
};
#define I2C_MASTER_MODE                      (0x0001)
#define I2C_SPEED                            (0x0006)
#define I2C_10BITADDR_SLAVE                  (0x0008)
#define I2C_10BITADDR_MASTER                 (0x0010)
#define I2C_RESTART_EN                       (0x0020)
#define I2C_SLAVE_DISABLE                    (0x0040)



#define I2C_TAR_REG                          (0x50001304) /* I2C Target Address Register */
#define I2C_TAR_REG_RESET                    (0x00000055)

/*=============================*/
struct __I2C_TAR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_IC_TAR                           : 10;
    volatile uint16_t BITFLD_GC_OR_START                      : 1;
    volatile uint16_t BITFLD_SPECIAL                          : 1;
};
#define IC_TAR                               (0x03FF)
#define GC_OR_START                          (0x0400)
#define SPECIAL                              (0x0800)



#define I2C_SAR_REG                          (0x50001308) /* I2C Slave Address Register */
#define I2C_SAR_REG_RESET                    (0x00000055)

/*=============================*/
struct __I2C_SAR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_IC_SAR                           : 10;
};
#define IC_SAR                               (0x03FF)



#define I2C_DATA_CMD_REG                     (0x50001310) /* I2C Rx/Tx Data Buffer and Command Register */
#define I2C_DATA_CMD_REG_RESET               (0x00000000)

/*=============================*/
struct __I2C_DATA_CMD_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DAT                              : 8;
    volatile uint16_t BITFLD_CMD                              : 1;
};
#define I2C_DAT                                  (0x00FF)
#define I2C_CMD                                  (0x0100)



#define I2C_SS_SCL_HCNT_REG                  (0x50001314) /* Standard Speed I2C Clock SCL High Count Register */
#define I2C_SS_SCL_HCNT_REG_RESET            (0x00000048)

/*=============================*/
struct __I2C_SS_SCL_HCNT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_IC_SS_SCL_HCNT                   : 16;
};
#define IC_SS_SCL_HCNT                       (0xFFFF)



#define I2C_SS_SCL_LCNT_REG                  (0x50001318) /* Standard Speed I2C Clock SCL Low Count Register */
#define I2C_SS_SCL_LCNT_REG_RESET            (0x0000004F)

/*=============================*/
struct __I2C_SS_SCL_LCNT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_IC_SS_SCL_LCNT                   : 16;
};
#define IC_SS_SCL_LCNT                       (0xFFFF)



#define I2C_FS_SCL_HCNT_REG                  (0x5000131C) /* Fast Speed I2C Clock SCL High Count Register */
#define I2C_FS_SCL_HCNT_REG_RESET            (0x00000008)

/*=============================*/
struct __I2C_FS_SCL_HCNT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_IC_FS_SCL_HCNT                   : 16;
};
#define IC_FS_SCL_HCNT                       (0xFFFF)



#define I2C_FS_SCL_LCNT_REG                  (0x50001320) /* Fast Speed I2C Clock SCL Low Count Register */
#define I2C_FS_SCL_LCNT_REG_RESET            (0x00000017)

/*=============================*/
struct __I2C_FS_SCL_LCNT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_IC_FS_SCL_LCNT                   : 16;
};
#define IC_FS_SCL_LCNT                       (0xFFFF)



#define I2C_INTR_STAT_REG                    (0x5000132C) /* I2C Interrupt Status Register */
#define I2C_INTR_STAT_REG_RESET              (0x00000000)

/*=============================*/
struct __I2C_INTR_STAT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_R_RX_UNDER                       : 1;
    volatile uint16_t BITFLD_R_RX_OVER                        : 1;
    volatile uint16_t BITFLD_R_RX_FULL                        : 1;
    volatile uint16_t BITFLD_R_TX_OVER                        : 1;
    volatile uint16_t BITFLD_R_TX_EMPTY                       : 1;
    volatile uint16_t BITFLD_R_RD_REQ                         : 1;
    volatile uint16_t BITFLD_R_TX_ABRT                        : 1;
    volatile uint16_t BITFLD_R_RX_DONE                        : 1;
    volatile uint16_t BITFLD_R_ACTIVITY                       : 1;
    volatile uint16_t BITFLD_R_STOP_DET                       : 1;
    volatile uint16_t BITFLD_R_START_DET                      : 1;
    volatile uint16_t BITFLD_R_GEN_CALL                       : 1;
};
#define R_RX_UNDER                           (0x0001)
#define R_RX_OVER                            (0x0002)
#define R_RX_FULL                            (0x0004)
#define R_TX_OVER                            (0x0008)
#define R_TX_EMPTY                           (0x0010)
#define R_RD_REQ                             (0x0020)
#define R_TX_ABRT                            (0x0040)
#define R_RX_DONE                            (0x0080)
#define R_ACTIVITY                           (0x0100)
#define R_STOP_DET                           (0x0200)
#define R_START_DET                          (0x0400)
#define R_GEN_CALL                           (0x0800)



#define I2C_INTR_MASK_REG                    (0x50001330) /* I2C Interrupt Mask Register */
#define I2C_INTR_MASK_REG_RESET              (0x000008FF)

/*=============================*/
struct __I2C_INTR_MASK_REG
/*=============================*/
{
    volatile uint16_t BITFLD_M_RX_UNDER                       : 1;
    volatile uint16_t BITFLD_M_RX_OVER                        : 1;
    volatile uint16_t BITFLD_M_RX_FULL                        : 1;
    volatile uint16_t BITFLD_M_TX_OVER                        : 1;
    volatile uint16_t BITFLD_M_TX_EMPTY                       : 1;
    volatile uint16_t BITFLD_M_RD_REQ                         : 1;
    volatile uint16_t BITFLD_M_TX_ABRT                        : 1;
    volatile uint16_t BITFLD_M_RX_DONE                        : 1;
    volatile uint16_t BITFLD_M_ACTIVITY                       : 1;
    volatile uint16_t BITFLD_M_STOP_DET                       : 1;
    volatile uint16_t BITFLD_M_START_DET                      : 1;
    volatile uint16_t BITFLD_M_GEN_CALL                       : 1;
};
#define M_RX_UNDER                           (0x0001)
#define M_RX_OVER                            (0x0002)
#define M_RX_FULL                            (0x0004)
#define M_TX_OVER                            (0x0008)
#define M_TX_EMPTY                           (0x0010)
#define M_RD_REQ                             (0x0020)
#define M_TX_ABRT                            (0x0040)
#define M_RX_DONE                            (0x0080)
#define M_ACTIVITY                           (0x0100)
#define M_STOP_DET                           (0x0200)
#define M_START_DET                          (0x0400)
#define M_GEN_CALL                           (0x0800)



#define I2C_RAW_INTR_STAT_REG                (0x50001334) /* I2C Raw Interrupt Status Register */
#define I2C_RAW_INTR_STAT_REG_RESET          (0x00000000)

/*=============================*/
struct __I2C_RAW_INTR_STAT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RX_UNDER                         : 1;
    volatile uint16_t BITFLD_RX_OVER                          : 1;
    volatile uint16_t BITFLD_RX_FULL                          : 1;
    volatile uint16_t BITFLD_TX_OVER                          : 1;
    volatile uint16_t BITFLD_TX_EMPTY                         : 1;
    volatile uint16_t BITFLD_RD_REQ                           : 1;
    volatile uint16_t BITFLD_TX_ABRT                          : 1;
    volatile uint16_t BITFLD_RX_DONE                          : 1;
    volatile uint16_t BITFLD_ACTIVITY                         : 1;
    volatile uint16_t BITFLD_STOP_DET                         : 1;
    volatile uint16_t BITFLD_START_DET                        : 1;
    volatile uint16_t BITFLD_GEN_CALL                         : 1;
};
#define RX_UNDER                             (0x0001)
#define RX_OVER                              (0x0002)
#define RX_FULL                              (0x0004)
#define TX_OVER                              (0x0008)
#define TX_EMPTY                             (0x0010)
#define RD_REQ                               (0x0020)
#define TX_ABRT                              (0x0040)
#define RX_DONE                              (0x0080)
#define ACTIVITY                             (0x0100)
#define STOP_DET                             (0x0200)
#define START_DET                            (0x0400)
#define GEN_CALL                             (0x0800)



#define I2C_RX_TL_REG                        (0x50001338) /* I2C Receive FIFO Threshold Register */
#define I2C_RX_TL_REG_RESET                  (0x00000000)

/*=============================*/
struct __I2C_RX_TL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RX_TL                            : 5;
};
#define RX_TL                                (0x001F)



#define I2C_TX_TL_REG                        (0x5000133C) /* I2C Transmit FIFO Threshold Register */
#define I2C_TX_TL_REG_RESET                  (0x00000000)

/*=============================*/
struct __I2C_TX_TL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RX_TL                            : 5;
};
#define RX_TL                                (0x001F)



#define I2C_CLR_INTR_REG                     (0x50001340) /* Clear Combined and Individual Interrupt Register */
#define I2C_CLR_INTR_REG_RESET               (0x00000000)

/*=============================*/
struct __I2C_CLR_INTR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CLR_INTR                         : 1;
};
#define CLR_INTR                             (0x0001)



#define I2C_CLR_RX_UNDER_REG                 (0x50001344) /* Clear RX_UNDER Interrupt Register */
#define I2C_CLR_RX_UNDER_REG_RESET           (0x00000000)

/*=============================*/
struct __I2C_CLR_RX_UNDER_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CLR_RX_UNDER                     : 1;
};
#define CLR_RX_UNDER                         (0x0001)



#define I2C_CLR_RX_OVER_REG                  (0x50001348) /* Clear RX_OVER Interrupt Register */
#define I2C_CLR_RX_OVER_REG_RESET            (0x00000000)

/*=============================*/
struct __I2C_CLR_RX_OVER_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CLR_RX_OVER                      : 1;
};
#define CLR_RX_OVER                          (0x0001)



#define I2C_CLR_TX_OVER_REG                  (0x5000134C) /* Clear TX_OVER Interrupt Register */
#define I2C_CLR_TX_OVER_REG_RESET            (0x00000000)

/*=============================*/
struct __I2C_CLR_TX_OVER_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CLR_TX_OVER                      : 1;
};
#define CLR_TX_OVER                          (0x0001)



#define I2C_CLR_RD_REQ_REG                   (0x50001350) /* Clear RD_REQ Interrupt Register */
#define I2C_CLR_RD_REQ_REG_RESET             (0x00000000)

/*=============================*/
struct __I2C_CLR_RD_REQ_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CLR_RD_REQ                       : 1;
};
#define CLR_RD_REQ                           (0x0001)



#define I2C_CLR_TX_ABRT_REG                  (0x50001354) /* Clear TX_ABRT Interrupt Register */
#define I2C_CLR_TX_ABRT_REG_RESET            (0x00000000)

/*=============================*/
struct __I2C_CLR_TX_ABRT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CLR_TX_ABRT                      : 1;
};
#define CLR_TX_ABRT                          (0x0001)



#define I2C_CLR_RX_DONE_REG                  (0x50001358) /* Clear RX_DONE Interrupt Register */
#define I2C_CLR_RX_DONE_REG_RESET            (0x00000000)

/*=============================*/
struct __I2C_CLR_RX_DONE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CLR_RX_DONE                      : 1;
};
#define CLR_RX_DONE                          (0x0001)



#define I2C_CLR_ACTIVITY_REG                 (0x5000135C) /* Clear ACTIVITY Interrupt Register */
#define I2C_CLR_ACTIVITY_REG_RESET           (0x00000000)

/*=============================*/
struct __I2C_CLR_ACTIVITY_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CLR_ACTIVITY                     : 1;
};
#define CLR_ACTIVITY                         (0x0001)



#define I2C_CLR_STOP_DET_REG                 (0x50001360) /* Clear STOP_DET Interrupt Register */
#define I2C_CLR_STOP_DET_REG_RESET           (0x00000000)

/*=============================*/
struct __I2C_CLR_STOP_DET_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CLR_ACTIVITY                     : 1;
};
#define CLR_ACTIVITY                         (0x0001)



#define I2C_CLR_START_DET_REG                (0x50001364) /* Clear START_DET Interrupt Register */
#define I2C_CLR_START_DET_REG_RESET          (0x00000000)

/*=============================*/
struct __I2C_CLR_START_DET_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CLR_START_DET                    : 1;
};
#define CLR_START_DET                        (0x0001)



#define I2C_CLR_GEN_CALL_REG                 (0x50001368) /* Clear GEN_CALL Interrupt Register */
#define I2C_CLR_GEN_CALL_REG_RESET           (0x00000000)

/*=============================*/
struct __I2C_CLR_GEN_CALL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CLR_GEN_CALL                     : 1;
};
#define CLR_GEN_CALL                         (0x0001)



#define I2C_ENABLE_REG                       (0x5000136C) /* I2C Enable Register */
#define I2C_ENABLE_REG_RESET                 (0x00000000)

/*=============================*/
struct __I2C_ENABLE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CTRL_ENABLE                      : 1;
};
#define CTRL_ENABLE                          (0x0001)



#define I2C_STATUS_REG                       (0x50001370) /* I2C Status Register */
#define I2C_STATUS_REG_RESET                 (0x00000006)

/*=============================*/
struct __I2C_STATUS_REG
/*=============================*/
{
    volatile uint16_t BITFLD_I2C_ACTIVITY                     : 1;
    volatile uint16_t BITFLD_TFNF                             : 1;
    volatile uint16_t BITFLD_TFE                              : 1;
    volatile uint16_t BITFLD_RFNE                             : 1;
    volatile uint16_t BITFLD_RFF                              : 1;
    volatile uint16_t BITFLD_MST_ACTIVITY                     : 1;
    volatile uint16_t BITFLD_SLV_ACTIVITY                     : 1;
};
#define I2C_ACTIVITY                         (0x0001)
#define TFNF                                 (0x0002)
#define TFE                                  (0x0004)
#define RFNE                                 (0x0008)
#define RFF                                  (0x0010)
#define MST_ACTIVITY                         (0x0020)
#define SLV_ACTIVITY                         (0x0040)



#define I2C_TXFLR_REG                        (0x50001374) /* I2C Transmit FIFO Level Register */
#define I2C_TXFLR_REG_RESET                  (0x00000000)

/*=============================*/
struct __I2C_TXFLR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_TXFLR                            : 6;
};
#define TXFLR                                (0x003F)



#define I2C_RXFLR_REG                        (0x50001378) /* I2C Receive FIFO Level Register */
#define I2C_RXFLR_REG_RESET                  (0x00000000)

/*=============================*/
struct __I2C_RXFLR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RXFLR                            : 6;
};
#define RXFLR                                (0x003F)



#define I2C_SDA_HOLD_REG                     (0x5000137C) /* I2C SDA Hold Time Length Register */
#define I2C_SDA_HOLD_REG_RESET               (0x00000001)

/*=============================*/
struct __I2C_SDA_HOLD_REG
/*=============================*/
{
    volatile uint16_t BITFLD_IC_SDA_HOLD                      : 16;
};
#define IC_SDA_HOLD                          (0xFFFF)



#define I2C_TX_ABRT_SOURCE_REG               (0x50001380) /* I2C Transmit Abort Source Register */
#define I2C_TX_ABRT_SOURCE_REG_RESET         (0x00000000)

/*=============================*/
struct __I2C_TX_ABRT_SOURCE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_ABRT_7B_ADDR_NOACK               : 1;
    volatile uint16_t BITFLD_ABRT_10ADDR1_NOACK               : 1;
    volatile uint16_t BITFLD_ABRT_10ADDR2_NOACK               : 1;
    volatile uint16_t BITFLD_ABRT_TXDATA_NOACK                : 1;
    volatile uint16_t BITFLD_ABRT_GCALL_NOACK                 : 1;
    volatile uint16_t BITFLD_ABRT_GCALL_READ                  : 1;
    volatile uint16_t BITFLD_ABRT_HS_ACKDET                   : 1;
    volatile uint16_t BITFLD_ABRT_SBYTE_ACKDET                : 1;
    volatile uint16_t BITFLD_ABRT_HS_NORSTRT                  : 1;
    volatile uint16_t BITFLD_ABRT_SBYTE_NORSTRT               : 1;
    volatile uint16_t BITFLD_ABRT_10B_RD_NORSTRT              : 1;
    volatile uint16_t BITFLD_ABRT_MASTER_DIS                  : 1;
    volatile uint16_t BITFLD_ARB_LOST                         : 1;
    volatile uint16_t BITFLD_ABRT_SLVFLUSH_TXFIFO             : 1;
    volatile uint16_t BITFLD_ABRT_SLV_ARBLOST                 : 1;
    volatile uint16_t BITFLD_ABRT_SLVRD_INTX                  : 1;
};
#define ABRT_7B_ADDR_NOACK                   (0x0001)
#define ABRT_10ADDR1_NOACK                   (0x0002)
#define ABRT_10ADDR2_NOACK                   (0x0004)
#define ABRT_TXDATA_NOACK                    (0x0008)
#define ABRT_GCALL_NOACK                     (0x0010)
#define ABRT_GCALL_READ                      (0x0020)
#define ABRT_HS_ACKDET                       (0x0040)
#define ABRT_SBYTE_ACKDET                    (0x0080)
#define ABRT_HS_NORSTRT                      (0x0100)
#define ABRT_SBYTE_NORSTRT                   (0x0200)
#define ABRT_10B_RD_NORSTRT                  (0x0400)
#define ABRT_MASTER_DIS                      (0x0800)
#define ARB_LOST                             (0x1000)
#define ABRT_SLVFLUSH_TXFIFO                 (0x2000)
#define ABRT_SLV_ARBLOST                     (0x4000)
#define ABRT_SLVRD_INTX                      (0x8000)



#define I2C_DMA_CR_REG                       (0x50001388) /* DMA Control Register */
#define I2C_DMA_CR_REG_RESET                 (0x00000000)

/*=============================*/
struct __I2C_DMA_CR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RDMAE                            : 1;
    volatile uint16_t BITFLD_TDMAE                            : 1;
};
#define RDMAE                                (0x0001)
#define TDMAE                                (0x0002)



#define I2C_DMA_TDLR_REG                     (0x5000138C) /* DMA Transmit Data Level Register */
#define I2C_DMA_TDLR_REG_RESET               (0x00000000)

/*=============================*/
struct __I2C_DMA_TDLR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMATDL                           : 5;
};
#define DMATDL                               (0x001F)



#define I2C_DMA_RDLR_REG                     (0x50001390) /* I2C Receive Data Level Register */
#define I2C_DMA_RDLR_REG_RESET               (0x00000000)

/*=============================*/
struct __I2C_DMA_RDLR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMARDL                           : 5;
};
#define DMARDL                               (0x001F)



#define I2C_SDA_SETUP_REG                    (0x50001394) /* I2C SDA Setup Register */
#define I2C_SDA_SETUP_REG_RESET              (0x00000064)

/*=============================*/
struct __I2C_SDA_SETUP_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SDA_SETUP                        : 8;
};
#define SDA_SETUP                            (0x00FF)



#define I2C_ACK_GENERAL_CALL_REG             (0x50001398) /* I2C ACK General Call Register */
#define I2C_ACK_GENERAL_CALL_REG_RESET       (0x00000000)

/*=============================*/
struct __I2C_ACK_GENERAL_CALL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_ACK_GEN_CALL                     : 1;
};
#define ACK_GEN_CALL                         (0x0001)



#define I2C_ENABLE_STATUS_REG                (0x5000139C) /* I2C Enable Status Register */
#define I2C_ENABLE_STATUS_REG_RESET          (0x00000000)

/*=============================*/
struct __I2C_ENABLE_STATUS_REG
/*=============================*/
{
    volatile uint16_t BITFLD_IC_EN                            : 1;
    volatile uint16_t BITFLD_SLV_DISABLED_WHILE_BUSY          : 1;
    volatile uint16_t BITFLD_SLV_RX_DATA_LOST                 : 1;
};
#define IC_EN                                (0x0001)
#define SLV_DISABLED_WHILE_BUSY              (0x0002)
#define SLV_RX_DATA_LOST                     (0x0004)



#define I2C_IC_FS_SPKLEN_REG                 (0x500013A0) /* I2C SS and FS spike suppression limit Size */
#define I2C_IC_FS_SPKLEN_REG_RESET           (0x00000001)

/*=============================*/
struct __I2C_IC_FS_SPKLEN_REG
/*=============================*/
{
    volatile uint16_t BITFLD_IC_FS_SPKLEN                     : 8;
};
#define IC_FS_SPKLEN                         (0x00FF)



#define I2C_COMP_PARAM1_REG                  (0x500013F4) /* Component Parameter Register */
#define I2C_COMP_PARAM1_REG_RESET            (0x00000000)

/*=============================*/
struct __I2C_COMP_PARAM1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_IC_COMP_PARAM1                   : 16;
};
#define IC_COMP_PARAM1                       (0xFFFF)



#define I2C_COMP_PARAM2_REG                  (0x500013F6) /* Component Parameter Register 2 */
#define I2C_COMP_PARAM2_REG_RESET            (0x00000000)

/*=============================*/
struct __I2C_COMP_PARAM2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_IC_COMP_PARAM2                   : 16;
};
#define IC_COMP_PARAM2                       (0xFFFF)



#define I2C_COMP_VERSION_REG                 (0x500013F8) /* I2C Component Version Register */
#define I2C_COMP_VERSION_REG_RESET           (0x0000302A)

/*=============================*/
struct __I2C_COMP_VERSION_REG
/*=============================*/
{
    volatile uint16_t BITFLD_IC_COMP_VERSION                  : 16;
};
#define IC_COMP_VERSION                      (0xFFFF)



#define I2C_COMP2_VERSION                    (0x500013FA) /* I2C Component2 Version Register */
#define I2C_COMP2_VERSION_RESET              (0x00003132)

/*=============================*/
struct __I2C_COMP2_VERSION
/*=============================*/
{
    volatile uint16_t BITFLD_IC_COMP2_VERSION                 : 16;
};
#define IC_COMP2_VERSION                     (0xFFFF)



#define I2C_COMP_TYPE_REG                    (0x500013FC) /* I2C Component Type Register */
#define I2C_COMP_TYPE_REG_RESET              (0x00000140)

/*=============================*/
struct __I2C_COMP_TYPE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_IC_COMP_TYPE                     : 16;
};
#define IC_COMP_TYPE                         (0xFFFF)



#define I2C_COMP_TYPE2_REG                   (0x500013FE) /* I2C Component2 Type Register */
#define I2C_COMP_TYPE2_REG_RESET             (0x00004457)

/*=============================*/
struct __I2C_COMP_TYPE2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_IC_COMP2_TYPE                    : 16;
};
#define IC_COMP2_TYPE                        (0xFFFF)



/*===========================================================================*/
/* memory map KBRD */
/*===========================================================================*/



#define GPIO_IRQ0_IN_SEL_REG                 (0x50001400) /* GPIO interrupt selection for GPIO_IRQ0 */
#define GPIO_IRQ0_IN_SEL_REG_RESET           (0x00000000)

/*=============================*/
struct __GPIO_IRQ0_IN_SEL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_KBRD_IRQ0_SEL                    : 6;
};
#define KBRD_IRQ0_SEL                        (0x003F)



#define GPIO_IRQ1_IN_SEL_REG                 (0x50001402) /* GPIO interrupt selection for GPIO_IRQ1 */
#define GPIO_IRQ1_IN_SEL_REG_RESET           (0x00000000)

/*=============================*/
struct __GPIO_IRQ1_IN_SEL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_KBRD_IRQ1_SEL                    : 6;
};
#define KBRD_IRQ1_SEL                        (0x003F)



#define GPIO_IRQ2_IN_SEL_REG                 (0x50001404) /* GPIO interrupt selection for GPIO_IRQ2 */
#define GPIO_IRQ2_IN_SEL_REG_RESET           (0x00000000)

/*=============================*/
struct __GPIO_IRQ2_IN_SEL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_KBRD_IRQ2_SEL                    : 6;
};
#define KBRD_IRQ2_SEL                        (0x003F)



#define GPIO_IRQ3_IN_SEL_REG                 (0x50001406) /* GPIO interrupt selection for GPIO_IRQ3 */
#define GPIO_IRQ3_IN_SEL_REG_RESET           (0x00000000)

/*=============================*/
struct __GPIO_IRQ3_IN_SEL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_KBRD_IRQ3_SEL                    : 6;
};
#define KBRD_IRQ3_SEL                        (0x003F)



#define GPIO_IRQ4_IN_SEL_REG                 (0x50001408) /* GPIO interrupt selection for GPIO_IRQ4 */
#define GPIO_IRQ4_IN_SEL_REG_RESET           (0x00000000)

/*=============================*/
struct __GPIO_IRQ4_IN_SEL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_KBRD_IRQ4_SEL                    : 6;
};
#define KBRD_IRQ4_SEL                        (0x003F)



#define GPIO_DEBOUNCE_REG                    (0x5000140C) /* debounce counter value for GPIO inputs */
#define GPIO_DEBOUNCE_REG_RESET              (0x00000000)

/*=============================*/
struct __GPIO_DEBOUNCE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DEB_VALUE                        : 6;
    volatile uint16_t                                         : 2;
    volatile uint16_t BITFLD_DEB_ENABLE0                      : 1;
    volatile uint16_t BITFLD_DEB_ENABLE1                      : 1;
    volatile uint16_t BITFLD_DEB_ENABLE2                      : 1;
    volatile uint16_t BITFLD_DEB_ENABLE3                      : 1;
    volatile uint16_t BITFLD_DEB_ENABLE4                      : 1;
    volatile uint16_t BITFLD_DEB_ENABLE_KBRD                  : 1;
};
#define DEB_VALUE                            (0x003F)
#define DEB_ENABLE0                          (0x0100)
#define DEB_ENABLE1                          (0x0200)
#define DEB_ENABLE2                          (0x0400)
#define DEB_ENABLE3                          (0x0800)
#define DEB_ENABLE4                          (0x1000)
#define DEB_ENABLE_KBRD                      (0x2000)



#define GPIO_RESET_IRQ_REG                   (0x5000140E) /* GPIO interrupt reset register */
#define GPIO_RESET_IRQ_REG_RESET             (0x00000000)

/*=============================*/
struct __GPIO_RESET_IRQ_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RESET_GPIO0_IRQ                  : 1;
    volatile uint16_t BITFLD_RESET_GPIO1_IRQ                  : 1;
    volatile uint16_t BITFLD_RESET_GPIO2_IRQ                  : 1;
    volatile uint16_t BITFLD_RESET_GPIO3_IRQ                  : 1;
    volatile uint16_t BITFLD_RESET_GPIO4_IRQ                  : 1;
    volatile uint16_t BITFLD_RESET_KBRD_IRQ                   : 1;
};
#define RESET_GPIO0_IRQ                      (0x0001)
#define RESET_GPIO1_IRQ                      (0x0002)
#define RESET_GPIO2_IRQ                      (0x0004)
#define RESET_GPIO3_IRQ                      (0x0008)
#define RESET_GPIO4_IRQ                      (0x0010)
#define RESET_KBRD_IRQ                       (0x0020)



#define GPIO_INT_LEVEL_CTRL_REG              (0x50001410) /* high or low level select for GPIO interrupts */
#define GPIO_INT_LEVEL_CTRL_REG_RESET        (0x00000000)

/*=============================*/
struct __GPIO_INT_LEVEL_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_INPUT_LEVEL0                     : 1;
    volatile uint16_t BITFLD_INPUT_LEVEL1                     : 1;
    volatile uint16_t BITFLD_INPUT_LEVEL2                     : 1;
    volatile uint16_t BITFLD_INPUT_LEVEL3                     : 1;
    volatile uint16_t BITFLD_INPUT_LEVEL4                     : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t                                         : 2;
    volatile uint16_t BITFLD_EDGE_LEVELn0                     : 1;
    volatile uint16_t BITFLD_EDGE_LEVELn1                     : 1;
    volatile uint16_t BITFLD_EDGE_LEVELn2                     : 1;
    volatile uint16_t BITFLD_EDGE_LEVELn3                     : 1;
    volatile uint16_t BITFLD_EDGE_LEVELn4                     : 1;
    volatile uint16_t                                         : 1;
};
#define INPUT_LEVEL0                         (0x0001)
#define INPUT_LEVEL1                         (0x0002)
#define INPUT_LEVEL2                         (0x0004)
#define INPUT_LEVEL3                         (0x0008)
#define INPUT_LEVEL4                         (0x0010)
#define EDGE_LEVELn0                         (0x0100)
#define EDGE_LEVELn1                         (0x0200)
#define EDGE_LEVELn2                         (0x0400)
#define EDGE_LEVELn3                         (0x0800)
#define EDGE_LEVELn4                         (0x1000)



#define KBRD_IRQ_IN_SEL0_REG                 (0x50001412) /* GPIO interrupt selection for KBRD_IRQ for P0 */
#define KBRD_IRQ_IN_SEL0_REG_RESET           (0x00000000)

/*=============================*/
struct __KBRD_IRQ_IN_SEL0_REG
/*=============================*/
{
    volatile uint16_t BITFLD_KBRD_P00_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P01_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P02_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P03_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P04_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P05_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P06_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P07_EN                      : 1;
    volatile uint16_t BITFLD_KEY_REPEAT                       : 6;
    volatile uint16_t BITFLD_KBRD_LEVEL                       : 1;
    volatile uint16_t BITFLD_KBRD_REL                         : 1;
};
#define KBRD_P00_EN                          (0x0001)
#define KBRD_P01_EN                          (0x0002)
#define KBRD_P02_EN                          (0x0004)
#define KBRD_P03_EN                          (0x0008)
#define KBRD_P04_EN                          (0x0010)
#define KBRD_P05_EN                          (0x0020)
#define KBRD_P06_EN                          (0x0040)
#define KBRD_P07_EN                          (0x0080)
#define KEY_REPEAT                           (0x3F00)
#define KBRD_LEVEL                           (0x4000)
#define KBRD_REL                             (0x8000)



#define KBRD_IRQ_IN_SEL1_REG                 (0x50001414) /* GPIO interrupt selection for KBRD_IRQ for P1 and P2 */
#define KBRD_IRQ_IN_SEL1_REG_RESET           (0x00000000)

/*=============================*/
struct __KBRD_IRQ_IN_SEL1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_KBRD_P20_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P21_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P22_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P23_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P24_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P25_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P26_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P27_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P28_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P29_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P10_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P11_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P12_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P13_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P14_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P15_EN                      : 1;
};
#define KBRD_P20_EN                          (0x0001)
#define KBRD_P21_EN                          (0x0002)
#define KBRD_P22_EN                          (0x0004)
#define KBRD_P23_EN                          (0x0008)
#define KBRD_P24_EN                          (0x0010)
#define KBRD_P25_EN                          (0x0020)
#define KBRD_P26_EN                          (0x0040)
#define KBRD_P27_EN                          (0x0080)
#define KBRD_P28_EN                          (0x0100)
#define KBRD_P29_EN                          (0x0200)
#define KBRD_P10_EN                          (0x0400)
#define KBRD_P11_EN                          (0x0800)
#define KBRD_P12_EN                          (0x1000)
#define KBRD_P13_EN                          (0x2000)
#define KBRD_P14_EN                          (0x4000)
#define KBRD_P15_EN                          (0x8000)



#define KBRD_IRQ_IN_SEL2_REG                 (0x50001416) /* GPIO interrupt selection for KBRD_IRQ for P3 */
#define KBRD_IRQ_IN_SEL2_REG_RESET           (0x00000000)

/*=============================*/
struct __KBRD_IRQ_IN_SEL2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_KBRD_P30_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P31_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P32_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P33_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P34_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P35_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P36_EN                      : 1;
    volatile uint16_t BITFLD_KBRD_P37_EN                      : 1;
};
#define KBRD_P30_EN                          (0x0001)
#define KBRD_P31_EN                          (0x0002)
#define KBRD_P32_EN                          (0x0004)
#define KBRD_P33_EN                          (0x0008)
#define KBRD_P34_EN                          (0x0010)
#define KBRD_P35_EN                          (0x0020)
#define KBRD_P36_EN                          (0x0040)
#define KBRD_P37_EN                          (0x0080)



/*===========================================================================*/
/* memory map OTPC */
/*===========================================================================*/



#define OTPC_MODE_REG                        (0x07F40000) /* Mode register */
#define OTPC_MODE_REG_RESET                  (0x00000000)

/*=============================*/
struct __OTPC_MODE_REG
/*=============================*/
{
    volatile uint32_t BITFLD_OTPC_MODE_MODE                   : 3;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_OTPC_MODE_USE_DMA                : 1;
    volatile uint32_t BITFLD_OTPC_MODE_FIFO_FLUSH             : 1;
    volatile uint32_t BITFLD_OTPC_MODE_ERR_RESP_DIS           : 1;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_OTPC_MODE_USE_SP_ROWS            : 1;
    volatile uint32_t BITFLD_OTPC_MODE_RLD_RR_REQ             : 1;
    volatile uint32_t BITFLD_OTPC_MODE_BCHK_ALG               : 2;
    volatile uint32_t BITFLD_OTPC_MODE_BCHK_THR               : 3;
};
#define OTPC_MODE_MODE                       (0x0007)
#define OTPC_MODE_USE_DMA                    (0x0010)
#define OTPC_MODE_FIFO_FLUSH                 (0x0020)
#define OTPC_MODE_ERR_RESP_DIS               (0x0040)
#define OTPC_MODE_USE_SP_ROWS                (0x0100)
#define OTPC_MODE_RLD_RR_REQ                 (0x0200)
#define OTPC_MODE_BCHK_ALG                   (0x0C00)
#define OTPC_MODE_BCHK_THR                   (0x7000)



#define OTPC_PCTRL_REG                       (0x07F40004) /* Bit-programming control register */
#define OTPC_PCTRL_REG_RESET                 (0x00000000)

/*=============================*/
struct __OTPC_PCTRL_REG
/*=============================*/
{
    volatile uint32_t BITFLD_OTPC_PCTRL_WADDR                 : 13;
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_OTPC_PCTRL_PRETRY                : 1;
    volatile uint32_t BITFLD_OTPC_PCTRL_PSTART                : 1;
};
#define OTPC_PCTRL_WADDR                     (0x1FFF)
#define OTPC_PCTRL_PRETRY                    (0x4000)
#define OTPC_PCTRL_PSTART                    (0x8000)



#define OTPC_STAT_REG                        (0x07F40008) /* Status register */
#define OTPC_STAT_REG_RESET                  (0x00000051)

/*=============================*/
struct __OTPC_STAT_REG
/*=============================*/
{
    volatile uint32_t BITFLD_OTPC_STAT_PRDY                   : 1;
    volatile uint32_t BITFLD_OTPC_STAT_PERR_UNC               : 1;
    volatile uint32_t BITFLD_OTPC_STAT_PERR_COR               : 1;
    volatile uint32_t BITFLD_OTPC_STAT_PZERO                  : 1;
    volatile uint32_t BITFLD_OTPC_STAT_TRDY                   : 1;
    volatile uint32_t BITFLD_OTPC_STAT_TERROR                 : 1;
    volatile uint32_t BITFLD_OTPC_STAT_ARDY                   : 1;
    volatile uint32_t BITFLD_OTPC_STAT_RERROR                 : 1;
    volatile uint32_t BITFLD_OTPC_STAT_FWORDS                 : 4;
    volatile uint32_t                                         : 4;
    volatile uint32_t BITFLD_OTPC_STAT_NWORDS                 : 14;
};
#define OTPC_STAT_PRDY                       (0x0001)
#define OTPC_STAT_PERR_UNC                   (0x0002)
#define OTPC_STAT_PERR_COR                   (0x0004)
#define OTPC_STAT_PZERO                      (0x0008)
#define OTPC_STAT_TRDY                       (0x0010)
#define OTPC_STAT_TERROR                     (0x0020)
#define OTPC_STAT_ARDY                       (0x0040)
#define OTPC_STAT_RERROR                     (0x0080)
#define OTPC_STAT_FWORDS                     (0x0F00)
#define OTPC_STAT_NWORDS                     (0x3FFF0000)



#define OTPC_AHBADR_REG                      (0x07F4000C) /* AHB master start address */
#define OTPC_AHBADR_REG_RESET                (0x07FC0000)

/*=============================*/
struct __OTPC_AHBADR_REG
/*=============================*/
{
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_OTPC_AHBADR                      : 30;
};
#define OTPC_AHBADR                          (0xFFFFFFFC)



#define OTPC_CELADR_REG                      (0x07F40010) /* Macrocell start address */
#define OTPC_CELADR_REG_RESET                (0x00000000)

/*=============================*/
struct __OTPC_CELADR_REG
/*=============================*/
{
    volatile uint32_t BITFLD_OTPC_CELADR                      : 14;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_OTPC_CELADR_LV                   : 14;
};
#define OTPC_CELADR                          (0x3FFF)
#define OTPC_CELADR_LV                       (0x3FFF0000)



#define OTPC_NWORDS_REG                      (0x07F40014) /* Number of words */
#define OTPC_NWORDS_REG_RESET                (0x00000000)

/*=============================*/
struct __OTPC_NWORDS_REG
/*=============================*/
{
    volatile uint32_t BITFLD_OTPC_NWORDS                      : 14;
};
#define OTPC_NWORDS                          (0x3FFF)



#define OTPC_FFPRT_REG                       (0x07F40018) /* Ports access to fifo logic */
#define OTPC_FFPRT_REG_RESET                 (0x00000000)

/*=============================*/
struct __OTPC_FFPRT_REG
/*=============================*/
{
    volatile uint32_t BITFLD_OTPC_FFPRT                       : 32;
};
#define OTPC_FFPRT                           (0xFFFFFFFF)



#define OTPC_FFRD_REG                        (0x07F4001C) /* The data which have taken with the latest read from the OTPC_FFPRT_REG */
#define OTPC_FFRD_REG_RESET                  (0x00000000)

/*=============================*/
struct __OTPC_FFRD_REG
/*=============================*/
{
    volatile uint32_t BITFLD_OTPC_FFRD                        : 32;
};
#define OTPC_FFRD                            (0xFFFFFFFF)



#define OTPC_PWORDL_REG                      (0x07F40020) /* The 32 lower bits of the 64-bit word that will be programmed, when the MPROG mode is used. */
#define OTPC_PWORDL_REG_RESET                (0x00000000)

/*=============================*/
struct __OTPC_PWORDL_REG
/*=============================*/
{
    volatile uint32_t BITFLD_OTPC_PWORDL                      : 32;
};
#define OTPC_PWORDL                          (0xFFFFFFFF)



#define OTPC_PWORDH_REG                      (0x07F40024) /* The 32 higher bits of the 64-bit word that will be programmed, when the MPROG mode is used. */
#define OTPC_PWORDH_REG_RESET                (0x00000000)

/*=============================*/
struct __OTPC_PWORDH_REG
/*=============================*/
{
    volatile uint32_t BITFLD_OTPC_PWORDH                      : 32;
};
#define OTPC_PWORDH                          (0xFFFFFFFF)



#define OTPC_TIM1_REG                        (0x07F40028) /* Various timing parameters of the OTP cell. */
#define OTPC_TIM1_REG_RESET                  (0x1A104F20)

/*=============================*/
struct __OTPC_TIM1_REG
/*=============================*/
{
    volatile uint32_t BITFLD_OTPC_TIM1_CC_T_CADX              : 8;
    volatile uint32_t BITFLD_OTPC_TIM1_CC_T_PW                : 8;
    volatile uint32_t BITFLD_OTPC_TIM1_CC_T_1US               : 6;
    volatile uint32_t BITFLD_OTPC_TIM1_CC_T_500NS             : 5;
    volatile uint32_t BITFLD_OTPC_TIM1_CC_T_200NS             : 4;
    volatile uint32_t BITFLD_OTPC_TIM1_CC_T_25NS              : 1;
};
#define OTPC_TIM1_CC_T_CADX                  (0x00FF)
#define OTPC_TIM1_CC_T_PW                    (0xFF00)
#define OTPC_TIM1_CC_T_1US                   (0x3F0000)
#define OTPC_TIM1_CC_T_500NS                 (0x7C00000)
#define OTPC_TIM1_CC_T_200NS                 (0x78000000)
#define OTPC_TIM1_CC_T_25NS                  (0x80000000)



#define OTPC_TIM2_REG                        (0x07F4002C) /* Various timing parameters of the OTP cell. */
#define OTPC_TIM2_REG_RESET                  (0x00010000)

/*=============================*/
struct __OTPC_TIM2_REG
/*=============================*/
{
    volatile uint32_t BITFLD_OTPC_TIM2_CC_STBY_THR            : 10;
    volatile uint32_t                                         : 6;
    volatile uint32_t BITFLD_OTPC_TIM2_CC_T_BCHK              : 7;
    volatile uint32_t BITFLD_OTPC_TIM2_RDENL_PROT             : 1;
};
#define OTPC_TIM2_CC_STBY_THR                (0x03FF)
#define OTPC_TIM2_CC_T_BCHK                  (0x7F0000)
#define OTPC_TIM2_RDENL_PROT                 (0x800000)



#define OTPC_BCSTS_REG                       (0x07F40034) /* Blank check status register */
#define OTPC_BCSTS_REG_RESET                 (0x00000000)

/*=============================*/
struct __OTPC_BCSTS_REG
/*=============================*/
{
    volatile uint32_t BITFLD_OTPC_BCHK_EQ1_WCNT               : 14;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_OTPC_BCHK_BEQ2_WCNT              : 14;
};
#define OTPC_BCHK_EQ1_WCNT                   (0x3FFF)
#define OTPC_BCHK_BEQ2_WCNT                  (0x3FFF0000)



/*===========================================================================*/
/* memory map PATCH */
/*===========================================================================*/
//reserved only



/*===========================================================================*/
/* memory map QDEC */
/*===========================================================================*/



#define QDEC_CTRL_REG                        (0x50000200) /* Quad Decoder control register */
#define QDEC_CTRL_REG_RESET                  (0x00000010)

/*=============================*/
struct __QDEC_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_QD_IRQ_MASK                      : 1;
    volatile uint16_t BITFLD_QD_IRQ_CLR                       : 1;
    volatile uint16_t BITFLD_QD_IRQ_STATUS                    : 1;
    volatile uint16_t BITFLD_QD_IRQ_THRES                     : 7;
};
#define QD_IRQ_MASK                          (0x0001)
#define QD_IRQ_CLR                           (0x0002)
#define QD_IRQ_STATUS                        (0x0004)
#define QD_IRQ_THRES                         (0x03F8)



#define QDEC_XCNT_REG                        (0x50000202) /* Counter value of the X Axis */
#define QDEC_XCNT_REG_RESET                  (0x00000000)

/*=============================*/
struct __QDEC_XCNT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_X_counter                        : 16;
};
#define X_counter                            (0xFFFF)



#define QDEC_YCNT_REG                        (0x50000204) /* Counter value of the Y Axis */
#define QDEC_YCNT_REG_RESET                  (0x00000000)

/*=============================*/
struct __QDEC_YCNT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_Y_counter                        : 16;
};
#define Y_counter                            (0xFFFF)



#define QDEC_CLOCKDIV_REG                    (0x50000206) /* Clock divider register */
#define QDEC_CLOCKDIV_REG_RESET              (0x00000000)

/*=============================*/
struct __QDEC_CLOCKDIV_REG
/*=============================*/
{
    volatile uint16_t BITFLD_clock_divider                    : 10;
};
#define clock_divider                        (0x03FF)



#define QDEC_CTRL2_REG                       (0x50000208) /* Quad Decoder control register */
#define QDEC_CTRL2_REG_RESET                 (0x00000000)

/*=============================*/
struct __QDEC_CTRL2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CHX_PORT_SEL                     : 4;
    volatile uint16_t BITFLD_CHY_PORT_SEL                     : 4;
    volatile uint16_t BITFLD_CHZ_PORT_SEL                     : 4;
};
#define CHX_PORT_SEL                         (0x000F)
#define CHY_PORT_SEL                         (0x00F0)
#define CHZ_PORT_SEL                         (0x0F00)



#define QDEC_ZCNT_REG                        (0x5000020A) /* Z_counter */
#define QDEC_ZCNT_REG_RESET                  (0x00000000)

/*=============================*/
struct __QDEC_ZCNT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_Z_counter                        : 16;
};
#define Z_counter                            (0xFFFF)



/*===========================================================================*/
/* memory map r_rfcu585_nl00 */
/*===========================================================================*/
//reserved only



/*===========================================================================*/
/* memory map rfpt580_gr01 */
/*===========================================================================*/
//reserved only



/*===========================================================================*/
/* memory map SPI */
/*===========================================================================*/



#define SPI_CTRL_REG                         (0x50001200) /* SPI control register 0 */
#define SPI_CTRL_REG_RESET                   (0x00000000)

/*=============================*/
struct __SPI_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SPI_ON                           : 1;
    volatile uint16_t BITFLD_SPI_PHA                          : 1;
    volatile uint16_t BITFLD_SPI_POL                          : 1;
    volatile uint16_t BITFLD_SPI_CLK                          : 2;
    volatile uint16_t BITFLD_SPI_DO                           : 1;
    volatile uint16_t BITFLD_SPI_SMN                          : 1;
    volatile uint16_t BITFLD_SPI_WORD                         : 2;
    volatile uint16_t BITFLD_SPI_RST                          : 1;
    volatile uint16_t BITFLD_SPI_FORCE_DO                     : 1;
    volatile uint16_t BITFLD_SPI_TXH                          : 1;
    volatile uint16_t BITFLD_SPI_DI                           : 1;
    volatile uint16_t BITFLD_SPI_INT_BIT                      : 1;
    volatile uint16_t BITFLD_SPI_MINT                         : 1;
    volatile uint16_t BITFLD_SPI_EN_CTRL                      : 1;
};
#define SPI_ON                               (0x0001)
#define SPI_PHA                              (0x0002)
#define SPI_POL                              (0x0004)
#define SPI_CLK                              (0x0018)
#define SPI_DO                               (0x0020)
#define SPI_SMN                              (0x0040)
#define SPI_WORD                             (0x0180)
#define SPI_RST                              (0x0200)
#define SPI_FORCE_DO                         (0x0400)
#define SPI_TXH                              (0x0800)
#define SPI_DI                               (0x1000)
#define SPI_INT_BIT                          (0x2000)
#define SPI_MINT                             (0x4000)
#define SPI_EN_CTRL                          (0x8000)



#define SPI_RX_TX_REG0                       (0x50001202) /* SPI RX/TX register0 */
#define SPI_RX_TX_REG0_RESET                 (0x00000000)

/*=============================*/
struct __SPI_RX_TX_REG0
/*=============================*/
{
    volatile uint16_t BITFLD_SPI_DATA0                        : 16;
};
#define SPI_DATA0                            (0xFFFF)



#define SPI_RX_TX_REG1                       (0x50001204) /* SPI RX/TX register1 */
#define SPI_RX_TX_REG1_RESET                 (0x00000000)

/*=============================*/
struct __SPI_RX_TX_REG1
/*=============================*/
{
    volatile uint16_t BITFLD_SPI_DATA1                        : 16;
};
#define SPI_DATA1                            (0xFFFF)



#define SPI_CLEAR_INT_REG                    (0x50001206) /* SPI clear interrupt register */
#define SPI_CLEAR_INT_REG_RESET              (0x00000000)

/*=============================*/
struct __SPI_CLEAR_INT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SPI_CLEAR_INT                    : 16;
};
#define SPI_CLEAR_INT                        (0xFFFF)



#define SPI_CTRL_REG1                        (0x50001208) /* SPI control register 1 */
#define SPI_CTRL_REG1_RESET                  (0x00000003)

/*=============================*/
struct __SPI_CTRL_REG1
/*=============================*/
{
    volatile uint16_t BITFLD_SPI_FIFO_MODE                    : 2;
    volatile uint16_t BITFLD_SPI_PRIORITY                     : 1;
    volatile uint16_t BITFLD_SPI_BUSY                         : 1;
    volatile uint16_t BITFLD_SPI_9BIT_VAL                     : 1;
};
#define SPI_FIFO_MODE                        (0x0003)
#define SPI_PRIORITY                         (0x0004)
#define SPI_BUSY                             (0x0008)
#define SPI_9BIT_VAL                         (0x0010)



/*===========================================================================*/
/* memory map Timer+3PWM */
/*===========================================================================*/



#define TIMER0_CTRL_REG                      (0x50003400) /* Timer0 control register */
#define TIMER0_CTRL_REG_RESET                (0x00000000)

/*=============================*/
struct __TIMER0_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_TIM0_CTRL                        : 1;
    volatile uint16_t BITFLD_TIM0_CLK_SEL                     : 1;
    volatile uint16_t BITFLD_TIM0_CLK_DIV                     : 1;
    volatile uint16_t BITFLD_PWM_MODE                         : 1;
};
#define TIM0_CTRL                            (0x0001)
#define TIM0_CLK_SEL                         (0x0002)
#define TIM0_CLK_DIV                         (0x0004)
#define PWM_MODE                             (0x0008)



#define TIMER0_ON_REG                        (0x50003402) /* Timer0 on control register */
#define TIMER0_ON_REG_RESET                  (0x00000000)

/*=============================*/
struct __TIMER0_ON_REG
/*=============================*/
{
    volatile uint16_t BITFLD_TIM0_ON                          : 16;
};
#define TIM0_ON                              (0xFFFF)



#define TIMER0_RELOAD_M_REG                  (0x50003404) /* 16 bits reload value for Timer0 */
#define TIMER0_RELOAD_M_REG_RESET            (0x00000000)

/*=============================*/
struct __TIMER0_RELOAD_M_REG
/*=============================*/
{
    volatile uint16_t BITFLD_TIM0_M                           : 16;
};
#define TIM0_M                               (0xFFFF)



#define TIMER0_RELOAD_N_REG                  (0x50003406) /* 16 bits reload value for Timer0 */
#define TIMER0_RELOAD_N_REG_RESET            (0x00000000)

/*=============================*/
struct __TIMER0_RELOAD_N_REG
/*=============================*/
{
    volatile uint16_t BITFLD_TIM0_N                           : 16;
};
#define TIM0_N                               (0xFFFF)



#define PWM2_DUTY_CYCLE                      (0x50003408) /* Duty Cycle for PWM2 */
#define PWM2_DUTY_CYCLE_RESET                (0x00000000)

/*=============================*/
struct __PWM2_DUTY_CYCLE
/*=============================*/
{
    volatile uint16_t BITFLD_DUTY_CYCLE                       : 14;
};
#define DUTY_CYCLE                           (0x3FFF)



#define PWM3_DUTY_CYCLE                      (0x5000340A) /* Duty Cycle for PWM3 */
#define PWM3_DUTY_CYCLE_RESET                (0x00000000)

/*=============================*/
struct __PWM3_DUTY_CYCLE
/*=============================*/
{
    volatile uint16_t BITFLD_DUTY_CYCLE                       : 14;
};
#define DUTY_CYCLE                           (0x3FFF)



#define PWM4_DUTY_CYCLE                      (0x5000340C) /* Duty Cycle for PWM4 */
#define PWM4_DUTY_CYCLE_RESET                (0x00000000)

/*=============================*/
struct __PWM4_DUTY_CYCLE
/*=============================*/
{
    volatile uint16_t BITFLD_DUTY_CYCLE                       : 14;
};
#define DUTY_CYCLE                           (0x3FFF)



#define TRIPLE_PWM_FREQUENCY                 (0x5000340E) /* Frequency for PWM 2,3 and 4 */
#define TRIPLE_PWM_FREQUENCY_RESET           (0x00000000)

/*=============================*/
struct __TRIPLE_PWM_FREQUENCY
/*=============================*/
{
    volatile uint16_t BITFLD_FREQ                             : 14;
};
#define FREQ                                 (0x3FFF)



#define TRIPLE_PWM_CTRL_REG                  (0x50003410) /* PWM 2 3 4 Control */
#define TRIPLE_PWM_CTRL_REG_RESET            (0x00000004)

/*=============================*/
struct __TRIPLE_PWM_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_TRIPLE_PWM_ENABLE                : 1;
    volatile uint16_t BITFLD_SW_PAUSE_EN                      : 1;
    volatile uint16_t BITFLD_HW_PAUSE_EN                      : 1;
};
#define TRIPLE_PWM_ENABLE                    (0x0001)
#define SW_PAUSE_EN                          (0x0002)
#define HW_PAUSE_EN                          (0x0004)



/*===========================================================================*/
/* memory map UART */
/*===========================================================================*/



#define UART_RBR_THR_DLL_REG                 (0x50001000) /* Receive Buffer Register */
#define UART_RBR_THR_DLL_REG_RESET           (0x00000000)

/*=============================*/
struct __UART_RBR_THR_DLL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RBR_THR_DLL                      : 8;
};
#define RBR_THR_DLL                          (0x00FF)



#define UART_IER_DLH_REG                     (0x50001004) /* Interrupt Enable Register */
#define UART_IER_DLH_REG_RESET               (0x00000000)

/*=============================*/
struct __UART_IER_DLH_REG
/*=============================*/
{
    volatile uint16_t BITFLD_ERBFI_dlh0                       : 1;
    volatile uint16_t BITFLD_ETBEI_dlh1                       : 1;
    volatile uint16_t BITFLD_ELSI_dhl2                        : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PTIME_dlh7                       : 1;
};
#define ERBFI_dlh0                           (0x0001)
#define ETBEI_dlh1                           (0x0002)
#define ELSI_dhl2                            (0x0004)
#define PTIME_dlh7                           (0x0080)



#define UART_IIR_FCR_REG                     (0x50001008) /* Interrupt Identification Register/FIFO Control Register */
#define UART_IIR_FCR_REG_RESET               (0x00000000)

/*=============================*/
struct __UART_IIR_FCR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_IIR_FCR                          : 16;
};
#define IIR_FCR                              (0xFFFF)



#define UART_LCR_REG                         (0x5000100C) /* Line Control Register */
#define UART_LCR_REG_RESET                   (0x00000000)

/*=============================*/
struct __UART_LCR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_DLS                         : 2;
    volatile uint16_t BITFLD_UART_STOP                        : 1;
    volatile uint16_t BITFLD_UART_PEN                         : 1;
    volatile uint16_t BITFLD_UART_EPS                         : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_UART_BC                          : 1;
    volatile uint16_t BITFLD_UART_DLAB                        : 1;
};
#define UART_DLS                             (0x0003)
#define UART_STOP                            (0x0004)
#define UART_PEN                             (0x0008)
#define UART_EPS                             (0x0010)
#define UART_BC                              (0x0040)
#define UART_DLAB                            (0x0080)



#define UART_MCR_REG                         (0x50001010) /* Modem Control Register */
#define UART_MCR_REG_RESET                   (0x00000000)

/*=============================*/
struct __UART_MCR_REG
/*=============================*/
{
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_UART_RTS                         : 1;
    volatile uint16_t BITFLD_UART_OUT1                        : 1;
    volatile uint16_t BITFLD_UART_OUT2                        : 1;
    volatile uint16_t BITFLD_UART_LB                          : 1;
    volatile uint16_t BITFLD_UART_AFCE                        : 1;
    volatile uint16_t BITFLD_UART_SIRE                        : 1;
};
#define UART_RTS                             (0x0002)
#define UART_OUT1                            (0x0004)
#define UART_OUT2                            (0x0008)
#define UART_LB                              (0x0010)
#define UART_AFCE                            (0x0020)
#define UART_SIRE                            (0x0040)



#define UART_LSR_REG                         (0x50001014) /* Line Status Register */
#define UART_LSR_REG_RESET                   (0x00000060)

/*=============================*/
struct __UART_LSR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_DR                          : 1;
    volatile uint16_t BITFLD_UART_OE                          : 1;
    volatile uint16_t BITFLD_UART_PE                          : 1;
    volatile uint16_t BITFLD_UART_FE                          : 1;
    volatile uint16_t BITFLD_UART_BI                          : 1;
    volatile uint16_t BITFLD_UART_THRE                        : 1;
    volatile uint16_t BITFLD_UART_TEMT                        : 1;
    volatile uint16_t BITFLD_UART_RFE                         : 1;
};
#define UART_DR                              (0x0001)
#define UART_OE                              (0x0002)
#define UART_PE                              (0x0004)
#define UART_FE                              (0x0008)
#define UART_BI                              (0x0010)
#define UART_THRE                            (0x0020)
#define UART_TEMT                            (0x0040)
#define UART_RFE                             (0x0080)



#define UART_MSR_REG                         (0x50001018) /* Modem Status Register */
#define UART_MSR_REG_RESET                   (0x00000000)

/*=============================*/
struct __UART_MSR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_DCTS                        : 1;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_UART_CTS                         : 1;
};
#define UART_DCTS                            (0x0001)
#define UART_CTS                             (0x0010)



#define UART_SCR_REG                         (0x5000101C) /* Scratchpad Register */
#define UART_SCR_REG_RESET                   (0x00000000)

/*=============================*/
struct __UART_SCR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_SCRATCH_PAD                 : 8;
};
#define UART_SCRATCH_PAD                     (0x00FF)



#define UART_SRBR_STHR0_REG                  (0x50001030) /* Shadow Receive/Transmit Buffer Register */
#define UART_SRBR_STHR0_REG_RESET            (0x00000000)

/*=============================*/
struct __UART_SRBR_STHR0_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART_SRBR_STHR1_REG                  (0x50001034) /* Shadow Receive/Transmit Buffer Register */
#define UART_SRBR_STHR1_REG_RESET            (0x00000000)

/*=============================*/
struct __UART_SRBR_STHR1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART_SRBR_STHR2_REG                  (0x50001038) /* Shadow Receive/Transmit Buffer Register */
#define UART_SRBR_STHR2_REG_RESET            (0x00000000)

/*=============================*/
struct __UART_SRBR_STHR2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART_SRBR_STHR3_REG                  (0x5000103C) /* Shadow Receive/Transmit Buffer Register */
#define UART_SRBR_STHR3_REG_RESET            (0x00000000)

/*=============================*/
struct __UART_SRBR_STHR3_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART_SRBR_STHR4_REG                  (0x50001040) /* Shadow Receive/Transmit Buffer Register */
#define UART_SRBR_STHR4_REG_RESET            (0x00000000)

/*=============================*/
struct __UART_SRBR_STHR4_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART_SRBR_STHR5_REG                  (0x50001044) /* Shadow Receive/Transmit Buffer Register */
#define UART_SRBR_STHR5_REG_RESET            (0x00000000)

/*=============================*/
struct __UART_SRBR_STHR5_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART_SRBR_STHR6_REG                  (0x50001048) /* Shadow Receive/Transmit Buffer Register */
#define UART_SRBR_STHR6_REG_RESET            (0x00000000)

/*=============================*/
struct __UART_SRBR_STHR6_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART_SRBR_STHR7_REG                  (0x5000104C) /* Shadow Receive/Transmit Buffer Register */
#define UART_SRBR_STHR7_REG_RESET            (0x00000000)

/*=============================*/
struct __UART_SRBR_STHR7_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART_SRBR_STHR8_REG                  (0x50001050) /* Shadow Receive/Transmit Buffer Register */
#define UART_SRBR_STHR8_REG_RESET            (0x00000000)

/*=============================*/
struct __UART_SRBR_STHR8_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART_SRBR_STHR9_REG                  (0x50001054) /* Shadow Receive/Transmit Buffer Register */
#define UART_SRBR_STHR9_REG_RESET            (0x00000000)

/*=============================*/
struct __UART_SRBR_STHR9_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART_SRBR_STHR10_REG                 (0x50001058) /* Shadow Receive/Transmit Buffer Register */
#define UART_SRBR_STHR10_REG_RESET           (0x00000000)

/*=============================*/
struct __UART_SRBR_STHR10_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART_SRBR_STHR11_REG                 (0x5000105C) /* Shadow Receive/Transmit Buffer Register */
#define UART_SRBR_STHR11_REG_RESET           (0x00000000)

/*=============================*/
struct __UART_SRBR_STHR11_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART_SRBR_STHR12_REG                 (0x50001060) /* Shadow Receive/Transmit Buffer Register */
#define UART_SRBR_STHR12_REG_RESET           (0x00000000)

/*=============================*/
struct __UART_SRBR_STHR12_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART_SRBR_STHR13_REG                 (0x50001064) /* Shadow Receive/Transmit Buffer Register */
#define UART_SRBR_STHR13_REG_RESET           (0x00000000)

/*=============================*/
struct __UART_SRBR_STHR13_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART_SRBR_STHR14_REG                 (0x50001068) /* Shadow Receive/Transmit Buffer Register */
#define UART_SRBR_STHR14_REG_RESET           (0x00000000)

/*=============================*/
struct __UART_SRBR_STHR14_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART_SRBR_STHR15_REG                 (0x5000106C) /* Shadow Receive/Transmit Buffer Register */
#define UART_SRBR_STHR15_REG_RESET           (0x00000000)

/*=============================*/
struct __UART_SRBR_STHR15_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART_FAR_REG                         (0x50001070) /* FIFO Access Register */
#define UART_FAR_REG_RESET                   (0x00000000)

/*=============================*/
struct __UART_FAR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_FAR                         : 1;
};
#define UART_FAR                             (0x0001)



#define UART_USR_REG                         (0x5000107C) /* UART Status register. */
#define UART_USR_REG_RESET                   (0x00000006)

/*=============================*/
struct __UART_USR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_BUSY                        : 1;
    volatile uint16_t BITFLD_UART_TFNF                        : 1;
    volatile uint16_t BITFLD_UART_TFE                         : 1;
    volatile uint16_t BITFLD_UART_RFNE                        : 1;
    volatile uint16_t BITFLD_UART_RFF                         : 1;
};
#define UART_BUSY                            (0x0001)
#define UART_TFNF                            (0x0002)
#define UART_TFE                             (0x0004)
#define UART_RFNE                            (0x0008)
#define UART_RFF                             (0x0010)



#define UART_TFL_REG                         (0x50001080) /* Transmit FIFO Level */
#define UART_TFL_REG_RESET                   (0x00000000)

/*=============================*/
struct __UART_TFL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_TRANSMIT_FIFO_LEVEL         : 16;
};
#define UART_TRANSMIT_FIFO_LEVEL             (0xFFFF)



#define UART_RFL_REG                         (0x50001084) /* Receive FIFO Level. */
#define UART_RFL_REG_RESET                   (0x00000000)

/*=============================*/
struct __UART_RFL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_RECEIVE_FIFO_LEVEL          : 16;
};
#define UART_RECEIVE_FIFO_LEVEL              (0xFFFF)



#define UART_SRR_REG                         (0x50001088) /* Software Reset Register. */
#define UART_SRR_REG_RESET                   (0x00000000)

/*=============================*/
struct __UART_SRR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_UR                          : 1;
    volatile uint16_t BITFLD_UART_RFR                         : 1;
    volatile uint16_t BITFLD_UART_XFR                         : 1;
};
#define UART_UR                              (0x0001)
#define UART_RFR                             (0x0002)
#define UART_XFR                             (0x0004)



#define UART_SRTS_REG                        (0x5000108C) /* Shadow Request to Send */
#define UART_SRTS_REG_RESET                  (0x00000000)

/*=============================*/
struct __UART_SRTS_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_SHADOW_REQUEST_TO_SEND      : 1;
};
#define UART_SHADOW_REQUEST_TO_SEND          (0x0001)



#define UART_SBCR_REG                        (0x50001090) /* Shadow Break Control Register */
#define UART_SBCR_REG_RESET                  (0x00000000)

/*=============================*/
struct __UART_SBCR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_SHADOW_BREAK_CONTROL        : 1;
};
#define UART_SHADOW_BREAK_CONTROL            (0x0001)



#define UART_SDMAM_REG                       (0x50001094) /* Shadow DMA Mode */
#define UART_SDMAM_REG_RESET                 (0x00000000)

/*=============================*/
struct __UART_SDMAM_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_SHADOW_DMA_MODE             : 1;
};
#define UART_SHADOW_DMA_MODE                 (0x0001)



#define UART_SFE_REG                         (0x50001098) /* Shadow FIFO Enable */
#define UART_SFE_REG_RESET                   (0x00000000)

/*=============================*/
struct __UART_SFE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_SHADOW_FIFO_ENABLE          : 1;
};
#define UART_SHADOW_FIFO_ENABLE              (0x0001)



#define UART_SRT_REG                         (0x5000109C) /* Shadow RCVR Trigger */
#define UART_SRT_REG_RESET                   (0x00000000)

/*=============================*/
struct __UART_SRT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_SHADOW_RCVR_TRIGGER         : 2;
};
#define UART_SHADOW_RCVR_TRIGGER             (0x0003)



#define UART_STET_REG                        (0x500010A0) /* Shadow TX Empty Trigger */
#define UART_STET_REG_RESET                  (0x00000000)

/*=============================*/
struct __UART_STET_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_SHADOW_TX_EMPTY_TRIGGER     : 2;
};
#define UART_SHADOW_TX_EMPTY_TRIGGER         (0x0003)



#define UART_HTX_REG                         (0x500010A4) /* Halt TX */
#define UART_HTX_REG_RESET                   (0x00000000)

/*=============================*/
struct __UART_HTX_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_HALT_TX                     : 1;
};
#define UART_HALT_TX                         (0x0001)



#define UART_DMASA_REG                       (0x500010A8) /* DMA Software Acknowledge */
#define UART_DMASA_REG_RESET                 (0x00000000)

/*=============================*/
struct __UART_DMASA_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMASA                            : 1;
};
#define DMASA                                (0x0001)



#define UART_DLF_REG                         (0x500010C0) /* Divisor Latch Fraction Register */
#define UART_DLF_REG_RESET                   (0x00000000)

/*=============================*/
struct __UART_DLF_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_DLF                         : 4;
};
#define UART_DLF                             (0x000F)



#define UART_CPR_REG                         (0x500010F4) /* Component Parameter Register */
#define UART_CPR_REG_RESET                   (0x00003D71)

/*=============================*/
struct __UART_CPR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CPR                              : 16;
};
#define CPR                                  (0xFFFF)



#define UART_UCV_REG                         (0x500010F8) /* Component Version */
#define UART_UCV_REG_RESET                   (0x3331352A)

/*=============================*/
struct __UART_UCV_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UCV                              : 16;
};
#define UCV                                  (0xFFFF)



#define UART_CTR_REG                         (0x500010FC) /* Component Type Register */
#define UART_CTR_REG_RESET                   (0x44570110)

/*=============================*/
struct __UART_CTR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CTR                              : 16;
};
#define CTR                                  (0xFFFF)



#define UART2_RBR_THR_DLL_REG                (0x50001100) /* Receive Buffer Register */
#define UART2_RBR_THR_DLL_REG_RESET          (0x00000000)

/*=============================*/
struct __UART2_RBR_THR_DLL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RBR_THR_DLL                      : 8;
};
#define RBR_THR_DLL                          (0x00FF)



#define UART2_IER_DLH_REG                    (0x50001104) /* Interrupt Enable Register */
#define UART2_IER_DLH_REG_RESET              (0x00000000)

/*=============================*/
struct __UART2_IER_DLH_REG
/*=============================*/
{
    volatile uint16_t BITFLD_ERBFI_dlh0                       : 1;
    volatile uint16_t BITFLD_ETBEI_dlh1                       : 1;
    volatile uint16_t BITFLD_ELSI_dhl2                        : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_PTIME_dlh7                       : 1;
};
#define ERBFI_dlh0                           (0x0001)
#define ETBEI_dlh1                           (0x0002)
#define ELSI_dhl2                            (0x0004)
#define PTIME_dlh7                           (0x0080)



#define UART2_IIR_FCR_REG                    (0x50001108) /* Interrupt Identification Register/FIFO Control Register */
#define UART2_IIR_FCR_REG_RESET              (0x00000000)

/*=============================*/
struct __UART2_IIR_FCR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_IIR_FCR                          : 16;
};
#define IIR_FCR                              (0xFFFF)



#define UART2_LCR_REG                        (0x5000110C) /* Line Control Register */
#define UART2_LCR_REG_RESET                  (0x00000000)

/*=============================*/
struct __UART2_LCR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_DLS                         : 2;
    volatile uint16_t BITFLD_UART_STOP                        : 1;
    volatile uint16_t BITFLD_UART_PEN                         : 1;
    volatile uint16_t BITFLD_UART_EPS                         : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_UART_BC                          : 1;
    volatile uint16_t BITFLD_UART_DLAB                        : 1;
};
#define UART_DLS                             (0x0003)
#define UART_STOP                            (0x0004)
#define UART_PEN                             (0x0008)
#define UART_EPS                             (0x0010)
#define UART_BC                              (0x0040)
#define UART_DLAB                            (0x0080)



#define UART2_MCR_REG                        (0x50001110) /* Modem Control Register */
#define UART2_MCR_REG_RESET                  (0x00000000)

/*=============================*/
struct __UART2_MCR_REG
/*=============================*/
{
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_UART_RTS                         : 1;
    volatile uint16_t BITFLD_UART_OUT1                        : 1;
    volatile uint16_t BITFLD_UART_OUT2                        : 1;
    volatile uint16_t BITFLD_UART_LB                          : 1;
    volatile uint16_t BITFLD_UART_AFCE                        : 1;
    volatile uint16_t BITFLD_UART_SIRE                        : 1;
};
#define UART_RTS                             (0x0002)
#define UART_OUT1                            (0x0004)
#define UART_OUT2                            (0x0008)
#define UART_LB                              (0x0010)
#define UART_AFCE                            (0x0020)
#define UART_SIRE                            (0x0040)



#define UART2_LSR_REG                        (0x50001114) /* Line Status Register */
#define UART2_LSR_REG_RESET                  (0x00000060)

/*=============================*/
struct __UART2_LSR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_DR                          : 1;
    volatile uint16_t BITFLD_UART_OE                          : 1;
    volatile uint16_t BITFLD_UART_PE                          : 1;
    volatile uint16_t BITFLD_UART_FE                          : 1;
    volatile uint16_t BITFLD_UART_BI                          : 1;
    volatile uint16_t BITFLD_UART_THRE                        : 1;
    volatile uint16_t BITFLD_UART_TEMT                        : 1;
    volatile uint16_t BITFLD_UART_RFE                         : 1;
};
#define UART_DR                              (0x0001)
#define UART_OE                              (0x0002)
#define UART_PE                              (0x0004)
#define UART_FE                              (0x0008)
#define UART_BI                              (0x0010)
#define UART_THRE                            (0x0020)
#define UART_TEMT                            (0x0040)
#define UART_RFE                             (0x0080)



#define UART2_MSR_REG                        (0x50001118) /* Modem Status Register */
#define UART2_MSR_REG_RESET                  (0x00000000)

/*=============================*/
struct __UART2_MSR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_DCTS                        : 1;
    volatile uint16_t                                         : 3;
    volatile uint16_t BITFLD_UART_CTS                         : 1;
};
#define UART_DCTS                            (0x0001)
#define UART_CTS                             (0x0010)



#define UART2_SCR_REG                        (0x5000111C) /* Scratchpad Register */
#define UART2_SCR_REG_RESET                  (0x00000000)

/*=============================*/
struct __UART2_SCR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_SCRATCH_PAD                 : 8;
};
#define UART_SCRATCH_PAD                     (0x00FF)



#define UART2_SRBR_STHR0_REG                 (0x50001130) /* Shadow Receive/Transmit Buffer Register */
#define UART2_SRBR_STHR0_REG_RESET           (0x00000000)

/*=============================*/
struct __UART2_SRBR_STHR0_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART2_SRBR_STHR1_REG                 (0x50001134) /* Shadow Receive/Transmit Buffer Register */
#define UART2_SRBR_STHR1_REG_RESET           (0x00000000)

/*=============================*/
struct __UART2_SRBR_STHR1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART2_SRBR_STHR2_REG                 (0x50001138) /* Shadow Receive/Transmit Buffer Register */
#define UART2_SRBR_STHR2_REG_RESET           (0x00000000)

/*=============================*/
struct __UART2_SRBR_STHR2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART2_SRBR_STHR3_REG                 (0x5000113C) /* Shadow Receive/Transmit Buffer Register */
#define UART2_SRBR_STHR3_REG_RESET           (0x00000000)

/*=============================*/
struct __UART2_SRBR_STHR3_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART2_SRBR_STHR4_REG                 (0x50001140) /* Shadow Receive/Transmit Buffer Register */
#define UART2_SRBR_STHR4_REG_RESET           (0x00000000)

/*=============================*/
struct __UART2_SRBR_STHR4_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART2_SRBR_STHR5_REG                 (0x50001144) /* Shadow Receive/Transmit Buffer Register */
#define UART2_SRBR_STHR5_REG_RESET           (0x00000000)

/*=============================*/
struct __UART2_SRBR_STHR5_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART2_SRBR_STHR6_REG                 (0x50001148) /* Shadow Receive/Transmit Buffer Register */
#define UART2_SRBR_STHR6_REG_RESET           (0x00000000)

/*=============================*/
struct __UART2_SRBR_STHR6_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART2_SRBR_STHR7_REG                 (0x5000114C) /* Shadow Receive/Transmit Buffer Register */
#define UART2_SRBR_STHR7_REG_RESET           (0x00000000)

/*=============================*/
struct __UART2_SRBR_STHR7_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART2_SRBR_STHR8_REG                 (0x50001150) /* Shadow Receive/Transmit Buffer Register */
#define UART2_SRBR_STHR8_REG_RESET           (0x00000000)

/*=============================*/
struct __UART2_SRBR_STHR8_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART2_SRBR_STHR9_REG                 (0x50001154) /* Shadow Receive/Transmit Buffer Register */
#define UART2_SRBR_STHR9_REG_RESET           (0x00000000)

/*=============================*/
struct __UART2_SRBR_STHR9_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART2_SRBR_STHR10_REG                (0x50001158) /* Shadow Receive/Transmit Buffer Register */
#define UART2_SRBR_STHR10_REG_RESET          (0x00000000)

/*=============================*/
struct __UART2_SRBR_STHR10_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART2_SRBR_STHR11_REG                (0x5000115C) /* Shadow Receive/Transmit Buffer Register */
#define UART2_SRBR_STHR11_REG_RESET          (0x00000000)

/*=============================*/
struct __UART2_SRBR_STHR11_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART2_SRBR_STHR12_REG                (0x50001160) /* Shadow Receive/Transmit Buffer Register */
#define UART2_SRBR_STHR12_REG_RESET          (0x00000000)

/*=============================*/
struct __UART2_SRBR_STHR12_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART2_SRBR_STHR13_REG                (0x50001164) /* Shadow Receive/Transmit Buffer Register */
#define UART2_SRBR_STHR13_REG_RESET          (0x00000000)

/*=============================*/
struct __UART2_SRBR_STHR13_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART2_SRBR_STHR14_REG                (0x50001168) /* Shadow Receive/Transmit Buffer Register */
#define UART2_SRBR_STHR14_REG_RESET          (0x00000000)

/*=============================*/
struct __UART2_SRBR_STHR14_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART2_SRBR_STHR15_REG                (0x5000116C) /* Shadow Receive/Transmit Buffer Register */
#define UART2_SRBR_STHR15_REG_RESET          (0x00000000)

/*=============================*/
struct __UART2_SRBR_STHR15_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SRBR_STHRx                       : 8;
};
#define SRBR_STHRx                           (0x00FF)



#define UART2_FAR_REG                        (0x50001170) /* FIFO Access Register */
#define UART2_FAR_REG_RESET                  (0x00000000)

/*=============================*/
struct __UART2_FAR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_FAR                         : 1;
};
#define UART_FAR                             (0x0001)



#define UART2_USR_REG                        (0x5000117C) /* UART Status register. */
#define UART2_USR_REG_RESET                  (0x00000006)

/*=============================*/
struct __UART2_USR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_BUSY                        : 1;
    volatile uint16_t BITFLD_UART_TFNF                        : 1;
    volatile uint16_t BITFLD_UART_TFE                         : 1;
    volatile uint16_t BITFLD_UART_RFNE                        : 1;
    volatile uint16_t BITFLD_UART_RFF                         : 1;
};
#define UART_BUSY                            (0x0001)
#define UART_TFNF                            (0x0002)
#define UART_TFE                             (0x0004)
#define UART_RFNE                            (0x0008)
#define UART_RFF                             (0x0010)



#define UART2_TFL_REG                        (0x50001180) /* Transmit FIFO Level */
#define UART2_TFL_REG_RESET                  (0x00000000)

/*=============================*/
struct __UART2_TFL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_TRANSMIT_FIFO_LEVEL         : 16;
};
#define UART_TRANSMIT_FIFO_LEVEL             (0xFFFF)



#define UART2_RFL_REG                        (0x50001184) /* Receive FIFO Level. */
#define UART2_RFL_REG_RESET                  (0x00000000)

/*=============================*/
struct __UART2_RFL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_RECEIVE_FIFO_LEVEL          : 16;
};
#define UART_RECEIVE_FIFO_LEVEL              (0xFFFF)



#define UART2_SRR_REG                        (0x50001188) /* Software Reset Register. */
#define UART2_SRR_REG_RESET                  (0x00000000)

/*=============================*/
struct __UART2_SRR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_UR                          : 1;
    volatile uint16_t BITFLD_UART_RFR                         : 1;
    volatile uint16_t BITFLD_UART_XFR                         : 1;
};
#define UART_UR                              (0x0001)
#define UART_RFR                             (0x0002)
#define UART_XFR                             (0x0004)



#define UART2_SRTS_REG                       (0x5000118C) /* Shadow Request to Send */
#define UART2_SRTS_REG_RESET                 (0x00000000)

/*=============================*/
struct __UART2_SRTS_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_SHADOW_REQUEST_TO_SEND      : 1;
};
#define UART_SHADOW_REQUEST_TO_SEND          (0x0001)



#define UART2_SBCR_REG                       (0x50001190) /* Shadow Break Control Register */
#define UART2_SBCR_REG_RESET                 (0x00000000)

/*=============================*/
struct __UART2_SBCR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_SHADOW_BREAK_CONTROL        : 1;
};
#define UART_SHADOW_BREAK_CONTROL            (0x0001)



#define UART2_SDMAM_REG                      (0x50001194) /* Shadow DMA Mode */
#define UART2_SDMAM_REG_RESET                (0x00000000)

/*=============================*/
struct __UART2_SDMAM_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_SHADOW_DMA_MODE             : 1;
};
#define UART_SHADOW_DMA_MODE                 (0x0001)



#define UART2_SFE_REG                        (0x50001198) /* Shadow FIFO Enable */
#define UART2_SFE_REG_RESET                  (0x00000000)

/*=============================*/
struct __UART2_SFE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_SHADOW_FIFO_ENABLE          : 1;
};
#define UART_SHADOW_FIFO_ENABLE              (0x0001)



#define UART2_SRT_REG                        (0x5000119C) /* Shadow RCVR Trigger */
#define UART2_SRT_REG_RESET                  (0x00000000)

/*=============================*/
struct __UART2_SRT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_SHADOW_RCVR_TRIGGER         : 2;
};
#define UART_SHADOW_RCVR_TRIGGER             (0x0003)



#define UART2_STET_REG                       (0x500011A0) /* Shadow TX Empty Trigger */
#define UART2_STET_REG_RESET                 (0x00000000)

/*=============================*/
struct __UART2_STET_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_SHADOW_TX_EMPTY_TRIGGER     : 2;
};
#define UART_SHADOW_TX_EMPTY_TRIGGER         (0x0003)



#define UART2_HTX_REG                        (0x500011A4) /* Halt TX */
#define UART2_HTX_REG_RESET                  (0x00000000)

/*=============================*/
struct __UART2_HTX_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_HALT_TX                     : 1;
};
#define UART_HALT_TX                         (0x0001)



#define UART2_DMASA_REG                      (0x500011A8) /* DMA Software Acknowledge */
#define UART2_DMASA_REG_RESET                (0x00000000)

/*=============================*/
struct __UART2_DMASA_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DMASA                            : 1;
};
#define DMASA                                (0x0001)



#define UART2_DLF_REG                        (0x500011C0) /* Divisor Latch Fraction Register */
#define UART2_DLF_REG_RESET                  (0x00000000)

/*=============================*/
struct __UART2_DLF_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UART_DLF                         : 4;
};
#define UART_DLF                             (0x000F)



#define UART2_CPR_REG                        (0x500011F4) /* Component Parameter Register */
#define UART2_CPR_REG_RESET                  (0x00003D71)

/*=============================*/
struct __UART2_CPR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CPR                              : 16;
};
#define CPR                                  (0xFFFF)



#define UART2_UCV_REG                        (0x500011F8) /* Component Version */
#define UART2_UCV_REG_RESET                  (0x3331352A)

/*=============================*/
struct __UART2_UCV_REG
/*=============================*/
{
    volatile uint16_t BITFLD_UCV                              : 16;
};
#define UCV                                  (0xFFFF)



#define UART2_CTR_REG                        (0x500011FC) /* Component Type Register */
#define UART2_CTR_REG_RESET                  (0x44570110)

/*=============================*/
struct __UART2_CTR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CTR                              : 16;
};
#define CTR                                  (0xFFFF)



/*===========================================================================*/
/* memory map Version */
/*===========================================================================*/



#define CHIP_ID1_REG                         (0x50003200) /* Chip identification register 1. */
#define CHIP_ID1_REG_RESET                   (0x00000035)

/*=============================*/
struct __CHIP_ID1_REG
/*=============================*/
{
    volatile uint8_t  BITFLD_CHIP_ID1                         : 8;
};
#define CHIP_ID1                             (0x00FF)



#define CHIP_ID2_REG                         (0x50003201) /* Chip identification register 2. */
#define CHIP_ID2_REG_RESET                   (0x00000038)

/*=============================*/
struct __CHIP_ID2_REG
/*=============================*/
{
    volatile uint8_t  BITFLD_CHIP_ID2                         : 8;
};
#define CHIP_ID2                             (0x00FF)



#define CHIP_ID3_REG                         (0x50003202) /* Chip identification register 3. */
#define CHIP_ID3_REG_RESET                   (0x00000035)

/*=============================*/
struct __CHIP_ID3_REG
/*=============================*/
{
    volatile uint8_t  BITFLD_CHIP_ID3                         : 8;
};
#define CHIP_ID3                             (0x00FF)



#define CHIP_SWC_REG                         (0x50003203) /* Software compatibility register. */
#define CHIP_SWC_REG_RESET                   (0x00000000)

/*=============================*/
struct __CHIP_SWC_REG
/*=============================*/
{
    volatile uint8_t  BITFLD_CHIP_SWC                         : 4;
};
#define CHIP_SWC                             (0x000F)



#define CHIP_REVISION_REG                    (0x50003204) /* Chip revision register. */
#define CHIP_REVISION_REG_RESET              (0x00000041)

/*=============================*/
struct __CHIP_REVISION_REG
/*=============================*/
{
    volatile uint8_t  BITFLD_REVISION_ID                      : 8;
};
#define REVISION_ID                          (0x00FF)



/*===========================================================================*/
/* memory map WDOG */
/*===========================================================================*/



#define WATCHDOG_REG                         (0x50003100) /* Watchdog timer register. */
#define WATCHDOG_REG_RESET                   (0x000000FF)

/*=============================*/
struct __WATCHDOG_REG
/*=============================*/
{
    volatile uint16_t BITFLD_WDOG_VAL                         : 8;
    volatile uint16_t BITFLD_WDOG_VAL_NEG                     : 1;
    volatile uint16_t BITFLD_WDOG_WEN                         : 7;
};
#define WDOG_VAL                             (0x00FF)
#define WDOG_VAL_NEG                         (0x0100)
#define WDOG_WEN                             (0xFE00)



#define WATCHDOG_CTRL_REG                    (0x50003102) /* Watchdog control register. */
#define WATCHDOG_CTRL_REG_RESET              (0x00000000)

/*=============================*/
struct __WATCHDOG_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_NMI_RST                          : 1;
    volatile uint16_t                                         : 1;
};
#define NMI_RST                              (0x0001)



/*===========================================================================*/
/* memory map WKUP */
/*===========================================================================*/



#define WKUP_CTRL_REG                        (0x50000100) /* Control register for the wakeup counter */
#define WKUP_CTRL_REG_RESET                  (0x00000000)

/*=============================*/
struct __WKUP_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_WKUP_DEB_VALUE                   : 6;
    volatile uint16_t BITFLD_WKUP_SFT_KEYHIT                  : 1;
    volatile uint16_t BITFLD_WKUP_ENABLE_IRQ                  : 1;
    volatile uint16_t                                         : 6;
};
#define WKUP_DEB_VALUE                       (0x003F)
#define WKUP_SFT_KEYHIT                      (0x0040)
#define WKUP_ENABLE_IRQ                      (0x0080)



#define WKUP_COMPARE_REG                     (0x50000102) /* Number of events before wakeup interrupt */
#define WKUP_COMPARE_REG_RESET               (0x00000000)

/*=============================*/
struct __WKUP_COMPARE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_COMPARE                          : 8;
};
#define COMPARE                              (0x00FF)



#define WKUP_RESET_IRQ_REG                   (0x50000104) /* Reset wakeup interrupt */
#define WKUP_RESET_IRQ_REG_RESET             (0x00000000)

/*=============================*/
struct __WKUP_RESET_IRQ_REG
/*=============================*/
{
    volatile uint16_t BITFLD_WKUP_IRQ_RST                     : 16;
};
#define WKUP_IRQ_RST                         (0xFFFF)



#define WKUP_COUNTER_REG                     (0x50000106) /* Actual number of events of the wakeup counter */
#define WKUP_COUNTER_REG_RESET               (0x00000000)

/*=============================*/
struct __WKUP_COUNTER_REG
/*=============================*/
{
    volatile uint16_t BITFLD_EVENT_VALUE                      : 8;
};
#define EVENT_VALUE                          (0x00FF)



#define WKUP_RESET_CNTR_REG                  (0x50000108) /* Reset the event counter */
#define WKUP_RESET_CNTR_REG_RESET            (0x00000000)

/*=============================*/
struct __WKUP_RESET_CNTR_REG
/*=============================*/
{
    volatile uint16_t BITFLD_WKUP_CNTR_RST                    : 16;
};
#define WKUP_CNTR_RST                        (0xFFFF)



#define WKUP_SELECT_P0_REG                   (0x5000010A) /* Select which inputs from P0 port can trigger wkup counter */
#define WKUP_SELECT_P0_REG_RESET             (0x00000000)

/*=============================*/
struct __WKUP_SELECT_P0_REG
/*=============================*/
{
    volatile uint16_t BITFLD_WKUP_SELECT_P0                   : 8;
};
#define WKUP_SELECT_P0                       (0x00FF)



#define WKUP_SELECT_P1_REG                   (0x5000010C) /* Select which inputs from P1 port can trigger wkup counter */
#define WKUP_SELECT_P1_REG_RESET             (0x00000000)

/*=============================*/
struct __WKUP_SELECT_P1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_WKUP_SELECT_P1                   : 6;
};
#define WKUP_SELECT_P1                       (0x003F)



#define WKUP_SELECT_P2_REG                   (0x5000010E) /* Select which inputs from P2 port can trigger wkup counter */
#define WKUP_SELECT_P2_REG_RESET             (0x00000000)

/*=============================*/
struct __WKUP_SELECT_P2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_WKUP_SELECT_P2                   : 10;
};
#define WKUP_SELECT_P2                       (0x03FF)



#define WKUP_SELECT_P3_REG                   (0x50000110) /* Select which inputs from P3 port can trigger wkup counter */
#define WKUP_SELECT_P3_REG_RESET             (0x00000000)

/*=============================*/
struct __WKUP_SELECT_P3_REG
/*=============================*/
{
    volatile uint16_t BITFLD_WKUP_SELECT_P3                   : 8;
};
#define WKUP_SELECT_P3                       (0x00FF)



#define WKUP_POL_P0_REG                      (0x50000112) /* Select the sensitivity polarity for each P0 input */
#define WKUP_POL_P0_REG_RESET                (0x00000000)

/*=============================*/
struct __WKUP_POL_P0_REG
/*=============================*/
{
    volatile uint16_t BITFLD_WKUP_POL_P0                      : 8;
};
#define WKUP_POL_P0                          (0x00FF)



#define WKUP_POL_P1_REG                      (0x50000114) /* Select the sensitivity polarity for each P1 input */
#define WKUP_POL_P1_REG_RESET                (0x00000000)

/*=============================*/
struct __WKUP_POL_P1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_WKUP_POL_P1                      : 6;
};
#define WKUP_POL_P1                          (0x003F)



#define WKUP_POL_P2_REG                      (0x50000116) /* Select the sensitivity polarity for each P2 input */
#define WKUP_POL_P2_REG_RESET                (0x00000000)

/*=============================*/
struct __WKUP_POL_P2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_WKUP_POL_P2                      : 10;
};
#define WKUP_POL_P2                          (0x03FF)



#define WKUP_POL_P3_REG                      (0x50000118) /* Select the sensitivity polarity for each P3 input */
#define WKUP_POL_P3_REG_RESET                (0x00000000)

/*=============================*/
struct __WKUP_POL_P3_REG
/*=============================*/
{
    volatile uint16_t BITFLD_WKUP_POL_P3                      : 8;
};
#define WKUP_POL_P3                          (0x00FF)



/*===========================================================================*/
/* Test mode spec map */
/*===========================================================================*/



#define OTPC_TEST_REG                        (0x07F40030) /* Test register for the control/status of the SECDED logic */
#define OTPC_TEST_REG_RESET                  (0x00000000)

/*=============================*/
struct __OTPC_TEST_REG
/*=============================*/
{
    volatile uint32_t BITFLD_OTPC_SECDED_COR_DIS              : 1;
    volatile uint32_t                                         : 15;
    volatile uint32_t BITFLD_OTPC_SECDED_RAW_ECC              : 8;
    volatile uint32_t BITFLD_OTPC_SECDED_STAT                 : 2;
    volatile uint32_t BITFLD_OTPC_USED_RR                     : 1;
};
#define OTPC_SECDED_COR_DIS                  (0x0001)
#define OTPC_SECDED_RAW_ECC                  (0xFF0000)
#define OTPC_SECDED_STAT                     (0x3000000)
#define OTPC_USED_RR                         (0x4000000)



#define PATCH_VALID_REG                      (0x40080000) /* Validity Control Register */
#define PATCH_VALID_REG_RESET                (0x00000000)

/*=============================*/
struct __PATCH_VALID_REG
/*=============================*/
{
    volatile uint32_t BITFLD_PATCH_VALID                      : 22;
};
#define PATCH_VALID                          (0x3FFFFF)



#define PATCH_ADDR0_REG                      (0x40080020) /* Patch Address field */
#define PATCH_ADDR0_REG_RESET                (0x07F00000)

/*=============================*/
struct __PATCH_ADDR0_REG
/*=============================*/
{
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_PATCH_ADDR_C                     : 16;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_C                         (0x1FFFE)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_ADDR1_REG                      (0x40080028) /* Patch Address field */
#define PATCH_ADDR1_REG_RESET                (0x07F00000)

/*=============================*/
struct __PATCH_ADDR1_REG
/*=============================*/
{
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_PATCH_ADDR_C                     : 16;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_C                         (0x1FFFE)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_ADDR2_REG                      (0x40080030) /* Patch Address field */
#define PATCH_ADDR2_REG_RESET                (0x07F00000)

/*=============================*/
struct __PATCH_ADDR2_REG
/*=============================*/
{
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_PATCH_ADDR_C                     : 16;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_C                         (0x1FFFE)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_ADDR3_REG                      (0x40080038) /* Patch Address field */
#define PATCH_ADDR3_REG_RESET                (0x07F00000)

/*=============================*/
struct __PATCH_ADDR3_REG
/*=============================*/
{
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_PATCH_ADDR_C                     : 16;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_C                         (0x1FFFE)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_ADDR4_REG                      (0x40080040) /* Patch Address field */
#define PATCH_ADDR4_REG_RESET                (0x07F00000)

/*=============================*/
struct __PATCH_ADDR4_REG
/*=============================*/
{
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_PATCH_ADDR_C                     : 16;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_C                         (0x1FFFE)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_ADDR5_REG                      (0x40080048) /* Patch Address field */
#define PATCH_ADDR5_REG_RESET                (0x07F00000)

/*=============================*/
struct __PATCH_ADDR5_REG
/*=============================*/
{
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_PATCH_ADDR_C                     : 16;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_C                         (0x1FFFE)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_ADDR6_REG                      (0x40080050) /* Patch Address field */
#define PATCH_ADDR6_REG_RESET                (0x07F00000)

/*=============================*/
struct __PATCH_ADDR6_REG
/*=============================*/
{
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_PATCH_ADDR_C                     : 16;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_C                         (0x1FFFE)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_ADDR7_REG                      (0x40080058) /* Patch Address field */
#define PATCH_ADDR7_REG_RESET                (0x07F00000)

/*=============================*/
struct __PATCH_ADDR7_REG
/*=============================*/
{
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_PATCH_ADDR_C                     : 16;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_C                         (0x1FFFE)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_ADDR8_REG                      (0x40080060) /* Patch Address field */
#define PATCH_ADDR8_REG_RESET                (0x07F00000)

/*=============================*/
struct __PATCH_ADDR8_REG
/*=============================*/
{
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_PATCH_ADDR_C                     : 16;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_C                         (0x1FFFE)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_ADDR9_REG                      (0x40080068) /* Patch Address field */
#define PATCH_ADDR9_REG_RESET                (0x07F00000)

/*=============================*/
struct __PATCH_ADDR9_REG
/*=============================*/
{
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_PATCH_ADDR_C                     : 16;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_C                         (0x1FFFE)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_ADDR10_REG                     (0x40080070) /* Patch Address field */
#define PATCH_ADDR10_REG_RESET               (0x07F00000)

/*=============================*/
struct __PATCH_ADDR10_REG
/*=============================*/
{
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_PATCH_ADDR_C                     : 16;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_C                         (0x1FFFE)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_ADDR11_REG                     (0x40080078) /* Patch Address field */
#define PATCH_ADDR11_REG_RESET               (0x07F00000)

/*=============================*/
struct __PATCH_ADDR11_REG
/*=============================*/
{
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_PATCH_ADDR_C                     : 16;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_C                         (0x1FFFE)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_ADDR12_REG                     (0x40080080) /* Patch Address field */
#define PATCH_ADDR12_REG_RESET               (0x07F00000)

/*=============================*/
struct __PATCH_ADDR12_REG
/*=============================*/
{
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_PATCH_ADDR_C                     : 16;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_C                         (0x1FFFE)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_ADDR13_REG                     (0x40080088) /* Patch Address field */
#define PATCH_ADDR13_REG_RESET               (0x07F00000)

/*=============================*/
struct __PATCH_ADDR13_REG
/*=============================*/
{
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_PATCH_ADDR_C                     : 16;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_C                         (0x1FFFE)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_ADDR14_REG                     (0x40080090) /* Patch Address field */
#define PATCH_ADDR14_REG_RESET               (0x07F00000)

/*=============================*/
struct __PATCH_ADDR14_REG
/*=============================*/
{
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_PATCH_ADDR_C                     : 16;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_C                         (0x1FFFE)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_ADDR15_REG                     (0x40080098) /* Patch Address field */
#define PATCH_ADDR15_REG_RESET               (0x07F00000)

/*=============================*/
struct __PATCH_ADDR15_REG
/*=============================*/
{
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_PATCH_ADDR_C                     : 16;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_C                         (0x1FFFE)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_ADDR16_REG                     (0x400800A0) /* Patch Address field */
#define PATCH_ADDR16_REG_RESET               (0x07F00000)

/*=============================*/
struct __PATCH_ADDR16_REG
/*=============================*/
{
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_PATCH_ADDR_C                     : 16;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_C                         (0x1FFFE)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_ADDR17_REG                     (0x400800A8) /* Patch Address field */
#define PATCH_ADDR17_REG_RESET               (0x07F00000)

/*=============================*/
struct __PATCH_ADDR17_REG
/*=============================*/
{
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_PATCH_ADDR_C                     : 16;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_C                         (0x1FFFE)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_ADDR18_REG                     (0x400800B0) /* Patch Address field */
#define PATCH_ADDR18_REG_RESET               (0x07F00000)

/*=============================*/
struct __PATCH_ADDR18_REG
/*=============================*/
{
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_PATCH_ADDR_C                     : 16;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_C                         (0x1FFFE)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_ADDR19_REG                     (0x400800B8) /* Patch Address field */
#define PATCH_ADDR19_REG_RESET               (0x07F00000)

/*=============================*/
struct __PATCH_ADDR19_REG
/*=============================*/
{
    volatile uint32_t                                         : 1;
    volatile uint32_t BITFLD_PATCH_ADDR_C                     : 16;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_C                         (0x1FFFE)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_ADDR20_REG                     (0x400800C0) /* Patch Address field */
#define PATCH_ADDR20_REG_RESET               (0x07F00000)

/*=============================*/
struct __PATCH_ADDR20_REG
/*=============================*/
{
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_D                     : 15;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_D                         (0x1FFFC)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_DATA20_REG                     (0x400800C4) /* Patch Data field */
#define PATCH_DATA20_REG_RESET               (0x00000000)

/*=============================*/
struct __PATCH_DATA20_REG
/*=============================*/
{
    volatile uint32_t BITFLD_PATCH_DATA                       : 32;
};
#define PATCH_DATA                           (0xFFFFFFFF)



#define PATCH_ADDR21_REG                     (0x400800C8) /* Patch Address field */
#define PATCH_ADDR21_REG_RESET               (0x07F00000)

/*=============================*/
struct __PATCH_ADDR21_REG
/*=============================*/
{
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_D                     : 15;
    volatile uint32_t                                         : 2;
    volatile uint32_t BITFLD_PATCH_ADDR_19                    : 1;
};
#define PATCH_ADDR_D                         (0x1FFFC)
#define PATCH_ADDR_19                        (0x80000)



#define PATCH_DATA21_REG                     (0x400800CC) /* Patch Data field */
#define PATCH_DATA21_REG_RESET               (0x00000000)

/*=============================*/
struct __PATCH_DATA21_REG
/*=============================*/
{
    volatile uint32_t BITFLD_PATCH_DATA                       : 32;
};
#define PATCH_DATA                           (0xFFFFFFFF)



#define STARTUP_STATUS_REG                   (0x5000002C) /* Startup timeout status */
#define STARTUP_STATUS_REG_RESET             (0x00000000)

/*=============================*/
struct __STARTUP_STATUS_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SU_BANDGAP_NOK                   : 1;
    volatile uint16_t BITFLD_SU_VDCDC_NOK                     : 1;
    volatile uint16_t BITFLD_SU_BOOST_VBAT_NOK                : 1;
    volatile uint16_t BITFLD_SU_LDO_DIG_NOK                   : 1;
};
#define SU_BANDGAP_NOK                       (0x0001)
#define SU_VDCDC_NOK                         (0x0002)
#define SU_BOOST_VBAT_NOK                    (0x0004)
#define SU_LDO_DIG_NOK                       (0x0008)



#define RF_IO_CTRL1_REG                      (0x50000030) /* (in CRG) */
#define RF_IO_CTRL1_REG_RESET                (0x0000000F)

/*=============================*/
struct __RF_IO_CTRL1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RFIO_TRIM1_CAP                   : 8;
};
#define RFIO_TRIM1_CAP                       (0x00FF)



#define RF_LNA_CTRL1_REG                     (0x50000032) /* (in CRG) */
#define RF_LNA_CTRL1_REG_RESET               (0x0000024E)

/*=============================*/
struct __RF_LNA_CTRL1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_LNA_TRIM_CD_LF                   : 6;
    volatile uint16_t BITFLD_LNA_TRIM_CD_HF                   : 6;
};
#define LNA_TRIM_CD_LF                       (0x003F)
#define LNA_TRIM_CD_HF                       (0x0FC0)



#define RF_LNA_CTRL2_REG                     (0x50000034) /* (in CRG) */
#define RF_LNA_CTRL2_REG_RESET               (0x00000226)

/*=============================*/
struct __RF_LNA_CTRL2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_LNA_TRIM_GM_HI                   : 6;
    volatile uint16_t BITFLD_LNA_TRIM_GM_LO                   : 6;
};
#define LNA_TRIM_GM_HI                       (0x003F)
#define LNA_TRIM_GM_LO                       (0x0FC0)



#define RF_LNA_CTRL3_REG                     (0x50000036) /* (in CRG) */
#define RF_LNA_CTRL3_REG_RESET               (0x00000007)

/*=============================*/
struct __RF_LNA_CTRL3_REG
/*=============================*/
{
    volatile uint16_t BITFLD_LNA_TRIM_CGS                     : 5;
};
#define LNA_TRIM_CGS                         (0x001F)



#define RF_RSSI_COMP_CTRL_REG                (0x50000038) /* (in CRG) */
#define RF_RSSI_COMP_CTRL_REG_RESET          (0x00008777)

/*=============================*/
struct __RF_RSSI_COMP_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RSSI_COMP01                      : 4;
    volatile uint16_t BITFLD_RSSI_COMP10                      : 4;
    volatile uint16_t BITFLD_RSSI_COMP11                      : 4;
    volatile uint16_t BITFLD_RSSI_COMP00                      : 4;
};
#define RSSI_COMP01                          (0x000F)
#define RSSI_COMP10                          (0x00F0)
#define RSSI_COMP11                          (0x0F00)
#define RSSI_COMP00                          (0xF000)



#define RF_VCO_CTRL_REG                      (0x5000003A) /*  */
#define RF_VCO_CTRL_REG_RESET                (0x00000001)

/*=============================*/
struct __RF_VCO_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_VCO_AMPL_SET                     : 4;
};
#define VCO_AMPL_SET                         (0x000F)



#define LDOS_DISABLE_REG                     (0x5000007E) /* Force off all LDOs */
#define LDOS_DISABLE_REG_RESET               (0x00000000)

/*=============================*/
struct __LDOS_DISABLE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_LDO_DIG_VBAT_DISABLE             : 1;
    volatile uint16_t BITFLD_LDO_DIG_VDCDC_DISABLE            : 1;
};
#define LDO_DIG_VBAT_DISABLE                 (0x0001)
#define LDO_DIG_VDCDC_DISABLE                (0x0002)



#define DCDC_CTRL_REG                        (0x50000080) /* DCDC control register */
#define DCDC_CTRL_REG_RESET                  (0x0000000C)

/*=============================*/
struct __DCDC_CTRL_REG
/*=============================*/
{
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_DCDC_VBAT1V_LEV                  : 3;
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_DCDC_MODE                        : 3;
    volatile uint16_t BITFLD_DCDC_DRIVE_NSW                   : 2;
    volatile uint16_t BITFLD_DCDC_DRIVE_PSW                   : 2;
    volatile uint16_t BITFLD_DCDC_DRIVE_OSW                   : 2;
    volatile uint16_t BITFLD_DCDC_TUNE                        : 2;
};
#define DCDC_VBAT1V_LEV                      (0x000E)
#define DCDC_MODE                            (0x00E0)
#define DCDC_DRIVE_NSW                       (0x0300)
#define DCDC_DRIVE_PSW                       (0x0C00)
#define DCDC_DRIVE_OSW                       (0x3000)
#define DCDC_TUNE                            (0xC000)



#define DCDC_CTRL2_REG                       (0x50000082) /* DCDC second control register */
#define DCDC_CTRL2_REG_RESET                 (0x00008C20)

/*=============================*/
struct __DCDC_CTRL2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DCDC_AUTO_CAL                    : 3;
    volatile uint16_t BITFLD_DCDC_CUR_LIM                     : 4;
    volatile uint16_t BITFLD_DCDC_TON                         : 2;
    volatile uint16_t BITFLD_DCDC_VBAT3V_LEV                  : 3;
    volatile uint16_t BITFLD_DCDC_VOLT_LEV                    : 4;
};
#define DCDC_AUTO_CAL                        (0x0007)
#define DCDC_CUR_LIM                         (0x0078)
#define DCDC_TON                             (0x0180)
#define DCDC_VBAT3V_LEV                      (0x0E00)
#define DCDC_VOLT_LEV                        (0xF000)



#define DCDC_CTRL3_REG                       (0x50000084) /* DCDC thirth control register */
#define DCDC_CTRL3_REG_RESET                 (0x00000015)

/*=============================*/
struct __DCDC_CTRL3_REG
/*=============================*/
{
    volatile uint16_t BITFLD_BUCK_ENABLE                      : 1;
    volatile uint16_t BITFLD_DCDC_IDLE_CLK                    : 2;
    volatile uint16_t BITFLD_DCDC_TIMEOUT                     : 2;
};
#define BUCK_ENABLE                          (0x0001)
#define DCDC_IDLE_CLK                        (0x0006)
#define DCDC_TIMEOUT                         (0x0018)



#define DCDC_CAL1_REG                        (0x50000086) /* DCDC first calibration register */
#define DCDC_CAL1_REG_RESET                  (0x00000000)

/*=============================*/
struct __DCDC_CAL1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DCDC_CAL1                        : 6;
};
#define DCDC_CAL1                            (0x003F)



#define DCDC_CAL2_REG                        (0x50000088) /* DCDC second calibration register */
#define DCDC_CAL2_REG_RESET                  (0x00000000)

/*=============================*/
struct __DCDC_CAL2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DCDC_CAL2                        : 6;
};
#define DCDC_CAL2                            (0x003F)



#define DCDC_CAL3_REG                        (0x5000008A) /* DCDC thirth calibration register */
#define DCDC_CAL3_REG_RESET                  (0x00000000)

/*=============================*/
struct __DCDC_CAL3_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DCDC_CAL3                        : 6;
};
#define DCDC_CAL3                            (0x003F)



#define RF_BMCW_REG                          (0x50002000) /*  */
#define RF_BMCW_REG_RESET                    (0x00000000)

/*=============================*/
struct __RF_BMCW_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CN_WR                            : 8;
    volatile uint16_t BITFLD_CN_SEL                           : 1;
};
#define CN_WR                                (0x00FF)
#define CN_SEL                               (0x0100)



#define RF_OVERRULE_REG                      (0x50002020) /*  */
#define RF_OVERRULE_REG_RESET                (0x00000000)

/*=============================*/
struct __RF_OVERRULE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_TX_DIS_WR                        : 1;
    volatile uint16_t BITFLD_TX_EN_WR                         : 1;
    volatile uint16_t BITFLD_RX_DIS_WR                        : 1;
    volatile uint16_t BITFLD_RX_EN_WR                         : 1;
};
#define TX_DIS_WR                            (0x0001)
#define TX_EN_WR                             (0x0002)
#define RX_DIS_WR                            (0x0004)
#define RX_EN_WR                             (0x0008)



#define RF_DIAGIRQ01_REG                     (0x50002022) /* BLE/RF Diagnostic IRQ Control Register */
#define RF_DIAGIRQ01_REG_RESET               (0x00000000)

/*=============================*/
struct __RF_DIAGIRQ01_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DIAGIRQ_MASK_0                   : 1;
    volatile uint16_t BITFLD_DIAGIRQ_WSEL_0                   : 2;
    volatile uint16_t BITFLD_DIAGIRQ_BSEL_0                   : 3;
    volatile uint16_t BITFLD_DIAGIRQ_EDGE_0                   : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_DIAGIRQ_MASK_1                   : 1;
    volatile uint16_t BITFLD_DIAGIRQ_WSEL_1                   : 2;
    volatile uint16_t BITFLD_DIAGIRQ_BSEL_1                   : 3;
    volatile uint16_t BITFLD_DIAGIRQ_EDGE_1                   : 1;
};
#define DIAGIRQ_MASK_0                       (0x0001)
#define DIAGIRQ_WSEL_0                       (0x0006)
#define DIAGIRQ_BSEL_0                       (0x0038)
#define DIAGIRQ_EDGE_0                       (0x0040)
#define DIAGIRQ_MASK_1                       (0x0100)
#define DIAGIRQ_WSEL_1                       (0x0600)
#define DIAGIRQ_BSEL_1                       (0x3800)
#define DIAGIRQ_EDGE_1                       (0x4000)



#define RF_DIAGIRQ23_REG                     (0x50002024) /* BLE/RF Diagnostic IRQ Control Register */
#define RF_DIAGIRQ23_REG_RESET               (0x00000000)

/*=============================*/
struct __RF_DIAGIRQ23_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DIAGIRQ_MASK_2                   : 1;
    volatile uint16_t BITFLD_DIAGIRQ_WSEL_2                   : 2;
    volatile uint16_t BITFLD_DIAGIRQ_BSEL_2                   : 3;
    volatile uint16_t BITFLD_DIAGIRQ_EDGE_2                   : 1;
    volatile uint16_t                                         : 1;
    volatile uint16_t BITFLD_DIAGIRQ_MASK_3                   : 1;
    volatile uint16_t BITFLD_DIAGIRQ_WSEL_3                   : 2;
    volatile uint16_t BITFLD_DIAGIRQ_BSEL_3                   : 3;
    volatile uint16_t BITFLD_DIAGIRQ_EDGE_3                   : 1;
};
#define DIAGIRQ_MASK_2                       (0x0001)
#define DIAGIRQ_WSEL_2                       (0x0006)
#define DIAGIRQ_BSEL_2                       (0x0038)
#define DIAGIRQ_EDGE_2                       (0x0040)
#define DIAGIRQ_MASK_3                       (0x0100)
#define DIAGIRQ_WSEL_3                       (0x0600)
#define DIAGIRQ_BSEL_3                       (0x3800)
#define DIAGIRQ_EDGE_3                       (0x4000)



#define RF_DIAGIRQ_STAT_REG                  (0x50002026) /* BLE/RF Diagnostic IRQ Status Register */
#define RF_DIAGIRQ_STAT_REG_RESET            (0x00000000)

/*=============================*/
struct __RF_DIAGIRQ_STAT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DIAGIRQ_STAT_0                   : 1;
    volatile uint16_t BITFLD_DIAGIRQ_STAT_1                   : 1;
    volatile uint16_t BITFLD_DIAGIRQ_STAT_2                   : 1;
    volatile uint16_t BITFLD_DIAGIRQ_STAT_3                   : 1;
};
#define DIAGIRQ_STAT_0                       (0x0001)
#define DIAGIRQ_STAT_1                       (0x0002)
#define DIAGIRQ_STAT_2                       (0x0004)
#define DIAGIRQ_STAT_3                       (0x0008)



#define RF_CALSTATE_REG                      (0x50002040) /*  */
#define RF_CALSTATE_REG_RESET                (0x00000000)

/*=============================*/
struct __RF_CALSTATE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CALSTATE                         : 4;
};
#define CALSTATE                             (0x000F)



#define RF_CALCAP1_REG                       (0x50002060) /*  */
#define RF_CALCAP1_REG_RESET                 (0x0000502A)

/*=============================*/
struct __RF_CALCAP1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_VCO_CALCAP_LOW                   : 16;
};
#define VCO_CALCAP_LOW                       (0xFFFF)



#define RF_CALCAP2_REG                       (0x50002062) /*  */
#define RF_CALCAP2_REG_RESET                 (0x00000001)

/*=============================*/
struct __RF_CALCAP2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_VCO_CALCAP_HIGH                  : 2;
};
#define VCO_CALCAP_HIGH                      (0x0003)



#define RF_SCAN_FEEDBACK_REG                 (0x50002080) /*  */
#define RF_SCAN_FEEDBACK_REG_RESET           (0x00000000)

/*=============================*/
struct __RF_SCAN_FEEDBACK_REG
/*=============================*/
{
    volatile uint16_t BITFLD_LF_RES                           : 4;
    volatile uint16_t BITFLD_CP_CUR                           : 4;
};
#define LF_RES                               (0x000F)
#define CP_CUR                               (0x00F0)



#define RF_CAL_CTRL_REG                      (0x50002200) /*  */
#define RF_CAL_CTRL_REG_RESET                (0x00000000)

/*=============================*/
struct __RF_CAL_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SO_CAL                           : 1;
    volatile uint16_t BITFLD_EO_CAL                           : 1;
    volatile uint16_t BITFLD_MGAIN_CAL_DIS                    : 1;
    volatile uint16_t BITFLD_IFF_CAL_DIS                      : 1;
    volatile uint16_t BITFLD_DC_OFFSET_CAL_DIS                : 1;
    volatile uint16_t BITFLD_VCO_CAL_DIS                      : 1;
};
#define SO_CAL                               (0x0001)
#define EO_CAL                               (0x0002)
#define MGAIN_CAL_DIS                        (0x0004)
#define IFF_CAL_DIS                          (0x0008)
#define DC_OFFSET_CAL_DIS                    (0x0010)
#define VCO_CAL_DIS                          (0x0020)



#define RF_REF_OSC_REG                       (0x50002202) /*  */
#define RF_REF_OSC_REG_RESET                 (0x000029AC)

/*=============================*/
struct __RF_REF_OSC_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CNT_RO                           : 6;
    volatile uint16_t BITFLD_CNT_CLK                          : 9;
};
#define CNT_RO                               (0x003F)
#define CNT_CLK                              (0x7FC0)



#define RF_IRQ_CTRL_REG                      (0x50002204) /*  */
#define RF_IRQ_CTRL_REG_RESET                (0x00000001)

/*=============================*/
struct __RF_IRQ_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_EO_CAL_CLEAR                     : 1;
};
#define EO_CAL_CLEAR                         (0x0001)



#define RF_IFF_RESULT_REG                    (0x50002300) /* Must be Retained */
#define RF_IFF_RESULT_REG_RESET              (0x00000010)

/*=============================*/
struct __RF_IFF_RESULT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_IF_CAL_CAP_RD                    : 5;
};
#define IF_CAL_CAP_RD                        (0x001F)



#define RF_ADCI_DC_OFFSET_REG                (0x50002310) /* Must be Retained */
#define RF_ADCI_DC_OFFSET_REG_RESET          (0x00008080)

/*=============================*/
struct __RF_ADCI_DC_OFFSET_REG
/*=============================*/
{
    volatile uint16_t BITFLD_ADC_OFFP_I_RD                    : 8;
    volatile uint16_t BITFLD_ADC_OFFN_I_RD                    : 8;
};
#define ADC_OFFP_I_RD                        (0x00FF)
#define ADC_OFFN_I_RD                        (0xFF00)



#define RF_ADCQ_DC_OFFSET_REG                (0x50002312) /* Must be Retained */
#define RF_ADCQ_DC_OFFSET_REG_RESET          (0x00008080)

/*=============================*/
struct __RF_ADCQ_DC_OFFSET_REG
/*=============================*/
{
    volatile uint16_t BITFLD_ADC_OFFP_Q_RD                    : 8;
    volatile uint16_t BITFLD_ADC_OFFN_Q_RD                    : 8;
};
#define ADC_OFFP_Q_RD                        (0x00FF)
#define ADC_OFFN_Q_RD                        (0xFF00)



#define RF_DC_OFFSET_RESULT_REG              (0x50002314) /* Must be Retained */
#define RF_DC_OFFSET_RESULT_REG_RESET        (0x00000000)

/*=============================*/
struct __RF_DC_OFFSET_RESULT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DCOFFSET_I_RD                    : 8;
    volatile uint16_t BITFLD_DCOFFSET_Q_RD                    : 8;
};
#define DCOFFSET_I_RD                        (0x00FF)
#define DCOFFSET_Q_RD                        (0xFF00)



#define RF_SYNTH_RESULT_REG                  (0x50002316) /* Must be Retained */
#define RF_SYNTH_RESULT_REG_RESET            (0x00000880)

/*=============================*/
struct __RF_SYNTH_RESULT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_GAUSS_GAIN_CAL_RD                : 8;
    volatile uint16_t BITFLD_VCO_FREQTRIM_RD                  : 4;
};
#define GAUSS_GAIN_CAL_RD                    (0x00FF)
#define VCO_FREQTRIM_RD                      (0x0F00)



#define RF_SYNTH_RESULT2_REG                 (0x50002318) /* Must be Retained */
#define RF_SYNTH_RESULT2_REG_RESET           (0x00001490)

/*=============================*/
struct __RF_SYNTH_RESULT2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_GAUSS_GAIN_RD                    : 8;
    volatile uint16_t BITFLD_CN_CAL_RD                        : 7;
};
#define GAUSS_GAIN_RD                        (0x00FF)
#define CN_CAL_RD                            (0x7F00)



#define RF_SYNTH_RESULT3_REG                 (0x5000231A) /*  */
#define RF_SYNTH_RESULT3_REG_RESET           (0x0000FFFF)

/*=============================*/
struct __RF_SYNTH_RESULT3_REG
/*=============================*/
{
    volatile uint16_t BITFLD_MDSTATE_RD                       : 16;
};
#define MDSTATE_RD                           (0xFFFF)



#define RF_CALTRIM_STEP1_REG                 (0x5000231C) /*  */
#define RF_CALTRIM_STEP1_REG_RESET           (0x00000000)

/*=============================*/
struct __RF_CALTRIM_STEP1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_MDSTATE_RD                       : 16;
};
#define MDSTATE_RD                           (0xFFFF)



#define RF_CALTRIM_STEP2_REG                 (0x5000231E) /*  */
#define RF_CALTRIM_STEP2_REG_RESET           (0x00000000)

/*=============================*/
struct __RF_CALTRIM_STEP2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_MDSTATE_RD                       : 16;
};
#define MDSTATE_RD                           (0xFFFF)



#define RF_CALTRIM_STEP3_REG                 (0x50002320) /*  */
#define RF_CALTRIM_STEP3_REG_RESET           (0x00000000)

/*=============================*/
struct __RF_CALTRIM_STEP3_REG
/*=============================*/
{
    volatile uint16_t BITFLD_MDSTATE_RD                       : 16;
};
#define MDSTATE_RD                           (0xFFFF)



#define RF_ENABLE_CONFIG1_REG                (0x50002400) /*  */
#define RF_ENABLE_CONFIG1_REG_RESET          (0x00000909)

/*=============================*/
struct __RF_ENABLE_CONFIG1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_lna_core_en                      : 8;
    volatile uint16_t BITFLD_lna_ldo_en                       : 8;
};
#define lna_core_en                          (0x00FF)
#define lna_ldo_en                           (0xFF00)



#define RF_ENABLE_CONFIG2_REG                (0x50002402) /*  */
#define RF_ENABLE_CONFIG2_REG_RESET          (0x00000922)

/*=============================*/
struct __RF_ENABLE_CONFIG2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_mix_ldo_en                       : 8;
    volatile uint16_t BITFLD_lna_cgm_en                       : 8;
};
#define mix_ldo_en                           (0x00FF)
#define lna_cgm_en                           (0xFF00)



#define RF_ENABLE_CONFIG3_REG                (0x50002404) /*  */
#define RF_ENABLE_CONFIG3_REG_RESET          (0x00002204)

/*=============================*/
struct __RF_ENABLE_CONFIG3_REG
/*=============================*/
{
    volatile uint16_t BITFLD_ifadc_ldo_en                     : 8;
    volatile uint16_t BITFLD_iff_ldo_en                       : 8;
};
#define ifadc_ldo_en                         (0x00FF)
#define iff_ldo_en                           (0xFF00)



#define RF_ENABLE_CONFIG4_REG                (0x50002406) /*  */
#define RF_ENABLE_CONFIG4_REG_RESET          (0x00002222)

/*=============================*/
struct __RF_ENABLE_CONFIG4_REG
/*=============================*/
{
    volatile uint16_t BITFLD_md_ldo_en                        : 8;
    volatile uint16_t BITFLD_vco_ldo_en                       : 8;
};
#define md_ldo_en                            (0x00FF)
#define vco_ldo_en                           (0xFF00)



#define RF_ENABLE_CONFIG5_REG                (0x50002408) /*  */
#define RF_ENABLE_CONFIG5_REG_RESET          (0x00002222)

/*=============================*/
struct __RF_ENABLE_CONFIG5_REG
/*=============================*/
{
    volatile uint16_t BITFLD_pa_ldo_en                        : 8;
    volatile uint16_t BITFLD_pfd_ldo_en                       : 8;
};
#define pa_ldo_en                            (0x00FF)
#define pfd_ldo_en                           (0xFF00)



#define RF_ENABLE_CONFIG6_REG                (0x5000240A) /*  */
#define RF_ENABLE_CONFIG6_REG_RESET          (0x00000022)

/*=============================*/
struct __RF_ENABLE_CONFIG6_REG
/*=============================*/
{
    volatile uint16_t BITFLD_vco_bias_en                      : 8;
    volatile uint16_t BITFLD_cp_switch_en                     : 8;
};
#define vco_bias_en                          (0x00FF)
#define cp_switch_en                         (0xFF00)



#define RF_ENABLE_CONFIG7_REG                (0x5000240C) /*  */
#define RF_ENABLE_CONFIG7_REG_RESET          (0x00003300)

/*=============================*/
struct __RF_ENABLE_CONFIG7_REG
/*=============================*/
{
    volatile uint16_t BITFLD_lna_ldo_zero                     : 8;
    volatile uint16_t BITFLD_cp_bias_en                       : 8;
};
#define lna_ldo_zero                         (0x00FF)
#define cp_bias_en                           (0xFF00)



#define RF_ENABLE_CONFIG8_REG                (0x5000240E) /*  */
#define RF_ENABLE_CONFIG8_REG_RESET          (0x0000C0A0)

/*=============================*/
struct __RF_ENABLE_CONFIG8_REG
/*=============================*/
{
    volatile uint16_t BITFLD_pa_en                            : 8;
    volatile uint16_t BITFLD_pa_ramp_en                       : 8;
};
#define pa_en                                (0x00FF)
#define pa_ramp_en                           (0xFF00)



#define RF_ENABLE_CONFIG9_REG                (0x50002410) /*  */
#define RF_ENABLE_CONFIG9_REG_RESET          (0x00000204)

/*=============================*/
struct __RF_ENABLE_CONFIG9_REG
/*=============================*/
{
    volatile uint16_t BITFLD_iff_en                           : 8;
    volatile uint16_t BITFLD_mix_en                           : 8;
};
#define iff_en                               (0x00FF)
#define mix_en                               (0xFF00)



#define RF_ENABLE_CONFIG10_REG               (0x50002412) /*  */
#define RF_ENABLE_CONFIG10_REG_RESET         (0x00000422)

/*=============================*/
struct __RF_ENABLE_CONFIG10_REG
/*=============================*/
{
    volatile uint16_t BITFLD_vco_en                           : 8;
    volatile uint16_t BITFLD_adc_en                           : 8;
};
#define vco_en                               (0x00FF)
#define adc_en                               (0xFF00)



#define RF_ENABLE_CONFIG11_REG               (0x50002414) /*  */
#define RF_ENABLE_CONFIG11_REG_RESET         (0x00003333)

/*=============================*/
struct __RF_ENABLE_CONFIG11_REG
/*=============================*/
{
    volatile uint16_t BITFLD_cp_en                            : 8;
    volatile uint16_t BITFLD_md_lobuf_en                      : 8;
};
#define cp_en                                (0x00FF)
#define md_lobuf_en                          (0xFF00)



#define RF_ENABLE_CONFIG12_REG               (0x50002416) /*  */
#define RF_ENABLE_CONFIG12_REG_RESET         (0x00003330)

/*=============================*/
struct __RF_ENABLE_CONFIG12_REG
/*=============================*/
{
    volatile uint16_t BITFLD_gauss_en                         : 8;
    volatile uint16_t BITFLD_pfd_en                           : 8;
};
#define gauss_en                             (0x00FF)
#define pfd_en                               (0xFF00)



#define RF_ENABLE_CONFIG13_REG               (0x50002418) /*  */
#define RF_ENABLE_CONFIG13_REG_RESET         (0x0000D030)

/*=============================*/
struct __RF_ENABLE_CONFIG13_REG
/*=============================*/
{
    volatile uint16_t BITFLD_lobuf_pa_en                      : 8;
    volatile uint16_t BITFLD_rfio_en                          : 8;
};
#define lobuf_pa_en                          (0x00FF)
#define rfio_en                              (0xFF00)



#define RF_ENABLE_CONFIG14_REG               (0x5000241A) /*  */
#define RF_ENABLE_CONFIG14_REG_RESET         (0x00000433)

/*=============================*/
struct __RF_ENABLE_CONFIG14_REG
/*=============================*/
{
    volatile uint16_t BITFLD_div2_en                          : 8;
    volatile uint16_t BITFLD_lobuf_rxiq_en                    : 8;
};
#define div2_en                              (0x00FF)
#define lobuf_rxiq_en                        (0xFF00)



#define RF_ENABLE_CONFIG15_REG               (0x5000241C) /*  */
#define RF_ENABLE_CONFIG15_REG_RESET         (0x00005555)

/*=============================*/
struct __RF_ENABLE_CONFIG15_REG
/*=============================*/
{
    volatile uint16_t BITFLD_vco_bias_sh_open_en              : 8;
    volatile uint16_t BITFLD_cp_bias_sh_open                  : 8;
};
#define vco_bias_sh_open_en                  (0x00FF)
#define cp_bias_sh_open                      (0xFF00)



#define RF_ENABLE_CONFIG16_REG               (0x5000241E) /*  */
#define RF_ENABLE_CONFIG16_REG_RESET         (0x00000650)

/*=============================*/
struct __RF_ENABLE_CONFIG16_REG
/*=============================*/
{
    volatile uint16_t BITFLD_gauss_bias_sh_open_en            : 8;
    volatile uint16_t BITFLD_iff_bias_sh_open_en              : 8;
};
#define gauss_bias_sh_open_en                (0x00FF)
#define iff_bias_sh_open_en                  (0xFF00)



#define RF_ENABLE_CONFIG17_REG               (0x50002420) /*  */
#define RF_ENABLE_CONFIG17_REG_RESET         (0x00000633)

/*=============================*/
struct __RF_ENABLE_CONFIG17_REG
/*=============================*/
{
    volatile uint16_t BITFLD_plldig_en                        : 8;
    volatile uint16_t BITFLD_mix_bias_sh_open_en              : 8;
};
#define plldig_en                            (0x00FF)
#define mix_bias_sh_open_en                  (0xFF00)



#define RF_ENABLE_CONFIG18_REG               (0x50002422) /*  */
#define RF_ENABLE_CONFIG18_REG_RESET         (0x00007708)

/*=============================*/
struct __RF_ENABLE_CONFIG18_REG
/*=============================*/
{
    volatile uint16_t BITFLD_dem_en                           : 8;
    volatile uint16_t BITFLD_pllclosed_en                     : 8;
};
#define dem_en                               (0x00FF)
#define pllclosed_en                         (0xFF00)



#define RF_ENABLE_CONFIG19_REG               (0x50002424) /*  */
#define RF_ENABLE_CONFIG19_REG_RESET         (0x000011EE)

/*=============================*/
struct __RF_ENABLE_CONFIG19_REG
/*=============================*/
{
    volatile uint16_t BITFLD_cal_en                           : 8;
    volatile uint16_t BITFLD_ldo_zero_en                      : 8;
};
#define cal_en                               (0x00FF)
#define ldo_zero_en                          (0xFF00)



#define RF_ENABLE_CONFIG20_REG               (0x50002426) /*  */
#define RF_ENABLE_CONFIG20_REG_RESET         (0x00000022)

/*=============================*/
struct __RF_ENABLE_CONFIG20_REG
/*=============================*/
{
    volatile uint16_t BITFLD_ldo_rfio_en                      : 8;
    volatile uint16_t BITFLD_tdc_en                           : 8;
};
#define ldo_rfio_en                          (0x00FF)
#define tdc_en                               (0xFF00)



#define RF_ENABLE_CONFIG21_REG               (0x50002428) /*  */
#define RF_ENABLE_CONFIG21_REG_RESET         (0x0000A0B0)

/*=============================*/
struct __RF_ENABLE_CONFIG21_REG
/*=============================*/
{
    volatile uint16_t BITFLD_rfio_bias_sh_open                : 8;
    volatile uint16_t BITFLD_rfio_bias_en                     : 8;
};
#define rfio_bias_sh_open                    (0x00FF)
#define rfio_bias_en                         (0xFF00)



#define RF_ENABLE_CONFIG22_REG               (0x5000242A) /*  */
#define RF_ENABLE_CONFIG22_REG_RESET         (0x00002208)

/*=============================*/
struct __RF_ENABLE_CONFIG22_REG
/*=============================*/
{
    volatile uint16_t BITFLD_adc_clk_en                       : 8;
    volatile uint16_t BITFLD_ldo_radio_en                     : 8;
};
#define adc_clk_en                           (0x00FF)
#define ldo_radio_en                         (0xFF00)



#define RF_ENABLE_CONFIG23_REG               (0x5000242C) /*  */
#define RF_ENABLE_CONFIG23_REG_RESET         (0x00000000)

/*=============================*/
struct __RF_ENABLE_CONFIG23_REG
/*=============================*/
{
    volatile uint16_t BITFLD_spare_en_3                       : 8;
    volatile uint16_t BITFLD_tr_pwm_off_en                    : 8;
};
#define spare_en_3                           (0x00FF)
#define tr_pwm_off_en                        (0xFF00)



#define RF_CNTRL_TIMER_1_REG                 (0x50002500) /*  */
#define RF_CNTRL_TIMER_1_REG_RESET           (0x00000800)

/*=============================*/
struct __RF_CNTRL_TIMER_1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SET_OFFSET                       : 8;
    volatile uint16_t BITFLD_RESET_OFFSET                     : 8;
};
#define SET_OFFSET                           (0x00FF)
#define RESET_OFFSET                         (0xFF00)



#define RF_CNTRL_TIMER_2_REG                 (0x50002502) /*  */
#define RF_CNTRL_TIMER_2_REG_RESET           (0x00000508)

/*=============================*/
struct __RF_CNTRL_TIMER_2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SET_OFFSET                       : 8;
    volatile uint16_t BITFLD_RESET_OFFSET                     : 8;
};
#define SET_OFFSET                           (0x00FF)
#define RESET_OFFSET                         (0xFF00)



#define RF_CNTRL_TIMER_3_REG                 (0x50002504) /*  */
#define RF_CNTRL_TIMER_3_REG_RESET           (0x00000410)

/*=============================*/
struct __RF_CNTRL_TIMER_3_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SET_OFFSET                       : 8;
    volatile uint16_t BITFLD_RESET_OFFSET                     : 8;
};
#define SET_OFFSET                           (0x00FF)
#define RESET_OFFSET                         (0xFF00)



#define RF_CNTRL_TIMER_4_REG                 (0x50002506) /*  */
#define RF_CNTRL_TIMER_4_REG_RESET           (0x0000022E)

/*=============================*/
struct __RF_CNTRL_TIMER_4_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SET_OFFSET                       : 8;
    volatile uint16_t BITFLD_RESET_OFFSET                     : 8;
};
#define SET_OFFSET                           (0x00FF)
#define RESET_OFFSET                         (0xFF00)



#define RF_CNTRL_TIMER_5_REG                 (0x50002508) /*  */
#define RF_CNTRL_TIMER_5_REG_RESET           (0x00000218)

/*=============================*/
struct __RF_CNTRL_TIMER_5_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SET_OFFSET                       : 8;
    volatile uint16_t BITFLD_RESET_OFFSET                     : 8;
};
#define SET_OFFSET                           (0x00FF)
#define RESET_OFFSET                         (0xFF00)



#define RF_CNTRL_TIMER_6_REG                 (0x5000250A) /*  */
#define RF_CNTRL_TIMER_6_REG_RESET           (0x00000232)

/*=============================*/
struct __RF_CNTRL_TIMER_6_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SET_OFFSET                       : 8;
    volatile uint16_t BITFLD_RESET_OFFSET                     : 8;
};
#define SET_OFFSET                           (0x00FF)
#define RESET_OFFSET                         (0xFF00)



#define RF_CNTRL_TIMER_7_REG                 (0x5000250C) /*  */
#define RF_CNTRL_TIMER_7_REG_RESET           (0x00000218)

/*=============================*/
struct __RF_CNTRL_TIMER_7_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SET_OFFSET                       : 8;
    volatile uint16_t BITFLD_RESET_OFFSET                     : 8;
};
#define SET_OFFSET                           (0x00FF)
#define RESET_OFFSET                         (0xFF00)



#define RF_CNTRL_TIMER_8_REG                 (0x5000250E) /*  */
#define RF_CNTRL_TIMER_8_REG_RESET           (0x0000023E)

/*=============================*/
struct __RF_CNTRL_TIMER_8_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SET_OFFSET                       : 8;
    volatile uint16_t BITFLD_RESET_OFFSET                     : 8;
};
#define SET_OFFSET                           (0x00FF)
#define RESET_OFFSET                         (0xFF00)



#define RF_CNTRL_TIMER_9_REG                 (0x50002510) /*  */
#define RF_CNTRL_TIMER_9_REG_RESET           (0x0000022E)

/*=============================*/
struct __RF_CNTRL_TIMER_9_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SET_OFFSET                       : 8;
    volatile uint16_t BITFLD_RESET_OFFSET                     : 8;
};
#define SET_OFFSET                           (0x00FF)
#define RESET_OFFSET                         (0xFF00)



#define RF_CNTRL_TIMER_10_REG                (0x50002512) /*  */
#define RF_CNTRL_TIMER_10_REG_RESET          (0x0000022E)

/*=============================*/
struct __RF_CNTRL_TIMER_10_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SET_OFFSET                       : 8;
    volatile uint16_t BITFLD_RESET_OFFSET                     : 8;
};
#define SET_OFFSET                           (0x00FF)
#define RESET_OFFSET                         (0xFF00)



#define RF_CNTRL_TIMER_11_REG                (0x50002514) /*  */
#define RF_CNTRL_TIMER_11_REG_RESET          (0x00000230)

/*=============================*/
struct __RF_CNTRL_TIMER_11_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SET_OFFSET                       : 8;
    volatile uint16_t BITFLD_RESET_OFFSET                     : 8;
};
#define SET_OFFSET                           (0x00FF)
#define RESET_OFFSET                         (0xFF00)



#define RF_CNTRL_TIMER_12_REG                (0x50002516) /*  */
#define RF_CNTRL_TIMER_12_REG_RESET          (0x00000239)

/*=============================*/
struct __RF_CNTRL_TIMER_12_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SET_OFFSET                       : 8;
    volatile uint16_t BITFLD_RESET_OFFSET                     : 8;
};
#define SET_OFFSET                           (0x00FF)
#define RESET_OFFSET                         (0xFF00)



#define RF_CNTRL_TIMER_13_REG                (0x50002518) /*  */
#define RF_CNTRL_TIMER_13_REG_RESET          (0x00000145)

/*=============================*/
struct __RF_CNTRL_TIMER_13_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SET_OFFSET                       : 8;
    volatile uint16_t BITFLD_RESET_OFFSET                     : 8;
};
#define SET_OFFSET                           (0x00FF)
#define RESET_OFFSET                         (0xFF00)



#define RF_CNTRL_TIMER_14_REG                (0x5000251A) /*  */
#define RF_CNTRL_TIMER_14_REG_RESET          (0x00002044)

/*=============================*/
struct __RF_CNTRL_TIMER_14_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SET_OFFSET                       : 8;
    volatile uint16_t BITFLD_RESET_OFFSET                     : 8;
};
#define SET_OFFSET                           (0x00FF)
#define RESET_OFFSET                         (0xFF00)



#define BIAS_CTRL1_REG                       (0x50002600) /*  */
#define BIAS_CTRL1_REG_RESET                 (0x00008888)

/*=============================*/
struct __BIAS_CTRL1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_MIX_BIAS_SET                     : 4;
    volatile uint16_t BITFLD_CP_BIAS_SET                      : 4;
    volatile uint16_t BITFLD_VCO_BIAS_SET                     : 4;
    volatile uint16_t BITFLD_IFF_BIAS_SET                     : 4;
};
#define MIX_BIAS_SET                         (0x000F)
#define CP_BIAS_SET                          (0x00F0)
#define VCO_BIAS_SET                         (0x0F00)
#define IFF_BIAS_SET                         (0xF000)



#define RF_SPARE1_REG                        (0x50002602) /*  */
#define RF_SPARE1_REG_RESET                  (0x00000000)

/*=============================*/
struct __RF_SPARE1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_IFF_REAL_MODE                    : 1;
    volatile uint16_t BITFLD_DCOC_DAC_REF_CUR_CTRL            : 2;
    volatile uint16_t BITFLD_RF_SPARE1                        : 13;
};
#define IFF_REAL_MODE                        (0x0001)
#define DCOC_DAC_REF_CUR_CTRL                (0x0006)
#define RF_SPARE1                            (0xFFF8)



#define RF_MIXER_CTRL1_REG                   (0x50002810) /*  */
#define RF_MIXER_CTRL1_REG_RESET             (0x00000035)

/*=============================*/
struct __RF_MIXER_CTRL1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_MIX_TRIM_GMBIAS                  : 4;
    volatile uint16_t BITFLD_MIX_TRIM_IBIAS                   : 4;
    volatile uint16_t BITFLD_MIX_TRIM_VCM                     : 4;
    volatile uint16_t BITFLD_MIX_SPARE                        : 4;
};
#define MIX_TRIM_GMBIAS                      (0x000F)
#define MIX_TRIM_IBIAS                       (0x00F0)
#define MIX_TRIM_VCM                         (0x0F00)
#define MIX_SPARE                            (0xF000)



#define RF_MIXER_CTRL2_REG                   (0x50002812) /*  */
#define RF_MIXER_CTRL2_REG_RESET             (0x0000000C)

/*=============================*/
struct __RF_MIXER_CTRL2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_MIX_CAL_CAP_WR                   : 5;
    volatile uint16_t BITFLD_MIX_CAL_CAP_SEL                  : 1;
};
#define MIX_CAL_CAP_WR                       (0x001F)
#define MIX_CAL_CAP_SEL                      (0x0020)



#define RF_IFF_CTRL1_REG                     (0x50002820) /*  */
#define RF_IFF_CTRL1_REG_RESET               (0x0000000C)

/*=============================*/
struct __RF_IFF_CTRL1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_IF_CAL_CAP_WR                    : 5;
    volatile uint16_t BITFLD_IF_CAL_CAP_SEL                   : 1;
    volatile uint16_t BITFLD_IF_MUTE                          : 1;
    volatile uint16_t BITFLD_RO_TO_PINS                       : 1;
    volatile uint16_t BITFLD_IFF_DCOC_DAC_DIS                 : 1;
};
#define IF_CAL_CAP_WR                        (0x001F)
#define IF_CAL_CAP_SEL                       (0x0020)
#define IF_MUTE                              (0x0040)
#define RO_TO_PINS                           (0x0080)
#define IFF_DCOC_DAC_DIS                     (0x0100)



#define RF_ADC_CTRL1_REG                     (0x50002830) /*  */
#define RF_ADC_CTRL1_REG_RESET               (0x00000000)

/*=============================*/
struct __RF_ADC_CTRL1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_ADC_DC_OFFSET_SEL                : 1;
    volatile uint16_t                                         : 12;
    volatile uint16_t BITFLD_ADC_MUTE                         : 1;
    volatile uint16_t BITFLD_ADC_SIGN                         : 1;
};
#define ADC_DC_OFFSET_SEL                    (0x0001)
#define ADC_MUTE                             (0x2000)
#define ADC_SIGN                             (0x4000)



#define RF_ADC_CTRL2_REG                     (0x50002832) /*  */
#define RF_ADC_CTRL2_REG_RESET               (0x00000000)

/*=============================*/
struct __RF_ADC_CTRL2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_ADC_OFFP_I_WR                    : 8;
    volatile uint16_t BITFLD_ADC_OFFN_I_WR                    : 8;
};
#define ADC_OFFP_I_WR                        (0x00FF)
#define ADC_OFFN_I_WR                        (0xFF00)



#define RF_ADC_CTRL3_REG                     (0x50002834) /*  */
#define RF_ADC_CTRL3_REG_RESET               (0x00000000)

/*=============================*/
struct __RF_ADC_CTRL3_REG
/*=============================*/
{
    volatile uint16_t BITFLD_ADC_OFFP_Q_WR                    : 8;
    volatile uint16_t BITFLD_ADC_OFFN_Q_WR                    : 8;
};
#define ADC_OFFP_Q_WR                        (0x00FF)
#define ADC_OFFN_Q_WR                        (0xFF00)



#define RF_DEM_CTRL_REG                      (0x50002840) /*  */
#define RF_DEM_CTRL_REG_RESET                (0x00000059)

/*=============================*/
struct __RF_DEM_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RXDATA_INV                       : 1;
    volatile uint16_t BITFLD_DEM_HSI_POL                      : 1;
    volatile uint16_t BITFLD_MATCH0101_TH                     : 4;
    volatile uint16_t BITFLD_EQUAL_EN                         : 1;
};
#define RXDATA_INV                           (0x0001)
#define DEM_HSI_POL                          (0x0002)
#define MATCH0101_TH                         (0x003C)
#define EQUAL_EN                             (0x0040)



#define RF_AGC_LUT_01_REG                    (0x50002850) /*  */
#define RF_AGC_LUT_01_REG_RESET              (0x00000100)

/*=============================*/
struct __RF_AGC_LUT_01_REG
/*=============================*/
{
    volatile uint16_t BITFLD_VGA2_GAIN0                       : 3;
    volatile uint16_t BITFLD_VGA1_GAIN0                       : 3;
    volatile uint16_t BITFLD_LNA_GAIN0                        : 2;
    volatile uint16_t BITFLD_VGA2_GAIN1                       : 3;
    volatile uint16_t BITFLD_VGA1_GAIN1                       : 3;
    volatile uint16_t BITFLD_LNA_GAIN1                        : 2;
};
#define VGA2_GAIN0                           (0x0007)
#define VGA1_GAIN0                           (0x0038)
#define LNA_GAIN0                            (0x00C0)
#define VGA2_GAIN1                           (0x0700)
#define VGA1_GAIN1                           (0x3800)
#define LNA_GAIN1                            (0xC000)



#define RF_AGC_LUT_23_REG                    (0x50002852) /*  */
#define RF_AGC_LUT_23_REG_RESET              (0x00004202)

/*=============================*/
struct __RF_AGC_LUT_23_REG
/*=============================*/
{
    volatile uint16_t BITFLD_VGA2_GAIN2                       : 3;
    volatile uint16_t BITFLD_VGA1_GAIN2                       : 3;
    volatile uint16_t BITFLD_LNA_GAIN2                        : 2;
    volatile uint16_t BITFLD_VGA2_GAIN3                       : 3;
    volatile uint16_t BITFLD_VGA1_GAIN3                       : 3;
    volatile uint16_t BITFLD_LNA_GAIN3                        : 2;
};
#define VGA2_GAIN2                           (0x0007)
#define VGA1_GAIN2                           (0x0038)
#define LNA_GAIN2                            (0x00C0)
#define VGA2_GAIN3                           (0x0700)
#define VGA1_GAIN3                           (0x3800)
#define LNA_GAIN3                            (0xC000)



#define RF_AGC_LUT_45_REG                    (0x50002854) /*  */
#define RF_AGC_LUT_45_REG_RESET              (0x00004B4A)

/*=============================*/
struct __RF_AGC_LUT_45_REG
/*=============================*/
{
    volatile uint16_t BITFLD_VGA2_GAIN4                       : 3;
    volatile uint16_t BITFLD_VGA1_GAIN4                       : 3;
    volatile uint16_t BITFLD_LNA_GAIN4                        : 2;
    volatile uint16_t BITFLD_VGA2_GAIN5                       : 3;
    volatile uint16_t BITFLD_VGA1_GAIN5                       : 3;
    volatile uint16_t BITFLD_LNA_GAIN5                        : 2;
};
#define VGA2_GAIN4                           (0x0007)
#define VGA1_GAIN4                           (0x0038)
#define LNA_GAIN4                            (0x00C0)
#define VGA2_GAIN5                           (0x0700)
#define VGA1_GAIN5                           (0x3800)
#define LNA_GAIN5                            (0xC000)



#define RF_AGC_LUT_67_REG                    (0x50002856) /*  */
#define RF_AGC_LUT_67_REG_RESET              (0x00005B53)

/*=============================*/
struct __RF_AGC_LUT_67_REG
/*=============================*/
{
    volatile uint16_t BITFLD_VGA2_GAIN6                       : 3;
    volatile uint16_t BITFLD_VGA1_GAIN6                       : 3;
    volatile uint16_t BITFLD_LNA_GAIN6                        : 2;
    volatile uint16_t BITFLD_VGA2_GAIN7                       : 3;
    volatile uint16_t BITFLD_VGA1_GAIN7                       : 3;
    volatile uint16_t BITFLD_LNA_GAIN7                        : 2;
};
#define VGA2_GAIN6                           (0x0007)
#define VGA1_GAIN6                           (0x0038)
#define LNA_GAIN6                            (0x00C0)
#define VGA2_GAIN7                           (0x0700)
#define VGA1_GAIN7                           (0x3800)
#define LNA_GAIN7                            (0xC000)



#define RF_AGC_LUT_89_REG                    (0x50002858) /*  */
#define RF_AGC_LUT_89_REG_RESET              (0x00006B63)

/*=============================*/
struct __RF_AGC_LUT_89_REG
/*=============================*/
{
    volatile uint16_t BITFLD_VGA2_GAIN8                       : 3;
    volatile uint16_t BITFLD_VGA1_GAIN8                       : 3;
    volatile uint16_t BITFLD_LNA_GAIN8                        : 2;
    volatile uint16_t BITFLD_VGA2_GAIN9                       : 3;
    volatile uint16_t BITFLD_VGA1_GAIN9                       : 3;
    volatile uint16_t BITFLD_LNA_GAIN9                        : 2;
};
#define VGA2_GAIN8                           (0x0007)
#define VGA1_GAIN8                           (0x0038)
#define LNA_GAIN8                            (0x00C0)
#define VGA2_GAIN9                           (0x0700)
#define VGA1_GAIN9                           (0x3800)
#define LNA_GAIN9                            (0xC000)



#define RF_AGC_CTRL1_REG                     (0x50002860) /*  */
#define RF_AGC_CTRL1_REG_RESET               (0x0000950D)

/*=============================*/
struct __RF_AGC_CTRL1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_AGC_TH_LOW                       : 7;
    volatile uint16_t BITFLD_AGC_TH_HIGH                      : 7;
    volatile uint16_t BITFLD_AGC_MODE                         : 2;
};
#define AGC_TH_LOW                           (0x007F)
#define AGC_TH_HIGH                          (0x3F80)
#define AGC_MODE                             (0xC000)



#define RF_AGC_CTRL2_REG                     (0x50002862) /*  */
#define RF_AGC_CTRL2_REG_RESET               (0x00000043)

/*=============================*/
struct __RF_AGC_CTRL2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RSSI_TH                          : 6;
    volatile uint16_t BITFLD_EN_FRZ_GAIN                      : 1;
    volatile uint16_t BITFLD_AGCSETTING_SEL                   : 1;
    volatile uint16_t BITFLD_AGCSETTING_WR                    : 4;
    volatile uint16_t BITFLD_SLOW_AGC                         : 1;
};
#define RSSI_TH                              (0x003F)
#define EN_FRZ_GAIN                          (0x0040)
#define AGCSETTING_SEL                       (0x0080)
#define AGCSETTING_WR                        (0x0F00)
#define SLOW_AGC                             (0x1000)



#define RF_AFC_CTRL_REG                      (0x50002864) /*  */
#define RF_AFC_CTRL_REG_RESET                (0x000000D5)

/*=============================*/
struct __RF_AFC_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_AFC_MODE                         : 4;
    volatile uint16_t BITFLD_POLE1                            : 2;
    volatile uint16_t BITFLD_POLE2                            : 2;
};
#define AFC_MODE                             (0x000F)
#define POLE1                                (0x0030)
#define POLE2                                (0x00C0)



#define RF_DC_OFFSET_CTRL1_REG               (0x50002866) /*  */
#define RF_DC_OFFSET_CTRL1_REG_RESET         (0x00008080)

/*=============================*/
struct __RF_DC_OFFSET_CTRL1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DCOFFSET_I_WR                    : 8;
    volatile uint16_t BITFLD_DCOFFSET_Q_WR                    : 8;
};
#define DCOFFSET_I_WR                        (0x00FF)
#define DCOFFSET_Q_WR                        (0xFF00)



#define RF_DC_OFFSET_CTRL2_REG               (0x50002868) /*  */
#define RF_DC_OFFSET_CTRL2_REG_RESET         (0x000001D2)

/*=============================*/
struct __RF_DC_OFFSET_CTRL2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DCOFFSET_SEL                     : 1;
    volatile uint16_t BITFLD_DCPARCAL_EN                      : 1;
    volatile uint16_t BITFLD_DCPOLE                           : 2;
    volatile uint16_t BITFLD_DCNSTEP                          : 3;
    volatile uint16_t BITFLD_DCNGAIN                          : 2;
};
#define DCOFFSET_SEL                         (0x0001)
#define DCPARCAL_EN                          (0x0002)
#define DCPOLE                               (0x000C)
#define DCNSTEP                              (0x0070)
#define DCNGAIN                              (0x0180)



#define RF_DC_OFFSET_CTRL3_REG               (0x5000286A) /*  */
#define RF_DC_OFFSET_CTRL3_REG_RESET         (0x0000DCE4)

/*=============================*/
struct __RF_DC_OFFSET_CTRL3_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DCBETA_I                         : 8;
    volatile uint16_t BITFLD_DCBETA_Q                         : 8;
};
#define DCBETA_I                             (0x00FF)
#define DCBETA_Q                             (0xFF00)



#define RF_DC_OFFSET_CTRL4_REG               (0x5000286C) /*  */
#define RF_DC_OFFSET_CTRL4_REG_RESET         (0x00009210)

/*=============================*/
struct __RF_DC_OFFSET_CTRL4_REG
/*=============================*/
{
    volatile uint16_t BITFLD_DCAGCSETTING_FULL0               : 4;
    volatile uint16_t BITFLD_DCAGCSETTING_FULL1               : 4;
    volatile uint16_t BITFLD_DCAGCSETTING_FULL2               : 4;
    volatile uint16_t BITFLD_DCAGCSETTING_FULL3               : 4;
};
#define DCAGCSETTING_FULL0                   (0x000F)
#define DCAGCSETTING_FULL1                   (0x00F0)
#define DCAGCSETTING_FULL2                   (0x0F00)
#define DCAGCSETTING_FULL3                   (0xF000)



#define RF_RADIG_SPARE_REG                   (0x50002870) /*  */
#define RF_RADIG_SPARE_REG_RESET             (0x00000000)

/*=============================*/
struct __RF_RADIG_SPARE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_LSI_GLOBAL                       : 1;
    volatile uint16_t BITFLD_CC_TEST_MODE_1                   : 1;
    volatile uint16_t BITFLD_CC_TEST_MODE_2                   : 1;
    volatile uint16_t BITFLD_RADIG_SPARE                      : 13;
};
#define LSI_GLOBAL                           (0x0001)
#define CC_TEST_MODE_1                       (0x0002)
#define CC_TEST_MODE_2                       (0x0004)
#define RADIG_SPARE                          (0xFFF8)



#define RF_AGC_RESULT_REG                    (0x50002900) /*  */
#define RF_AGC_RESULT_REG_RESET              (0x00000000)

/*=============================*/
struct __RF_AGC_RESULT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_AFC_RD                           : 8;
    volatile uint16_t BITFLD_AGCSETTING_RD                    : 4;
};
#define AFC_RD                               (0x00FF)
#define AGCSETTING_RD                        (0x0F00)



#define RF_RSSI_RESULT_REG                   (0x50002902) /*  */
#define RF_RSSI_RESULT_REG_RESET             (0x00000000)

/*=============================*/
struct __RF_RSSI_RESULT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RSSI_PH_RD                       : 6;
    volatile uint16_t BITFLD_RSSI_AVG_RD                      : 10;
};
#define RSSI_PH_RD                           (0x003F)
#define RSSI_AVG_RD                          (0xFFC0)



#define RF_PA_CTRL_REG                       (0x50002A00) /* Removed obsolete values of bits 10:7, pa_pw back to 4 */
#define RF_PA_CTRL_REG_RESET                 (0x00007853)

/*=============================*/
struct __RF_PA_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_PA_GAIN                          : 2;
    volatile uint16_t BITFLD_PA_PW                            : 3;
    volatile uint16_t BITFLD_PA_RAMPSPEED                     : 2;
    volatile uint16_t                                         : 4;
    volatile uint16_t BITFLD_LEVEL_LDO_RFPA                   : 4;
};
#define PA_GAIN                              (0x0003)
#define PA_PW                                (0x001C)
#define PA_RAMPSPEED                         (0x0060)
#define LEVEL_LDO_RFPA                       (0x7800)



#define RF_SYNTH_CTRL1_REG                   (0x50002C00) /*  */
#define RF_SYNTH_CTRL1_REG_RESET             (0x00002962)

/*=============================*/
struct __RF_SYNTH_CTRL1_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CHANNEL_ZERO                     : 12;
    volatile uint16_t BITFLD_SGN                              : 1;
    volatile uint16_t BITFLD_CS                               : 1;
    volatile uint16_t BITFLD_PLL_HSI_POL                      : 1;
};
#define CHANNEL_ZERO                         (0x0FFF)
#define SGN                                  (0x1000)
#define CS                                   (0x2000)
#define PLL_HSI_POL                          (0x4000)



#define RF_SYNTH_CTRL2_REG                   (0x50002C02) /*  */
#define RF_SYNTH_CTRL2_REG_RESET             (0x0000108B)

/*=============================*/
struct __RF_SYNTH_CTRL2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SD_ORDER_RX                      : 2;
    volatile uint16_t BITFLD_SD_ORDER_TX                      : 2;
    volatile uint16_t BITFLD_MODINDEX                         : 2;
    volatile uint16_t BITFLD_DELAY                            : 2;
    volatile uint16_t BITFLD_GAUSS_INV                        : 1;
    volatile uint16_t BITFLD_GAUSS_86                         : 1;
    volatile uint16_t BITFLD_TXDATA_INV                       : 1;
    volatile uint16_t BITFLD_EO_PACKET_DIS                    : 1;
    volatile uint16_t BITFLD_BT_SEL                           : 1;
};
#define SD_ORDER_RX                          (0x0003)
#define SD_ORDER_TX                          (0x000C)
#define MODINDEX                             (0x0030)
#define DELAY                                (0x00C0)
#define GAUSS_INV                            (0x0100)
#define GAUSS_86                             (0x0200)
#define TXDATA_INV                           (0x0400)
#define EO_PACKET_DIS                        (0x0800)
#define BT_SEL                               (0x1000)



#define RF_SYNTH_CTRL3_REG                   (0x50002C04) /*  */
#define RF_SYNTH_CTRL3_REG_RESET             (0x00000003)

/*=============================*/
struct __RF_SYNTH_CTRL3_REG
/*=============================*/
{
    volatile uint16_t BITFLD_MODVAL_WR                        : 14;
    volatile uint16_t BITFLD_MODVAL_SEL                       : 1;
};
#define MODVAL_WR                            (0x3FFF)
#define MODVAL_SEL                           (0x4000)



#define RF_VCOCAL_CTRL_REG                   (0x50002C06) /*  */
#define RF_VCOCAL_CTRL_REG_RESET             (0x000000C3)

/*=============================*/
struct __RF_VCOCAL_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_VCO_FREQTRIM_WR                  : 4;
    volatile uint16_t BITFLD_VCO_FREQTRIM_SEL                 : 1;
    volatile uint16_t BITFLD_VCO_FREQTRIM_OFF_SEL             : 1;
    volatile uint16_t BITFLD_VCOCAL_PERIOD                    : 2;
    volatile uint16_t BITFLD_VCOCAL_MD_STATE_DLY_SEL          : 1;
};
#define VCO_FREQTRIM_WR                      (0x000F)
#define VCO_FREQTRIM_SEL                     (0x0010)
#define VCO_FREQTRIM_OFF_SEL                 (0x0020)
#define VCOCAL_PERIOD                        (0x00C0)
#define VCOCAL_MD_STATE_DLY_SEL              (0x0100)



#define RF_MGAIN_CTRL_REG                    (0x50002C08) /*  */
#define RF_MGAIN_CTRL_REG_RESET              (0x00001403)

/*=============================*/
struct __RF_MGAIN_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_GAUSS_GAIN_WR                    : 8;
    volatile uint16_t BITFLD_GAUSS_GAIN_SEL                   : 1;
    volatile uint16_t BITFLD_MGAIN_DBL_TRANSMIT               : 1;
    volatile uint16_t BITFLD_MGAIN_CMP_INV                    : 1;
    volatile uint16_t BITFLD_MGAIN_AVER                       : 2;
};
#define GAUSS_GAIN_WR                        (0x00FF)
#define GAUSS_GAIN_SEL                       (0x0100)
#define MGAIN_DBL_TRANSMIT                   (0x0200)
#define MGAIN_CMP_INV                        (0x0400)
#define MGAIN_AVER                           (0x1800)



#define RF_MGAIN_CTRL2_REG                   (0x50002C0A) /*  */
#define RF_MGAIN_CTRL2_REG_RESET             (0x00000688)

/*=============================*/
struct __RF_MGAIN_CTRL2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_MGAIN_TRANSMIT_LENGTH            : 7;
    volatile uint16_t BITFLD_KMOD_ALPHA                       : 6;
};
#define MGAIN_TRANSMIT_LENGTH                (0x007F)
#define KMOD_ALPHA                           (0x1F80)



#define RF_MGC_CTRL_REG                      (0x50002C10) /*  */
#define RF_MGC_CTRL_REG_RESET                (0x00000000)

/*=============================*/
struct __RF_MGC_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_MGC_GAIN_SET                     : 1;
    volatile uint16_t BITFLD_MGC_POLE_SW                      : 1;
    volatile uint16_t BITFLD_GAUSS_DAC_CTRL                   : 2;
};
#define MGC_GAIN_SET                         (0x0001)
#define MGC_POLE_SW                          (0x0002)
#define GAUSS_DAC_CTRL                       (0x000C)



#define RF_VCOVAR_CTRL_REG                   (0x50002C20) /*  */
#define RF_VCOVAR_CTRL_REG_RESET             (0x0000A492)

/*=============================*/
struct __RF_VCOVAR_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_TUNE_VAR_V0                      : 3;
    volatile uint16_t BITFLD_TUNE_VAR_V1                      : 3;
    volatile uint16_t BITFLD_TUNE_VAR_V2                      : 3;
    volatile uint16_t BITFLD_TUNE_VAR_V3                      : 3;
    volatile uint16_t BITFLD_MOD_VAR_V0                       : 2;
    volatile uint16_t BITFLD_MOD_VAR_V1                       : 2;
};
#define TUNE_VAR_V0                          (0x0007)
#define TUNE_VAR_V1                          (0x0038)
#define TUNE_VAR_V2                          (0x01C0)
#define TUNE_VAR_V3                          (0x0E00)
#define MOD_VAR_V0                           (0x3000)
#define MOD_VAR_V1                           (0xC000)



#define RF_VCO_CALCAP_BIT14_REG              (0x50002C22) /*  */
#define RF_VCO_CALCAP_BIT14_REG_RESET        (0x0000D59D)

/*=============================*/
struct __RF_VCO_CALCAP_BIT14_REG
/*=============================*/
{
    volatile uint16_t BITFLD_VCO_CALCAP_BIT14                 : 16;
};
#define VCO_CALCAP_BIT14                     (0xFFFF)



#define RF_VCO_CALCAP_BIT15_REG              (0x50002C24) /*  */
#define RF_VCO_CALCAP_BIT15_REG_RESET        (0x00000000)

/*=============================*/
struct __RF_VCO_CALCAP_BIT15_REG
/*=============================*/
{
    volatile uint16_t BITFLD_VCO_CALCAP_BIT15                 : 16;
};
#define VCO_CALCAP_BIT15                     (0xFFFF)



#define RF_PFD_CTRL_REG                      (0x50002C40) /*  */
#define RF_PFD_CTRL_REG_RESET                (0x00000000)

/*=============================*/
struct __RF_PFD_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_FIXED_CUR_SET                    : 2;
    volatile uint16_t BITFLD_FIXED_CUR_EN                     : 1;
    volatile uint16_t BITFLD_PFD_POLARITY                     : 1;
};
#define FIXED_CUR_SET                        (0x0003)
#define FIXED_CUR_EN                         (0x0004)
#define PFD_POLARITY                         (0x0008)



#define RF_CP_CTRL_REG                       (0x50002C50) /*  */
#define RF_CP_CTRL_REG_RESET                 (0x00007F7F)

/*=============================*/
struct __RF_CP_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CP_CUR_SET_RX                    : 4;
    volatile uint16_t BITFLD_CP_CUR_SET_TX                    : 4;
    volatile uint16_t BITFLD_CP_CUR_RX                        : 4;
    volatile uint16_t BITFLD_CP_CUR_TX                        : 4;
};
#define CP_CUR_SET_RX                        (0x000F)
#define CP_CUR_SET_TX                        (0x00F0)
#define CP_CUR_RX                            (0x0F00)
#define CP_CUR_TX                            (0xF000)



#define RF_LF_RES_CTRL_REG                   (0x50002C52) /* LF resistor setting */
#define RF_LF_RES_CTRL_REG_RESET             (0x00007F7F)

/*=============================*/
struct __RF_LF_RES_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_LF_RES_SET_RX                    : 4;
    volatile uint16_t BITFLD_LF_RES_SET_TX                    : 4;
    volatile uint16_t BITFLD_LF_RES_RX                        : 4;
    volatile uint16_t BITFLD_LF_RES_TX                        : 4;
};
#define LF_RES_SET_RX                        (0x000F)
#define LF_RES_SET_TX                        (0x00F0)
#define LF_RES_RX                            (0x0F00)
#define LF_RES_TX                            (0xF000)



#define RF_LF_CTRL_REG                       (0x50002C60) /*  */
#define RF_LF_CTRL_REG_RESET                 (0x00000050)

/*=============================*/
struct __RF_LF_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_LF_CAL_CAP_WR                    : 5;
    volatile uint16_t BITFLD_LF_CAL_CAP_SEL                   : 1;
    volatile uint16_t BITFLD_LF_SHORT_R4                      : 1;
    volatile uint16_t                                         : 1;
};
#define LF_CAL_CAP_WR                        (0x001F)
#define LF_CAL_CAP_SEL                       (0x0020)
#define LF_SHORT_R4                          (0x0040)



#define RF_TDC_CTRL_REG                      (0x50002C70) /* TDC settings */
#define RF_TDC_CTRL_REG_RESET                (0x00000000)

/*=============================*/
struct __RF_TDC_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_CTRL_FAST                        : 4;
    volatile uint16_t BITFLD_CTRL_SLOW                        : 4;
    volatile uint16_t BITFLD_CAL_PH_1                         : 1;
    volatile uint16_t BITFLD_CAL_PH_2                         : 1;
    volatile uint16_t BITFLD_REF_CTRL                         : 2;
    volatile uint16_t BITFLD_TDC_CONNECT                      : 1;
};
#define CTRL_FAST                            (0x000F)
#define CTRL_SLOW                            (0x00F0)
#define CAL_PH_1                             (0x0100)
#define CAL_PH_2                             (0x0200)
#define REF_CTRL                             (0x0C00)
#define TDC_CONNECT                          (0x1000)



#define SCAN_OBSERVE_REG                     (0x500030A0) /*  */
#define SCAN_OBSERVE_REG_RESET               (0x00000000)

/*=============================*/
struct __SCAN_OBSERVE_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SCAN_FEEDBACK_MUX                : 16;
};
#define SCAN_FEEDBACK_MUX                    (0xFFFF)



#define TEST_CTRL_REG                        (0x500030F0) /*  */
#define TEST_CTRL_REG_RESET                  (0x00000000)

/*=============================*/
struct __TEST_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_SHOW_CLOCKS                      : 1;
    volatile uint16_t BITFLD_ENABLE_RFPT                      : 1;
    volatile uint16_t BITFLD_SHOW_DC_STATE                    : 1;
    volatile uint16_t BITFLD_SHOW_DC_COMP                     : 1;
    volatile uint16_t BITFLD_XTAL16M_CAP_TEST_EN              : 1;
    volatile uint16_t BITFLD_SHOW_IF_RO                       : 1;
    volatile uint16_t BITFLD_PLL_TST_MODE                     : 1;
    volatile uint16_t                                         : 1;
};
#define SHOW_CLOCKS                          (0x0001)
#define ENABLE_RFPT                          (0x0002)
#define SHOW_DC_STATE                        (0x0004)
#define SHOW_DC_COMP                         (0x0008)
#define XTAL16M_CAP_TEST_EN                  (0x0010)
#define SHOW_IF_RO                           (0x0020)
#define PLL_TST_MODE                         (0x0040)



#define TEST_CTRL2_REG                       (0x500030F2) /*  */
#define TEST_CTRL2_REG_RESET                 (0x00000000)

/*=============================*/
struct __TEST_CTRL2_REG
/*=============================*/
{
    volatile uint16_t BITFLD_ANA_TESTMUX_CTRL                 : 4;
    volatile uint16_t                                         : 4;
    volatile uint16_t BITFLD_RF_IN_TESTMUX_CTRL               : 2;
};
#define ANA_TESTMUX_CTRL                     (0x000F)
#define RF_IN_TESTMUX_CTRL                   (0x0300)



#define TEST_CTRL3_REG                       (0x500030F4) /*  */
#define TEST_CTRL3_REG_RESET                 (0x00000000)

/*=============================*/
struct __TEST_CTRL3_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RF_TEST_OUT_SEL                  : 6;
    volatile uint16_t                                         : 2;
    volatile uint16_t BITFLD_RF_TEST_OUT_PARAM                : 8;
};
#define RF_TEST_OUT_SEL                      (0x003F)
#define RF_TEST_OUT_PARAM                    (0xFF00)



#define TEST_CTRL4_REG                       (0x500030F6) /*  */
#define TEST_CTRL4_REG_RESET                 (0x00000000)

/*=============================*/
struct __TEST_CTRL4_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RF_TEST_IN_SEL                   : 3;
    volatile uint16_t                                         : 5;
    volatile uint16_t BITFLD_RF_TEST_IN_PARAM                 : 8;
};
#define RF_TEST_IN_SEL                       (0x0007)
#define RF_TEST_IN_PARAM                     (0xFF00)



#define TEST_CTRL5_REG                       (0x500030F8) /*  */
#define TEST_CTRL5_REG_RESET                 (0x00000000)

/*=============================*/
struct __TEST_CTRL5_REG
/*=============================*/
{
    volatile uint16_t BITFLD_TEST_VDD                         : 1;
    volatile uint16_t BITFLD_TEST_OTP_VDD                     : 1;
    volatile uint16_t BITFLD_TEST_OTP_OTA                     : 1;
    volatile uint16_t BITFLD_TEST_OTP_VSS                     : 1;
    volatile uint16_t                                         : 4;
    volatile uint16_t BITFLD_TEST_STRUCT                      : 4;
    volatile uint16_t BITFLD_DCDC_NSW                         : 1;
    volatile uint16_t BITFLD_DCDC_PSW                         : 1;
    volatile uint16_t BITFLD_DCDC_OUTSW                       : 1;
    volatile uint16_t BITFLD_DCDC_FORCE_IDLE                  : 1;
};
#define TEST_VDD                             (0x0001)
#define TEST_OTP_VDD                         (0x0002)
#define TEST_OTP_OTA                         (0x0004)
#define TEST_OTP_VSS                         (0x0008)
#define TEST_STRUCT                          (0x0F00)
#define DCDC_NSW                             (0x1000)
#define DCDC_PSW                             (0x2000)
#define DCDC_OUTSW                           (0x4000)
#define DCDC_FORCE_IDLE                      (0x8000)



#define BIST_CTRL_REG                        (0x500030FA) /*  */
#define BIST_CTRL_REG_RESET                  (0x000006C0)

/*=============================*/
struct __BIST_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RAM_BIST_CONFIG                  : 2;
    volatile uint16_t BITFLD_ROMBIST_ENABLE                   : 1;
    volatile uint16_t BITFLD_SYSRAM12_BIST_ENABLE             : 1;
    volatile uint16_t BITFLD_SHOW_BIST                        : 1;
    volatile uint16_t BITFLD_ROM_BIST_BUSY                    : 1;
    volatile uint16_t BITFLD_SYSRAM34_BIST_LINE_FAIL          : 1;
    volatile uint16_t BITFLD_SYSRAM34_BIST_FAIL               : 1;
    volatile uint16_t BITFLD_SYSRAM34_BIST_BUSY               : 1;
    volatile uint16_t BITFLD_SYSRAM12_BIST_LINE_FAIL          : 1;
    volatile uint16_t BITFLD_SYSRAM12_BIST_FAIL               : 1;
    volatile uint16_t BITFLD_SYSRAM12_BIST_BUSY               : 1;
    volatile uint16_t BITFLD_RAM_BIST_PATTERN                 : 2;
    volatile uint16_t BITFLD_SYSRAM34_BIST_ENABLE             : 1;
};
#define RAM_BIST_CONFIG                      (0x0003)
#define ROMBIST_ENABLE                       (0x0004)
#define SYSRAM12_BIST_ENABLE                 (0x0008)
#define SHOW_BIST                            (0x0010)
#define ROM_BIST_BUSY                        (0x0020)
#define SYSRAM34_BIST_LINE_FAIL              (0x0040)
#define SYSRAM34_BIST_FAIL                   (0x0080)
#define SYSRAM34_BIST_BUSY                   (0x0100)
#define SYSRAM12_BIST_LINE_FAIL              (0x0200)
#define SYSRAM12_BIST_FAIL                   (0x0400)
#define SYSRAM12_BIST_BUSY                   (0x0800)
#define RAM_BIST_PATTERN                     (0x3000)
#define SYSRAM34_BIST_ENABLE                 (0x4000)



#define ROMBIST_RESULTL_REG                  (0x500030FC) /*  */
#define ROMBIST_RESULTL_REG_RESET            (0x00000000)

/*=============================*/
struct __ROMBIST_RESULTL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_ROMBIST_RESULTL                  : 16;
};
#define ROMBIST_RESULTL                      (0xFFFF)



#define ROMBIST_RESULTH_REG                  (0x500030FE) /*  */
#define ROMBIST_RESULTH_REG_RESET            (0x00000000)

/*=============================*/
struct __ROMBIST_RESULTH_REG
/*=============================*/
{
    volatile uint16_t BITFLD_ROMBIST_RESULTH                  : 16;
};
#define ROMBIST_RESULTH                      (0xFFFF)



#define CHIP_CONFIG1_REG                     (0x50003205) /* Chip configuration register 1. */
#define CHIP_CONFIG1_REG_RESET               (0x00000020)

/*=============================*/
struct __CHIP_CONFIG1_REG
/*=============================*/
{
    volatile uint8_t  BITFLD_CHIP_CONFIG1                     : 8;
};
#define CHIP_CONFIG1                         (0x00FF)



#define CHIP_CONFIG2_REG                     (0x50003206) /* Chip configuration register 2. */
#define CHIP_CONFIG2_REG_RESET               (0x00000020)

/*=============================*/
struct __CHIP_CONFIG2_REG
/*=============================*/
{
    volatile uint8_t  BITFLD_CHIP_CONFIG2                     : 8;
};
#define CHIP_CONFIG2                         (0x00FF)



#define CHIP_CONFIG3_REG                     (0x50003207) /* Chip configuration register 3. */
#define CHIP_CONFIG3_REG_RESET               (0x00000020)

/*=============================*/
struct __CHIP_CONFIG3_REG
/*=============================*/
{
    volatile uint8_t  BITFLD_CHIP_CONFIG3                     : 8;
};
#define CHIP_CONFIG3                         (0x00FF)



#define CHIP_TEST1_REG                       (0x5000320A) /* Chip test register 1. */
#define CHIP_TEST1_REG_RESET                 (0x00000000)
//register CHIP_TEST1_REG found for which all fields are reserved, omitting.



#define CHIP_TEST2_REG                       (0x5000320B) /* Chip test register 2. */
#define CHIP_TEST2_REG_RESET                 (0x00000020)
//register CHIP_TEST2_REG found for which all fields are reserved, omitting.



#define RFPT_CTRL_REG                        (0x50003500) /* Control register */
#define RFPT_CTRL_REG_RESET                  (0x00000000)

/*=============================*/
struct __RFPT_CTRL_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RFPT_PACK_EN                     : 1;
    volatile uint16_t BITFLD_RFPT_PACK_SEL                    : 1;
};
#define RFPT_PACK_EN                         (0x0001)
#define RFPT_PACK_SEL                        (0x0002)



#define RFPT_ADDR_REG                        (0x50003502) /* AHB master start address */
#define RFPT_ADDR_REG_RESET                  (0x00000000)

/*=============================*/
struct __RFPT_ADDR_REG
/*=============================*/
{
    volatile uint16_t                                         : 2;
    volatile uint16_t BITFLD_RFPT_ADDR                        : 14;
};
#define RFPT_ADDR                            (0xFFFC)



#define RFPT_LEN_REG                         (0x50003504) /* Data length register */
#define RFPT_LEN_REG_RESET                   (0x00000000)

/*=============================*/
struct __RFPT_LEN_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RFPT_LEN                         : 14;
};
#define RFPT_LEN                             (0x3FFF)



#define RFPT_STAT_REG                        (0x50003506) /* Status register */
#define RFPT_STAT_REG_RESET                  (0x00000000)

/*=============================*/
struct __RFPT_STAT_REG
/*=============================*/
{
    volatile uint16_t BITFLD_RFPT_ACTIVE                      : 1;
    volatile uint16_t BITFLD_RFPT_OFLOW_STK                   : 1;
};
#define RFPT_ACTIVE                          (0x0001)
#define RFPT_OFLOW_STK                       (0x0002)
/*===========================================================================*/ 
/*===================== End of automatic generated code =====================*/ 
/*===========================================================================*/ 



/*===========================================================================*/ 
/*========================== Start of footer part ===========================*/ 
/*===========================================================================*/ 
/*----------------------------*/
/* Data access macros         */
/*----------------------------*/

#define SHIF16(a) ((a)&0x0001? 0:(a)&0x0002? 1:(a)&0x0004? 2:(a)&0x0008? 3: \
                   (a)&0x0010? 4:(a)&0x0020? 5:(a)&0x0040? 6:(a)&0x0080? 7: \
                   (a)&0x0100? 8:(a)&0x0200? 9:(a)&0x0400?10:(a)&0x0800?11: \
                   (a)&0x1000?12:(a)&0x2000?13:(a)&0x4000?14:           15)

#define SHIF32(a)((a)&0x00000001? 0:(a)&0x00000002? 1:(a)&0x00000004? 2:(a)&0x00000008? 3:\
                  (a)&0x00000010? 4:(a)&0x00000020? 5:(a)&0x00000040? 6:(a)&0x00000080? 7:\
                  (a)&0x00000100? 8:(a)&0x00000200? 9:(a)&0x00000400?10:(a)&0x00000800?11:\
                  (a)&0x00001000?12:(a)&0x00002000?13:(a)&0x00004000?14:(a)&0x00008000?15:\
                  (a)&0x00010000?16:(a)&0x00020000?17:(a)&0x00040000?18:(a)&0x00080000?19:\
                  (a)&0x00100000?20:(a)&0x00200000?21:(a)&0x00400000?22:(a)&0x00800000?23:\
                  (a)&0x01000000?24:(a)&0x02000000?25:(a)&0x04000000?26:(a)&0x08000000?27:\
                  (a)&0x10000000?28:(a)&0x20000000?29:(a)&0x40000000?30:               31)

#define SetWord8(a,d)  (* ( volatile uint8_t*) (a)=(d) )
#define SetWord16(a,d) (* ( volatile uint16_t*)(a)=(d) )
#define SetWord32(a,d) (* ( volatile uint32_t*)(a)=(d) )
#define SetWord64(a,d) (* ( volatile uint64_t*)(a)=(d) )

#define GetWord8(a)    (* ( volatile uint8_t*) (a) )
#define GetWord16(a)   (* ( volatile uint16_t*)(a) )
#define GetWord32(a)   (* ( volatile uint32_t*)(a) )
#define GetWord64(a)   (* ( volatile uint64_t*)(a) )

/* Aliases for backwards compatibility (only the Byte versions). */
#define SetByte(a,d) SetWord8((a),(d))
#define GetByte(a)   GetWord8((a))

#define SetBits16(a,f,d) ( SetWord16( (a), (GetWord16(a)&(~(uint16_t)(f))) | (((uint16_t)(d))<<SHIF16((f))) ))
#define SetBits32(a,f,d) ( SetWord32( (a), (GetWord32(a)&(~(uint32_t)(f))) | (((uint32_t)(d))<<SHIF32((f))) ))

// old-fashioned way. Allowed on APB-2 under the following conditions:
// * armcc is used (gcc for ARM seems to work as well?)
// * bitfields are marked with 'volatile' keyword and the correct word size
#define GetBits(a,f)    ( ((volatile struct __##a *)(a))->BITFLD_##f )
#define SetBits(a,f,d) (( ((volatile struct __##a *)(a))->BITFLD_##f )=(d) )

#define GetBits16(a,f) ( (GetWord16(a)&( (uint16_t)(f) )) >> SHIF16(f) )
#define GetBits32(a,f) ( (GetWord32(a)&( (uint32_t)(f) )) >> SHIF32(f) )

// Macro's used in the recommended_settings function
// All fields are combined into a single mask.
#define SetWordMasked16(a,m,d) SetWord16(a, (GetWord16(a)&~m) | (d&m))
#define SetWordMasked32(a,m,d) SetWord32(a, (GetWord32(a)&~m) | (d&m))


/*===========================================================================*/ 
/*=========================== End of footer part ============================*/ 
/*===========================================================================*/ 



// _DA14585_586_H_
#endif

