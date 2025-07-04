/**
 ****************************************************************************************
 * @addtogroup Drivers
 * @{
 * @addtogroup WLAN
 * @brief WLAN coexistence API
 * @{
 *
 * @file wlan_coex.h
 *
 * @brief Definition of API for the WLAN coexistence.
 *
 * Copyright (C) 2019-2025 Renesas Electronics Corporation and/or its affiliates.
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

#ifndef _WLAN_COEX_H_
#define _WLAN_COEX_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>
#include "gpio.h"

/*
 * DEFINES
 ****************************************************************************************
 */
 
#if (!WLAN_COEX_ONE_WIRE)
/// BLE priorities
typedef enum
{
#if !defined (__DA14531_01__)
    /// Active scan
    WLAN_COEX_BLE_PRIO_SCAN   = 0x01,
#endif
    /// Advertise
    WLAN_COEX_BLE_PRIO_ADV    = 0x02,
#if !defined (__DA14531_01__)
    /// Connection request
    WLAN_COEX_BLE_PRIO_CONREQ = 0x04,
#endif
    /// Control packet
    WLAN_COEX_BLE_PRIO_LLCP   = 0x10,
    /// Data packet
    WLAN_COEX_BLE_PRIO_DATA   = 0x20,
    /// Missed events
    WLAN_COEX_BLE_PRIO_MISSED = 0x40,
}wlan_coex_ble_prio_t;
#endif

/// WLAN coexistence configuration struct
typedef struct
{
    /// BLE event in progress signal pin
    GPIO_PORT ble_eip_port;
    GPIO_PIN ble_eip_pin;

#if (!WLAN_COEX_ONE_WIRE)
    /// 2.4GHz external device event in progress signal pin
    GPIO_PORT ext_24g_eip_port;
    GPIO_PIN ext_24g_eip_pin;

    /// BLE priority indication signal pin
    GPIO_PORT ble_prio_port;
    GPIO_PIN ble_prio_pin;

#if defined (CFG_WLAN_COEX_DEBUG)
    /// BLE radio overruled signal pin
    GPIO_PORT debug_a_port;
    GPIO_PIN debug_a_pin;

    /// Debug signal to be generated upon the reception of the 2.4GHz event in progress signal
    GPIO_PORT debug_b_port;
    GPIO_PIN debug_b_pin;
#endif

    /// GPIO IRQ number fired from external 2.4GHz device event in progress signal
    uint8_t irq;
#endif
} wlan_coex_cfg_t;


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize the WLAN-BLE coexistence mode
 *
 * @note This function must be called during system initialization (e.g. from  user_on_init())
 ****************************************************************************************
 */
void wlan_coex_init(void);

/**
 ****************************************************************************************
 * @brief Configure the required GPIOs used in WLAN-BLE coexistence mode
 ****************************************************************************************
 */
void wlan_coex_gpio_cfg(void);

/**
 ****************************************************************************************
 * @brief Drive to inactive state the pin used for the BLE event in progress signal
 *
 * @note If the polarity of the BLE event in progress signal is active low then
 *       GPIO is configured in high state else in low.
 ****************************************************************************************
 */
void wlan_coex_set_ble_eip_pin_inactive(void);

/**
 ****************************************************************************************
 * @brief Enable/disable the BLE priority over the WLAN module.
 *
 * @param[in] level The BLE priority level - 1 to enable the BLE priority, 0
 *            to disable the BLE priority
 *
 * @note This function must be called from dlg_event_isr() and dlg_eventapfa_isr() with
 * level = 0.
 ****************************************************************************************
 */
void wlan_coex_prio_level(int level);

/**
 ****************************************************************************************
 * @brief Configure WLAN_COEX_BLE_EVENT signal.
 *
 * @note This signal is generated by BLE diagnostics and must be configured after BLE has
 * been powered up.
 ****************************************************************************************
 */
void wlan_coex_BLE_set(void);

/**
 ****************************************************************************************
 * @brief Called within BLE event start and end event
 *
 * @note This function must be called from dlg_cscnt_isr() and dlg_finetgtim_isr()
 ****************************************************************************************
 */
void wlan_coex_finetimer_isr(void);

#if (!WLAN_COEX_ONE_WIRE)

/**
 ****************************************************************************************
 * @brief Add priority criteria to WLAN coexistence
 *
 * @param[in] priority  The priority type
 * @param[in] conhdl    The connection handle. Use LLD_ADV_HDL for advertising and
 *                      scan events
 * @param[in] missed    The number of missed events before asserting BLE priority signal
 *                      (for WLAN_COEX_BLE_PRIO_MISSED priority)
 ****************************************************************************************
 */
void wlan_coex_prio_criteria_add(wlan_coex_ble_prio_t priority, uint16_t conhdl, uint16_t missed);

/**
 ****************************************************************************************
 * @brief Delete priority criteria to WLAN coexistence
 *
 * @param[in] priority  The priority type
 * @param[in] conhdl    The connection handle. Use LLD_ADV_HDL for advertising and
 *                      scan events
 * @param[in] missed    Ignored
 ****************************************************************************************
 */
void wlan_coex_prio_criteria_del(wlan_coex_ble_prio_t priority, uint16_t conhdl, uint16_t missed);

/**
 ****************************************************************************************
 * @brief Prepare WLAN COEX for going to sleep
 ****************************************************************************************
 */
void wlan_coex_going_to_sleep(void);

#endif // WLAN_COEX_ONE_WIRE

#endif  //_WLAN_COEX_H_

///@}
///@}
