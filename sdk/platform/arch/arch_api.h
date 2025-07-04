/**
 ****************************************************************************************
 * @addtogroup ARCH Architecture
 * @{
 * @addtogroup ARCH_SYSTEM
 * @brief System API
 * @{
 *
 * @file arch_api.h
 *
 * @brief Architecture functions API.
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

#ifndef _ARCH_API_H_
#define _ARCH_API_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdbool.h>
#include <string.h>
#include "nvds.h"
#include "arch.h"
#include "arch_wdg.h"
#include "gpio.h"
#if !defined (__NON_BLE_EXAMPLE__)
#include "rwble.h"
#endif

/*
##############################################################################
#   Sleep mode API                                                           #
##############################################################################
*/

typedef enum
{
    ARCH_SLEEP_OFF,
    ARCH_EXT_SLEEP_ON,
    ARCH_EXT_SLEEP_OTP_COPY_ON,
} sleep_state_t;


/// Arch Sleep environment structure
struct arch_sleep_env_tag
{
    sleep_state_t slp_state;
};

#if defined (__DA14531__)
/// RAM block (1, 2 or 3) power control when system power domain is down
typedef enum
{
    /// RAM block is turned on (retained) when system power domain is down
    PD_SYS_DOWN_RAM_ON  = 0U,

    /// RAM block is turned off (not retained) when system power domain is down
    PD_SYS_DOWN_RAM_OFF = 1U,
} pd_sys_down_ram_t;

/// Selects which memory block is located at address 0x0000 for execution
typedef enum
{
    /// ROM at address 0
    REMAP_ADDR0_TO_ROM  = 0U,

    /// OTP at address 0
    REMAP_ADDR0_TO_OTP  = 1U,

    /// RAM1 at address 0
    REMAP_ADDR0_TO_RAM1 = 2U,

    /// RAM2 at address 0
    REMAP_ADDR0_TO_RAM2 = 3U,

    /// RAM3 at address 0
    REMAP_ADDR0_TO_RAM3 = 3U,
} remap_addr0_t;

/// Selects which memory block is located at address 0x0000 for execution after
/// waking up from stateful hibernation
typedef enum
{
    /// RAM1 at address 0
    STATEFUL_HIBERN_REMAP_ADDR0_TO_RAM1 = 2U,

    /// RAM2 at address 0
    STATEFUL_HIBERN_REMAP_ADDR0_TO_RAM2 = 3U,

    /// RAM3 at address 0
    STATEFUL_HIBERN_REMAP_ADDR0_TO_RAM3 = 3U,
} stateful_hibern_remap_addr0_t;
#endif

/**
 ****************************************************************************************
 * @brief       Disable all sleep modes. The system operates in active / idle modes only.
 ****************************************************************************************
 */
void arch_disable_sleep(void);

/**
 ****************************************************************************************
 * @brief       Activates extended sleep mode. Selects betweeen extended sleep with OTP
 *              copy on or extended sleep with OTP copy off.
 * @param[in]   otp_copy        OTP copy on or off
 ****************************************************************************************
 */
void arch_set_extended_sleep(bool otp_copy);

#if defined (__DA14531__)
/**
 ****************************************************************************************
 * @brief       Activates deep sleep mode.
 * @param[in]   ram1           Selects the RAM1 state (on/off) during the hibernation.
 * @param[in]   ram2           Selects the RAM2 state (on/off) during the hibernation.
 * @param[in]   ram3           Selects the RAM3 state (on/off) during the hibernation.
 * @param[in]   pad_latch_en   true = Enable latching of pads state during sleep,
 *                             false = Disable latching of pads state during sleep.
 * @details     The system can wake-up from the following sources:
 *                  - POR (power on reset)
 *                  - GPIO(s) using the wake-up controller
 *                  - Interrupt generated by the RTC
 *                  - Interrupt generated by the Timer1
 *
 * @note        The wake-up condition must be configured prior to putting the system in
 *              deep sleep.
 *              If RTC and Timer1 are not going to be used as wake-up sources, it is
 *              recommended to close the PD_TIM prior to putting the system in deep sleep.
 *              The exit from the deep sleep state causes a system reboot.
 ****************************************************************************************
 */
void arch_set_deep_sleep(pd_sys_down_ram_t ram1,
                         pd_sys_down_ram_t ram2,
                         pd_sys_down_ram_t ram3,
                         bool pad_latch_en);

#else
/**
 ****************************************************************************************
 * @brief       Activates deep sleep mode. The system can boot again under a POR 
 *              (power on reset) or external wake-up interrupt condition.
 * @param[in]   ext_wakeup      External wake-up interrupt flag:
 *                                  - true: an external interrupt can wake-up and reboot
 *                                          the system
 *                                  - false: no external interrupt can wake-up and reboot
 *                                           the system
 *
 * @note        A POR condition always reboots the system.
 *              The GPIO(s), which may generate the POR or the wake-up interrupt, must 
 *              have already been configured before putting the system in deep sleep.
 ****************************************************************************************
 */
void arch_set_deep_sleep(bool ext_wakeup);
#endif

/**
 ****************************************************************************************
 * @brief       Activates selected sleep mode.
 * @param       sleep_state     Selected sleep mode.
 ****************************************************************************************
 */
void arch_set_sleep_mode(sleep_state_t sleep_state);

/**
 ****************************************************************************************
 * @brief       Get the current sleep mode of operation.
 * @return      The current sleep mode of operation.
 ****************************************************************************************
 */
uint8_t arch_get_sleep_mode(void);

/**
 ****************************************************************************************
 * @brief       Restore the sleep mode to what it was before disabling. App should not modify the sleep mode directly.
 * @details     Restores the sleep mode. 
 ****************************************************************************************
 */
void arch_restore_sleep_mode(void);

/**
 ****************************************************************************************
 * @brief       Disable sleep but save the sleep mode status. Store the sleep mode used by the app.
 ****************************************************************************************
 */
void arch_force_active_mode(void);

/**
 ****************************************************************************************
 * @brief       Puts the BLE core to permanent sleep. Only an external event can wake it up.\
 *              BLE sleeps forever waiting a forced wakeup. After waking up from an external 
 *              event, if the system has to wake BLE up it needs to restore the default mode 
 *              of operation by calling app_ble_ext_wakeup_off() or the BLE won't be able to
 *              wake up in order to serve BLE events!
 *
 * @exception   Warning Assertion
 *              if rwip_env.ext_wakeup_enable is 0 since it wouldn't be possible to wake-up the 
 *              BLE core in this case.
 ****************************************************************************************
 */
void arch_ble_ext_wakeup_on(void);

/**
 ****************************************************************************************
 * @brief       Takes the BLE core out of the permanent sleep mode.Restore BLE cores' operation to default mode. The BLE core will wake up every 
 *              "CFG_MAX_SLEEP_DURATION_EXTERNAL_WAKEUP_MS" even if no BLE events are scheduled. If an event is to be scheduled 
 *              earlier, then BLE will wake up sooner to serve it.
 ****************************************************************************************
 */
void arch_ble_ext_wakeup_off(void);

/**
 ****************************************************************************************
 * @brief       Checks whether the BLE core is in permanent sleep mode or not.
 *              Returns the current mode of operation of the BLE core (external wakeup or default).
 * @return      The BLE core's mode of operation (extended wakeup or normal).
 * @retval      The BLE core's sleep mode.
 *              <ul>
 *                  <li> false, if default mode is selected
 *                  <li> true, if BLE sleeps forever waiting for a forced wakeup
 *              </ul>
 ****************************************************************************************
 */
bool arch_ble_ext_wakeup_get(void);

/**
 ****************************************************************************************
 * @brief       Wake the BLE core via an external request. If the BLE core is sleeping (permanently or not and external wake-up is enabled)
 *              then this function wakes it up. 
 *              A call to app_ble_ext_wakeup_off() should follow if the BLE core was in permanent sleep.
 * @return      status
 * @retval      The status of the requested operation.
 *              <ul>
 *                  <li> false, if the BLE core is not sleeping
 *                  <li> true, if the BLE core was woken-up successfully
 *              </ul>
 ****************************************************************************************
 */
bool arch_ble_force_wakeup(void);

/**
 ****************************************************************************************
 * @brief       Modifies the system startup sleep delay. It has effect ONLY if the low
 *              power clock is the XTAL32K.
 * @param[in]   delay       Delay in BLE slots (0.625 usec)
 ****************************************************************************************
 */
__STATIC_FORCEINLINE void arch_startup_sleep_delay_set(uint32_t delay)
{
    startup_sleep_delay = delay;
}

/*
##############################################################################
#   BLE events API                                                           #
##############################################################################
*/

/*Last BLE event. Used for application's synchronisation with BLE activity */
typedef enum 
{
    BLE_EVT_SLP,
    BLE_EVT_CSCNT,
    BLE_EVT_RX,
    BLE_EVT_TX,
    BLE_EVT_END,

}last_ble_evt;

/**
 ****************************************************************************************
 * @brief       Used for application's tasks synchronisation with ble events. 
 * @return      Last BLE event. See last_ble_evt enumaration
 ****************************************************************************************
 */
last_ble_evt arch_last_rwble_evt_get(void);

/*
##############################################################################
#   DEVELOPMENT DEBUG API                                                    #
##############################################################################
*/

#if defined (__DA14531__)
/**
 ****************************************************************************************
 * @brief Creates S/W cursor in power profiler tool(Smart Snippets Studio).
          Used for Development/Debugging.
 * @param[in] port          GPIO port
 * @param[in] pin           GPIO pin
 ****************************************************************************************
 */
void arch_set_pxact_gpio(GPIO_PORT port, GPIO_PIN pin);
#else
/**
 ****************************************************************************************
 * @brief Creates S/W cursor in power profiler tool(Smart Snippets Studio).
 *        Used for Development/Debugging.
 ****************************************************************************************
 */
void arch_set_pxact_gpio(void);
#endif

/*
##############################################################################
#   MISC                                                                     #
##############################################################################
*/

/*
 * LOW POWER CLOCK - Used for CFG_LP_CLK configuration
 ****************************************************************************************
 */
#define LP_CLK_XTAL32       0x00
#define LP_CLK_RCX20        0xAA
#define LP_CLK_FROM_OTP     0xFF


/*
##############################################################################
#   MAIN LOOP CALLBACKS API                                                  #
##############################################################################
*/

typedef enum {
    GOTO_SLEEP = 0,
    KEEP_POWERED
} arch_main_loop_callback_ret_t;

/*
 * Main loop callback functions' structure. Developer must set application's callback functions in
 * the initialization of the structure in user_callback_config.h 
 ****************************************************
 */
struct arch_main_loop_callbacks {
    void (*app_on_init)(void);
    arch_main_loop_callback_ret_t (*app_on_ble_powered)(void);
    arch_main_loop_callback_ret_t (*app_on_system_powered)(void);
    void (*app_before_sleep)(void);
    sleep_mode_t (*app_validate_sleep)(sleep_mode_t sleep_mode);
    void (*app_going_to_sleep)(sleep_mode_t sleep_mode);
    void (*app_resume_from_sleep)(void);  
};    


/*
##############################################################################
#   BLE METRICS API                                                          #
##############################################################################
*/
#if (BLE_METRICS)

typedef struct
{
    uint32_t    rx_pkt;
    uint32_t    rx_err;
    uint32_t    rx_err_crc;
    uint32_t    rx_err_sync;
    uint16_t    rx_rssi;
}arch_ble_metrics_t;

extern arch_ble_metrics_t            metrics;

/**
 ****************************************************************************************
 * @brief Returns a pointer to ble metrics structure keeping device's statistics. 
 *
 * @return     arch_ble_metrics_t          Pointer to ble metrics structure
 ****************************************************************************************
 */
__STATIC_FORCEINLINE arch_ble_metrics_t * arch_ble_metrics_get(void)
{
    return &metrics;
}

/**
 ****************************************************************************************
 * @brief Reset the counters of ble metrics structure. 
 ****************************************************************************************
 */
__STATIC_FORCEINLINE void arch_ble_metrics_reset(void)
{
    memset(&metrics, 0, sizeof(arch_ble_metrics_t));
}

#endif

#endif // _ARCH_API_H_

/// @}
/// @}
