/**
 ****************************************************************************************
 *
 * @file app_bass_task.c
 *
 * @brief Battery server application task.
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

/**
 ****************************************************************************************
 * @addtogroup APPTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_APP_PRESENT)

#if (BLE_BATT_SERVER)

#include "bass_task.h"
#include "app_bass.h"
#include "app_bass_task.h"
#include "gpio.h"
#include "app_entry_point.h"
#include "app.h"
#include "user_callback_config.h"

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

static int bass_enable_rsp_handler(ke_msg_id_t const msgid,
                                   struct bass_enable_rsp const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

static int bass_batt_level_upd_rsp_handler(ke_msg_id_t const msgid,
                                           struct bass_batt_level_upd_rsp const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    CALLBACK_ARGS_1(user_app_bass_cb.on_batt_level_upd_rsp, param->status)
    
    return (KE_MSG_CONSUMED);
}

static int bass_batt_level_ntf_cfg_ind_handler(ke_msg_id_t const msgid,
                                               struct bass_batt_level_ntf_cfg_ind const *ind,
                                               ke_task_id_t const dest_id,
                                               ke_task_id_t const src_id)
{
    CALLBACK_ARGS_2(user_app_bass_cb.on_batt_level_ntf_cfg_ind, ind->conidx, ind->ntf_cfg)
    
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles Battery Level polling timer
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int app_bass_timer_handler(ke_msg_id_t const msgid,
                                  void const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    app_batt_lvl();

    app_timer_set(APP_BASS_TIMER, dest_id, bat_poll_timeout);

    return (KE_MSG_CONSUMED);
}

#ifndef CUSTOM_BATTERY_LEVEL_ALERT_LED_HANDLING
/**
 ****************************************************************************************
 * @brief Handles Battery Alert time.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int app_bass_alert_timer_handler(ke_msg_id_t const msgid,
                                        void const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    // Read LED GPIO state
    if (bat_lvl_alert_used)
    {
        if (bat_led_state)
        {
            GPIO_SetInactive(bat_led_port, bat_led_pin);
            bat_led_state = 0;
            app_timer_set(APP_BASS_ALERT_TIMER, dest_id, 20);
        }
        else
        {
            GPIO_SetActive(bat_led_port, bat_led_pin);
            bat_led_state = 1;
            app_timer_set(APP_BASS_ALERT_TIMER, dest_id, 5);
        }
    }
    return (KE_MSG_CONSUMED);
}
#endif // CUSTOM_BATTERY_LEVEL_ALERT_LED_HANDLING

/*
 * LOCAL VARIABLES DEFINITION
 ****************************************************************************************
 */

static const struct ke_msg_handler app_bass_process_handlers[] =
{
    {BASS_ENABLE_RSP,                       (ke_msg_func_t)bass_enable_rsp_handler},
    {BASS_BATT_LEVEL_UPD_RSP,               (ke_msg_func_t)bass_batt_level_upd_rsp_handler},
    {BASS_BATT_LEVEL_NTF_CFG_IND,           (ke_msg_func_t)bass_batt_level_ntf_cfg_ind_handler},
    {APP_BASS_TIMER,                        (ke_msg_func_t)app_bass_timer_handler},
    {APP_BASS_ALERT_TIMER,                  (ke_msg_func_t)app_bass_alert_timer_handler},
};

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

enum process_event_response app_bass_process_handler(ke_msg_id_t const msgid,
                                                     void const *param,
                                                     ke_task_id_t const dest_id,
                                                     ke_task_id_t const src_id,
                                                     enum ke_msg_status_tag *msg_ret)
{
    return app_std_process_event(msgid, param,src_id, dest_id, msg_ret, app_bass_process_handlers,
                                         sizeof(app_bass_process_handlers) / sizeof(struct ke_msg_handler));
}

#endif //(BLE_BATT_SERVER)

#endif //(BLE_APP_PRESENT)

/// @} APPTASK
