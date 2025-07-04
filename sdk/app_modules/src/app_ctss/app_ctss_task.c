/**
 ****************************************************************************************
 *
 * @file app_ctss_task.c
 *
 * @brief Current Time Service Application Task
 *
 * Copyright (C) 2017-2025 Renesas Electronics Corporation and/or its affiliates.
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
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"        // SW Configuration

#if (BLE_APP_PRESENT)

#if (BLE_CTS_SERVER)
#include <stdint.h>
#include "ctss_task.h"          // Current Time Service Server Task API
#include "ctss.h"               // Current Time Service Definitions
#include "app_ctss.h"           // Current Time Service Application Definitions
#include "app_ctss_task.h"      // Current Time Service Application Task API
#include "app_task.h"           // Application Task API
#include "app_entry_point.h"
#include "app.h"
#include "user_profiles_config.h"
#include "user_callback_config.h"

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref CTSS_NTF_CURR_TIME_CFM message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int ctss_cur_time_notify_handler(ke_msg_id_t const msgid,
                                        const struct ctss_ntf_curr_time_rsp *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    CALLBACK_ARGS_1(user_app_ctss_cb.on_cur_time_notified, param->status);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref CTSS_READ_REQ_IND message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int ctss_read_req_ind_handler(ke_msg_id_t const msgid,
                                     const struct ctss_read_req_ind *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    // Initialize length
    uint8_t len = 0;
    // buffer that can hold current_time, local_time or ref_time
    uint8_t data[10];
    struct cts_curr_time current_time;
    struct cts_loc_time_info local_time;
    struct cts_ref_time_info ref_time;

    // Check requested value
    switch (param->value)
    {
    case CTSS_IDX_CURRENT_TIME_VAL:
        // Set information
        user_app_ctss_cb.on_cur_time_read_req(&current_time);
        ctss_pack_curr_time_value(&current_time, data);
        len = 10;
        break;

    case CTSS_IDX_LOCAL_TIME_INFO_VAL:
        // Set information
        data[0] = local_time.time_zone;
        data[1] = local_time.dst_offset;
        len = 2;
        break;

    case CTSS_IDX_REF_TIME_INFO_VAL:
        // Set information
        user_app_ctss_cb.on_ref_time_info_read_req(&ref_time);
        data[0] = ref_time.time_source;
        data[1] = ref_time.time_accuracy;
        data[2] = ref_time.days_update;
        data[3] = ref_time.hours_update;
        len = 4;
        break;

    default:
        ASSERT_ERROR(0);
        break;
    }

    // Allocate confirmation to send the value
    struct ctss_read_rsp *rsp = KE_MSG_ALLOC_DYN(CTSS_READ_RSP,
            src_id, dest_id,
            ctss_read_rsp,
            len);

    // Set parameters
    rsp->value = param->value;
    rsp->length = len;
    if (len)
    {
        // Copy data
        memcpy(rsp->data, data, len);
    }
    // Send message
    KE_MSG_SEND(rsp);

    return KE_MSG_CONSUMED;
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref CTSS_WRITE_CUR_TIME_REQ_IND message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int ctss_cur_time_write_req_ind_handler(ke_msg_id_t const msgid,
                                               const struct ctss_cur_time_write_ind *param,
                                               ke_task_id_t const dest_id,
                                               ke_task_id_t const src_id)
{
    uint8_t status = ATT_ERR_WRITE_NOT_PERMITTED;

    if (user_app_ctss_cb.on_cur_time_write_req)
    {
        status = user_app_ctss_cb.on_cur_time_write_req(&param->current_time);
    }

    // Allocate confirmation to send the value
    struct ctss_write_rsp *rsp = KE_MSG_ALLOC(CTSS_WRITE_RSP,
                                              src_id, dest_id,
                                              ctss_write_rsp);

    // Set parameters
    rsp->value = param->value;
    rsp->status = status;

    // Send message
    KE_MSG_SEND(rsp);

    return KE_MSG_CONSUMED;
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref CTSS_WRITE_LOC_TIME_REQ_IND message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int ctss_loc_time_info_write_req_ind_handler(ke_msg_id_t const msgid,
                                                    const struct ctss_loc_time_info_write_ind *param,
                                                    ke_task_id_t const dest_id,
                                                    ke_task_id_t const src_id)
{
    if (user_app_ctss_cb.on_loc_time_info_write_req)
    {
        user_app_ctss_cb.on_loc_time_info_write_req(&param->local_time_info);
    }

    return KE_MSG_CONSUMED;
}

/*
 * LOCAL VARIABLES DEFINITION
 ****************************************************************************************
 */

static const struct ke_msg_handler app_ctss_process_handlers[] =
{
    {CTSS_READ_REQ_IND,           (ke_msg_func_t) ctss_read_req_ind_handler},
    {CTSS_WRITE_LOC_TIME_REQ_IND, (ke_msg_func_t) ctss_loc_time_info_write_req_ind_handler},
    {CTSS_WRITE_CUR_TIME_REQ_IND, (ke_msg_func_t) ctss_cur_time_write_req_ind_handler},
    {CTSS_NTF_CURR_TIME_CFM,      (ke_msg_func_t) ctss_cur_time_notify_handler},
};

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

enum process_event_response app_ctss_process_handler(ke_msg_id_t const msgid,
                                                     void const *param,
                                                     ke_task_id_t const dest_id,
                                                     ke_task_id_t const src_id,
                                                     enum ke_msg_status_tag *msg_ret)
{
    return (app_std_process_event(msgid, param, src_id, dest_id, msg_ret, app_ctss_process_handlers,
                                         sizeof(app_ctss_process_handlers) / sizeof(struct ke_msg_handler)));
}

#endif //(BLE_CTS_SERVER)

#endif //BLE_APP_PRESENT

/// @} APP
