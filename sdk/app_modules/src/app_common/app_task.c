/**
 ****************************************************************************************
 *
 * @file app_task.c
 *
 * @brief RW APP Task implementation
 *
 * Copyright (C) RivieraWaves 2009-2013
 * Copyright (C) 2017-2022 Modified by Renesas Electronics Corporation and/or its affiliates.
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

#include "rwip_config.h"            // SW configuration

#if (BLE_APP_PRESENT)

#include <string.h>
#include "gapc_task.h"          // GAP Controller Task API
#include "gapm_task.h"          // GAP Manager Task API
#include "gap.h"                // GAP Definitions
#include "gapc.h"               // GAPC Definitions
#include "gattc_task.h"         // GATT Controller Task API
#include "co_error.h"           // Error Codes Definition
#include "co_bt.h"
#include "arch.h"               // Platform Definitions
#include "app_task.h"           // Application Task API
#include "app.h"                // Application Definition
#include "app_security.h"       // Application Definition
#include "app_security_task.h"  // Application Security Task API
#include "app_mid.h"
#include "app_callback.h"
#include "app_default_handlers.h"
#include "app_entry_point.h"
#include "app_utils.h"
#include "user_callback_config.h"
#include "user_modules_config.h"

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

extern const struct app_entry_catch_rest catch_rest;

static uint8_t app_gap_current_handled_connection_idx         __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY

uint8_t app_gap_get_current_handled_connection_idx(void)
{
    return app_gap_current_handled_connection_idx;
}

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (__APP_TASK_BUILD_FOR_DA14531__) ||         \
    (__APP_TASK_BUILD_FOR_DA1458X__) ||         \
    ((__APP_TASK_BUILD_FOR_DA14535__ || __APP_TASK_BUILD_FOR_DA14531_01__) && defined (__EXCLUDE_ROM_APP_TASK__))
/**
 ****************************************************************************************
 * @brief Handles ready indication from the GAP. - Reset the stack.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_device_ready_ind_handler(ke_msg_id_t const msgid,
                                         void const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
    if (ke_state_get(dest_id) == APP_DISABLED)
    {
        // reset the lower layers.
        app_gapm_reset_op();
    }
    else
    {
        // APP_DISABLED state is used to wait the GAP_READY_EVT message
        ASSERT_ERROR(0);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles GAP manager command complete events.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapm_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    switch(param->operation)
    {
        // reset completed
        case GAPM_RESET:
        {
            if(param->status != GAP_ERR_NO_ERROR)
            {
                ASSERT_ERROR(0); // unexpected error
            }
            else
            {
                // set device configuration
                app_easy_gap_dev_configure();
            }
        }
        break;

        // device configuration updated
        case GAPM_SET_DEV_CONFIG:
        {
            if(param->status != GAP_ERR_NO_ERROR)
            {
                ASSERT_ERROR(0); // unexpected error
            }
            else
            {
                CALLBACK_ARGS_0(user_app_callbacks.app_on_set_dev_config_complete)
            }
        }
        break;

        // Non connectable advertising completed
        case GAPM_ADV_NON_CONN:
        {
            CALLBACK_ARGS_1(user_app_callbacks.app_on_adv_nonconn_complete, param->status)
        }
        break;

        // Undirected connectable advertising completed
        case GAPM_ADV_UNDIRECT:
        {
            CALLBACK_ARGS_1(user_app_callbacks.app_on_adv_undirect_complete, param->status)
        }
        break;

        // Directed connectable advertising completed
        case GAPM_ADV_DIRECT:
        case GAPM_ADV_DIRECT_LDC:
        {
            CALLBACK_ARGS_1(user_app_callbacks.app_on_adv_direct_complete, param->status)
        }
        break;

        case GAPM_SCAN_ACTIVE:
        case GAPM_SCAN_PASSIVE:
        {
            CALLBACK_ARGS_1(user_app_callbacks.app_on_scanning_completed, param->status)
        }
        break;

        case GAPM_CONNECTION_DIRECT:
            if (param->status == GAP_ERR_CANCELED)
            {
                CALLBACK_ARGS_0(user_app_callbacks.app_on_connect_failed)
            }
        break;

#if (BLE_APP_SEC) && (!EXCLUDE_DLG_SEC)
        case GAPM_RESOLV_ADDR:
            if (param->status == GAP_ERR_NOT_FOUND)
            {
                CALLBACK_ARGS_1(user_app_callbacks.app_on_addr_resolve_failed, KE_IDX_GET(src_id))
            }
        break;
#if !defined (__DA14531_01__) && !defined (__DA14535__)
// Controller Privacy Completion Events (not for DA14531_01, DA14535)
#if (USER_CFG_ADDRESS_MODE == APP_CFG_CNTL_PRIV_RPA_PUB) || (USER_CFG_ADDRESS_MODE == APP_CFG_CNTL_PRIV_RPA_RAND)
        case GAPM_GET_RAL_SIZE:
        case GAPM_GET_RAL_LOC_ADDR:
        case GAPM_GET_RAL_PEER_ADDR:
        case GAPM_ADD_DEV_IN_RAL:
        case GAPM_RMV_DEV_FRM_RAL:
        case GAPM_CLEAR_RAL:
        {
            CALLBACK_ARGS_1(user_app_callbacks.app_on_ral_cmp_evt, param)
        }
        break;
#endif // Controller Privacy Completion Events
#endif // DA14531_01 or DA14535
#endif // (BLE_APP_SEC)

        case GAPM_CANCEL:
        default:
          CALLBACK_ARGS_4(catch_rest.cb, msgid, param, dest_id, src_id);
        break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles connection complete event from the GAP. Will enable profile.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_connection_req_ind_handler(ke_msg_id_t const msgid,
                                           struct gapc_connection_req_ind const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    // Connection Index
    if (ke_state_get(dest_id) == APP_CONNECTABLE)
    {
        uint8_t conidx = KE_IDX_GET(src_id);
        ASSERT_WARNING(conidx < APP_EASY_MAX_ACTIVE_CONNECTION);
        app_env[conidx].conidx = conidx;

        if (conidx != GAP_INVALID_CONIDX)
        {
            app_env[conidx].connection_active = true;
            ke_state_set(TASK_APP, APP_CONNECTED);
            // Retrieve the connection info from the parameters
            app_env[conidx].conhdl = param->conhdl;
            app_env[conidx].peer_addr_type = param->peer_addr_type;
            memcpy(app_env[conidx].peer_addr.addr, param->peer_addr.addr, BD_ADDR_LEN);
            #if (BLE_APP_SEC)
                // send connection confirmation
                app_easy_gap_confirm(conidx, (enum gap_auth) app_sec_env[conidx].auth, 1);
            #else
                app_easy_gap_confirm(conidx, GAP_AUTH_REQ_NO_MITM_NO_BOND, 1);
            #endif
        }
        CALLBACK_ARGS_2(user_app_callbacks.app_on_connection, conidx, param)
    }
    else
    {
        // APP_CONNECTABLE state is used to wait the GAP_LE_CREATE_CONN_REQ_CMP_EVT message
        ASSERT_ERROR(0);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles GAP controller command complete events.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapc_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    switch(param->operation)
    {
        // reset completed
        case GAPC_UPDATE_PARAMS:
        {
            if ((param->status != CO_ERROR_NO_ERROR))
            {
                // it's application specific what to do when the Param Upd request is rejected
                CALLBACK_ARGS_1(user_app_callbacks.app_on_update_params_rejected, param->status)
            }
            else
            {
                CALLBACK_ARGS_0(user_app_callbacks.app_on_update_params_complete)
            }
        }
        break;

        default:
        {
            if ((param->operation == GAPC_DISCONNECT) && (param->status == GAP_ERR_COMMAND_DISALLOWED))
            {
                // Application can receive from the ble stack the following allowed
                // message with parameters:
                // param->operation = GAPC_DISCONNECT
                // param->status    = GAP_ERR_COMMAND_DISALLOWED
                // In that case it is better to continue, instead of ending up to
                // the ASSERT_ERROR() breakpoint that follows.
            }
            else if (param->status != GAP_ERR_NO_ERROR)
            {
                ASSERT_ERROR(0); // unexpected error
            }
            CALLBACK_ARGS_4(catch_rest.cb, msgid, param, dest_id, src_id);
        }
        break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles disconnection complete event from the GAP.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_disconnect_ind_handler(ke_msg_id_t const msgid,
                                       struct gapc_disconnect_ind const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t conidx = KE_IDX_GET(src_id);

    if (state == APP_CONNECTED)
    {
        app_env[conidx].conidx = GAP_INVALID_CONIDX;
        app_env[conidx].connection_active = false;
        CALLBACK_ARGS_1(user_app_callbacks.app_on_disconnect, param)
    }
    else
    {
        // We are not in Connected State
        ASSERT_ERROR(0);
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles GAPC_GET_DEV_INFO_REQ_IND event.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
*/
static int gapc_get_dev_info_req_ind_handler(ke_msg_id_t msgid,
                                             struct gapc_get_dev_info_req_ind *param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    switch(param->req)
    {
        case GAPC_DEV_NAME:
        {
            ASSERT_ERROR(USER_DEVICE_NAME_LEN <= BD_NAME_SIZE);

            struct app_device_name device_name = {0};

            CALLBACK_ARGS_1(user_app_callbacks.app_on_get_dev_name, &device_name)

            struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC_DYN(GAPC_GET_DEV_INFO_CFM,
                                                                 src_id,
                                                                 dest_id,
                                                                 gapc_get_dev_info_cfm,
                                                                 device_name.length);
            cfm->req = GAPC_DEV_NAME;
            cfm->info.name.length = device_name.length;
            memcpy(cfm->info.name.value, device_name.name , cfm->info.name.length);

            KE_MSG_SEND(cfm);
        }
        break;

        case GAPC_DEV_APPEARANCE:
        {
            uint16_t appearance = 0x0000;

            CALLBACK_ARGS_1(user_app_callbacks.app_on_get_dev_appearance, &appearance)

            struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
                                                             src_id,
                                                             dest_id,
                                                             gapc_get_dev_info_cfm);
            cfm->req = GAPC_DEV_APPEARANCE;
            cfm->info.appearance = appearance;

            KE_MSG_SEND(cfm);
        }
        break;

        case GAPC_DEV_SLV_PREF_PARAMS:
        {
            struct gap_slv_pref slv_params = {0};

            struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
                                                             src_id,
                                                             dest_id,
                                                             gapc_get_dev_info_cfm);

            CALLBACK_ARGS_1(user_app_callbacks.app_on_get_dev_slv_pref_params, &slv_params)

            cfm->req = GAPC_DEV_SLV_PREF_PARAMS;
            cfm->info.slv_params.con_intv_min = slv_params.con_intv_min;
            cfm->info.slv_params.con_intv_max = slv_params.con_intv_max;
            cfm->info.slv_params.slave_latency = slv_params.slave_latency;
            cfm->info.slv_params.conn_timeout = slv_params.conn_timeout;

            KE_MSG_SEND(cfm);
        }
        break;

        default: /* Do Nothing */ break;
    }

    return (KE_MSG_CONSUMED);
}
#endif // (__APP_TASK_BUILD_FOR_DA14531__) ||         \
       // (__APP_TASK_BUILD_FOR_DA1458X__) ||         \
       // ((__APP_TASK_BUILD_FOR_DA14535__ || __APP_TASK_BUILD_FOR_DA14531_01__) && defined (__EXCLUDE_ROM_APP_TASK__))

#if (__APP_TASK_BUILD_FOR_DA1458X__) || (__EXCLUDE_ROM_APP_TASK__)

/*
 ****************************************************************************************
 * @brief Handles GAPM_ADV_REPORT_IND event.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
*/
static int gapm_adv_report_ind_handler(ke_msg_id_t msgid,
                                       struct gapm_adv_report_ind *param,
                                       ke_task_id_t dest_id,
                                       ke_task_id_t src_id)
{
    CALLBACK_ARGS_1(user_app_callbacks.app_on_adv_report_ind, param)
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles GAPC_SECURITY_IND event.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
*/
#if (BLE_APP_SEC)
static int gapc_security_ind_handler(ke_msg_id_t msgid,
                                     struct gapc_security_ind *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    CALLBACK_ARGS_1(user_app_callbacks.app_on_security_req_ind, param->auth)
    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles GAPC_SET_DEV_INFO_REQ_IND event.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
*/
static int gapc_set_dev_info_req_ind_handler(ke_msg_id_t msgid,
                                             struct gapc_set_dev_info_req_ind *param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    uint8_t status = GAP_ERR_REJECTED;
    uint8_t conidx = KE_IDX_GET(src_id);

    CALLBACK_ARGS_2(user_app_callbacks.app_on_set_dev_info, param, &status)

    // Set Device configuration
    struct gapc_set_dev_info_cfm* cfm = KE_MSG_ALLOC(GAPC_SET_DEV_INFO_CFM,
                                                     KE_BUILD_ID(TASK_GAPC, conidx),
                                                     TASK_APP,
                                                     gapc_set_dev_info_cfm);

    // Local device accept or reject device info modification
    cfm->status = status;
    cfm->req = param->req;

    // Send message
    KE_MSG_SEND(cfm);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles GAPM_PROFILE_ADDED_IND event.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
*/
static int gapm_profile_added_ind_handler(ke_msg_id_t msgid,
                                          struct gapm_profile_added_ind *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
     // Current State
     uint8_t state = ke_state_get(dest_id);

     if (state == APP_DB_INIT)
     {
         // Add the next requested profile/service
        if (app_db_init())
        {
            // Go to the connectable state
            ke_state_set(TASK_APP, APP_CONNECTABLE);

            // No profiles/services to add in the DB -> Start Advertising
            CALLBACK_ARGS_0(user_app_callbacks.app_on_db_init_complete);
        }
     }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles GAPC_PARAM_UPDATE_REQ_IND event.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
*/
static int gapc_param_update_req_ind_handler(ke_msg_id_t msgid,
                                             struct gapc_param_update_req_ind *param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    struct gapc_param_update_cfm* cfm = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CFM,
                                                     src_id,
                                                     dest_id,
                                                     gapc_param_update_cfm);

    CALLBACK_ARGS_2(user_app_callbacks.app_on_update_params_request, param, cfm)

    // Send message
    KE_MSG_SEND(cfm);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles GAPC_LE_PKT_SIZE_IND event.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
*/
static int gapc_le_pkt_size_ind_handler(ke_msg_id_t msgid,
                                        struct gapc_le_pkt_size_ind *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(src_id);
    CALLBACK_ARGS_2(user_app_callbacks.app_on_data_length_change, conidx, param)

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles GATTC_SVC_CHANGED_CFG_IND message.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
*/
static int gattc_svc_changed_cfg_ind_handler(ke_msg_id_t msgid,
                                             struct gattc_svc_changed_cfg *param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(src_id);
    CALLBACK_ARGS_2(user_app_callbacks.app_on_svc_changed_cfg_ind, conidx, param->ind_cfg);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles GAPC_PEER_FEATURES_IND message.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
*/
static int gapc_peer_features_ind_handler(ke_msg_id_t msgid,
                                          struct gapc_peer_features_ind const *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(src_id);

    CALLBACK_ARGS_2(user_app_callbacks.app_on_get_peer_features, conidx, param);

    return (KE_MSG_CONSUMED);
}
#endif // (__APP_TASK_BUILD_FOR_DA1458X__) || (__EXCLUDE_ROM_APP_TASK__)

#if (APP_IDX_MAX > 1)
/**
 ****************************************************************************************
 * @brief Handles ready indication from the GAP. - Reset the stack.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_device_ready_ind_handler_MUL_CON(ke_msg_id_t const msgid,
                                         void const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
    if (ke_state_get(KE_BUILD_ID(KE_TYPE_GET(dest_id), KE_IDX_GET(src_id))) == APP_DISABLED)
    {
        // reset the lower layers.
        app_gapm_reset_op();
    }
    else
    {
        // APP_DISABLED state is used to wait the GAP_READY_EVT message
        ASSERT_ERROR(0);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles connection complete event from the GAP. Will enable profile.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_connection_req_ind_handler_MUL_CON(ke_msg_id_t const msgid,
                                           struct gapc_connection_req_ind const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(src_id);
    uint8_t state = ke_state_get(KE_BUILD_ID(KE_TYPE_GET(dest_id), conidx));

    if (state == APP_CONNECTABLE)
    {
        ASSERT_WARNING(conidx < APP_IDX_MAX);
        app_env[conidx].conidx = conidx;

        if (conidx != GAP_INVALID_CONIDX)
        {
            app_env[conidx].connection_active = true;
            ke_state_set(KE_BUILD_ID(KE_TYPE_GET(dest_id), conidx), APP_CONNECTED);
            // Retrieve the connection info from the parameters
            app_env[conidx].conhdl = param->conhdl;
            app_env[conidx].peer_addr_type = param->peer_addr_type;
            memcpy(app_env[conidx].peer_addr.addr, param->peer_addr.addr, BD_ADDR_LEN);
            #if (BLE_APP_SEC)
                // send connection confirmation
                app_easy_gap_confirm(conidx, (enum gap_auth) app_sec_env[conidx].auth, 1);
            #else
                app_easy_gap_confirm(conidx, GAP_AUTH_REQ_NO_MITM_NO_BOND, 1);
            #endif
        }
        CALLBACK_ARGS_2(user_app_callbacks.app_on_connection, conidx, param)
    }
    else
    {
        // APP_CONNECTABLE state is used to wait the GAP_LE_CREATE_CONN_REQ_CMP_EVT message
        ASSERT_ERROR(0);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles disconnection complete event from the GAP.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_disconnect_ind_handler_MUL_CON(ke_msg_id_t const msgid,
                                       struct gapc_disconnect_ind const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(src_id);
    uint8_t state = ke_state_get(KE_BUILD_ID(KE_TYPE_GET(dest_id), conidx));

    if (state == APP_CONNECTED)
    {
        // Go back to connectable state
        ke_state_set(KE_BUILD_ID(KE_TYPE_GET(dest_id), conidx), APP_CONNECTABLE);
        app_env[conidx].conidx = GAP_INVALID_CONIDX;
        app_env[conidx].connection_active = false;
        CALLBACK_ARGS_1(user_app_callbacks.app_on_disconnect, param)
    }
    else
    {
        // We are not in Connected State
        ASSERT_ERROR(0);
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles GAPM_PROFILE_ADDED_IND event.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
*/
static int gapm_profile_added_ind_handler_MUL_CON(ke_msg_id_t msgid,
                                          struct gapm_profile_added_ind *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
     // Current State
     uint8_t state = ke_state_get(KE_BUILD_ID(KE_TYPE_GET(dest_id), KE_IDX_GET(src_id)));

     if (state == APP_DB_INIT)
     {
         // Add the next requested profile/service
        if (app_db_init())
        {
            // Go to the connectable state
            for(uint8_t idx = 0; idx < APP_IDX_MAX; idx++)
                ke_state_set(KE_BUILD_ID(TASK_APP, idx), APP_CONNECTABLE);

            // No profiles/services to add in the DB -> Start Advertising
            CALLBACK_ARGS_0(user_app_callbacks.app_on_db_init_complete);
        }
     }

    return (KE_MSG_CONSUMED);
}

#endif // (APP_IDX_MAX > 1)

extern int gapm_cmp_evt_handler_ROM(ke_msg_id_t msgid,
                                                 struct gapm_cmp_evt const *param,
                                                 ke_task_id_t const dest_id,
                                                 ke_task_id_t const src_id);
                                                 
extern int gapc_set_dev_info_req_ind_handler_ROM(ke_msg_id_t msgid,
                                                 struct gapc_set_dev_info_req_ind const *param,
                                                 ke_task_id_t const dest_id,
                                                 ke_task_id_t const src_id);

extern int gapm_profile_added_ind_handler_ROM(ke_msg_id_t msgid,
                                              struct gapm_profile_added_ind const *param,
                                              ke_task_id_t const dest_id,
                                              ke_task_id_t const src_id);

extern int gapm_adv_report_ind_handler_ROM(ke_msg_id_t msgid,
                                           struct gapm_adv_report_ind const *param,
                                           ke_task_id_t dest_id,
                                           ke_task_id_t src_id);

extern int gapc_param_update_req_ind_handler_ROM(ke_msg_id_t msgid,
                                                 struct gapc_param_update_req_ind const *param,
                                                 ke_task_id_t const dest_id,
                                                 ke_task_id_t const src_id);

extern int gapc_le_pkt_size_ind_handler_ROM(ke_msg_id_t msgid,
                                            struct gapc_le_pkt_size_ind const *param,
                                            ke_task_id_t const dest_id,
                                            ke_task_id_t const src_id);

extern int gattc_svc_changed_cfg_ind_handler_ROM(ke_msg_id_t msgid,
                                                 struct gattc_svc_changed_cfg const *param,
                                                 ke_task_id_t const dest_id,
                                                 ke_task_id_t const src_id);

extern int gapc_peer_features_ind_handler_ROM(ke_msg_id_t msgid,
                                              struct gapc_peer_features_ind const *param,
                                              ke_task_id_t const dest_id,
                                              ke_task_id_t const src_id);

extern int gapc_security_ind_handler_ROM(ke_msg_id_t msgid,
                                         struct gapc_security_ind const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id);

extern int gapc_get_dev_info_req_ind_handler_ROM(ke_msg_id_t msgid,
                                         struct gapc_get_dev_info_req_ind const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id);

extern int gapc_cmp_evt_handler_ROM(ke_msg_id_t msgid,
                                         struct gapc_cmp_evt const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id);

#if defined(__EXCLUDE_ROM_APP_TASK__) || (APP_IDX_MAX > 1) || __APP_TASK_BUILD_FOR_DA14531__ || __APP_TASK_BUILD_FOR_DA1458X__
/**
 ****************************************************************************************
 * @brief Handles ready indication from the GAP. - Reset the stack.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_device_ready_ind_handler_wrapper(ke_msg_id_t const msgid,
                                         void const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
    app_gap_current_handled_connection_idx = KE_IDX_GET(src_id);

#if APP_IDX_MAX > 1
    return gapm_device_ready_ind_handler_MUL_CON(msgid, param, dest_id, src_id);
#elif (__EXCLUDE_ROM_APP_TASK__) || (__APP_TASK_BUILD_FOR_DA1458X__) || (__APP_TASK_BUILD_FOR_DA14531__)
    return gapm_device_ready_ind_handler(msgid, param, dest_id, src_id);
#endif
}

/**
 ****************************************************************************************
 * @brief Handles GAP manager command complete events.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_cmp_evt_handler_wrapper(ke_msg_id_t const msgid,
                                struct gapm_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
#if (__EXCLUDE_ROM_APP_TASK__) || (__APP_TASK_BUILD_FOR_DA1458X__) || (__APP_TASK_BUILD_FOR_DA14531__)
    return gapm_cmp_evt_handler(msgid, param, dest_id, src_id);
#else
    return gapm_cmp_evt_handler_ROM(msgid, param, dest_id, src_id);
#endif
}

/**
 ****************************************************************************************
 * @brief Handles connection complete event from the GAP. Will enable profile.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_connection_req_ind_handler_wrapper(ke_msg_id_t const msgid,
                                           struct gapc_connection_req_ind const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    app_gap_current_handled_connection_idx = KE_IDX_GET(src_id);

#if APP_IDX_MAX > 1
    return gapc_connection_req_ind_handler_MUL_CON(msgid, param, dest_id, src_id);
#elif (__EXCLUDE_ROM_APP_TASK__) || (__APP_TASK_BUILD_FOR_DA1458X__) || (__APP_TASK_BUILD_FOR_DA14531__)
    return gapc_connection_req_ind_handler(msgid, param, dest_id, src_id);
#endif
}

/**
 ****************************************************************************************
 * @brief Handles GAP controller command complete events.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_cmp_evt_handler_wrapper(ke_msg_id_t const msgid,
                                struct gapc_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    app_gap_current_handled_connection_idx = KE_IDX_GET(src_id);

#if (__EXCLUDE_ROM_APP_TASK__) || (__APP_TASK_BUILD_FOR_DA1458X__) || (__APP_TASK_BUILD_FOR_DA14531__)
    return gapc_cmp_evt_handler(msgid, param, dest_id, src_id);
#else
    return gapc_cmp_evt_handler_ROM(msgid, param, dest_id, src_id);
#endif
}

/**
 ****************************************************************************************
 * @brief Handles disconnection complete event from the GAP.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_disconnect_ind_handler_wrapper(ke_msg_id_t const msgid,
                                       struct gapc_disconnect_ind const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    app_gap_current_handled_connection_idx = KE_IDX_GET(src_id);

#if APP_IDX_MAX > 1
    return gapc_disconnect_ind_handler_MUL_CON(msgid, param, dest_id, src_id);
#elif (__EXCLUDE_ROM_APP_TASK__) || (__APP_TASK_BUILD_FOR_DA1458X__) || (__APP_TASK_BUILD_FOR_DA14531__)
    return gapc_disconnect_ind_handler(msgid, param, dest_id, src_id);
#endif
}

/*
 ****************************************************************************************
 * @brief Handles GAPC_GET_DEV_INFO_REQ_IND event.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
*/
static int gapc_get_dev_info_req_ind_handler_wrapper(ke_msg_id_t msgid,
                                             struct gapc_get_dev_info_req_ind *param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    app_gap_current_handled_connection_idx = KE_IDX_GET(src_id);

#if (__EXCLUDE_ROM_APP_TASK__) || (__APP_TASK_BUILD_FOR_DA1458X__) || (__APP_TASK_BUILD_FOR_DA14531__)
    return gapc_get_dev_info_req_ind_handler(msgid, param, dest_id, src_id);
#else
    return gapc_get_dev_info_req_ind_handler_ROM(msgid, param, dest_id, src_id);
#endif
}

/*
 ****************************************************************************************
 * @brief Handles GAPM_ADV_REPORT_IND event.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
*/
static int gapm_adv_report_ind_handler_wrapper(ke_msg_id_t msgid,
                                       struct gapm_adv_report_ind *param,
                                       ke_task_id_t dest_id,
                                       ke_task_id_t src_id)
{
#if (__EXCLUDE_ROM_APP_TASK__) || (__APP_TASK_BUILD_FOR_DA1458X__)
    return gapm_adv_report_ind_handler(msgid, param, dest_id, src_id);
#else
    return gapm_adv_report_ind_handler_ROM(msgid, param, dest_id, src_id);
#endif
}

/*
 ****************************************************************************************
 * @brief Handles GAPC_SECURITY_IND event.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
*/
#if (BLE_APP_SEC)
static int gapc_security_ind_handler_wrapper(ke_msg_id_t msgid,
                                     struct gapc_security_ind *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    app_gap_current_handled_connection_idx = KE_IDX_GET(src_id);

#if (__EXCLUDE_ROM_APP_TASK__) || (__APP_TASK_BUILD_FOR_DA1458X__)
    return gapc_security_ind_handler(msgid, param, dest_id, src_id);
#else
    return gapc_security_ind_handler_ROM(msgid, param, dest_id, src_id);
#endif
}
#endif

/*
 ****************************************************************************************
 * @brief Handles GAPC_SET_DEV_INFO_REQ_IND event.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
*/
static int gapc_set_dev_info_req_ind_handler_wrapper(ke_msg_id_t msgid,
                                             struct gapc_set_dev_info_req_ind *param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    app_gap_current_handled_connection_idx = KE_IDX_GET(src_id);

#if (__EXCLUDE_ROM_APP_TASK__) || (__APP_TASK_BUILD_FOR_DA1458X__)
    return gapc_set_dev_info_req_ind_handler(msgid, param, dest_id, src_id);
#else
    return gapc_set_dev_info_req_ind_handler_ROM(msgid, param, dest_id, src_id);
#endif
}

/*
 ****************************************************************************************
 * @brief Handles GAPM_PROFILE_ADDED_IND event.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
*/
static int gapm_profile_added_ind_handler_wrapper(ke_msg_id_t msgid,
                                          struct gapm_profile_added_ind *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
#if APP_IDX_MAX > 1
    return gapm_profile_added_ind_handler_MUL_CON(msgid, param, dest_id, src_id);
#else
  #if (__EXCLUDE_ROM_APP_TASK__) || (__APP_TASK_BUILD_FOR_DA1458X__)
    return gapm_profile_added_ind_handler(msgid, param, dest_id, src_id);
  #else
    return gapm_profile_added_ind_handler_ROM(msgid, param, dest_id, src_id);
  #endif
#endif
}

/*
 ****************************************************************************************
 * @brief Handles GAPC_PARAM_UPDATE_REQ_IND event.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
*/
static int gapc_param_update_req_ind_handler_wrapper(ke_msg_id_t msgid,
                                             struct gapc_param_update_req_ind *param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    app_gap_current_handled_connection_idx = KE_IDX_GET(src_id);

#if (__EXCLUDE_ROM_APP_TASK__) || (__APP_TASK_BUILD_FOR_DA1458X__)
    return gapc_param_update_req_ind_handler(msgid, param, dest_id, src_id);
#else
    return gapc_param_update_req_ind_handler_ROM(msgid, param, dest_id, src_id);
#endif

}

/*
 ****************************************************************************************
 * @brief Handles GAPC_LE_PKT_SIZE_IND event.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
*/
static int gapc_le_pkt_size_ind_handler_wrapper(ke_msg_id_t msgid,
                                        struct gapc_le_pkt_size_ind *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    app_gap_current_handled_connection_idx = KE_IDX_GET(src_id);

#if (__EXCLUDE_ROM_APP_TASK__) || (__APP_TASK_BUILD_FOR_DA1458X__)
    return gapc_le_pkt_size_ind_handler(msgid, param, dest_id, src_id);
#else
    return gapc_le_pkt_size_ind_handler_ROM(msgid, param, dest_id, src_id);
#endif
}

/*
 ****************************************************************************************
 * @brief Handles GATTC_SVC_CHANGED_CFG_IND message.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
*/
static int gattc_svc_changed_cfg_ind_handler_wrapper(ke_msg_id_t msgid,
                                             struct gattc_svc_changed_cfg *param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    app_gap_current_handled_connection_idx = KE_IDX_GET(src_id);

#if (__EXCLUDE_ROM_APP_TASK__) || (__APP_TASK_BUILD_FOR_DA1458X__)
    return gattc_svc_changed_cfg_ind_handler(msgid, param, dest_id, src_id);
#else
    return gattc_svc_changed_cfg_ind_handler_ROM(msgid, param, dest_id, src_id);
#endif
}

/*
 ****************************************************************************************
 * @brief Handles GAPC_PEER_FEATURES_IND message.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
*/
static int gapc_peer_features_ind_handler_wrapper(ke_msg_id_t msgid,
                                          struct gapc_peer_features_ind const *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    app_gap_current_handled_connection_idx = KE_IDX_GET(src_id);

#if (__EXCLUDE_ROM_APP_TASK__) || (__APP_TASK_BUILD_FOR_DA1458X__)
    return gapc_peer_features_ind_handler(msgid, param, dest_id, src_id);
#else
    return gapc_peer_features_ind_handler_ROM(msgid, param, dest_id, src_id);
#endif
}

/*
 * LOCAL VARIABLES DEFINITION
 ****************************************************************************************
 */


static const struct ke_msg_handler app_gap_process_handlers[]=
{
    {GAPM_DEVICE_READY_IND,                 (ke_msg_func_t)gapm_device_ready_ind_handler_wrapper    },
    {GAPM_CMP_EVT,                          (ke_msg_func_t)gapm_cmp_evt_handler_wrapper             },
    {GAPC_CMP_EVT,                          (ke_msg_func_t)gapc_cmp_evt_handler_wrapper             },
    {GAPC_CONNECTION_REQ_IND,               (ke_msg_func_t)gapc_connection_req_ind_handler_wrapper  },
    {GAPC_DISCONNECT_IND,                   (ke_msg_func_t)gapc_disconnect_ind_handler_wrapper      },
    {GAPC_GET_DEV_INFO_REQ_IND,             (ke_msg_func_t)gapc_get_dev_info_req_ind_handler_wrapper},
    {GAPC_SET_DEV_INFO_REQ_IND,             (ke_msg_func_t)gapc_set_dev_info_req_ind_handler_wrapper},
    {GAPM_PROFILE_ADDED_IND,                (ke_msg_func_t)gapm_profile_added_ind_handler_wrapper   },
    {GAPM_ADV_REPORT_IND,                   (ke_msg_func_t)gapm_adv_report_ind_handler_wrapper      },
    {GAPC_PARAM_UPDATE_REQ_IND,             (ke_msg_func_t)gapc_param_update_req_ind_handler_wrapper},
    {GAPC_LE_PKT_SIZE_IND,                  (ke_msg_func_t)gapc_le_pkt_size_ind_handler_wrapper     },
    {GATTC_SVC_CHANGED_CFG_IND,             (ke_msg_func_t)gattc_svc_changed_cfg_ind_handler_wrapper},
    {GAPC_PEER_FEATURES_IND,                (ke_msg_func_t)gapc_peer_features_ind_handler_wrapper   },
    
#if (BLE_APP_SEC)
    {GAPC_SECURITY_IND,                     (ke_msg_func_t)gapc_security_ind_handler_wrapper        },
#endif
};

/* Default State handlers definition. */
const struct ke_msg_handler app_default_state[] =
{
    {KE_MSG_DEFAULT_HANDLER,                (ke_msg_func_t)app_entry_point_handler},
};

/*
 * GLOBAL VARIABLES DEFINITION
 ****************************************************************************************
 */

/* Specifies the message handlers that are common to all states. */
const struct ke_state_handler APP_DEFAULT_HANDLER_VAR = KE_STATE_HANDLER(app_default_state);

#endif // defined(__EXCLUDE_ROM_APP_TASK__) || (APP_IDX_MAX > 1) || __APP_TASK_BUILD_FOR_DA14531__ || __APP_TASK_BUILD_FOR_DA1458X__


/* Defines the place holder for the states of all the task instances. */
ke_state_t app_state[APP_IDX_MAX] __SECTION_ZERO("retention_mem_area0"); //RETENTION MEMORY

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (__APP_TASK_BUILD_FOR_DA14531__) ||         \
    (__APP_TASK_BUILD_FOR_DA1458X__) ||         \
    ((__APP_TASK_BUILD_FOR_DA14535__ || __APP_TASK_BUILD_FOR_DA14531_01__) && (defined (__EXCLUDE_ROM_APP_TASK__) || (APP_IDX_MAX > 1)))
enum process_event_response APP_GAP_PROCESS_HANDLER_FUNC(ke_msg_id_t const msgid,
                                                    void const *param,
                                                    ke_task_id_t const dest_id,
                                                    ke_task_id_t const src_id,
                                                    enum ke_msg_status_tag *msg_ret)
{
    return app_std_process_event(msgid, param, src_id, dest_id, msg_ret, app_gap_process_handlers,
                                         sizeof(app_gap_process_handlers) / sizeof(struct ke_msg_handler));
}
#endif

#endif // BLE_APP_PRESENT

/// @} APPTASK
