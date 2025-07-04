/**
 ****************************************************************************************
 *
 * @file ctss.h
 *
 * @brief Header file - Current Time Service Server.
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

#ifndef _CTSS_H_
#define _CTSS_H_

/**
 ****************************************************************************************
 * @addtogroup CTSS Current Time Service Server
 * @ingroup CTS
 * @brief Current Time Service Server
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "prf.h"
#include "cts_common.h"

#if (BLE_CTS_SERVER)

/*
 * DEFINES
 ****************************************************************************************
 */

#define CTSS_CURRENT_TIME_VAL_LEN        (10)

/*
 * MACROS
 ****************************************************************************************
 */

#define CTSS_IS_SUPPORTED(mask) (((ctss_env.features & mask) == mask))

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Database Configuration Flags
enum ctss_features
{
    // Optional Characteristics
    CTSS_LOC_TIME_INFO_SUP      = 0x01,
    CTSS_REF_TIME_INFO_SUP      = 0x02,

    // Current Time Characteristic's write enabled
    CTSS_CURRENT_TIME_WRITE_SUP = 0x10,
    // Local Time Characteristic's write enabled
    CTSS_LOC_TIME_WRITE_SUP     = 0x20,
};

/// Current Time Service Attribute indices
enum
{
    CTSS_IDX_SVC,

    CTSS_IDX_CURRENT_TIME_CHAR,
    CTSS_IDX_CURRENT_TIME_VAL,
    CTSS_IDX_CURRENT_TIME_CFG,

    CTSS_IDX_LOCAL_TIME_INFO_CHAR,
    CTSS_IDX_LOCAL_TIME_INFO_VAL,

    CTSS_IDX_REF_TIME_INFO_CHAR,
    CTSS_IDX_REF_TIME_INFO_VAL,

    CTSS_IDX_NB,
};

/// Current Time Service Characteristics
enum
{
    CTSS_CURRENT_TIME_CHAR,
    CTSS_LOCAL_TIME_INFO_CHAR,
    CTSS_REF_TIME_INFO_CHAR,

    CTSS_CHAR_MAX,
};

/*
 * STRUCTURES
 ****************************************************************************************
 */

/// Parameters for the database creation
struct ctss_db_cfg
{
    /// Database configuration @see enum ctss_features
    uint8_t features;
};

struct ctss_env_tag
{
    /// Connection Info
    prf_env_t prf_env;

    /// CTS Start Handle
    uint16_t start_hdl;

    /// CTS Attribute Table
    uint8_t att_tbl[CTSS_IDX_NB];

    struct cts_loc_time_info loc_time_info;

    /// Notification configuration of peer devices.
    uint8_t ntf_cfg[BLE_CONNECTION_MAX];

    /// On-going operation
    struct ke_msg *operation;

    /// Connection index to use for notifications
    uint8_t cursor;

    /// CTSS task state
    ke_state_t state;

    /// Current time, stored here during nitification
    uint8_t pckd_time[CTSS_CURRENT_TIME_VAL_LEN];

    /// Database configuration
    uint8_t features;

    /// Last requested value
    uint8_t  req_val;
    /// Last connection index which requested value
    uint8_t  req_conidx;
};

extern struct ctss_env_tag ctss_env;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieve CTS service profile interface
 *
 * @return CTS service profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs *ctss_prf_itf_get(void);

/**
 ****************************************************************************************
 * @brief Send a CTS_ERROR_IND message to the application
 * @param[in] con_info Connection information
 * @param[in] status Status code of error
 * @param[in] msg_id Failing message ID
 ****************************************************************************************
 */
void ctss_error_ind_send(struct ctss_env_tag *ctss_env, uint8_t status, ke_msg_id_t msg_id);

/**
 ****************************************************************************************
 * @brief Pack Current Time value
 * @param[in] p_curr_time_val Current Time value to be packed
 * @param[out] p_pckd_time Packed Current Time value
 ****************************************************************************************
 */
void ctss_pack_curr_time_value(const struct cts_curr_time *p_curr_time_val, uint8_t *p_pckd_time);

/**
 ****************************************************************************************
 * @brief Unpack Current Time value
 * @param[in] p_pckd_time Packed Current Time value
 * @param[out] p_curr_time_val Current Time value to be packed
 ****************************************************************************************
 */
void ctss_unpack_curr_time_value(const uint8_t *p_pckd_time, struct cts_curr_time *p_curr_time_val);

/**
 ****************************************************************************************
 * @brief Disable actions grouped in getting back to IDLE and sending configuration to requester task
 * @param[in] conhdl Handle of connection for which this service is disabled
 ****************************************************************************************
 */
void ctss_disable(uint16_t conhdl);

/**
 ****************************************************************************************
 * @brief Convert attribute index to handle
 * @param[in] ctss_env Current time service environment
 * @param[in] att_idx  Attribute index (on of CTSS_IDX_...)
 * @return attribute handle
 ****************************************************************************************
 */
uint16_t ctss_get_att_handle(const struct ctss_env_tag *ctss_env, uint8_t att_idx);

/**
 ****************************************************************************************
 * @brief Convert handle to attribute index
 * @param[in] ctss_env Current time service environment
 * @param[in] handle   Attribute handle
 * @return attribute index (on of CTSS_IDX_...)
 ****************************************************************************************
 */
uint8_t ctss_get_att_idx(const struct ctss_env_tag *ctss_env, uint16_t handle);

/**
 ****************************************************************************************
 * @brief  This function fully manage notification of current time service
 ****************************************************************************************
 */
void ctss_exe_operation(void);

#endif /* #if (BLE_CTS_SERVER) */

/// @} CTSS

#endif /* _CTSS_H_ */
