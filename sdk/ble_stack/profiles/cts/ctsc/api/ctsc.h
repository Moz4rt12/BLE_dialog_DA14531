/**
 ****************************************************************************************
 *
 * @file ctsc.h
 *
 * @brief Header file - Current Time Service Client.
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

#ifndef CTSC_H_
#define CTSC_H_

/**
 ****************************************************************************************
 * @addtogroup CTSC Current Time Service Client
 * @ingroup CTS
 * @brief Current Time Service Client
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_CTS_CLIENT)

#include <stdint.h>
#include <stdbool.h>
#include "ke_task.h"
#include "prf_utils.h"
#include "cts_common.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum number of Current Time Information Service Client task instances
#define CTSC_IDX_MAX    (BLE_CONNECTION_MAX)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Characteristics
enum ctsc_char
{
    // Current Time Service
    CTSC_CHAR_CTS_CURRENT_TIME,
    // Local Time Information
    CTSC_CHAR_CTS_LOC_TIME_INFO,
    // Reference Time Information
    CTSC_CHAR_CTS_REF_TIME_INFO,

    CTSC_CHAR_MAX
};

/// Characteristic Descriptor
enum ctsc_desc
{
    // Current Time Client Config
    CTSC_DESC_CTS_CT_CLI_CFG,

    CTSC_DESC_MAX,

    // Bit is added to descriptor for read and write request
    CTSC_DESC_MASK = 0x10
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/**
 * Structure containing the characteristics handles, value handles and descriptor for
 * the Current Time Service
 */
struct ctsc_cts_content
{
    /// Service info
    struct prf_svc svc;

    /// Characteristic info:
    struct prf_char_inf chars[CTSC_CHAR_MAX];

    /// Descriptor handles:
    struct prf_char_desc_inf descs[CTSC_DESC_MAX];
};

/**
 * Environment variable for each Connections
 */
struct ctsc_cnx_env
{
    /// Current Time Service Characteristics
    struct ctsc_cts_content cts;
    /// counter used to check service uniqueness
    uint8_t nb_svc;
    /// Last char. code requested to read.
    uint8_t last_char_code;
    /// Pending response code, for time consuming requests
    ke_msg_id_t rsp_msg_id;
};

/**
 * Current Time Service Client environment variable
 */
struct ctsc_env_tag
{
    /// profile environment
    prf_env_t prf_env;
    /// Environment variable pointer for each connections
    struct ctsc_cnx_env *env[CTSC_IDX_MAX];
    /// State of different task instances
    ke_state_t state[CTSC_IDX_MAX];
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieve CTS client profile interface
 *
 * @return CTS client profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs *ctsc_prf_itf_get(void);

/**
 ****************************************************************************************
 * @brief Send Current Timer ATT DB discovery results to CTSC host.
 *
 * @param[in] ctsc_env  CTSC environment
 * @param[in] conidx    Conection index
 * @param[in] status    Operation status
 ****************************************************************************************
 */
void ctsc_enable_rsp_send(struct ctsc_env_tag *ctsc_env, uint8_t conidx, uint8_t status);

/**
 ****************************************************************************************
 * @brief Get characteristic information.
 *
 * @param[in] conidx    Conection index
 * @param[in] char_code Characteristic id from @ref enum ctsc_char
 ****************************************************************************************
 */
uint8_t ctsc_get_char_info(uint8_t conidx, enum ctsc_char char_code,
                           struct prf_char_inf *char_inf);

/**
 ****************************************************************************************
 * @brief Pack Current Time value
 * @param[in] p_curr_time_val Current Time value to be packed
 * @param[out] p_pckd_time Packed Current Time value
 ****************************************************************************************
 */
void ctsc_pack_curr_time_value(const struct cts_curr_time *p_curr_time_val,
                               uint8_t *p_pckd_time);

/**
 ****************************************************************************************
 * @brief Unpack Current Time value
 * @param[in] p_pckd_time Packed Current Time value
 * @param[out] p_curr_time_val Current Time value to be packed
 ****************************************************************************************
 */
void ctsc_unpack_curr_time_value(const uint8_t *p_pckd_time,
                                 struct cts_curr_time *p_curr_time_val);

#endif // BLE_CTS_CLIENT

/// @} CTSC

#endif // CTSC_H_

