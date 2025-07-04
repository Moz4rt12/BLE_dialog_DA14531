/**
 ****************************************************************************************
 *
 * @file bcsc.h
 *
 * @brief Header file - Body Composition Service Client Declaration.
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

#ifndef BCSC_H_
#define BCSC_H_

/**
 ****************************************************************************************
 * @addtogroup BCSC Center Client
 * @ingroup BCSC
 * @brief Body Composition Service Client
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_BCS_CLIENT)

#include "ke_task.h"
#include "prf_types.h"
#include "prf_utils.h"
#include "bcs_common.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum number of ANCC task instances
#define BCSC_IDX_MAX                    (BLE_CONNECTION_MAX)

/// BCSC Characteristics
enum bcsc_bcs_chars
{
    /// Body Composition Feature
    BCSC_CHAR_BC_FEATURE,
    /// Body Composition Measurement
    BCSC_CHAR_BC_MEAS,
    /// Number of characteristics
    BCSC_CHAR_MAX,
};

/// Characteristic descriptors
enum
{
    /// Body Composition Measurement client cfg
    BCSC_DESC_MEAS_CLI_CFG,
    BCSC_DESC_MAX,
    BCSC_DESC_MASK = 0x10,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
///Structure containing the characteristics handles, value handles and descriptors

struct bcs_content
{
    /// service info
    struct prf_svc svc;
    /// If service is a secondary service
    bool is_secondary;
    /// characteristics
    struct prf_char_inf chars[BCSC_CHAR_MAX];
    /// descriptors
    struct prf_char_desc_inf descs[BCSC_DESC_MAX];
};

/// Environment variable for each Connections
struct bcsc_cnx_env
{
    /// Last requested UUID(to keep track of the two services and char)
    uint16_t last_uuid_req;
    /// Last Char. for which a read request has been sent
    uint8_t last_char_code;
    /// counter used to check service uniqueness
    uint8_t nb_svc;
    /// BCS characteristics
    struct bcs_content bcs;
};

/// Weight Scale Client environment variable
struct bcsc_env_tag
{
    /// profile environment
    prf_env_t prf_env;
    /// Environment variable pointer for each connections
    struct bcsc_cnx_env* env[BCSC_IDX_MAX];
    /// State of different task instances
    ke_state_t state[BCSC_IDX_MAX];
};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieve BCS client profile interface
 *
 * @return BCS client profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs* bcsc_prf_itf_get(void);

/**
 ****************************************************************************************
 * @brief Send enable confirmation to application.
 * @param status Status of the enable: either OK or error encountered while discovery.
 ****************************************************************************************
 */
void bcsc_enable_rsp_send(struct bcsc_env_tag *bcsc_env, uint8_t conidx, uint8_t status);

/**
 ****************************************************************************************
 * @brief Send indication configuration confirmation to application.
 * @param status Status of the operation
 ****************************************************************************************
 */
void bcsc_bc_meas_cfg_ind_rsp_send(struct bcsc_env_tag * bcsc_env, uint8_t conidx, uint8_t status);

/**
 ****************************************************************************************
 * @brief Unpack Body Composition measurement data into a comprehensive structure.
 * @param[out] pmeas_val    Pointer to BC measurement structure destination.
 * @param[out] pmeas_flags  Pointer to BC measurement flags destination.
 * @param[in] packed_bp     Pointer of the packed data of Body Composition Measurement
 *                          information.
 ****************************************************************************************
 */
void bcsc_unpack_meas_value(bcs_meas_t *pmeas_val, uint16_t *pmeas_flags, uint8_t *packed_bp);

#endif // (BLE_BCS_CLIENT)

/// @} BCSC

#endif //BCSC_H_
