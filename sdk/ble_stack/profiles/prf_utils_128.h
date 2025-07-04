/**
 ****************************************************************************************
 *
 * @file prf_utils_128.h
 *
 * @brief  Header file - Profile Utilities for attributes with 128-bit UUIDs
 *
 * Copyright (C) 2015-2025 Renesas Electronics Corporation and/or its affiliates.
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

#ifndef _PRF_UTILS_128_H_
#define _PRF_UTILS_128_H_

/**
 ****************************************************************************************
 * @addtogroup PRF_UTILS
 * @ingroup PROFILE
 *
 * @brief Definitions of shared profiles functions that can be used by several profiles
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_CLIENT_PRF)
#include "ke_msg.h"
#include "prf_types.h"
#include "gattc_task.h"
#include "gapc.h"
#include "gapc_task.h"
#include "attm_db.h"
#include "prf_utils.h"
#endif /* (BLE_CLIENT_PRF) */

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

#if (BLE_CLIENT_PRF)

/// Characteristic definition
struct prf_char_def_128
{
    /// Characteristic UUID
    uint8_t uuid[ATT_UUID_128_LEN];
    /// Requirement Attribute Flag
    uint8_t req_flag;
    /// Mandatory Properties
    uint8_t prop_mand;
};

/**
 ****************************************************************************************
 * @brief Discover services by 128-bit UUID. The function is based on the
 *        GATTC_SDP_DISC_SVC message.
 * @param[in] prf_env   Pointer to profile information
 * @param[in] conidx    Connection index
 * @param[in] uuid_128  128-bit service UUID
 ****************************************************************************************
 */
void prf_disc_svc_sdp_send_128(prf_env_t *prf_env, uint8_t conidx, const uint8_t* uuid_128);

/**
 ****************************************************************************************
 * @brief Discover services by 128-bit UUID. The function is based on the
 *        GATTC_DISC_CMD message.
 * @param[in] prf_env   Pointer to profile information
 * @param[in] conidx    Connection index
 * @param[in] uuid_128  128-bit service UUID
 ****************************************************************************************
 */
void prf_disc_svc_send_128(prf_env_t *prf_env, uint8_t conidx, const uint8_t *uuid_128);

/**
 ****************************************************************************************
 * @brief Discover characteristic by 128-bit UUID.
 * @param[in] prf_env   Pointer to profile information
 * @param[in] conidx    Connection index
 * @param[in] uuid_128  128-bit service UUID
 ****************************************************************************************
 */
void prf_disc_char_by_uuid_128_send(prf_env_t *prf_env, uint8_t conidx, const uint8_t *uuid_128);

/**
 ****************************************************************************************
 * @brief Extract information of the service according to the service description based
 *        on 128-bit UUIDs.
 * @param[in]     param            Service information
 * @param[in]     nb_chars         Length of provided arrays (chars and chars_req)
 * @param[in]     chars_req        Characteristics requirements (128 bit uuids)
 * @param[in|out] chars            Characteristics
 * @param[in]     nb_descs         Length of provided arrays (descs and descs_req)
 * @param[in]     descs_req        Descriptors requirements
 * @param[in|out] descs            Descriptors
 ****************************************************************************************
 */
void prf_extract_svc_sdp_info_128(struct gattc_sdp_svc_ind const *param,
                                  uint8_t nb_chars,
                                  const struct prf_char_def_128* chars_req,
                                  struct prf_char_inf* chars,
                                  uint8_t nb_descs,
                                  const struct prf_char_desc_def* descs_req,
                                  struct prf_char_desc_inf* descs);

/**
 ****************************************************************************************
 * @brief Check service characteristic validity.
 *
 * For each characteristic in service it verifies handles.
 *
 * If some handles are not present, it checks if they shall be present or they are optional.
 *
 * @param[in] nb_chars   Number of Characteristics in the service
 * @param[in] chars      Characteristics values (char handles, val handles, properties)
 * @param[in] chars_req  Characteristics requirements.
 *
 * @return 0x1 if service is valid, 0x00 else.
 ****************************************************************************************
 */
uint8_t prf_check_svc_char_validity_128(uint8_t nb_chars,
                                        const struct prf_char_inf* chars,
                                        const struct prf_char_def_128* chars_req);

/**
 ****************************************************************************************
 * @brief Look for requested characteristics in the list provided by the
 *        GATTC_DISC_CHAR_IND message.
 * @param svc_ehdl         End handle of the service for which characteristics are discovered
 * @param nb_chars         Length of provided arrays (chars and chars_req)
 * @param chars            Characteristics
 * @param chars_req        Characteristics requirements
 * @param param            List of found characteristics
 * @param last_found_char  Characteristics code of the last characteristic that has been found.
 ****************************************************************************************
 */
void prf_search_chars_128(uint16_t svc_ehdl,
                          uint8_t nb_chars,
                          struct prf_char_inf* chars,
                          const struct prf_char_def_128* chars_req,
                          const struct gattc_disc_char_ind* param,
                          uint8_t* last_found_char);

#endif //(BLE_CLIENT_PRF)
/// @} prf_utils

#endif /* _PRF_UTILS_128_H_ */
