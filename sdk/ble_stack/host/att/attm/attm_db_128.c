/**
 ****************************************************************************************
 *
 * @file attm_db_128.c
 *
 * @brief ATTM functions that handle the creation of service database of 128bits long UUID.
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

/*
 * INCLUDES
 ****************************************************************************************
 */

#if !defined (__DA14531__) || defined (__EXCLUDE_ROM_ATTM_DB_128__)

#include "attm_db_128.h"
#include "attm_db.h"
#include "ke_mem.h"

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

uint8_t attm_svc_create_db_128(uint8_t svc_idx, uint16_t *shdl, uint8_t *cfg_flag, uint8_t max_nb_att,
                               uint8_t *att_tbl, ke_task_id_t const dest_id,
                               const struct attm_desc_128 *att_db, uint8_t svc_perm)
{
    int32_t db_cfg;
    uint8_t nb_att = 0;
    uint8_t status;
    uint8_t i;
    uint8_t att_idx = 0;
    struct gattm_svc_desc *svc_desc = NULL;
    const uint16_t att_decl_char = ATT_DECL_CHARACTERISTIC;

    if (cfg_flag != NULL)
    {
        // Number of attributes is limited to 32
        memcpy(&db_cfg, cfg_flag, sizeof(uint32_t));
    }
    else
    {
        // Set all bits to 1
        db_cfg = -1;
    }

    // Compute number of attributes and maximal payload size
    for (i = svc_idx + 1; i < max_nb_att; i++)
    {
        // check within db_cfg flag if attribute is enabled or not
        if (((db_cfg >> i) & 1) == 1)
        {
            nb_att++;
        }
    }

    // Allocate database descriptor
    svc_desc = (struct gattm_svc_desc *) ke_malloc(sizeof(struct gattm_svc_desc) +
                                nb_att * sizeof(struct gattm_att_desc),
                                KE_MEM_NON_RETENTION);

    svc_desc->start_hdl = *shdl;
    svc_desc->task_id = dest_id;

    if (att_db[svc_idx].uuid_size == ATT_UUID_16_LEN)
    {
        svc_desc->perm = svc_perm | PERM_VAL(SVC_UUID_LEN, PERM_UUID_16);
    }
    else if (att_db[svc_idx].uuid_size == ATT_UUID_32_LEN)
    {
        svc_desc->perm = svc_perm | PERM_VAL(SVC_UUID_LEN, PERM_UUID_32);
    }
    else
    {
        svc_desc->perm = svc_perm | PERM_VAL(SVC_UUID_LEN, PERM_UUID_128);
    }

    *(struct att_uuid_128 *)(&svc_desc->uuid) = *(struct att_uuid_128 *)att_db[svc_idx].value;

    for (i = svc_idx + 1; i < max_nb_att; i++)
    {
        // check within db_cfg flag if attribute is enabled or not
        if (((db_cfg >> i) & 1) == 1)
        {
            // Add attribute
            if (att_db[i].uuid_size == ATT_UUID_16_LEN)
            {
                svc_desc->atts[att_idx] = (struct gattm_att_desc) {
                    .uuid = {att_db[i].uuid[0], att_db[i].uuid[1]},
                    .perm = att_db[i].perm | PERM_VAL(UUID_LEN, PERM_UUID_16),
                    .max_len = att_db[i].max_length
                };
            }
            else if (att_db[i].uuid_size == ATT_UUID_32_LEN)
            {
                svc_desc->atts[att_idx] = (struct gattm_att_desc) {
                    .uuid = {att_db[i].uuid[0], att_db[i].uuid[1], att_db[i].uuid[2], att_db[i].uuid[3]},
                    .perm = att_db[i].perm | PERM_VAL(UUID_LEN, PERM_UUID_32),
                    .max_len = att_db[i].max_length
                };
            }
            else if (att_db[i].uuid_size == ATT_UUID_128_LEN)
            {
                memcpy(svc_desc->atts[att_idx].uuid, att_db[i].uuid, sizeof(struct att_uuid_128));
                svc_desc->atts[att_idx].perm = att_db[i].perm | PERM_VAL(UUID_LEN, PERM_UUID_128);
                svc_desc->atts[att_idx].max_len = att_db[i].max_length;
            }
            att_idx++;
        }
    }
    svc_desc->nb_att = att_idx;

    status = attmdb_add_service(svc_desc);
    if (status == ATT_ERR_NO_ERROR)
    {
        *shdl = svc_desc->start_hdl;
        att_idx = 0;

        for (i = svc_idx + 1; i < max_nb_att && (status == ATT_ERR_NO_ERROR); i++)
        {
            // check within db_cfg flag if attribute is enabled or not
            if (((db_cfg >> i) & 1) == 0)
                continue;
            att_idx++;

            // If attribute is marked with RI, or is empty don't store it in DB
            if (att_db[i].max_length & PERM(RI, ENABLE) || att_db[i].length == 0)
                continue;

            if(att_db[i].uuid_size == ATT_UUID_16_LEN &&
              (memcmp(att_db[i].uuid, &att_decl_char, sizeof(att_decl_char)) == 0))
                  continue;

            status = attmdb_att_set_value(svc_desc->start_hdl + att_idx,
                                        att_db[i].length, 0, att_db[i].value);
        }
    }

    ke_free(svc_desc);

    return status;
}

#endif
