/**
 ****************************************************************************************
 *
 * @file aes_cmac.c
 *
 * @brief AES-CMAC implementation.
 *
 * Copyright (C) 2018-2025 Renesas Electronics Corporation and/or its affiliates.
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
 * @addtogroup aes_cmac
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "aes_cmac.h"
#include "aes_cbc.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define R128_LSB (0x87)          // according to NIST SP 800-38b: 120*0 || 10000111
#define MSb_MASK (0x80)

/*
 * STRUCTURES
 ****************************************************************************************
 */

struct aes_cmac_subkeys {
    uint8_t k1[AES_CMAC_BLK_SIZE_128];
    uint8_t k2[AES_CMAC_BLK_SIZE_128];
};

/*
 * STATIC FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Prepare subkey for AES-CMAC
 * @details Sec 6.1 of NIST SP 800-38b
 * @param[out] dst      destination buffer
 * @param[in] src       source buffer
 * @param[in] xor       data to xor the output with
 ****************************************************************************************
 */
static void subkey_prepare(uint8_t *dst, const uint8_t *src, const uint8_t *xor)
{
    uint8_t overflow = 0;

    // Shift entire array left by one bit
    for (int8_t i = AES_CMAC_BLK_SIZE_128 - 1; i >= 0; i--)
    {
        dst[i] = (src[i] << 1) | overflow;
        overflow = (src[i] >> 7);
    }

    // Optional xor
    if (src[0] & MSb_MASK)
    {
        aes_array_xor(dst, xor, AES_CMAC_BLK_SIZE_128, dst);
    }
}

/**
 ****************************************************************************************
 * @brief Generate subkeys
 * @details Sec 6.1 of NIST SP 800-38b
 * @param[in] key           Input AES key
 * @param[out] subkeys      Subkeys structure for the output data,
 ****************************************************************************************
 */
static void generate_subkeys(const uint8_t *key, struct aes_cmac_subkeys *subkeys)
{
    uint8_t z_blk[AES_CMAC_BLK_SIZE_128] = {0};
    uint8_t l_blk[AES_CMAC_BLK_SIZE_128] = {0};
    uint8_t r_blk[AES_CMAC_BLK_SIZE_128] = {0};

    // Prepare R block
    r_blk[AES_CMAC_BLK_SIZE_128 - 1] = R128_LSB;

    // Prepare L block [Step 1, Sec 6.1 of NIST SP 800-38b]
    aes_cbc_encrypt(z_blk, 1, l_blk, 1, key, AES_CMAC_BLK_SIZE_128, NULL);

    // Generate K1 [Step 2, Sec 6.1 of NIST SP 800-38b]
    subkey_prepare(subkeys->k1, l_blk, r_blk);

    // Generate K2 [Step 3, Sec 6.1 of NIST SP 800-38b]
    subkey_prepare(subkeys->k2, subkeys->k1, r_blk);
}

/*
 * PUBLIC FUNCTION DEFINITIONS
 ****************************************************************************************
 */

uint8_t aes_cmac_generate(const uint8_t *payload, uint16_t payload_len,
                          const uint8_t *key, uint8_t *mac, uint8_t mac_len)
{
    uint8_t n = payload_len ? aes_get_blk_num(payload_len) : 1; // [NIST SP 800-38b sec. 6.2, step 2]
    struct aes_cmac_subkeys subkeys;
    uint8_t status = AES_CBC_ERR_NO_ERR;

    // Temporary MAC storage
    uint8_t temp_mac[AES_CBC_BLK_SIZE] = {0};

    // We use the provided input buffer in 128-bit wide blocks to correctly feed the AES
    // block encryption routine. We need an intermediate buffer only for the last chunk.
    uint8_t temp_mn[AES_CBC_BLK_SIZE] = {0};

    // Generate subkeys [NIST SP 800-38b sec. 6.2, step 1]
    generate_subkeys(key, &subkeys);

    // Prepare last block [NIST SP 800-38b sec. 6.2, step 4]
    if ((payload_len == 0) || (payload_len % AES_CBC_BLK_SIZE))
    {
        // Mn is an incomplete block (or there is no payload at all)
        uint8_t rem_len = payload_len ? payload_len - ((n - 1) * AES_CBC_BLK_SIZE) : 0;

        if (rem_len)
        {
            memcpy(temp_mn, &payload[(n - 1) * AES_CBC_BLK_SIZE], rem_len);
        }
        // Fill the empty space with 10^j pattern (single 1 and the rest of bits are 0).
        // Since buffer was zeroed, we set just the most significant bit in the first byte.
        temp_mn[rem_len] = (1 << 7);

        aes_array_xor(subkeys.k2, temp_mn, AES_CBC_BLK_SIZE, temp_mn);
    }
    else
    {
        // Mn is a complete block
        memcpy(temp_mn, &payload[(n - 1) * AES_CBC_BLK_SIZE],
               AES_CBC_BLK_SIZE);

        aes_array_xor(subkeys.k1, temp_mn, AES_CBC_BLK_SIZE, temp_mn);
    }

    // [NIST SP 800-38b sec. 6.2, steps 6,7,8]
    // Encrypt all but the last block which is stored in temp_mn
    if (n > 1)
    {
        status = aes_cbc_encrypt(payload, n - 1, temp_mac, 1, key, AES_CMAC_BLK_SIZE_128,
                                 NULL); // IV = C0 = 0
    }

    // Encrypt the last block (get the final ciphertext block Cn)
    if (status == AES_CBC_ERR_NO_ERR)
    {
        status = aes_cbc_encrypt(temp_mn, 1, temp_mac, 1, key, AES_CMAC_BLK_SIZE_128,
                                 temp_mac); // IV = Cn-1
        if (status == AES_CBC_ERR_NO_ERR)
        {
            memcpy(mac, temp_mac, mac_len);
        }
    }

    return status;
}

bool aes_cmac_verify(const uint8_t *payload, uint16_t payload_len,
                     const uint8_t *key, const uint8_t *mac, uint8_t mac_len)
{
    uint8_t cmac[AES_CMAC_BLK_SIZE_128];

    aes_cmac_generate(payload, payload_len, key, cmac, mac_len);

    return (memcmp(cmac, mac, mac_len) ? false : true);
}
/// @} aes_cmac
