/**
 ****************************************************************************************
 *
 * @file aes_api.c
 *
 * @brief AES.
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

#include <stdint.h>
#include "rwip.h"
#include "sw_aes.h"
#include "gapm_task.h"
#include "aes.h"
#include "aes_api.h"
#include "llm.h"
#include "reg_blecore.h"
#include "co_bt.h"

/* using a dummy IV vector filled with zeroes for the software decryption since the chip does not use IV for encryption */
extern uint8_t IV[];

int aes_set_key(const uint8_t *userKey, const uint32_t bits, AES_KEY *key, uint8_t enc_dec)
{
    if(enc_dec == AES_ENCRYPT)
    {
        uint32_t *rk;
        rk = key->ks;
        rk[0] = GETU32(userKey     );
        rk[1] = GETU32(userKey +  4);
        rk[2] = GETU32(userKey +  8);
        rk[3] = GETU32(userKey + 12);

        return 0;
    }
    if (enc_dec == AES_DECRYPT)
    {
        if (bits == 128)
           AES_set_key(key, userKey, IV, AES_MODE_128);
      else if (bits == 256)
           AES_set_key(key, userKey, IV, AES_MODE_256);

      AES_convert_key(key);
    }
    return 0; /* now we always return 0 since the handling functions changed to void */
}


static void reverse_uint8memcpy(uint8_t * dst, const uint8_t * src, size_t n)
{
    while (n--) *dst++ = *src--;
}

int aes_enc_dec(uint8_t *in, uint8_t *out, AES_KEY *key, uint8_t enc_dec, uint8_t ble_flags)
{
    if(ble_flags & BLE_SAFE_MASK)
        rwip_schedule();

    if(enc_dec == AES_ENCRYPT)
    {
        // Check if an encryption is pending
        if (llm_le_env.enc_pend)
            return -1;

        // copy the key in the register dedicated for the encryption
        ble_aeskey31_0_set(key->ks[3]);
        ble_aeskey63_32_set(key->ks[2]);
        ble_aeskey95_64_set(key->ks[1]);
        ble_aeskey127_96_set(key->ks[0]);

        // copy data from sys ram to exchange memory (in reverse order)
        reverse_uint8memcpy((uint8_t *)(EM_BLE_ENC_PLAIN_OFFSET + EM_BASE_ADDR), &in[15], ENC_DATA_LEN);

        // set the pointer on the data to encrypt.
        ble_aesptr_set(EM_BLE_ENC_PLAIN_OFFSET);

        // start the encryption
        ble_aescntl_set(BLE_AES_START_BIT);

        while(GetWord32(BLE_AESCNTL_REG)==1)
            if(ble_flags & BLE_SAFE_MASK)
                rwip_schedule();

        // copy data from em to sys ram
        em_rd(&out[0], EM_BLE_ENC_CIPHER_OFFSET, ENC_DATA_LEN);
    }
    else
    {
#if USE_AES_DECRYPT
        uint8_t j;

        uint32_t temp[4];
        
        for(j = 0; j < ENC_DATA_LEN; j += 4)
        {
            temp[j/4] = GETU32(&in[j]);
        }
        
        AES_decrypt(key, &temp[0]);
        //decrypt is performed in place, so copy to out afterwards
        for(j = 0 ; j < ENC_DATA_LEN ; j += 4)
        {
            PUTU32(&out[j], temp[j/4]);
        }
        
#else
        return -6;
#endif
    }

    return 0;
}
