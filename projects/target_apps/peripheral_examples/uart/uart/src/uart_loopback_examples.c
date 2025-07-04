/**
 ****************************************************************************************
 *
 * @file uart_loopback_examples.c
 *
 * @brief UART loopback examples.
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

#include "uart.h"
#include "uart_utils.h"
#include "uart_common.h"
#include "ring_buffer.h"


static uint8_t rxbuf[1];
static uint8_t txbuf[1];
static volatile uint8_t tx_in_progress = 0;
static uart_t* uart_id;
volatile uint16_t total_cnt=0;

static void uart_read_cb(uint16_t length)
{
    total_cnt += length;
    // Put received byte in ring buffer
    buffer_put_byte(rxbuf[0]);

    // Start the next asynchronous read of 1 character.
    uart_receive(uart_id, rxbuf, 1, UART_OP_INTR);
}

static void uart_write_cb(uint16_t length)
{
    tx_in_progress = 0;
}

void uart_loopback_interrupt_example(uart_t *uart)
{
    total_cnt = 0;
    uart_id = uart;

    printf_string(uart, "\n\r\n\r****************************************\n\r");
    if (uart_id == UART1)
    {
        printf_string(uart_id, "* UART1 Loopback Interrupt example *\n\r");
    }
    else
    {
        printf_string(uart_id, "* UART2 Loopback Interrupt example *\n\r");
    }
    printf_string(uart_id, "****************************************\n\r\n\r");

    uart_register_rx_cb(uart_id, uart_read_cb);
    uart_register_tx_cb(uart_id, uart_write_cb);

    uart_receive(uart_id, rxbuf, 1, UART_OP_INTR);
    while(total_cnt < 20)
    {
        int status;
        // Attemp to get a byte from the ring buffer
        status = buffer_get_byte(&txbuf[0]);

        // Check if a character was read successfully from the ring buffer
        if (!status)
        {
            // Write the character to UART2 and then wait synchronously for completion.
            tx_in_progress = 1;
            uart_send(uart_id, txbuf, 1, UART_OP_INTR);
            while (tx_in_progress);
        }
    }

    printf_string(uart, "\r\n");
    printf_string(uart_id, "****************************************\n\r\n\r");
}
