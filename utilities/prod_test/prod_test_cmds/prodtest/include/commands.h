/**
 ****************************************************************************************
 *
 * @file commands.h
 *
 * @brief Commands header file.
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

#ifndef _COMMANDS_H_
#define _COMMANDS_H_

/* status codes */
#define SC_NO_ERROR                                 0
#define SC_MISSING_COMMAND                          1
#define SC_INVALID_COMMAND                          2
#define SC_WRONG_NUMBER_OF_ARGUMENTS                3
#define SC_INVALID_COM_PORT_NUMBER                  4
#define SC_INVALID_FREQUENCY_ARG                    5
#define SC_INVALID_DATA_LENGTH_ARG                  6
#define SC_INVALID_PAYLOAD_TYPE_ARG                 7
#define SC_COM_PORT_INIT_ERROR                      8
#define SC_RX_TIMEOUT                               9
#define SC_UNEXPECTED_EVENT                        10
#define SC_INVALID_NUMBER_OF_PACKETS_ARG           11
#define SC_INVALID_UNMODULATED_CMD_MODE_ARG        12
#define SC_COM_PORT_NOT_SPECIFIED                  13
#define SC_INVALID_SLEEP_CMD_MODE_ARG              14
#define SC_INVALID_SLEEP_CMD_MINUTES_ARG           15
#define SC_INVALID_SLEEP_CMD_SECONDS_ARG           16
#define SC_INVALID_XTAL_TRIMMING_CMD_OPERATION_ARG 17
#define SC_INVALID_XTAL_TRIMMING_CMD_TRIM_VALUE_ARG 18
#define SC_INVALID_OTP_CMD_OPERATION_ARG            19
#define SC_INVALID_OTP_CMD_TRIM_VALUE_ARG           20
#define SC_INVALID_OTP_CMD_BDADDR_ARG               21
#define SC_INVALID_OTP_ADDRESS_ARG                  22
#define SC_INVALID_NUMBER_OF_OTP_WORDS_ARG          23
#define SC_INVALID_WORD_VALUE_ARG                   24
#define SC_INVALID_GPIO_ARG                         25
#define SC_XTAL_TRIMMING_CAL_OUT_OF_RANGE_ERROR     26
#define SC_XTAL_TRIMMING_CAL_FREQ_NOT_CONNECTED     27
#define SC_INVALID_REGISTER_ADDRESS_ARG             28
#define SC_INVALID_REGISTER_VALUE_ARG               29

#define SC_HCI_STANDARD_ERROR_CODE_BASE           1000


/* timeout while waiting for events*/
#define RX_TIMEOUT_MILLIS 10000 

/* Maximum number of words that can be read or written by a command at once */
#define MAX_READ_WRITE_OTP_WORDS 60

/* command handlers */
int starttest_tx_param_len_3_handler(int argc, char **argv);
int starttest_tx_param_len_5_handler(int argc, char **argv);
int starttest_rx_default_handler(int argc, char **argv);
int starttest_rx_readback_values_handler(int argc, char **argv);
int stoptest_rx_readback_values_handler(int argc, char **argv);
int stoptest_handler(int argc, char **argv);
int starttest_unmodulated_handler(int argc, char **argv);
int starttest_tx_continue_handler(int argc, char **argv);
int stoptest_tx_continue_handler(int argc, char **argv);
int reset_handler(int argc, char **argv);
int sleep_cmd_handler(int argc, char **argv);
int xtal_trimming_cmd_handler(int argc, char **argv);
int otp_cmd_handler(int argc, char **argv);
int otp_read_cmd_handler(int argc, char **argv);
int otp_write_cmd_handler(int argc, char **argv);
int read_reg32_cmd_handler(int argc, char **argv);
int write_reg32_cmd_handler(int argc, char **argv);
int read_reg16_cmd_handler(int argc, char **argv);
int write_reg16_cmd_handler(int argc, char **argv);

/* utils*/
long parse_number(int *return_status, const char * str);

#endif /* _COMMANDS_H_ */
