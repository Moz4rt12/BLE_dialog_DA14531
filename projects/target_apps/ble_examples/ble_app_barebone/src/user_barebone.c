/**
 ****************************************************************************************
 *
 * @file user_barebone.c
 *
 * @brief Barebone project source code.
 *
 * Copyright (C) 2015-2023 Renesas Electronics Corporation and/or its affiliates.
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
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"             // SW configuration
#include "gattc_task.h"
#include "gap.h"
#include "app_easy_timer.h"
#include "user_barebone.h"
#include "co_bt.h"

// USER INCLUSIONS **********************************************************************
#include "battery.h"
#include "arch_system.h"
#include <rtc.h>
#include "uart.h"
#include "ext_wakeup_uart.h"

// **************************************************************************************

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */	
 
 #define BUFFER_SIZE 8
 
// USER GLOBAL VARIABLES ****************************************************************

//***************************************************************************************

// Manufacturer Specific Data ADV structure type
struct mnf_specific_data_ad_structure
{
    uint8_t ad_structure_size;
    uint8_t ad_structure_type;
    uint8_t company_id[APP_AD_MSD_COMPANY_ID_LEN];
    uint8_t proprietary_data[APP_AD_MSD_DATA_LEN];
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

// USER GLOBAL VARIABLES ****************************************************************

timer_hnd adv_timer;          // Timer para controlar o tempo de advertising
timer_hnd pause_timer;        // Timer para controlar o tempo de pausa
bool is_advertising = false;  // Flag para verificar se está anunciando
uint32_t ttime_milli = 0;			// Total de tempo de milisegundos

uint8_t rx_byte = 0;
uint8_t *p_rx;
// uint8_t* tx_byte = NULL;
uint8_t buffer[BUFFER_SIZE];

uint8_t flag_df_is_ready = 0;

//***************************************************************************************

uint8_t app_connection_idx                      __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
timer_hnd app_adv_data_update_timer_used        __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
timer_hnd app_param_update_request_timer_used   __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY

// Retained variables
struct mnf_specific_data_ad_structure mnf_data  __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
// Index of manufacturer data in advertising data or scan response data (when MSB is 1)
uint8_t mnf_data_index                          __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint8_t stored_adv_data_len                     __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint8_t stored_scan_rsp_data_len                __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint8_t stored_adv_data[ADV_DATA_LEN]           __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint8_t stored_scan_rsp_data[SCAN_RSP_DATA_LEN] __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY

// USER STRUCTS *************************************************************************

typedef struct
{
	uint8_t frame_type; 		// Frame type
	uint8_t version; 				// TML version
	uint16_t batt_value;		// Battery Voltage
	uint16_t temp_value;  	// Temperature value
	uint32_t adv_PDU_count;	// Advertising PDU count
	uint32_t TSPR_value;		// Time since Power-on or reboot
	uint16_t IMEI_num;			// IMEI number
	uint8_t status_RF;			// Status from RF
	uint8_t status_GPS;			// Status from GPS
	//uint8_t checksum;				// Checksum da mensagem
	
}data_frame_t;

// Inicialização dos valores da struct *************************************************
data_frame_t data_frame = {
	
    0x20,          // frame_type (uint8_t)
    0x00,          // version (uint8_t)
    0x0000,        // batt_value (uint16_t)
    0x0000,        // temp_value (uint16_t)
    0x00000000,    // adv_PDU_count (uint32_t)
    0x00000000,    // TSPR_value (uint32_t)
    0x0000,        // IMEI_num (uint16_t)
    0x00,          // status_RF (uint8_t)
    0x00           // status_GPS (uint8_t)
};

rtc_time_t time = {0};
// rtc_calendar_t 	clndr = {0};

// CallBacks *****************************************************************************

void Uart_Receive_Callback(uint16_t data_cnt);

// ***************************************************************************************

uart_cfg_t uart_config = {
	
		.baud_rate = UART_BAUDRATE_9600,
    .data_bits = UART_DATABITS_8,
    .parity = UART_PARITY_NONE,
    .stop_bits = UART_STOPBITS_1,
    .auto_flow_control = UART_AFCE_DIS,
    .use_fifo = UART_FIFO_EN,
    .tx_fifo_tr_lvl = UART_TX_FIFO_LEVEL_0,
    .rx_fifo_tr_lvl = UART_RX_FIFO_LEVEL_0,
    .intr_priority = 0,
		.uart_rx_cb = Uart_Receive_Callback

};

// **************************************************************************************

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
*/

/*
 * USER FUNCTION PROTOTYPE
 ****************************************************************************************
*/

void update_data_frame(data_frame_t *data_frame);

// **************************************************************************************

// USER FUNCTION DEFINITIONS ************************************************************

// Inverte 2 bytes (uint16_t) -> 0x1234 vira 0x3412
uint16_t swap16(uint16_t x) 
{
    return (x << 8) | (x >> 8);
}

// Inverte 4 bytes (uint32_t) -> 0x12345678 vira 0x78563412
uint32_t swap32(uint32_t x) 
{
    return (x << 24) | ((x << 8) & 0x00FF0000) | ((x >> 8) & 0x0000FF00) | (x >> 24);
}

void convert_to_big_endian(data_frame_t* frame) 
{
    
		frame->batt_value = swap16(frame->batt_value);
    frame->temp_value = swap16(frame->temp_value);
    frame->IMEI_num = swap16(frame->IMEI_num);
    frame->adv_PDU_count = swap32(frame->adv_PDU_count);
    frame->TSPR_value = swap32(frame->TSPR_value);
	
}

void Check_Data_Frame(uint8_t* buffer, uint8_t length, data_frame_t* data_frame)
{
	uint16_t IMEI_temp = 0;
	uint8_t status_RF_temp, status_GPS_temp = 0;
	
	for (uint8_t i = 0; i < length; i++)
	{
		if (buffer[i] == 0xAA)
		{
			// Verifica se há dados suficientes no buffer
			if ((i + 1 + 7) <= length) // 7 bytes: [2 IMEI + 1 RF + 1 GPS + 1 checksum + 0xBB + 0xBB]
			{
				uint8_t* payload = &buffer[i + 1];

				if (payload[5] == 0xBB && payload[6] == 0xBB)
				{
					// Extrai dados temporários
					IMEI_temp = (payload[0] << 8) | payload[1];
					status_RF_temp = payload[2];
					status_GPS_temp = payload[3];
					
					uint8_t checksum_recebido = payload[4];
					uint8_t checksum_calculado = payload[0] + payload[1] + payload[2] + payload[3];

					if (checksum_recebido == checksum_calculado)
					{
						flag_df_is_ready = 1;
						data_frame->IMEI_num = (payload[0] << 8) | payload[1];
						data_frame->status_RF = payload[2];
						data_frame->status_GPS = payload[3];
						//data_frame->checksum = checksum_recebido;
						
						update_data_frame(data_frame);
					}
					
					else
					{
						flag_df_is_ready = 0; // Checksum inválido
					}

					break;
				}
			}
		}
	}
}

void Uart_Receive_Callback(uint16_t data_cnt)
{
	
	uart_receive(UART1, &rx_byte, data_cnt, UART_OP_INTR); 
	
	*p_rx = rx_byte;
	
	if(p_rx >= buffer + BUFFER_SIZE)
	{
		p_rx = buffer; // retorna o ponteiro para o inicio do buffer
		Check_Data_Frame(buffer, BUFFER_SIZE, &data_frame);
	}
	
	else{
		p_rx++;
	}
}

void initialize_uart(uart_cfg_t* uart_config)
{
	
	// Limpeza do buffer de armazenamento dos bytes
	memset(buffer, 0, sizeof(buffer));
	
	p_rx = buffer; // Aponta para o primeiro byte do vetor	
	
	// Inicialização da UART1 para recebimento via Interrupção
	uart_initialize(UART1, uart_config);
	uart_enable(UART1);
	uart_baudrate_setf(UART1, UART_BAUDRATE_9600);
	
	uart_register_rx_cb(UART1, Uart_Receive_Callback);
	
}

uint32_t convert_time_to_miliseconds(rtc_time_t* time)
{
	uint32_t ttime_ms = 0;
	
	ttime_ms = ((uint32_t)(time->hour) * 3600000) + ((uint32_t)(time->minute) * 60000) + ((uint32_t)(time->sec) * 1000);
	
	return ttime_ms;
}

void update_data_frame(data_frame_t *data_frame)
{
	
	uint8_t t_value[2] = {0}; // Salvar os valores de temperatura vindos no formato Little Endian
													  // e armazena-los dentro do buffer.
	
	
	// Teste para um valor de tempertura (TODO: Remover do código assim que o sens. funcionar)
	t_value[0] = 0x28; // LSB
	t_value[1] = 0x00; // HSB
	// *************************************************************************************
	
	// Atualização dos valores da struct
	data_frame->batt_value = battery_get_voltage(BATT_CR2032);   // Atualização dos valores de bateria
	memcpy(&data_frame->temp_value, t_value, sizeof(t_value));   // Atualização dos valores de temperatura
	data_frame->adv_PDU_count++;															 	 // Atualização dos valores de contagem PDU
	data_frame->TSPR_value = convert_time_to_miliseconds(&time); // Atualização do valor de hr/min/sec em ms
	
}

// **************************************************************************************

/**
 ****************************************************************************************
 * @brief Initialize Manufacturer Specific Data
 ****************************************************************************************
 */
static void mnf_data_init()
{
    mnf_data.ad_structure_size = sizeof(struct mnf_specific_data_ad_structure ) - sizeof(uint8_t); // minus the size of the ad_structure_size field
    mnf_data.ad_structure_type = GAP_AD_TYPE_MANU_SPECIFIC_DATA;
    mnf_data.company_id[0] = APP_AD_MSD_COMPANY_ID & 0xFF; // LSB
    mnf_data.company_id[1] = (APP_AD_MSD_COMPANY_ID >> 8 )& 0xFF; // MSB
    mnf_data.proprietary_data[0] = 0;
    mnf_data.proprietary_data[1] = 0;
}

/**
 ****************************************************************************************
 * @brief Update Manufacturer Specific Data
 ***********************************																																																																														*****************************************************
 */
static void mnf_data_update()
{
    uint16_t data;

    data = mnf_data.proprietary_data[0] | (mnf_data.proprietary_data[1] << 8);
    data += 1;
    mnf_data.proprietary_data[0] = data & 0xFF;
    mnf_data.proprietary_data[1] = (data >> 8) & 0xFF;

    if (data == 0xFFFF) {
         mnf_data.proprietary_data[0] = 0;
         mnf_data.proprietary_data[1] = 0;
    }
}

/**
 ****************************************************************************************
 * @brief Add an AD structure in the Advertising or Scan Response Data of the
 *        GAPM_START_ADVERTISE_CMD parameter struct.
 * @param[in] cmd               GAPM_START_ADVERTISE_CMD parameter struct
 * @param[in] ad_struct_data    AD structure buffer
 * @param[in] ad_struct_len     AD structure length
 * @param[in] adv_connectable   Connectable advertising event or not. It controls whether
 *                              the advertising data use the full 31 bytes length or only
 *                              28 bytes (Document CCSv6 - Part 1.3 Flags).
 ****************************************************************************************
 */
static void app_add_ad_struct(struct gapm_start_advertise_cmd *cmd, void *ad_struct_data, uint8_t ad_struct_len, uint8_t adv_connectable)
{
    uint8_t adv_data_max_size = (adv_connectable) ? (ADV_DATA_LEN - 3) : (ADV_DATA_LEN);
    
    if ((adv_data_max_size - cmd->info.host.adv_data_len) >= ad_struct_len)
    {
        // Append manufacturer data to advertising data
        memcpy(&cmd->info.host.adv_data[cmd->info.host.adv_data_len], ad_struct_data, ad_struct_len);

        // Update Advertising Data Length
        cmd->info.host.adv_data_len += ad_struct_len;
        
        // Store index of manufacturer data which are included in the advertising data
        mnf_data_index = cmd->info.host.adv_data_len - sizeof(struct mnf_specific_data_ad_structure);
    }
    else if ((SCAN_RSP_DATA_LEN - cmd->info.host.scan_rsp_data_len) >= ad_struct_len)
    {
        // Append manufacturer data to scan response data
        memcpy(&cmd->info.host.scan_rsp_data[cmd->info.host.scan_rsp_data_len], ad_struct_data, ad_struct_len);

        // Update Scan Response Data Length
        cmd->info.host.scan_rsp_data_len += ad_struct_len;
        
        // Store index of manufacturer data which are included in the scan response data
        mnf_data_index = cmd->info.host.scan_rsp_data_len - sizeof(struct mnf_specific_data_ad_structure);
        // Mark that manufacturer data is in scan response and not advertising data
        mnf_data_index |= 0x80;
    }
    else
    {
        // Manufacturer Specific Data do not fit in either Advertising Data or Scan Response Data
        ASSERT_WARNING(0);
    }
    // Store advertising data length
    stored_adv_data_len = cmd->info.host.adv_data_len;
    // Store advertising data
    memcpy(stored_adv_data, cmd->info.host.adv_data, stored_adv_data_len);
    // Store scan response data length
    stored_scan_rsp_data_len = cmd->info.host.scan_rsp_data_len;
    // Store scan_response data
    memcpy(stored_scan_rsp_data, cmd->info.host.scan_rsp_data, stored_scan_rsp_data_len);
}

/**
 ****************************************************************************************
 * @brief Advertisement data update timer callback function.
 ****************************************************************************************
*/
static void adv_data_update_timer_cb()
{
    // If mnd_data_index has MSB set, manufacturer data is stored in scan response
    uint8_t *mnf_data_storage = (mnf_data_index & 0x80) ? stored_scan_rsp_data : stored_adv_data;

    // Update manufacturer data
    mnf_data_update();

    // Update the selected fields of the advertising data (manufacturer data)
    memcpy(mnf_data_storage + (mnf_data_index & 0x7F), &mnf_data, sizeof(struct mnf_specific_data_ad_structure));

    // Update advertising data on the fly
    app_easy_gap_update_adv_data(stored_adv_data, stored_adv_data_len, stored_scan_rsp_data, stored_scan_rsp_data_len);
    
    // Restart timer for the next advertising update
    app_adv_data_update_timer_used = app_easy_timer(APP_ADV_DATA_UPDATE_TO, adv_data_update_timer_cb);
}

/**
 ****************************************************************************************
 * @brief Parameter update request timer callback function.
 ****************************************************************************************
*/

//***************************************************************************************************************
// Função para parada do advertising
static void stop_adv_timer_cb() 
{
    // Para o advertising
    app_easy_gap_advertise_stop();
    is_advertising = false;

    // Agenda o timer para REINICIAR o advertising após 2 minutos
    pause_timer = app_easy_timer(120000, user_app_adv_start); // 120.000 ms = 2 minutos
}

// **************************************************************************************************************

static void param_update_request_timer_cb()
{
    app_easy_gap_param_update_start(app_connection_idx);
    app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;
}

void user_app_init(void)
{
		convert_to_big_endian(&data_frame);
	
    // Inicializa os dados de advertising
    memcpy(stored_adv_data, &data_frame, sizeof(data_frame));
		stored_adv_data_len = sizeof(data_frame);
    
    // Inicia o advertising pela primeira vez
    // user_app_adv_start();
		
		// Inicializa o RTC
		rtc_clk_config(RTC_DIV_DENOM_1000, 32000);
		rtc_clock_enable();
		rtc_start();
	
		// Atualiza o valor total em milisegundos
		ttime_milli = convert_time_to_miliseconds(&time);
	
		// Inicializa o BLE
    default_app_on_init();
	
}

void user_app_adv_start(void)
{
     if (!is_advertising) 
		 {
        struct gapm_start_advertise_cmd* cmd = app_easy_gap_undirected_advertise_get_active();
        
        // Adiciona dados customizados (opcional)
        // app_add_ad_struct(cmd, &mnf_data, sizeof(mnf_data), 1);
        
        // Inicia o advertising
        app_easy_gap_undirected_advertise_start();
        is_advertising = true;
			 
				if(flag_df_is_ready)
				{
					// Atualização do DF
					update_data_frame(&data_frame);
					flag_df_is_ready = 0;
				}
			 
				// Converte para big-endian (Eddystone)
				// convert_to_big_endian(&data_frame);

        // Agenda o timer para PARAR o advertising após 30 segundos
        adv_timer = app_easy_timer(30000, stop_adv_timer_cb); // 30.000 ms = 30 segundos
     }
}

void user_app_connection(uint8_t connection_idx, struct gapc_connection_req_ind const *param)
{
    if (app_env[connection_idx].conidx != GAP_INVALID_CONIDX)
    {
        app_connection_idx = connection_idx;

        // Stop the advertising data update timer
        app_easy_timer_cancel(app_adv_data_update_timer_used);

        // Check if the parameters of the established connection are the preferred ones.
        // If not then schedule a connection parameter update request.
        if ((param->con_interval < user_connection_param_conf.intv_min) ||
            (param->con_interval > user_connection_param_conf.intv_max) ||
            (param->con_latency != user_connection_param_conf.latency) ||
            (param->sup_to != user_connection_param_conf.time_out))
        {
            // Connection params are not these that we expect
            app_param_update_request_timer_used = app_easy_timer(APP_PARAM_UPDATE_REQUEST_TO, param_update_request_timer_cb);
        }
    }
    else
    {
        // No connection has been established, restart advertising
        user_app_adv_start();
    }

    default_app_on_connection(connection_idx, param);
}

void user_app_adv_undirect_complete(uint8_t status)
{
    // If advertising was canceled then update advertising data and start advertising again
    if (status == GAP_ERR_CANCELED)
    {
        // user_app_adv_start();
				is_advertising = false;
    }
}

void user_app_disconnect(struct gapc_disconnect_ind const *param)
{
    // Cancel the parameter update request timer
    //if (app_param_update_request_timer_used != EASY_TIMER_INVALID_TIMER)
    //{
    //    app_easy_timer_cancel(app_param_update_request_timer_used);
    //    app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;
    //}
    // Update manufacturer data for the next advertsing event
    //mnf_data_update();
    // Restart Advertising
    //user_app_adv_start();
	
	  app_easy_timer_cancel(adv_timer);
    app_easy_timer_cancel(pause_timer);
    is_advertising = false;
}

void user_catch_rest_hndl(ke_msg_id_t const msgid,
                          void const *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id)
{
    switch(msgid)
    {
        case GAPC_PARAM_UPDATED_IND:
        {
            // Cast the "param" pointer to the appropriate message structure
            struct gapc_param_updated_ind const *msg_param = (struct gapc_param_updated_ind const *)(param);

            // Check if updated Conn Params filled to preferred ones
            if ((msg_param->con_interval >= user_connection_param_conf.intv_min) &&
                (msg_param->con_interval <= user_connection_param_conf.intv_max) &&
                (msg_param->con_latency == user_connection_param_conf.latency) &&
                (msg_param->sup_to == user_connection_param_conf.time_out))
            {
            }
        } break;

        case GATTC_EVENT_REQ_IND:
        {
            // Confirm unhandled indication to avoid GATT timeout
            struct gattc_event_ind const *ind = (struct gattc_event_ind const *) param;
            struct gattc_event_cfm *cfm = KE_MSG_ALLOC(GATTC_EVENT_CFM, src_id, dest_id, gattc_event_cfm);
            cfm->handle = ind->handle;
            KE_MSG_SEND(cfm);
        } break;

        default:
            break;
    }
}
	


/// @} APP
