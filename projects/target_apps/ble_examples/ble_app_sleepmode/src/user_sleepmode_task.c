/**
 ****************************************************************************************
 *
 * @file user_sleepmode_task.c
 *
 * @brief Sleep mode project source code.
 *
 * Copyright (C) 2016-2025 Renesas Electronics Corporation and/or its affiliates.
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

#include "user_sleepmode_task.h"
#include "wkupct_quadec.h"
#include "gpio.h"
#include "timer0_2.h"
#include "timer0.h"
#include "adc.h"
#include "custs1_task.h"
#include "user_custs1_def.h"
#include "user_sleepmode.h"
#include "user_periph_setup.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define APP_PWM_ON      (1000)
#define APP_PWM_HIGH    (500)
#define APP_PWM_LOW     (200)
#define APP_NOTES_NUM   (26)

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

struct app_proj_env_tag app_sleep_env __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY

const uint16_t notes[APP_NOTES_NUM] =
{
    1046, 987, 767, 932, 328, 880, 830, 609, 783, 991, 739, 989, 698, 456, 659, 255, 622,
    254, 587, 554, 365, 523, 251, 493, 466, 440
};

static const adc_config_t adc_cfg =
{
#if defined (__DA14531__)
    .input_mode       = ADC_INPUT_MODE_SINGLE_ENDED,
    .input            = ADC_INPUT_SE_VDD,
    .smpl_time_mult   = 2,
    .continuous       = false,
    .interval_mult    = 0,
    .input_attenuator = ADC_INPUT_ATTN_4X,
    .chopping         = false,
    .oversampling     = 0
#else
    .mode  = ADC_INPUT_MODE_SINGLE_ENDED,
    .input = ADC_INPUT_SE_VDD_REF,
    .sign  = false,
    .attn  = true
#endif
};

/*
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Timer callback function. Update PWM settings.
 ****************************************************************************************
 */
static void user_app_pwm_callback_function(void);

/**
 ****************************************************************************************
 * @brief Enable PWM timer.
 ****************************************************************************************
 */
static void user_app_enable_pwm(void);

/**
 ****************************************************************************************
 * @brief Disable PWM timer.
 ****************************************************************************************
 */
static void user_app_disable_pwm(void);

/**
 ****************************************************************************************
 * @brief Set button event configuration.
 * @param[in] next_event    Next event number
 ****************************************************************************************
 */
static void user_app_set_button_event(uint8_t next_event);

/**
 ****************************************************************************************
 * @brief Disable button action.
 ****************************************************************************************
 */
static void user_app_disable_button(void);

/**
 ****************************************************************************************
 * @brief Callback function for button action. Update button state characteristic.
 ****************************************************************************************
 */
static void user_app_button_press_cb(void);

/**
 ****************************************************************************************
 * @brief Read ADC val and update ADC_VAL2 characteristic.
 ****************************************************************************************
*/
static void user_app_get_adc_val(void);

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

static void user_app_pwm_callback_function(void)
{
    static uint8_t change_delay     __SECTION_ZERO("retention_mem_area0");
    static uint8_t notes_idx        __SECTION_ZERO("retention_mem_area0");

    if (change_delay == 10)
    {
        change_delay = 0;
        timer0_set_pwm_on_counter(0xFFFF);
        timer0_set_pwm_high_counter(notes[notes_idx] / 3 * 2);
        timer0_set_pwm_low_counter(notes[notes_idx] / 3);
        notes_idx = (notes_idx + 1) % (APP_NOTES_NUM - 1);
    }
    change_delay++;
}

static void user_app_enable_pwm(void)
{
    if(!app_sleep_env.custs1_pwm_enabled)
    {
        tim0_2_clk_div_config_t clk_div_config =
        {
            .clk_div = TIM0_2_CLK_DIV_1
        };

        app_sleep_env.custs1_pwm_enabled = 1;

        // Disable sleep mode
        arch_force_active_mode();

        // Enable the Timer0/Timer2 input clock
        timer0_2_clk_enable();
        // Set the Timer0/Timer2 input clock division factor to 8, so 16 MHz / 8 = 2 MHz input clock
        timer0_2_clk_div_set(&clk_div_config);
        // Init timer 0
        timer0_init(TIM0_CLK_FAST, PWM_MODE_ONE, TIM0_CLK_NO_DIV);
        // Set timing parameters
        timer0_set(APP_PWM_ON, APP_PWM_HIGH, APP_PWM_LOW);
        // Register IRQ callback
        timer0_register_callback(user_app_pwm_callback_function);
        // Enable IRQ
        timer0_enable_irq();
        // Start timer
        timer0_start();
    }
}

static void user_app_disable_pwm(void)
{
    if(app_sleep_env.custs1_pwm_enabled)
    {
        app_sleep_env.custs1_pwm_enabled = 0;

        arch_restore_sleep_mode();

        timer0_stop();

        // Disable the Timer0/Timer2 input clock
        timer0_2_clk_disable();
    }
}

static void user_app_set_button_event(uint8_t next_event)
{

    wkupct_register_callback(user_app_button_press_cb);

    wkupct_enable_irq(WKUPCT_PIN_SELECT(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN),
                      WKUPCT_PIN_POLARITY(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN, next_event), // polarity
                      1, // 1 event
                      10); // debouncing time = 10 ms

}

static void user_app_disable_button(void)
{
    wkupct_disable_irq();
}

static void user_app_button_press_cb(void)
{
    uint8_t next_btn_event = 0;

    // Read button state
    if(GPIO_GetPinStatus(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN))
    {
        app_sleep_env.custs1_btn_state = CUSTS1_BTN_STATE_RELEASED;
        next_btn_event = WKUPCT_PIN_POLARITY_LOW;
    }
    else
    {
        app_sleep_env.custs1_btn_state = CUSTS1_BTN_STATE_PRESSED;
        next_btn_event = WKUPCT_PIN_POLARITY_HIGH;
    }

    // Update button characteristic
    struct custs1_val_ntf_ind_req *req = KE_MSG_ALLOC_DYN(CUSTS1_VAL_NTF_REQ,
                                                          prf_get_task_from_id(TASK_ID_CUSTS1),
                                                          TASK_APP,
                                                          custs1_val_ntf_ind_req,
                                                          DEF_SVC1_BUTTON_STATE_CHAR_LEN);

    req->conidx = app_env->conidx;
    req->notification = true;
    req->handle = SVC1_IDX_BUTTON_STATE_VAL;
    req->length = DEF_SVC1_BUTTON_STATE_CHAR_LEN;
    req->value[0] = app_sleep_env.custs1_btn_state;

    KE_MSG_SEND(req);

    // Configure next button event
    user_app_set_button_event(next_btn_event);
}

static void user_app_get_adc_val(void)
{
    if (app_sleep_env.custs1_adcval2_enabled)
    {
        uint16_t adc_sample;

        adc_offset_calibrate(ADC_INPUT_MODE_SINGLE_ENDED);

        adc_init(&adc_cfg);
        adc_sample = adc_get_sample();
        adc_disable();

        struct custs1_val_set_req *req = KE_MSG_ALLOC_DYN(CUSTS1_VAL_SET_REQ,
                                                          prf_get_task_from_id(TASK_ID_CUSTS1),
                                                          TASK_APP,
                                                          custs1_val_set_req,
                                                          DEF_SVC1_ADC_VAL_2_CHAR_LEN);

       req->conidx = app_env->conidx;
       req->handle = SVC1_IDX_ADC_VAL_2_VAL;
       req->length = DEF_SVC1_ADC_VAL_2_CHAR_LEN;
       memcpy(req->value, &adc_sample, DEF_SVC1_ADC_VAL_1_CHAR_LEN);

       KE_MSG_SEND(req);
    }
}

void user_app_enable_periphs(void)
{
    // Update button state characteristic
    user_app_button_press_cb();
    // Get initial ADC value if enabled
    user_app_get_adc_val();
}

void user_app_disable_periphs(void)
{
    user_app_disable_pwm();
    user_app_disable_button();
}

void user_svc1_ctrl_wr_ind_handler(ke_msg_id_t const msgid,
                                     struct custs1_val_write_ind const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    if(param->handle == SVC1_IDX_CONTROL_POINT_VAL)
    {
        if(param->value[0] == CUSTS1_CP_CMD_PWM_ENABLE)
        {
            user_app_enable_pwm();
        }
        else if(param->value[0] == CUSTS1_CP_CMD_PWM_DISABLE)
        {
            user_app_disable_pwm();
        }
        else if(param->value[0] == CUSTS1_CP_CMD_ADC_VAL_2_ENABLE)
        {
            app_sleep_env.custs1_adcval2_enabled = 1;
            user_app_get_adc_val();
        }
        else if(param->value[0] == CUSTS1_CP_CMD_ADC_VAL_2_DISABLE)
        {
            app_sleep_env.custs1_adcval2_enabled = 0;
        }
    }
}

/// @} APP
