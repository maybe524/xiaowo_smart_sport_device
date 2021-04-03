/** \file main.cpp ******************************************************
*
* Project: STM32F103C8T6+MAX30102
* Edited by Anning
* Shop website: https://shop108071095.taobao.com
* Email: anning865@126.com
* ------------------------------------------------------------------------- */
/*******************************************************************************
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/
/*!\mainpage Main Page
*
* \section intro_sec Introduction
*
* This is the code documentation for the MAXREFDES117# subsystem reference design.
*
*  The Files page contains the File List page and the Globals page.
*
*  The Globals page contains the Functions, Variables, and Macros sub-pages.
*
* \image html MAXREFDES117_Block_Diagram.png "MAXREFDES117# System Block Diagram"
*
* \image html MAXREFDES117_firmware_Flowchart.png "MAXREFDES117# Firmware Flowchart"
*
*/
//#include "stm32f103c8t6.h"
//#include "mbed.h"
#include "common.h"
#include "algorithm.h"
#include "max30102_driver.h"

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_error.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "app_pwm.h"
#include "app.h"

#define MAX30102_INT_PIN    19
#define MAX_BRIGHTNESS 255
#define MAX30102_LEN_PIN    9

uint32_t aun_ir_buffer[500]; //IR LED sensor data
int32_t n_ir_buffer_length;    //data length
uint32_t aun_red_buffer[500];    //Red LED sensor data
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;
bool is_need_stop_hr_spo2 = false;
static common_event_callback_t *p_user_callback = NULL;

//Serial pc(SERIAL_TX, SERIAL_RX);    //initializes the serial port, TX-PA2, RX-PA3

//PwmOut pwmled(PB_3);  //initializes the pwm output PB3 that connects to the LED
//DigitalIn INT(PB_7);  //pin PB7 connects to the interrupt output pin of the MAX30102
//DigitalOut led(PC_13); //PC13 connects to the on board user LED

APP_PWM_INSTANCE(PWM0, 0);                   // Create the instance "PWM1" using TIMER1.

static volatile bool max30102_pwm_ready_flag;            // A flag indicating PWM status.

int max30102_set_hr_spo2(bool on)
{
    is_need_stop_hr_spo2 = !on;
    return 0;
}

static void max30102_pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    max30102_pwm_ready_flag = true;
}

static int max30102_pwm_test(void)
{
    ret_code_t err_code;
    uint32_t value;

    /* 2-channel PWM, 200Hz, output on DK LED pins. */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(5000L, BSP_LED_0);
    /* Switch the polarity of the second channel. */
    pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
    /* Initialize and enable PWM. */
    err_code = app_pwm_init(&PWM0,&pwm1_cfg, max30102_pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM0);

    while (true) {
        for (uint8_t i = 0; i < 40; ++i) {
            value = (i < 20) ? (i * 5) : (100 - (i - 20) * 5);
            max30102_pwm_ready_flag = false;
            /* Set the duty cycle - keep trying until PWM is ready... */
            while (app_pwm_channel_duty_set(&PWM0, 0, value) == NRF_ERROR_BUSY);
            /* ... or wait for callback. */
            while (!max30102_pwm_ready_flag);
            APP_ERROR_CHECK(app_pwm_channel_duty_set(&PWM0, 1, value));
            nrf_delay_ms(25);
        }
    }
}


// the setup routine runs once when you press reset:
int max30102_init(void)
{
    uint8_t revision_id = 0, part_id = 0;
    bool ret;

    NRF_LOG_INFO("max30102_init");
    max30102_twi_init();
    ret = maxim_max30102_read_reg(REG_REV_ID, &revision_id, 1);
    if (!ret) {
        NRF_LOG_INFO("max30102 read fail");
        return -1;
    }
    maxim_max30102_read_reg(REG_PART_ID, &part_id, 1);
    NRF_LOG_INFO("revision_id: 0x%02x, part_id: 0x%02x", revision_id, part_id);
    if (part_id != 0x15) {
        NRF_LOG_INFO("max30102_init fail");
        return -2;
    }
    // 中断是低有效，所有要上拉
    nrf_gpio_cfg_input(MAX30102_INT_PIN, NRF_GPIO_PIN_PULLUP);
    // nrf_gpio_cfg_input(MAX30102_LED_PIN, NRF_GPIO_PIN_PULLUP);

    maxim_max30102_reset(); //resets the MAX30102
    //read and clear status register
    maxim_max30102_read_reg(0, &uch_dummy, 1);
    ret = maxim_max30102_init();  //initializes the MAX30102
    NRF_LOG_INFO("maxim_max30102_init, ret: %d", ret);
    if (!ret)
        return -3;

    return (int)true;
}

int max30102_exit(void)
{
    NRF_LOG_INFO("max30102_module_exit");
    nrf_gpio_cfg_default(MAX30102_INT_PIN);
    maxim_max30102_reset();

    return 0;
}

int max30102_user_event_callback_init(common_event_callback_t p_call_back)
{
    p_user_callback = p_call_back;

    return 0;
}

int max30102_collect_data(void)
{
    int i;
    int32_t n_brightness;
    uint32_t un_min, un_max, un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats
    uint32_t gpio_value = 0, timeout_cnt = 100000;
    int8_t intr_status_1, intr_status_2, intr_enable_1, intr_enable_2;
    float f_temp;
    struct event_hr_spo2 {
        unsigned int hr, spo2;
    };
    struct event_hr_spo2 user_event_hr_spo2 = {0};

    n_brightness = 0;
    un_min = 0x3FFFF;
    un_max = 0;
    n_ir_buffer_length = 500; //buffer length of 100 stores 5 seconds of samples running at 100sps
    intr_enable_1 = 0;
    intr_enable_2 = 0;

    NRF_LOG_INFO("max30102_collect_data test");

    maxim_max30102_read_reg(REG_INTR_ENABLE_1, &intr_status_1, 1);
    maxim_max30102_read_reg(REG_INTR_ENABLE_2, &intr_status_2, 1);
    NRF_LOG_INFO("enable_1: 0x%x, enable_2: 0x%x", intr_status_1, intr_status_2);

    //read the first 500 samples, and determine the signal range
    for (i = 0; i < n_ir_buffer_length; i++) {
        //while(INT.read()==1);   //wait until the interrupt pin asserts
        timeout_cnt = 100000;
        while (1) {
            gpio_value = nrf_gpio_pin_read(MAX30102_INT_PIN);
            timeout_cnt--;
            intr_status_1 = 0;
            intr_status_2 = 0;
            maxim_max30102_read_reg(REG_INTR_STATUS_1, &intr_status_1, 1);
            maxim_max30102_read_reg(REG_INTR_STATUS_2, &intr_status_2, 1);
            NRF_LOG_INFO("1 status_1: 0x%02x, status_2: 0x%02x, gpio_value: %d", intr_status_1, intr_status_2, gpio_value);
            if (!gpio_value || !timeout_cnt)
                break;
            // vTaskDelay(1);
        }
        if (!timeout_cnt)
            NRF_LOG_INFO("waiting timeout");
        // NRF_LOG_INFO("nrf_gpio_pin_read low");
        maxim_max30102_read_fifo((aun_red_buffer + i), (aun_ir_buffer + i));  //read from MAX30102 FIFO
        if (un_min>aun_red_buffer[i]) {
            un_min=aun_red_buffer[i];    //update signal min
        }
        if (un_max<aun_red_buffer[i]) {
            un_max=aun_red_buffer[i];    //update signal max
        }
        NRF_LOG_INFO("red = %i, ir = %i", aun_red_buffer[i], aun_ir_buffer[i]);
    }
    un_prev_data = aun_red_buffer[i];

    //calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);

    //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
    while (1) {
        i = 0;
        un_min = 0x3FFFF;
        un_max = 0;

        if (is_need_stop_hr_spo2) {
            NRF_LOG_INFO("max30102_collect_data stop collect data\n");
            break;
        }
        //dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
        for (i = 100; i < 500; i++) {
            aun_red_buffer[i - 100] = aun_red_buffer[i];
            aun_ir_buffer[i - 100] = aun_ir_buffer[i];
            //update the signal min and max
            if (un_min > aun_red_buffer[i]) {
                un_min = aun_red_buffer[i];
            }
            if(un_max < aun_red_buffer[i]) {
                un_max = aun_red_buffer[i];
            }
        }

        //take 100 sets of samples before calculating the heart rate.
        for (i = 400; i < 500; i++) {
            un_prev_data = aun_red_buffer[i - 1];
            //while(INT.read()==1);
            timeout_cnt = 100000;
            while (true) {
                gpio_value = nrf_gpio_pin_read(MAX30102_INT_PIN);
                timeout_cnt--;
                if (!gpio_value || !timeout_cnt)
                    break;
                // intr_status_1 = 0;
                // intr_status_2 = 0;
                // maxim_max30102_read_reg(REG_INTR_STATUS_1, &intr_status_1, 1);
                // maxim_max30102_read_reg(REG_INTR_STATUS_2, &intr_status_2, 1);
                // NRF_LOG_INFO("2 status_1: 0x%02x, status_2: 0x%02x, gpio_value: %d", intr_status_1, intr_status_2, gpio_value);
                // vTaskDelay(1);
            }
            // NRF_LOG_INFO("exit waiting, remain: %d, gpio_value: %d", timeout_cnt, gpio_value);
            maxim_max30102_read_fifo((aun_red_buffer + i), (aun_ir_buffer + i));

            //just to determine the brightness of LED according to the deviation of adjacent two AD data
            if (aun_red_buffer[i] > un_prev_data) {
                f_temp = aun_red_buffer[i] - un_prev_data;
                f_temp /= (un_max-un_min);
                f_temp *= MAX_BRIGHTNESS;
                n_brightness -= (int)f_temp;
                if (n_brightness < 0) {
                    n_brightness = 0;
                }
            }
            else {
                f_temp = un_prev_data - aun_red_buffer[i];
                f_temp /= (un_max-un_min);
                f_temp *= MAX_BRIGHTNESS;
                n_brightness += (int)f_temp;
                if (n_brightness > MAX_BRIGHTNESS) {
                    n_brightness = MAX_BRIGHTNESS;
                }
            }

            //pwmled.write(1-(float)n_brightness/256);//pwm control led brightness
			//if(n_brightness<120)
            //    led=1;
			//else
			//	led=0;

            //send samples and calculation result to terminal program through UART
            NRF_LOG_INFO("red: %08d, ir: %08d, HR: %08d, HRvalid: %d, SpO2: %08d, SPO2Valid: %d", \
                aun_red_buffer[i], \
                aun_ir_buffer[i], \
                n_heart_rate, \
                ch_hr_valid, \
                n_sp02, \
                ch_spo2_valid);
        }
        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, \
                n_ir_buffer_length, \
                aun_red_buffer, \
                &n_sp02, \
                &ch_spo2_valid, \
                &n_heart_rate, \
                &ch_hr_valid);
        if (p_user_callback && ch_hr_valid && ch_spo2_valid) {
            user_event_hr_spo2.hr = n_heart_rate;
            user_event_hr_spo2.spo2 = n_sp02;
            p_user_callback(1, (unsigned long)&user_event_hr_spo2);
        }
    }
}

int max30102_collect_raw_data(uint32_t *aun_red, uint32_t *aun_ir)
{
    maxim_max30102_read_fifo(aun_red, aun_ir);

    return 0;
}
