#include "common.h"
#include "app.h"

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_error.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "app_pwm.h"

#define MOTOR_PIN   10
APP_PWM_INSTANCE(PWM1, 1);                   // Create the instance "PWM1" using TIMER1.

static volatile bool ready_flag;            // A flag indicating PWM status.
static TaskHandle_t m_vibr_thread;
static bool is_need_vibr_test = false;

int vibr_set_test(void)
{
    NRF_LOG_INFO("vibr_set_test");
    is_need_vibr_test = true;
    return 0;
}

int vibr_set_close_test(void)
{
    NRF_LOG_INFO("vibr_set_close_test");
    is_need_vibr_test = false;
    return 0;
}

static void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    ready_flag = true;
}

static int vibr_test(void)
{
    ret_code_t err_code;
    uint32_t value;
    
    /* 2-channel PWM, 200Hz, output on DK LED pins. */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(5000L, MOTOR_PIN);
    /* Switch the polarity of the second channel. */
    pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
    /* Initialize and enable PWM. */
    err_code = app_pwm_init(&PWM1,&pwm1_cfg, pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM1);

    while (true) {
        if (!is_need_vibr_test)
            break;
        for (uint8_t i = 0; i < 40; ++i) {
            value = (i < 20) ? (i * 5) : (100 - (i - 20) * 5);
            ready_flag = false;
            /* Set the duty cycle - keep trying until PWM is ready... */
            while (app_pwm_channel_duty_set(&PWM1, 0, value) == NRF_ERROR_BUSY);
            /* ... or wait for callback. */
            while (!ready_flag);
            APP_ERROR_CHECK(app_pwm_channel_duty_set(&PWM1, 1, value));
            nrf_delay_ms(25);
        }
    }
}

static void vibr_service_thread(void *arg)
{
    int step = 0, ret = 0;
    
    NRF_LOG_INFO("vibr_service_thread start");
    while (true) {
TASK_GEN_ENTRY_STEP(0) {
        if (!is_need_vibr_test)
            vTaskDelay(300);
        else
            step++;
      }

TASK_GEN_ENTRY_STEP(1) {
        vibr_test();
        step = 0;
      }
    }
}

int app_vibr_init(void)
{
    BaseType_t ret;
    
    NRF_LOG_INFO("app_vibr_init");
    ret = xTaskCreate(vibr_service_thread, "VIBR", 64, NULL, 2, &m_vibr_thread);
    if (ret != pdPASS){
        NRF_LOG_INFO("xTaskCreate fail, ret: %d", ret);
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
	
    return 0;
}
