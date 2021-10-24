#include "common.h"
#include "app.h"

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_error.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "app_pwm.h"

#define MOTOR_PIN   3
APP_PWM_INSTANCE(PWM1, 1);                   // Create the instance "PWM1" using TIMER1.

static volatile bool ready_flag;            // A flag indicating PWM status.
static TaskHandle_t m_vibr_thread;
static bool is_need_vibr_test = 0, is_vibr_pwm_inited = false;
static bool is_need_show_power_by_vbir_led = 0;
extern bool g_is_app_init_done;

int vibr_set_test(void)
{
    MSG_DEBUG("vibr_set_test");
    is_need_vibr_test = true;
    return 0;
}

int vibr_set_close_test(void)
{
    MSG_DEBUG("vibr_set_close_test");
    is_need_vibr_test = false;
    return 0;
}

int vibr_set_show_power_led(void)
{
    MSG_DEBUG("vibr_set_show_power_led");
    is_need_show_power_by_vbir_led = true;
    return 0;
}

int vibr_set_off_power_led(void)
{
    MSG_DEBUG("vibr_set_off_power_led");
    is_need_show_power_by_vbir_led = false;
    return 0;
}


static void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    ready_flag = true;
}

static void vibr_pwm_init(void)
{
    ret_code_t err_code;
    uint32_t value;
    
    NRF_LOG_INFO("vibr_pwm_init: %d", is_vibr_pwm_inited);
    if (is_vibr_pwm_inited)
        return;
    /* 2-channel PWM, 200Hz, output on DK LED pins. */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(5000L, MOTOR_PIN);   // 5000L
    /* Switch the polarity of the second channel. */
    pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
    /* Initialize and enable PWM. */
    err_code = app_pwm_init(&PWM1, &pwm1_cfg, pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
    is_vibr_pwm_inited = true;
}

static int vibr_test(void)
{
    ret_code_t err_code;
    uint32_t value;
    uint32_t time_turnon_led_gap = 0, time_exit_gap = 0;
    
    NRF_LOG_INFO("vibr_test start");
    vibr_pwm_init();
    app_pwm_enable(&PWM1);
    while (true) {
#if 1
        if (!is_need_vibr_test || !app_get_bleconn_status() || time_exit_gap >= 400)
            break;
#endif
        for (uint8_t i = 0; i < 40; ++i) {
            value = (i < 20) ? (i * 5) : (100 - (i - 20) * 5);
            ready_flag = false;
            /* Set the duty cycle - keep trying until PWM is ready... */
            while (app_pwm_channel_duty_set(&PWM1, 0, value) == NRF_ERROR_BUSY);
            /* ... or wait for callback. */
            while (!ready_flag);
            nrf_delay_ms(25);
            time_exit_gap++;
        }
    }
    while (app_pwm_channel_duty_set(&PWM1, 0, 0) == NRF_ERROR_BUSY);
    app_pwm_disable(&PWM1);
    is_need_vibr_test = false;
    nrf_gpio_cfg_input(MOTOR_PIN, NRF_GPIO_PIN_INPUT_DISCONNECT);
    NRF_LOG_INFO("vibr_test done");
    return 0;
}

///< 接口仅适合在任务里边调用，因为会调用任务切换函数
static int vibr_run_mix(void)
{
    ret_code_t err_code;
    uint32_t value;
    int ret = 0;
    uint32_t power_percent = 0;
    int is_pwm_inited = 0;
    uint8_t i = 0;
    uint32_t time_turnon_led_gap = 40, time_exit_pwm_gap = 0, time_exit_show_gap = 0;
    char led_status = 1;
    bool is_vibr_run_mix_inited = false;
    unsigned int pwm_timeout = 0;
    bool is_need_switch_on = false;
    unsigned int vbir_keep_run_timestamp = 0;

    NRF_LOG_INFO("vibr_run_mix start");

    while (true) {
retry:
        NRF_LOG_INFO("vibr_run_mix power_percent: %02d, time: %04d, charging: %d, %d, %d", \
                power_percent, time_exit_show_gap, battery_get_power_charging(),\
                is_need_show_power_by_vbir_led, is_need_vibr_test);
        if (!is_need_show_power_by_vbir_led && !is_need_vibr_test) {
            NRF_LOG_INFO("timeout: %d, show_power: %d, vibr_test: %d", \
                time_exit_show_gap, \
                is_need_show_power_by_vbir_led, \
                is_need_vibr_test);
            break;
        }

        if (is_need_show_power_by_vbir_led) {
            ret = battery_get_power_percent(&power_percent);
            if (ret && time_exit_show_gap < 20) {
                goto next1;
            }
            else if (!ret && time_exit_show_gap >= 10) {
                is_need_show_power_by_vbir_led = false;
                goto next1;
            }

            ///< LED灯根据电量显示
            ///< 1) 绿色表示电量大于80%；
            ///< 2) 红色表示低于20%；
            ///< 3）红灯闪烁 + 马达震动5秒表示电量低于10%
            if (power_percent < 10) {
                if (!is_vibr_run_mix_inited) {
                    vibr_pwm_init();
                    app_pwm_enable(&PWM1);
                    is_vibr_run_mix_inited = true;
                }
                for (i = 0; i < 40; ++i) {
                    value = (i < 20) ? (i * 5) : (100 - (i - 20) * 5);
                    ready_flag = false;
                    /* Set the duty cycle - keep trying until PWM is ready... */
                    pwm_timeout = 1000;
                    while (true) {
                        if (!pwm_timeout || app_pwm_channel_duty_set(&PWM1, 0, value) != NRF_ERROR_BUSY)
                            break;
                        pwm_timeout--;
                        vTaskDelay(25);
                    }
                    /* ... or wait for callback. */
                    pwm_timeout = 1000;
                    while (true) {
                        if (ready_flag || !pwm_timeout)
                            break;
                        pwm_timeout--;
                        vTaskDelay(25);
                    }
                    vTaskDelay(25);
                    vbir_keep_run_timestamp++;
                    if (vbir_keep_run_timestamp > 12) {
                        vbir_keep_run_timestamp = 0;
                        led_3gpio_set_led(is_need_switch_on, 0, 0);
                        is_need_switch_on = is_need_switch_on ? false : true;
                    }
                }
                while (true) {
                    if (app_pwm_channel_duty_set(&PWM1, 0, 0) != NRF_ERROR_BUSY)
                        break;
                    vTaskDelay(25);
                }
            }
            else if (10 < power_percent && power_percent <= 20)
                led_3gpio_set_led(1, 0, 0);
            else if (20 < power_percent)
                led_3gpio_set_led(0, 1, 0);
         }

next1:
        if (is_need_vibr_test) {
#if 1
            if (!is_need_vibr_test || !app_get_bleconn_status() || time_exit_show_gap >= 10) {
                is_need_vibr_test = false;
                goto next2;
            }
#endif
            if (!is_vibr_run_mix_inited) {
                vibr_pwm_init();
                app_pwm_enable(&PWM1);
                is_vibr_run_mix_inited = true;
            }
            for (uint8_t i = 0; i < 40; ++i) {
                value = (i < 20) ? (i * 5) : (100 - (i - 20) * 5);
                ready_flag = false;
                /* Set the duty cycle - keep trying until PWM is ready... */
                pwm_timeout = 1000;
                while (true) {
                    if (!pwm_timeout || app_pwm_channel_duty_set(&PWM1, 0, value) != NRF_ERROR_BUSY)
                        break;
                    pwm_timeout--;
                    vTaskDelay(25);
                }
                /* ... or wait for callback. */
                pwm_timeout = 1000;
                while (true) {
                    if (ready_flag || !pwm_timeout)
                        break;
                    pwm_timeout--;
                    vTaskDelay(25);
                }
                vTaskDelay(25);
            }
            
            pwm_timeout = 1000;
            while (true) {
                if (!pwm_timeout || app_pwm_channel_duty_set(&PWM1, 0, 0) != NRF_ERROR_BUSY)
                    break;
                pwm_timeout--;
                vTaskDelay(25);
            }
        }

next2:
        vTaskDelay(300);
        time_exit_show_gap++;
        time_turnon_led_gap++;
    }

    ///< 关闭三色灯
    led_3gpio_set_led(0, 0, 0);
    ///< 关闭标志位
    is_need_show_power_by_vbir_led = false;
    is_need_vibr_test = false;

    if (is_vibr_run_mix_inited) {
        app_pwm_disable(&PWM1);
        nrf_gpio_cfg_input(MOTOR_PIN, NRF_GPIO_PIN_INPUT_DISCONNECT);
    }
    
    NRF_LOG_INFO("vibr_run_mix done");
    return 0;
}

static void vibr_service_thread(void *arg)
{
    int step = 0, ret = 0;
    
    NRF_LOG_INFO("vibr_service_thread start");
    while (true) {
TASK_GEN_ENTRY_STEP(0) {
        if (g_is_app_init_done && (is_need_vibr_test || is_need_show_power_by_vbir_led))
            step++;
        else
            vTaskDelay(300);
      }

TASK_GEN_ENTRY_STEP(1) {
        vibr_run_mix();
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
