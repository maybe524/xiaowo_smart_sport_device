#include "common.h"
#include "app.h"

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_error.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "app_pwm.h"

APP_PWM_INSTANCE(PWM1,1);                   // Create the instance "PWM1" using TIMER1.

static volatile bool ready_flag;            // A flag indicating PWM status.

void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    ready_flag = true;
}

int vibr_test(void)
{
    ret_code_t err_code;

    /* 2-channel PWM, 200Hz, output on DK LED pins. */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(5000L, BSP_LED_0, BSP_LED_1);

    /* Switch the polarity of the second channel. */
    pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;

    /* Initialize and enable PWM. */
    err_code = app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM1);

    uint32_t value;
    while (true)
    {
        for (uint8_t i = 0; i < 40; ++i)
        {
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

int app_vibr_init(void)
{
    NRF_LOG_INFO("app_vibr_init");
    
    
    
    return 0;
}