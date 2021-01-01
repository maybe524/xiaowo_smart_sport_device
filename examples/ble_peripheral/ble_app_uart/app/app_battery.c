#include "common.h"
#include "app.h"
#include "app_protocol.h"
#include "nrf_drv_saadc.h"

#define SAMPLES_IN_BUFFER 1
#define GPIO_PIN_ON_OFF     6

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600                 /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6                   /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  1000                    /**< Typical forward voltage drop of the diode . */
#define ADC_RES_10BIT                   1024                /**< Maximum digital value for 10-bit ADC conversion. */

// VP = (RESULT * REFERENCE / 2^10) * 6
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

static TaskHandle_t m_battery_service_thread;
static bool is_need_upload_power_percent = false;
static nrf_saadc_value_t     m_buffer_pool[SAMPLES_IN_BUFFER];
static bool is_saadc_sample_done = false;

int battery_get_power_percent(void)
{
    is_need_upload_power_percent = true;
    return 0;
}

static void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    int i;
    ret_code_t err_code;
    nrf_saadc_value_t adcResult = 0;
    float batteryPercentage = 0;

    if (p_event->type == NRF_DRV_SAADC_EVT_DONE) {
        // 配置好缓存，为下一次转换做准备
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
        adcResult = p_event->data.done.p_buffer[0];
        batteryPercentage = (((adcResult * 0.6) / 81) * 1.6) * 1000;
        is_saadc_sample_done = true;
    }
    NRF_LOG_INFO("adcResult: %d, VBAT: %d", adcResult, batteryPercentage);
}

static void battery_saadc_uninit(void)
{
    nrfx_saadc_uninit();
}

static void battery_saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool, SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

static void battery_saadc_sample(void)
{
    ret_code_t errCode;
    
    errCode = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(errCode);
}


// 处理电量上报业务
static void battery_service_thread(void *arg)
{
    int step = 0, ret = 0;
    struct app_d2h_power_percent *power_percent = NULL;
    struct app_gen_command app_cmd;
    unsigned int timeout = 0;
    
    NRF_LOG_INFO("battery_service_thread start");
    nrf_gpio_cfg_output(GPIO_PIN_ON_OFF);
    nrf_gpio_pin_set(GPIO_PIN_ON_OFF);
    battery_saadc_init();
    
    while (true) {
#if 1
TASK_GEN_ENTRY_STEP(0) {
        if (!is_need_upload_power_percent) {
            vTaskDelay(300);
            continue;
        }
        is_need_upload_power_percent = false;
        memset(&app_cmd, 0, sizeof(struct app_gen_command));
        power_percent = (struct app_d2h_power_percent *)app_cmd.buff;
        power_percent->percent = 50;
        app_cmd.id = CMD_D2H_ID_GET_BAT_PERCENT;
        app_cmd.flags = CMD_D2H_ID;
        app_send_2host((uint8_t *)&app_cmd, sizeof(struct app_gen_command));
      }
#endif
TASK_GEN_ENTRY_STEP(1) {
        NRF_LOG_INFO("battery_saadc_sample start");
        is_saadc_sample_done = false;
        battery_saadc_sample();
        timeout = 1000;
        while (true) {
            if (is_saadc_sample_done || !timeout)
                break;
            vTaskDelay(300);
            timeout--;
        }
        NRF_LOG_INFO("battery_saadc_sample done, timeout: %d", timeout);
    }
  }
}


int app_battery_init(void)
{    
    BaseType_t ret;
    
    NRF_LOG_INFO("app_battery_init");
    ret = xTaskCreate(battery_service_thread, "BATTERY", 64, NULL, 2, &m_battery_service_thread);
    if (ret != pdPASS){
        NRF_LOG_INFO("xTaskCreate fail, ret: %d", ret);
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    
    return 0;
}