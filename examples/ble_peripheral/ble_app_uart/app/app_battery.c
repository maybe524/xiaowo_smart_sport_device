#include "common.h"
#include "app.h"
#include "app_protocol.h"

static TaskHandle_t m_battery_service_thread;
static bool is_need_upload_power_percent = false;

int battery_get_power_percent(void)
{
    is_need_upload_power_percent = true;
    return 0;
}

#if 0
static void battery_saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

}
#endif

// 处理电量上报业务
static void battery_service_thread(void *arg)
{
    int step = 0, ret = 0;
    struct app_d2h_power_percent *power_percent = NULL;
    struct app_gen_command app_cmd;
    
    NRF_LOG_INFO("battery_service_thread start");
    while (true) {
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