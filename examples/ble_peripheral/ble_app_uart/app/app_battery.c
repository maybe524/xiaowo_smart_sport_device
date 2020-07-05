#include "common.h"
#include "app.h"


static TaskHandle_t m_battery_service_thread;

// 处理电量上报业务
static void battery_service_thread(void *arg)
{
    int step = 0, ret = 0;
    
    NRF_LOG_INFO("battery_service_thread start");
    while (true) {
TASK_GEN_ENTRY_STEP(0) {
        vTaskDelay(300);
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