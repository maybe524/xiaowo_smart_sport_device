#include "common.h"
#include "app.h"
#include "app_storage.h"

static TaskHandle_t m_hr_oximeter_thread;

// 处理心率血氧的业务逻辑
static void hr_oximeter_service_thread(void *arg)
{
    int step = 0, ret = 0;
    
    NRF_LOG_INFO("hr_oximeter_service_thread start");
    max30102_init();
    while (true) {
TASK_GEN_ENTRY_STEP(0) {
        vTaskDelay(300);
      }
  }
}

int app_hr_oximeter_init(void)
{
    BaseType_t ret;
#if 1
    NRF_LOG_INFO("app_hr_oximeter_init");
    ret = xTaskCreate(hr_oximeter_service_thread, "HROXI", 64, NULL, 2, &m_hr_oximeter_thread);
    if (ret != pdPASS){
        NRF_LOG_INFO("xTaskCreate fail, ret: %d", ret);
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#else
    max30102_init();
#endif
    return 0;
}