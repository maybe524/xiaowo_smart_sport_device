#include "common.h"
#include "app.h"
#include "app_storage.h"

static TaskHandle_t m_hr_oximeter_thread;
static bool is_need_open_hr = false, is_need_open_oximter = false;

int hr_oximeter_open_hr(void)
{
    is_need_open_hr = true;
    return 0;
}

int hr_oximeter_open_oximeter(void)
{
    is_need_open_oximter = true;
    return 0;
}

int hr_oximeter_open_all(void)
{
    hr_oximeter_open_hr();
    hr_oximeter_open_oximeter();
    max30102_set_hr_spo2(true);
    return 0;
}


int hr_oximeter_close_hr(void)
{
    is_need_open_hr = true;
    return 0;
}

int hr_oximeter_close_oximeter(void)
{
    is_need_open_oximter = true;
    return 0;
}

int hr_oximeter_close_all(void)
{
    hr_oximeter_close_hr();
    hr_oximeter_close_oximeter();
    max30102_set_hr_spo2(false);
    return 0;
}

// 处理心率血氧的业务逻辑
static void hr_oximeter_service_thread(void *arg)
{
    int step = 0, ret = 0;
    
    NRF_LOG_INFO("hr_oximeter_service_thread start");
    while (true) {
TASK_GEN_ENTRY_STEP(0) {
        if (!is_need_open_hr && !is_need_open_oximter)
            vTaskDelay(200);
        else {
            NRF_LOG_INFO("enter hr_oximeter");
            step++;
        }
      }

TASK_GEN_ENTRY_STEP(1) {
        if (!is_need_open_hr && !is_need_open_oximter) {
            step++;
            continue;
        }
        ret = (int)max30102_init();
        if (!ret) {
            NRF_LOG_INFO("oximeter sensor init fail, retry...");
            vTaskDelay(3000);
            continue;
        }
        vTaskDelay(300);
      }

TASK_GEN_ENTRY_STEP(2) {
        step = 0;
      }
    }
}

int app_hr_oximeter_init(void)
{
    BaseType_t ret;

    NRF_LOG_INFO("app_hr_oximeter_init");
    ret = xTaskCreate(hr_oximeter_service_thread, "HROXI", 256, NULL, 2, &m_hr_oximeter_thread);
    if (ret != pdPASS){
        NRF_LOG_INFO("xTaskCreate fail, ret: %d", ret);
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    return 0;
}