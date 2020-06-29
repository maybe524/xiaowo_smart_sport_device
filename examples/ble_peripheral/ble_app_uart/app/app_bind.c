#include "common.h"
#include "app.h"
#include "app_storage.h"

static TaskHandle_t m_bind_thread;
static bool is_need_bind = false;
static uint8_t bind_num_buff[4] = {0};

// 处理绑定解绑的业务逻辑
static void bind_thread(void *arg)
{
    int step = 0, ret = 0;
    
    NRF_LOG_INFO("bind_thread start");
    while (true) {
TASK_GEN_ENTRY_STEP(0) {
        if (!is_need_bind)
            vTaskDelay(300);
        else
            step++;
      }

TASK_GEN_ENTRY_STEP(1) {
        NRF_LOG_INFO("new bind num: 0x%x-0x%x-0x%x-0x%x", 
            bind_num_buff[0], bind_num_buff[1], 
            bind_num_buff[2], bind_num_buff[3]);
        // ret = storage_set(STORAGE_ID_BIND_NUM, bind_num_buff, sizeof(bind_num_buff));
        step = 0;
        is_need_bind = false;
      }
    }
}

int bind_mark_bind_num(uint8_t *num_array)
{
    if (is_need_bind) {
        NRF_LOG_INFO("bind_num_ready is busy");
        return 0;
    }
    memset(bind_num_buff, 0, sizeof(bind_num_buff));
    memcpy(bind_num_buff, num_array, sizeof(bind_num_buff));
    is_need_bind = true;
    NRF_LOG_INFO("bind_num_ready is ready!");
    
    return 0;
}

int bind_mark_unbind(void)
{
    NRF_LOG_INFO("mark_unbind");
    return 0;
}

int app_bind_init(void)
{
    BaseType_t ret;
    uint8_t buff[4] = {0};
    
    NRF_LOG_INFO("app_bind_init");
    storage_get(STORAGE_ID_BIND_NUM, buff, sizeof(buff));
    NRF_LOG_INFO("bind num: 0x%02x-0x%02x-0x%02x-0x%02x", buff[0], buff[1], buff[2], buff[3]);
    ret = xTaskCreate(bind_thread, "BIND", 64, NULL, 2, &m_bind_thread);
    if (ret != pdPASS){
        NRF_LOG_INFO("xTaskCreate fail, ret: %d", ret);
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    
    return 0;
}
