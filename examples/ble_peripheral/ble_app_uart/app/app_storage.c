#include "common.h"
#include "app.h"
#include "app_storage.h"

#define STORAGE_BASE_ADDR   0x3e000

static TaskHandle_t m_storage_thread;
#if defined   (__CC_ARM) /*!< ARM Compiler */ // MDK, 保持数组的起始地址是8的倍数
__align(4)
static struct app_storage_fmt ___storage_cache = {0};
__align(1)
#endif
static bool is_need_storage_update = false;
extern bool is_erased_done;

nrf_fstorage_t *get_fstorage_ins(void);
void power_manage(void);

int storage_set(uint32_t id, void *data, uint32_t len)
{
    struct app_storage_fmt *storage = &___storage_cache;
    
    if (!id)
        return -1;
    switch (id) {
    case STORAGE_ID_BIND_NUM:
        memcpy(storage->bind_num, data, len);
        break;
    default:
        NRF_LOG_INFO("id error, return");
        return -2;
    }
    is_need_storage_update = true;
    return 0;
}

int storage_get(uint32_t id, void *data, uint32_t len)
{
    struct app_storage_fmt *storage = &___storage_cache;
    
    if (!id)
        return -1;
    switch (id) {
    case STORAGE_ID_BIND_NUM:
        memcpy(data, storage->bind_num, len);
        break;
    default:
        break;
    }

    return 0;
}

static int storage_sync(bool is_task_wait)
{
    uint32_t timeout_cnt = 100, page_count = 0;
    ret_code_t ret;
    struct app_storage_fmt *storage = &___storage_cache;
    nrf_fstorage_t *storage_fds = get_fstorage_ins();

    // 平台定义接口
    NRF_LOG_INFO("storage_sync start.");
    // 清除Flash
    page_count = (storage_fds->end_addr - storage_fds->start_addr) / 4096;
    NRF_LOG_INFO("erase page count: %d", page_count);
    is_erased_done = false;
    ret = nrf_fstorage_erase(storage_fds, storage_fds->start_addr, page_count, NULL);
    timeout_cnt = 100;
    NRF_LOG_INFO("waiting done.");
    while (true) {
        if (is_task_wait)
            vTaskDelay(300);
        else
            power_manage();
        if (is_erased_done)
            break;
        timeout_cnt--;
        if (!timeout_cnt)
            break;
    }
    NRF_LOG_INFO("storage_sync clear done, timeout_cnt: %d", timeout_cnt);
    // 写到Flash上去
    NRF_LOG_INFO("storage_sync write, size: %d, storage: 0x%08x", sizeof(struct app_storage_fmt), storage);
    ret = nrf_fstorage_write(storage_fds, storage_fds->start_addr, storage, sizeof(struct app_storage_fmt), NULL);
    APP_ERROR_CHECK(ret);
    timeout_cnt = 100;
    while (true) {
        ret = nrf_fstorage_is_busy(storage_fds);
        if (!ret)
            break;
        else if (is_task_wait)
            vTaskDelay(300);
        else
            power_manage();
        timeout_cnt--;
        if (!timeout_cnt)
            break;
    }
    NRF_LOG_INFO("storage_sync write done, timeout: %d", timeout_cnt);
    
    return timeout_cnt ? 0 : -1;
}

static void storage_thread(void *arg)
{
    int step = 0, ret = 0;
    
    NRF_LOG_INFO("storage_thread start");
    while (true) {
TASK_GEN_ENTRY_STEP(0) {
        if (!is_need_storage_update)
            vTaskDelay(300);
        else
            step++;
      }

TASK_GEN_ENTRY_STEP(1) {
        ret = storage_sync(true);
        step = 0;
        is_need_storage_update = false;
      }
    }
}

int app_storage_init(void)
{
    BaseType_t ret;
    ret_code_t rc;
    struct app_storage_fmt *storage = &___storage_cache;
    nrf_fstorage_t *storage_fds = get_fstorage_ins();
    
    NRF_LOG_INFO("app_storage_init");
    rc = nrf_fstorage_read(storage_fds, storage_fds->start_addr, storage, sizeof(struct app_storage_fmt));
    NRF_LOG_INFO("nrf_fstorage_write, rc: %d", rc);
    
    ret = xTaskCreate(storage_thread, "BIND", 32, NULL, 2, &m_storage_thread);
    if (ret != pdPASS){
        NRF_LOG_INFO("xTaskCreate fail, ret: %d", ret);
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    return 0;
}