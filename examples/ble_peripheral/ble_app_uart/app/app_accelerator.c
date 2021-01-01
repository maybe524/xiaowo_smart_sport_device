#include "common.h"
#include "app.h"
#include "app_protocol.h"
#include "app_storage.h"
#include "nrf_drv_spi.h"
#include "lis3dh_driver.h"

// #define CONFIG_TEST_ACCELERATOR

static TaskHandle_t m_accelerator_service_thread;
static bool is_need_sync_2host = false;
static uint32_t accelerator_data_id = 0;

int accelerator_set_send2host(void)
{
    NRF_LOG_INFO("accelerator_set_send2host");
    is_need_sync_2host = true;
    return 0;
}

int accelerator_set_close_send2host(void)
{
    NRF_LOG_INFO("accelerator_set_close_send2host");
    is_need_sync_2host = false;
    return 0;
}

// 处理加速度业务逻辑
static void accelerator_service_thread(void *arg)
{
    int step = 0, ret = 0;
	AxesRaw_t data; //定义一个结构体变量，用于保存从LIS3DH中读取的加速度数据
    status_t read_ret;
    struct app_d2h_accelerator_data *acc_p;
    struct app_gen_command app_cmd;

#ifdef CONFIG_TEST_ACCELERATOR
    is_need_sync_2host = true;
#endif
    
    while (true) {
TASK_GEN_ENTRY_STEP(0) {
        if (!is_need_sync_2host)
            vTaskDelay(100);
        else {
            NRF_LOG_INFO("detect one to collect accelerator");
            ret = (int)LIS3DH_Init();
            if (ret) {
                vTaskDelay(1000);
                continue;
            }
            step++;
        }
      }

// 收集accelerator数据并且上报
TASK_GEN_ENTRY_STEP(1) {
        if (!is_need_sync_2host || !app_get_bleconn_status()) {
            step++;
            continue;
        }
        //读取LIS3DH加速度数据，读取成功后，通过串口打印出数据
        read_ret = LIS3DH_GetAccAxesRaw(&data);
        if (read_ret) {
            memset(&app_cmd, 0, sizeof(struct app_gen_command));
            app_cmd.id = CMD_D2H_ID_GET_TASK;
            app_cmd.flags = 0x02;
            app_cmd.len = 8;
            acc_p = (struct app_d2h_accelerator_data *)app_cmd.buff;
            acc_p->opt_task = CHANG_TO_BIGENDING(0x04);
            acc_p->opt_id = 0x02;
            acc_p->x = CHANG_TO_BIGENDING(data.AXIS_X);
            acc_p->y = CHANG_TO_BIGENDING(data.AXIS_Y);
            acc_p->z = CHANG_TO_BIGENDING(data.AXIS_Z);
            app_send_2host((uint8_t *)&app_cmd, sizeof(struct app_gen_command));
            NRF_LOG_INFO("id: %06d, x: %6d, y: %6d, z: %6d", accelerator_data_id, data.AXIS_X, data.AXIS_Y, data.AXIS_Z);	
        }
        vTaskDelay(200);
      }

// 处理结束后的操作
TASK_GEN_ENTRY_STEP(2) {
        NRF_LOG_INFO("detect stop send accelerator data");
        is_need_sync_2host = false;
        step = 0;
      }
    }
}

int app_accelerator_init(void)
{
    BaseType_t ret;
    
    NRF_LOG_INFO("int app_accelerator_init(void)");
    ret = xTaskCreate(accelerator_service_thread, "ACCEL", 128, NULL, 2, &m_accelerator_service_thread);
    if (ret != pdPASS){
        NRF_LOG_INFO("xTaskCreate fail, ret: %d", ret);
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
	
    return 0;
}