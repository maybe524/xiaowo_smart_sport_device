#include "common.h"
#include "app.h"
#include "app_storage.h"
#include "app_protocol.h"

static TaskHandle_t m_hr_oximeter_thread;
static volatile bool is_need_open_hr = false, is_need_open_oximter = false;
extern bool is_need_stop_hr_spo2;

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
    is_need_open_hr = false;
    return 0;
}

int hr_oximeter_close_oximeter(void)
{
    is_need_open_oximter = false;
    return 0;
}

int hr_oximeter_close_all(void)
{
    hr_oximeter_close_hr();
    hr_oximeter_close_oximeter();
    max30102_set_hr_spo2(false);
    return 0;
}

// 心率血氧计算完成返回的事件
static int hr_oximeter_max30102_event_callback(int event, unsigned long data)
{
    switch (event) {
    case 1: {
        struct event_hr_spo2 {
            unsigned int hr, spo2;
        };
        struct event_hr_spo2 *p_event_val = (struct event_hr_spo2 *)data;
        struct app_gen_command app_cmd;
        struct app_d2h_hr_spo2_data *p_hr_spo2 = NULL;

        memset(&app_cmd, 0, sizeof(struct app_gen_command));
        app_cmd.id = CMD_D2H_ID_GET_TASK;
        app_cmd.flags = 0x02;
        app_cmd.len = 7;
        p_hr_spo2 = (struct app_d2h_hr_spo2_data *)app_cmd.buff;
        p_hr_spo2->opt_task = CHANG_TO_BIGENDING(0x03);
        p_hr_spo2->opt_id = 0x02;
        p_hr_spo2->hr = p_event_val->hr;
        p_hr_spo2->spo2 = p_event_val->spo2;
        app_send_2host((uint8_t *)&app_cmd, sizeof(struct app_gen_command));
        break;
    }
    default:
        break;
    }

    return 0;
}

// 处理心率血氧的业务逻辑，获取原始数据
static void hr_oximeter_service_thread(void *arg)
{
    int step = 0, ret = 0;
    struct app_gen_command app_cmd;
    struct app_d2h_red_ir_data *red_ir = NULL;
    uint32_t aun_red, aun_ir;
    uint32_t timeout_cnt = 0;

    NRF_LOG_INFO("hr_oximeter_service_thread start");
    while (true) {
TASK_GEN_ENTRY_STEP(0) {
        if (!is_need_open_hr && !is_need_open_oximter)
            vTaskDelay(200);
        else {
            NRF_LOG_INFO("enter hr_oximeter");
            timeout_cnt = 10;
            step++;
        }
      }

TASK_GEN_ENTRY_STEP(1) {
        if (!is_need_open_hr && !is_need_open_oximter) {
            step++;
            continue;
        }
        ret = (int)max30102_init();
        if (!ret && !timeout_cnt) {
            step = 0;
            is_need_open_hr = false;
            is_need_open_oximter = false;
            continue;
        }
        else if (!ret && timeout_cnt) {
            timeout_cnt--;
            NRF_LOG_INFO("oximeter sensor init fail, retry...");
            vTaskDelay(3000);
            continue;
        }
        max30102_user_event_callback_init(hr_oximeter_max30102_event_callback);
        max30102_collect_data();
        // memset(&app_cmd, 0, sizeof(struct app_gen_command));
#if 0
        app_cmd.id = CMD_D2H_ID_GET_TASK;
        app_cmd.flags = 0x02;
        app_cmd.len = 7;
        red_ir = (struct app_d2h_red_ir_data *)app_cmd.buff;
        while (true) {
            if (is_need_stop_hr_spo2)
                break;
            ret = maxim_max30102_read_fifo(&aun_red, &aun_ir);
            red_ir->opt_task = CHANG_TO_BIGENDING(0x03);
            red_ir->opt_id = 0x03;
            red_ir->red = CHANG_TO_BIGENDING_32(aun_red);
            red_ir->ir = CHANG_TO_BIGENDING_32(aun_ir);
            app_send_2host((uint8_t *)&app_cmd, sizeof(struct app_gen_command));
        }
#endif
        step++;
      }

TASK_GEN_ENTRY_STEP(2) {
        ret = max30102_exit();
        is_need_stop_hr_spo2 = false;
        is_need_open_hr = false;
        is_need_open_oximter = false;
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

#if 0
    is_need_open_hr = true;
    is_need_open_oximter = true;
#endif

    return 0;
}