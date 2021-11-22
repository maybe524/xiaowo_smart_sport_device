#include "common.h"
#include "app.h"
#include "app_storage.h"
#include "app_protocol.h"

static TaskHandle_t m_hr_oximeter_thread;
volatile bool s_is_need_open_hr = false, s_is_need_open_oximter = false;
extern bool g_is_need_stop_hr_spo2;
extern bool g_is_app_init_done;
bool g_is_hr_oximeter_busy = false;
bool g_is_hr_oximeter_need_red_ir_raw_data = false;

int hr_oximeter_open_hr(void)
{
    s_is_need_open_hr = true;
    return 0;
}

int hr_oximeter_open_oximeter(void)
{
    s_is_need_open_oximter = true;
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
    s_is_need_open_hr = false;
    return 0;
}

int hr_oximeter_close_oximeter(void)
{
    s_is_need_open_oximter = false;
    return 0;
}

int hr_oximeter_close_all(void)
{
    hr_oximeter_close_hr();
    hr_oximeter_close_oximeter();
    max30102_set_hr_spo2(false);
    return 0;
}

int hr_oximeter_set_red_ir_raw_data(int en)
{    
    g_is_hr_oximeter_need_red_ir_raw_data = en ? true : false;
    
    return 0;
}

int hr_oximeter_check_condiction(void)
{
    if (!app_get_bleconn_status() || \
        (!s_is_need_open_hr && !g_is_hr_oximeter_need_red_ir_raw_data))
    {
        return 0;
    }

    return 1;
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
        p_hr_spo2->opt_id = 0x02;       ///< 表示测量结果
        p_hr_spo2->hr = p_event_val->hr;
        p_hr_spo2->spo2 = p_event_val->spo2;
        app_send_2host((uint8_t *)&app_cmd, sizeof(struct app_gen_command));
        break;
    }
    
    case 2: {
        struct event_red_ir {
            unsigned int red, ir;
        };
        struct event_red_ir *p_event_val = (struct event_red_ir *)data;
        struct app_gen_command app_cmd;
        struct app_d2h_red_ir_data *p_red_ir = NULL;
        
        memset(&app_cmd, 0, sizeof(struct app_gen_command));
        app_cmd.id = CMD_D2H_ID_GET_TASK;
        app_cmd.flags = 0x02;
        app_cmd.len = 8;
        p_red_ir = (struct app_d2h_red_ir_data *)app_cmd.buff;
        p_red_ir->opt_task = CHANG_TO_BIGENDING(0x03);
        p_red_ir->opt_id = 0x03;        ///< 表示采样数据
        p_red_ir->red = p_event_val->red;
        p_red_ir->ir = p_event_val->ir;
        app_send_2host((uint8_t *)&app_cmd, sizeof(struct app_gen_command));
        break;
    }
    
    default:
        break;
    }

    return 0;
}

static void hr_oximeter_test(void)
{
#if 0
    {
        unsigned int id = 0;
        struct test_io_info {
            int id;
            unsigned int val;
        };
        struct test_io_info test_io_info_array[] = {
            {0, NRF_GPIO_PIN_S0S1},
            {1, NRF_GPIO_PIN_H0S1},
            {2, NRF_GPIO_PIN_S0H1},
            {3, NRF_GPIO_PIN_H0H1},
            {4, NRF_GPIO_PIN_D0S1},
            {5, NRF_GPIO_PIN_D0H1},
            {6, NRF_GPIO_PIN_S0D1},
            {7, NRF_GPIO_PIN_H0D1}
        };

        while (1) {
#if 1
            NRF_LOG_INFO("test input");
            nrf_gpio_cfg(
                19,
                NRF_GPIO_PIN_DIR_INPUT,
                NRF_GPIO_PIN_INPUT_CONNECT,
                NRF_GPIO_PIN_PULLDOWN,
                test_io_info_array[id].val,
                NRF_GPIO_PIN_NOSENSE
            );
            vTaskDelay(2000);

            NRF_LOG_INFO("test output");
            nrf_gpio_cfg(
                19,
                NRF_GPIO_PIN_DIR_OUTPUT,
                NRF_GPIO_PIN_INPUT_DISCONNECT,
                NRF_GPIO_PIN_NOPULL,
                test_io_info_array[id].val,
                NRF_GPIO_PIN_NOSENSE
            );
            nrf_gpio_pin_clear(19);
#endif
            NRF_LOG_INFO("test_io_info_array: %d", id);
            vTaskDelay(2000);
#if 0
            id++;
            if (id >= (sizeof(test_io_info_array) / sizeof(test_io_info_array[0]) - 1))
                id = 0;
#endif
        }
    }
#endif

#if 0
    while (1) {
        led_3gpio_set_led(0, 0, 1);
        vTaskDelay(1000);
        led_3gpio_set_led(0, 1, 0);
        vTaskDelay(1000);
        led_3gpio_set_led(0, 1, 1);
        vTaskDelay(1000);
        led_3gpio_set_led(1, 0, 0);
        vTaskDelay(1000);
        led_3gpio_set_led(1, 0, 1);
        vTaskDelay(1000);
        led_3gpio_set_led(1, 1, 1);
        vTaskDelay(1000);
    }
#endif
}

// 处理心率血氧的业务逻辑，获取原始数据
static void hr_oximeter_service_thread(void *arg)
{
    int step = 0, ret = 0;
    struct app_gen_command app_cmd;
    struct app_d2h_red_ir_data *red_ir = NULL;
    uint32_t aun_red, aun_ir;
    uint32_t timeout_cnt = 0;
    uint32_t gpio_test = 0, gpio_test_num = 0;

    NRF_LOG_INFO("hr_oximeter_service_thread start");
    hr_oximeter_test();

    while (true) {
TASK_GEN_ENTRY_STEP(0) {
        // 解决在未初始化时，芯片的灯会常亮问题
        if (!g_is_app_init_done) {
            vTaskDelay(200);
            continue;
        }
        max30102_api_init();
        max30102_api_exit();
        step++;
      }

TASK_GEN_ENTRY_STEP(1) {
        if (!s_is_need_open_hr && !s_is_need_open_oximter && \
                !g_is_hr_oximeter_need_red_ir_raw_data)
        {
            vTaskDelay(200);
        }
        else {
            NRF_LOG_INFO("enter hr_oximeter");
            timeout_cnt = 10;
            step++;
        }
      }

TASK_GEN_ENTRY_STEP(2) {
        if (!s_is_need_open_hr && !s_is_need_open_oximter && \
            !g_is_hr_oximeter_need_red_ir_raw_data)
        {
            step++;
            continue;
        }
        ret = (int)max30102_api_init();
        if (!ret && !timeout_cnt) {
            step = 1;
            s_is_need_open_hr = false;
            s_is_need_open_oximter = false;
            continue;
        }
        else if (!ret && timeout_cnt) {
            timeout_cnt--;
            NRF_LOG_INFO("oximeter sensor init fail, retry...");
            vTaskDelay(3000);
            continue;
        }
        g_is_hr_oximeter_busy = true;
        max30102_user_event_callback_init(hr_oximeter_max30102_event_callback);
        max30102_api_collect_data();
        // memset(&app_cmd, 0, sizeof(struct app_gen_command));
#if 0
        app_cmd.id = CMD_D2H_ID_GET_TASK;
        app_cmd.flags = 0x02;
        app_cmd.len = 7;
        red_ir = (struct app_d2h_red_ir_data *)app_cmd.buff;
        while (true) {
            if (g_is_need_stop_hr_spo2)
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

TASK_GEN_ENTRY_STEP(3) {
        ret = max30102_api_exit();
        g_is_need_stop_hr_spo2 = false;
        s_is_need_open_hr = false;
        s_is_need_open_oximter = false;
        g_is_hr_oximeter_busy = false;
        g_is_hr_oximeter_need_red_ir_raw_data = false;
        step = 1;
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
    s_is_need_open_hr = true;
    s_is_need_open_oximter = true;
#endif

    return 0;
}