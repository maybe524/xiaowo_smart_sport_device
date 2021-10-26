#include "common.h"
#include "app.h"
#include "app_protocol.h"
#include "nrf_drv_saadc.h"
#include "ws2812b_2020_drv.h"

#define SAMPLES_IN_BUFFER   1
#define GPIO_PIN_ON_OFF     9
#define GPIO_PIN_VBUS_ADC   13

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600                 /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6                   /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  1000                    /**< Typical forward voltage drop of the diode . */
#define ADC_RES_10BIT                   1024                /**< Maximum digital value for 10-bit ADC conversion. */

// VP = (RESULT * REFERENCE / 2^10) * 6
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

static TaskHandle_t m_battery_service_thread;
static bool is_need_upload_power_percent = true;
static nrf_saadc_value_t m_buffer_pool[SAMPLES_IN_BUFFER];
static volatile bool is_saadc_sample_done = false;
static float s_battery_charging_curr_vol = -1;
static unsigned int s_current_time = 0;
static unsigned int s_battery_inited = 0;
static unsigned int is_len_show_on = 0;
static unsigned int s_battery_update_time = 0;
static int s_is_battery_charging = 0;
static unsigned int s_is_need_show_battery_percent = 0;
static float s_battery_deviation_value = 10.875;
///< 假设目前的充电是按2个小时充满，那么从2900充到3700计算，每分钟充6.7毫安时，
///< 即每分钟按0.8%
static float s_battery_mah_per_min = 6.7;
static float s_battery_percentage_bak = 0;

struct battery_charging_adc_map {
    unsigned int charging_adc;
    unsigned int final_adc;
};

///< 作废
static const struct battery_charging_adc_map s_battery_charging_adc_map[] = {
    {3994,	3638},
    {3875,	3460},
    {3840,	3389},
    {3816,	3342},
    {3792,	3354},
    {3780,	3306},
    {3757,	3104},
    {3709,	3069},
    {3579,	3045},
    {3531,	3022},
    {3460,	2915},
    {3410,	2800},
    {2749,	2666},
    {2654,	2560},
};

static const unsigned int battery_charging_full_voltage = 4150;
static const unsigned int battery_charging_low_voltage = 2900;

static bool s_is_battery_charging_need_update_adc = false;
static short s_battery_charging_vol_array[10] = {0};
static unsigned int s_battery_charging_vol_avg = 0;
static unsigned int s_battery_charging_vol_cnt = 0;
static bool s_is_battery_charging_vol_array_inited = false;
static bool s_is_battery_charging_full = false;
static unsigned int s_battery_charging_keep_time = 0;

#define ARRAY_LEN(array)    (sizeof(array) / sizeof(array[0]))
#define BATTERY_KEEP_CHARGUNG_TIME_IN_MS  (5 * 60 * 1000) 

extern int sgm_31324_drv_openled(bool is_red_open, bool is_green_open, bool is_blue_open);

int battery_set_all_power(unsigned int value)
{    
    return 0;
}

int battery_check_power_percent(void)
{
    is_need_upload_power_percent = true;
    return 0;
}

bool battery_check_power_percent_busy(void)
{
    return is_need_upload_power_percent;
}

int battery_get_power_percent(unsigned int *p_power_percent)
{
    int ret = 0;
    unsigned int power_percent = 0;
    
    if (s_battery_charging_curr_vol < 0) {
        ret = -1;
        goto L1;
    }
    if (s_battery_charging_curr_vol <= battery_charging_low_voltage) {
        ret = -2;
        goto L1;
    }
    
    power_percent = ((s_battery_charging_curr_vol - battery_charging_low_voltage) / (battery_charging_full_voltage - battery_charging_low_voltage)) * 100;
    power_percent = power_percent > 100 ? 100 : power_percent;
L1:
    *p_power_percent = power_percent;
    
    return ret;
}

int battery_get_power_charging(void)
{
    return s_is_battery_charging;
}

static void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    int i;
    ret_code_t err_code;
    nrf_saadc_value_t adc_result = 0;
    unsigned int power_percent = 0;

    if (p_event->type == NRF_DRV_SAADC_EVT_DONE) {
        // 配置好缓存，为下一次转换做准备
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
        adc_result = p_event->data.done.p_buffer[0];
        ///< 根据电路计算
        ///< s_battery_charging_curr_vol = ((((adc_result + s_battery_deviation_value) * 0.6) / 81) * 1.6) * 1000;
        ///< ((261*(600/1024))*6)/1000/10*34.3=vbt=3.147v
        s_battery_charging_curr_vol = adc_result * 12.05;
        is_saadc_sample_done = true;
    }
    
    if (s_is_battery_charging) {
        if (!s_is_battery_charging_need_update_adc) {
            s_is_battery_charging_need_update_adc = true;
        }
    }
    
    ///< 更新电池电量
    battery_get_power_percent(&power_percent);
    NRF_LOG_INFO("----------------------");
    NRF_LOG_INFO("time stamp: %d", app_get_time_stamp());
    NRF_LOG_INFO("build: %s %s", __DATE__, __TIME__);
    NRF_LOG_INFO("is_charging: %d", s_is_battery_charging);
    NRF_LOG_INFO("adc_result: %d, curr voltage: %d, percent: %d", adc_result, s_battery_charging_curr_vol, power_percent);
    NRF_LOG_INFO("full voltage: %d, low voltage: %d", battery_charging_full_voltage, battery_charging_low_voltage);
    NRF_LOG_INFO("----------------------");
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
    struct app_d2h_power_percent *p_power_percent = NULL;
    struct app_gen_command app_cmd;
    unsigned int timeout = 0;
    unsigned int power_percent = 0, power_percent_new = 0;
    int is_battery_charging_bak = 0;
    int is_need_update_led = 0;
    unsigned int battery_charging_start_time = 0, battery_charging_keep_time = 0;
    float battery_charging_add = 0;

    NRF_LOG_INFO("battery_service_thread start");
    battery_saadc_init();
    nrf_gpio_cfg(
        GPIO_PIN_VBUS_ADC,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE);
    ///< 默认启动时，先来一次采样
    is_need_upload_power_percent = true;

    while (true) {
TASK_GEN_ENTRY_STEP(0) {
        vTaskDelay(500);
        s_is_battery_charging = nrf_gpio_pin_read(GPIO_PIN_VBUS_ADC);
    
        ///< 定时测量
        s_battery_update_time++;
        if ((s_is_battery_charging && s_battery_update_time >= 20) || \
                (!s_is_battery_charging && s_battery_update_time >= 100))
        {
            s_battery_update_time = 0;
            is_need_upload_power_percent = true;
        }

        ///< 当充电和不充电时，都采样一次电池
        if (is_battery_charging_bak != s_is_battery_charging) {
            is_battery_charging_bak = s_is_battery_charging;
            is_need_upload_power_percent = true;
            s_is_battery_charging_need_update_adc = false;
            s_is_battery_charging_vol_array_inited = false;
            s_is_battery_charging_full = false;
            s_battery_charging_vol_cnt = 0;
            s_battery_charging_keep_time = 0;
            led_3gpio_set_led(0, 0, 0);
        }
        
        ///< 判断是否需要采样电量
        if (s_is_battery_charging)
            step = 1;
        else if (is_need_upload_power_percent)
            step = 4;
      }

///< 充电前的初始化，获取电量
TASK_GEN_ENTRY_STEP(1) {
        unsigned char x = 0, len = 0;
        
        if (!s_is_battery_charging_need_update_adc) {
            step = 4;
            continue;
        }

        s_battery_charging_vol_array[s_battery_charging_vol_cnt] = s_battery_charging_curr_vol;
        s_battery_charging_vol_cnt++;
        if (s_battery_charging_vol_cnt > ARRAY_LEN(s_battery_charging_vol_array) - 1) {
            s_battery_charging_vol_cnt = 0;
            s_is_battery_charging_vol_array_inited = true;
        }
        
        len = s_is_battery_charging_vol_array_inited ? \
            ARRAY_LEN(s_battery_charging_vol_array) : s_battery_charging_vol_cnt;
        s_battery_charging_vol_avg = 0;
        
        for (x = 0 ; x < len ; x++)
            s_battery_charging_vol_avg += s_battery_charging_vol_array[x];
        s_battery_charging_vol_avg /= len;
        
        if (s_is_battery_charging && !s_is_battery_charging_full)
            step = 2;
        else if (s_is_battery_charging && s_is_battery_charging_full)
            step = 3;
        else
            step = 4;
      }

///< 处理充电过程
TASK_GEN_ENTRY_STEP(2) {
        if (s_battery_charging_vol_avg > battery_charging_full_voltage) {
            if (!s_battery_charging_keep_time)
                s_battery_charging_keep_time = app_get_time_stamp();
            if (app_get_time_stamp() - s_battery_charging_keep_time >= BATTERY_KEEP_CHARGUNG_TIME_IN_MS) {
                s_is_battery_charging_full = true;
                step = 3;
                continue;
            }
            led_3gpio_set_led(0, 1, 0);
        }
        else if (s_battery_charging_vol_avg <= battery_charging_full_voltage) {
            led_3gpio_set_led(1, 0, 0);
        }
    
        NRF_LOG_INFO("handle low vol, curr vol: %d, avg vol: %d, keep time: %d", \
            s_battery_charging_curr_vol, \
            s_battery_charging_vol_avg, \
            s_battery_charging_keep_time ? app_get_time_stamp() - s_battery_charging_keep_time : 0);
        step = 4;
      }

///< 处理充满以后电量自己会掉下来问题
TASK_GEN_ENTRY_STEP(3) {
        NRF_LOG_INFO("handle full vol, curr vol: %d, avg vol: %d, keep time: %d", \
            s_battery_charging_curr_vol, \
            s_battery_charging_vol_avg, \
            s_battery_charging_keep_time ? app_get_time_stamp() - s_battery_charging_keep_time : 0);
        led_3gpio_set_led(0, 0, 0);
        step = 4;
      }

TASK_GEN_ENTRY_STEP(4) {
        if (!is_need_upload_power_percent) {
            step = 0;
            continue;
        }

        nrf_gpio_cfg_output(GPIO_PIN_ON_OFF);
        nrf_gpio_pin_set(GPIO_PIN_ON_OFF);
        vTaskDelay(300);
        is_need_upload_power_percent = false;
        NRF_LOG_INFO("battery_saadc_sample start");
        is_saadc_sample_done = false;
        battery_saadc_sample();
        timeout = 1000;
        while (true) {
            if (is_saadc_sample_done || !timeout)
                break;
            vTaskDelay(200);
            timeout--;
        }
        NRF_LOG_INFO("battery_saadc_sample done, timeout: %d", timeout);
        nrf_gpio_cfg_default(GPIO_PIN_ON_OFF);
        battery_get_power_percent(&power_percent);
        power_percent_new = power_percent;
        step++;
    }

TASK_GEN_ENTRY_STEP(5) {
        memset(&app_cmd, 0, sizeof(struct app_gen_command));
        p_power_percent = (struct app_d2h_power_percent *)app_cmd.buff;
        p_power_percent->percent = power_percent_new;
        app_cmd.id = CMD_D2H_ID_GET_BAT_PERCENT;
        app_cmd.flags = CMD_D2H_ID;
        app_send_2host((uint8_t *)&app_cmd, sizeof(struct app_gen_command));
        NRF_LOG_INFO("battery send to app: %02d", p_power_percent->percent);
        step = 0;
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