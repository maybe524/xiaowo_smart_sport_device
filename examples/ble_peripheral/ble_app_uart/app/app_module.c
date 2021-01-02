#include "common.h"
#include "app.h"

#include "app_button.h"

// 心率血氧芯片的中断
#define INTERRUPT_MAX30102      19
#define INTERRUPT_LIS3DH_INT1   26
#define INTERRUPT_LIS3DH_INT2   27

#define BUTTON_DETECTION_DELAY  APP_TIMER_TICKS(50)

static void interrupt_event_handler(uint8_t pin_no, uint8_t button_action)
{
    NRF_LOG_INFO("pin_no: %d, button_action: %d", pin_no, button_action);
}

static void app_interrupt_init(void)
{
    uint32_t err_code;
    static app_button_cfg_t interrupt_list[] = {
        {INTERRUPT_MAX30102, true, BUTTON_PULL, interrupt_event_handler},
        {INTERRUPT_LIS3DH_INT1, false, BUTTON_PULL, interrupt_event_handler},
        {INTERRUPT_LIS3DH_INT2, false, BUTTON_PULL, interrupt_event_handler},
    };    
    NRF_LOG_INFO("%s %d", __func__, __LINE__);
    err_code = app_button_init(interrupt_list, ARRAY_SIZE(interrupt_list), BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}

int app_module_init(void)
{
    NRF_LOG_INFO("app_module_init, start");
    app_storage_init();
    app_bind_init();
    app_vibr_init();
    app_accelerator_init();
    app_battery_init();
    app_hr_oximeter_init();
    // app_interrupt_init();
    NRF_LOG_INFO("app_module_init, done");
    
    return 0;
}
