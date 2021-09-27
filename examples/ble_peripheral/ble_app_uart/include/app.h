#pragma once

#define CONFIG_NOT_SUPPORT_STORAGE

#define MSG_DEBUG(format, args...) do { \
        extern bool g_is_debug_mode;    \
        if (g_is_debug_mode) {          \
            NRF_LOG_INFO(format, ##args)    \
        }     \
    } while (0)

int app_bind_init(void);
int app_storage_init(void);
int app_vibr_init(void);
int app_accelerator_init(void);
int app_hr_oximeter_init(void);
int app_vibr_init(void);
int app_time_init(void);
int app_set_bind_num(uint8_t *num_array);

bool app_send_2host(uint8_t *data_array, uint16_t length);
bool app_get_bleconn_status(void);


typedef int (common_event_callback_t)(int event, unsigned long data);