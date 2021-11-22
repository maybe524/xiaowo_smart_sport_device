#pragma once

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_nvmc.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define TASK_GEN_ENTRY_STEP(s)      if ((step) == (s))
#define BIG_ENDING_16(v)            ((v) >> 8 | ((v) & 0xFF) << 8)
#define CHECK_MARKS(v, m)           ((v) & (m))

extern unsigned int xPortGetSysTick(void);
extern unsigned int app_get_time_stamp(void);

//#define kprintf(format, ...)        NRF_LOG_INFO("%08d: "format"", app_get_time_stamp(), ##__VA_ARGS__)
#define kprintf        NRF_LOG_INFO

#define ARRAY_SIZE(array)           (sizeof(array) / sizeof(array[0]))
