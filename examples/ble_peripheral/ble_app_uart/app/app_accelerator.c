#include "common.h"
#include "app.h"
#include "app_storage.h"
#include "nrf_drv_spi.h"
#include "lis3dh_drive.h"

static TaskHandle_t m_accelerator_service_thread;

#if 0
#define MY_SPI_SS_PIN		25
#define MY_SPI_MISO_PIN	24
#define MY_SPI_MOSI_PIN	23
#define MY_SPI_SCK_PIN	22

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic"
static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

/**
 * @brief SPI user event handler.
 * @param event
 */
static void spi_event_handler(nrf_drv_spi_evt_t const *p_event, void *p_context)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.");
    if (m_rx_buf[0] != 0) {
        NRF_LOG_INFO(" Received:");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}

static int accelerator_spi_init(void)
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = MY_SPI_SS_PIN;
    spi_config.miso_pin = MY_SPI_MISO_PIN;
    spi_config.mosi_pin = MY_SPI_MOSI_PIN;
    spi_config.sck_pin  = MY_SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

    NRF_LOG_INFO("SPI example started.");
    while (true) {
        // Reset rx buffer and transfer done flag
        memset(m_rx_buf, 0, m_length);
        spi_xfer_done = false;
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
        while (!spi_xfer_done) {
            vTaskDelay(200);
        }
        vTaskDelay(200);
    }
}
#endif

// 处理加速度业务逻辑
static void accelerator_service_thread(void *arg)
{
    int step = 0, ret = 0;
	AxesRaw_t data; //定义一个结构体变量，用于保存从LIS3DH中读取的加速度数据
    
    LIS3DH_Init();
    while (true) {
TASK_GEN_ENTRY_STEP(0) {
        //读取LIS3DH加速度数据，读取成功后，通过串口打印出数据
        if (LIS3DH_GetAccAxesRaw(&data)) {
            NRF_LOG_INFO("X=%6d Y=%6d Z=%6d \r\n", data.AXIS_X, data.AXIS_Y, data.AXIS_Z);	
        }
        vTaskDelay(200);
      }
    }
}

int app_accelerator_init(void)
{
    BaseType_t ret;
    
    NRF_LOG_INFO("int app_accelerator_init(void)");
    ret = xTaskCreate(accelerator_service_thread, "ACCEL", 64, NULL, 2, &m_accelerator_service_thread);
    if (ret != pdPASS){
        NRF_LOG_INFO("xTaskCreate fail, ret: %d", ret);
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
	
    return 0;
}