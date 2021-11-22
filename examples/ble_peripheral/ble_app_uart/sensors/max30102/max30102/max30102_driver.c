/** \file max30102.cpp ******************************************************
*
* Project: MAXREFDES117#
* Filename: max30102.cpp
* Description: This module is an embedded controller driver for the MAX30102
*
*
* --------------------------------------------------------------------
*
* This code follows the following naming conventions:
*
* char              ch_pmod_value
* char (array)      s_pmod_s_string[16]
* float             f_pmod_value
* int32_t           n_pmod_value
* int32_t (array)   an_pmod_value[16]
* int16_t           w_pmod_value
* int16_t (array)   aw_pmod_value[16]
* uint16_t          uw_pmod_value
* uint16_t (array)  auw_pmod_value[16]
* uint8_t           uch_pmod_value
* uint8_t (array)   auch_pmod_buffer[16]
* uint32_t          un_pmod_value
* int32_t *         pn_pmod_value
*
* ------------------------------------------------------------------------- */
/*******************************************************************************
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/
// #include "mbed.h"
#include "common.h"
#include "max30102_driver.h"
#include "nrf_drv_twi.h"

//I2C i2c(I2C_SDA, I2C_SCL);//SDA-PB9,SCL-PB8
/* Mode for LM75B. */
#define NORMAL_MODE 0U
/* TWI instance ID. */
#define TWI_INSTANCE_ID     1

//MAX30102 I2C器件地址，读写地址是0xAF、0xAE，右移一位得0x57
#define MAX30102_ADDRESS 0x57

/* Indicates if operation on TWI has ended. */
volatile bool m_xfer_done = false;

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample;
static bool is_max30102_twi_inited = false;
bool is_max30102_i2c_busy = false;

/* TWI instance. */
const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint8_t temp)
{
    NRF_LOG_INFO("Temperature: %d Celsius degrees.", temp);
}

/**
 * @brief TWI events handler.
 */
void max30102_twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{

    switch (p_event->type) {
    case NRF_DRV_TWI_EVT_DONE:
#if 0
        if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            data_handler(m_sample);
#endif
        m_xfer_done = true;
        break;
    case NRF_DRV_TWI_EVT_ADDRESS_NACK:
        NRF_LOG_INFO("no ack");
        //m_xfer_done = true;
        break;
    default:
        break;
    }
}

/**
 * @brief UART initialization.
 */
void max30102_twi_init(void)
{
    ret_code_t err_code;
    const nrf_drv_twi_config_t twi_max30102_config = {
#if 1
       .scl                = 20,    //16,
       .sda                = 21,    //15,
#else
       .scl                = 16,
       .sda                = 15,
#endif
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = true,
    };
    
    NRF_LOG_INFO("is_max30102_twi_inited: %d", is_max30102_twi_inited);
    if (is_max30102_twi_inited)
        return;
    err_code = nrf_drv_twi_init(&m_twi, &twi_max30102_config, max30102_twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
    is_max30102_twi_inited = true;
    NRF_LOG_INFO("max30102_twi_init done");
}

bool max30102_drv_write_reg(uint8_t uch_addr, uint8_t uch_data)
/**
* \brief        Write a value to a MAX30102 register
* \par          Details
*               This function writes a value to a MAX30102 register
*
* \param[in]    uch_addr    - register address
* \param[in]    uch_data    - register data
*
* \retval       true on success
*/
{
    bool ret = true;
    ret_code_t err_code;
    /* Writing to LM75B_REG_CONF "0" set temperature sensor in NORMAL mode. */
    uint32_t timeout_cnt = 100000;
    uint8_t buff[2] = {uch_addr, uch_data};

#if 0
    if (is_max30102_i2c_busy && timeout_cnt) {
        vTaskDelay(1);
        timeout_cnt--;
    }
    if (!timeout_cnt) {
        return false;
    }
    is_max30102_i2c_busy = true;
#endif
    
    m_xfer_done = false;
    timeout_cnt = 10000;
    err_code = nrf_drv_twi_tx(&m_twi, MAX30102_ADDRESS, buff, sizeof(buff), false);
    APP_ERROR_CHECK(err_code);
    while (!m_xfer_done && timeout_cnt--)
        vTaskDelay(1);
    if (!m_xfer_done || !timeout_cnt) {
        ret = false;
        goto L1;
    }
    
L1:
    is_max30102_i2c_busy = false;
    
    return ret;
}

bool max30102_drv_read_reg(uint8_t uch_addr, uint8_t *puch_data, uint8_t len)
/**
* \brief        Read a MAX30102 register
* \par          Details
*               This function reads a MAX30102 register
*
* \param[in]    uch_addr    - register address
* \param[out]   puch_data    - pointer that stores the register data
*
* \retval       true on success
*/
{
    bool ret = true;
    ret_code_t err_code;
    uint32_t timeout_cnt = 100000;

#if 0
    if (is_max30102_i2c_busy && timeout_cnt) {
        vTaskDelay(1);
        timeout_cnt--;
    }
    if (!timeout_cnt) {
        return false;
    }
    is_max30102_i2c_busy = true;
#endif

    m_xfer_done = false;
    timeout_cnt = 100000;
    err_code = nrf_drv_twi_tx(&m_twi, MAX30102_ADDRESS, &uch_addr, 1, true);
    APP_ERROR_CHECK(err_code);
    while (!m_xfer_done && timeout_cnt--)
        vTaskDelay(1);
    if (!m_xfer_done || !timeout_cnt) {
        NRF_LOG_INFO("%s %d", __func__, __LINE__);
        ret = false;
        goto L1;
    }
    
    m_xfer_done = false;
    /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
    err_code = nrf_drv_twi_rx(&m_twi, MAX30102_ADDRESS, puch_data, len);
    APP_ERROR_CHECK(err_code);
    timeout_cnt = 100000;
    while (!m_xfer_done && timeout_cnt--)
        vTaskDelay(1);
    if (!m_xfer_done || !timeout_cnt) {
        NRF_LOG_INFO("%s %d", __func__, __LINE__);
        ret = false;
        goto L1;
    }

L1:
    is_max30102_i2c_busy = false;
    
    return ret;
}

bool max30102_drv_init()
/**
* \brief        Initialize the MAX30102
* \par          Details
*               This function initializes the MAX30102
*
* \param        None
*
* \retval       true on success
*/
{
    int i;
    unsigned char val = 0;

#if 1
    //使能A_FULL_EN 和 PPG_RDY_EN 中断
    ///< 0x02
    max30102_drv_write_reg(REG_INTR_ENABLE_1, 0xc0); // INTR setting
    ///< 0x03
    max30102_drv_write_reg(REG_INTR_ENABLE_2, 0x00);
    ///< 0x04
    max30102_drv_write_reg(REG_FIFO_WR_PTR, 0x00);  // FIFO_WR_PTR[4:0]，FIFO指针清零
    ///< 0x05
    max30102_drv_write_reg(REG_OVF_COUNTER, 0x00);  // OVF_COUNTER[4:0]，FIFO指针清零
    ///< 0x06
    max30102_drv_write_reg(REG_FIFO_RD_PTR, 0x00);  // FIFO_RD_PTR[4:0]，FIFO指针清零
    ///< 0x08
    max30102_drv_write_reg(REG_FIFO_CONFIG, 0x0f);  // sample avg = 1, fifo rollover=false, fifo almost full = 17
    ///< SPO2模式其实会产生两个通道的数据Red（心率）和IR（血氧）
    ///< 0x09
    max30102_drv_write_reg(REG_MODE_CONFIG, 0x03);  //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
    
    ///< 0x0a
    ///< 18Bit的分辨率；100Hz；
    ///< [6:5]量程是8192
    max30102_drv_write_reg(REG_SPO2_CONFIG, 0x47);  // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)
    ///< 0x0c
    max30102_drv_write_reg(REG_LED1_PA, 0x40);   // Choose value for ~ 7mA for LED1
    ///< 0x0d
    max30102_drv_write_reg(REG_LED2_PA, 0x40);   // Choose value for ~ 7mA for LED2
    ///< 0x10
    max30102_drv_write_reg(REG_PILOT_PA, 0x7f);  // Choose value for ~ 25mA for Pilot LED
    ///< 0x30寄存器
    ///< PROX模式只是在检测是否有手指放在上边
    // max30102_drv_write_reg(REG_PROX_INT_THRESH, 0x7f);   // Choose value for ~ 25mA for Pilot LED
#else
    setupDriver();
#endif

    for (i = 0; i < 0x31; i++) {
        val = 0;
        max30102_drv_read_reg(i, &val, 1);
        NRF_LOG_INFO("max30102 regconf 0x%02x => 0x%02x", i, val);
    }
    
    return true;  
}

bool max30102_drv_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
/**
* \brief        Read a set of samples from the MAX30102 FIFO register
* \par          Details
*               This function reads a set of samples from the MAX30102 FIFO register
*
* \param[out]   *pun_red_led   - pointer that stores the red LED reading data
* \param[out]   *pun_ir_led    - pointer that stores the IR LED reading data
*
* \retval       true on success
*/
{
    uint32_t un_temp;
    unsigned char uch_temp;
    char ach_i2c_data[6];
    bool ret;
  
    *pun_red_led=0;
    *pun_ir_led=0;
    
    //read and clear status register
    //max30102_drv_read_reg(REG_INTR_STATUS_1, &uch_temp, 1);
    //max30102_drv_read_reg(REG_INTR_STATUS_2, &uch_temp, 1);
  
    ach_i2c_data[0]=REG_FIFO_DATA;
#ifdef HXW
    if(i2c.write(I2C_WRITE_ADDR, ach_i2c_data, 1, true)!=0)
        return false;
    if(i2c.read(I2C_READ_ADDR, ach_i2c_data, 6, false)!=0)
    {
        return false;
    }
#endif
    ret = max30102_drv_read_reg(REG_FIFO_DATA, (uint8_t *)ach_i2c_data, 6);
    if (!ret)
        return ret;
    ///< Red值
    un_temp = (unsigned char) ach_i2c_data[0];
    un_temp <<= 16;
    *pun_red_led += un_temp;
    un_temp = (unsigned char) ach_i2c_data[1];
    un_temp <<= 8;
    *pun_red_led += un_temp;
    un_temp = (unsigned char) ach_i2c_data[2];
    *pun_red_led += un_temp;
  
    ///< IR值
    un_temp = (unsigned char) ach_i2c_data[3];
    un_temp <<= 16;
    *pun_ir_led += un_temp;
    un_temp = (unsigned char) ach_i2c_data[4];
    un_temp <<= 8;
    *pun_ir_led += un_temp;
    un_temp = (unsigned char) ach_i2c_data[5];
    *pun_ir_led += un_temp;
    *pun_red_led &= 0x03FFFF;  //Mask MSB [23:18]
    *pun_ir_led  &= 0x03FFFF;  //Mask MSB [23:18]
  
    return true;
}

bool max30102_drv_reset()
/**
* \brief        Reset the MAX30102
* \par          Details
*               This function resets the MAX30102
*
* \param        None
*
* \retval       true on success
*/
{
    if(!max30102_drv_write_reg(REG_MODE_CONFIG,0x40))
        return false;
    else
        return true;    
}