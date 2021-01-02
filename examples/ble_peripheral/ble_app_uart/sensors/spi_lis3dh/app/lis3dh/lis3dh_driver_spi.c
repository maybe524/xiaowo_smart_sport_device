#include "app_error.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "lis3dh_drive.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define CONFIG_LIS3DH_USE_I2C

#if 0
//引脚定义
#define SPI_SS_PIN   25  //SPI片选 nRF52832只能使用GPIO作为片选
#define SPI_SCK_PIN  22  //SPI时钟
#define SPI_MISO_PIN 24  //SPI主入从出
#define SPI_MOSI_PIN 23  //SPI主出从入
#else
/*
*  2020年12月28日最新原理图修改
*/
#define SPI_SS_PIN   25  //SPI片选 nRF52832只能使用GPIO作为片选
#define SPI_SCK_PIN  22  //SPI时钟
#define SPI_MISO_PIN 24  //SPI主入从出
#define SPI_MOSI_PIN 23  //SPI主出从入
#endif


#define SPI_BUFSIZE 8
#define SPI_INSTANCE  0 /**< SPI instance index. */

#define FIFO_STREAM_MODE
#define LIS3DH_ODR_FREQ LIS3DH_ODR_100Hz

uint8_t   SPI_Tx_Buf[SPI_BUFSIZE];
uint8_t   SPI_Rx_Buf[SPI_BUFSIZE];
volatile  uint8_t   SPIReadLength, SPIWriteLength;
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static bool is_lis3dh_spi_inited = false;

static volatile bool m_xfer_done = false;

//SPI事件处理函数，该函数中置位传输完成标志spi_xfer_done
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
}

/**
 * @brief TWI events handler.
 */
void LIS3DH_Handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type) {
    case NRFDRV_TWI_EVT_DONE:
#if 0
        if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            data_handler(m_sample);
#endif
        m_xfer_done = true;
        break;
    case NRF_DRV_TWI_EVT_ADDRESS_NACK:
        NRF_LOG_INFO("no ack");
        break;
    default:
        break;
    }
}

/*****************************************************************************
** 描  述：读LIS3DSH寄存器
** 参  数：[in]reg：寄存器地址
**         [out]reg：读取的寄存器数据
** 返回值：true-读寄存器成功
******************************************************************************/
bool LIS3DH_ReadReg(uint8_t Reg, uint8_t* Data)
{
    unsigned int timeout_cnt = 1000;
    
    SPI_Tx_Buf[0] = Reg | LIS3DH_READBIT;
    //SPI传输完成标志设置为false
	spi_xfer_done = false;
	//启动数据传输
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, SPI_Tx_Buf, 2, SPI_Rx_Buf, 2));
	//等待传输完成
	while (spi_xfer_done == false && timeout_cnt--)
        vTaskDelay(1);
    if (!spi_xfer_done && !timeout_cnt)
        return false;
      
    /* Send received value back to the caller */
    *Data = SPI_Rx_Buf[1];

    return true;
}

/*****************************************************************************
** 描  述：写LIS3DH寄存器
** 参  数：[in]WriteAddr：寄存器地址
**         [in]data：数据
** 返回值：true-写寄存器成功
******************************************************************************/
u8_t LIS3DH_WriteReg(uint8_t WriteAddr, uint8_t Data)
{
    unsigned int timeout_cnt = 1000;
    
    SPIWriteLength = 2;//发送的数据长度
    SPIReadLength = 0; //接收的数据长度
    SPI_Tx_Buf[0] = WriteAddr;
    SPI_Tx_Buf[1] = (Data);
	//SPI传输完成标志设置为false
	spi_xfer_done = false;
	//启动数据传输
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, SPI_Tx_Buf, SPIWriteLength, SPI_Rx_Buf, SPIReadLength));
	//等待传输完成
	while (spi_xfer_done == false && timeout_cnt--)
        vTaskDelay(1);
    if (!spi_xfer_done && !timeout_cnt)
        return false;
    return true;
}

uint8_t LIS3DH_SPI_Init(void)
{
#ifdef CONFIG_LIS3DH_USE_I2C
    ret_code_t err_code;
    const nrf_drv_twi_config_t twi_lis3dh_config = {
       .scl                = 25,
       .sda                = 26,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };
    
    if (is_lis3dh_spi_inited)
        return -1;
    err_code = nrf_drv_twi_init(&m_twi, &twi_lis3dh_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
#else
    /* 初始化SPI0 */
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    
    if (is_lis3dh_spi_inited)
        return -1;
    spi_config.ss_pin   = SPI_SS_PIN;//nRF52832只能使用GPIO作为片选，所以这个单独定义了SPI CS管脚
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
	spi_config.frequency = NRF_DRV_SPI_FREQ_4M;
    //初始化SPI
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
    nrf_delay_ms(500);
    is_lis3dh_spi_inited = true;
#endif
    
    return 0;
}

/**********************************************************************************************
 * 描  述 : 初始化LIS3DH
 * 入  参 : 无
 * 返回值 : true-初始化成功
 **************************************************/
uint8_t LIS3DH_Init(void)
{
    uint8_t whoami, cnt = 0;
    bool ret;
	
retry:
    LIS3DH_SPI_Init();
    /*读取WHO_AM_I判断LIS3DH是否存在 */
	ret = LIS3DH_ReadReg(LIS3DH_WHO_AM_I, &whoami);
    if (!ret)
        return 1;
	NRF_LOG_INFO("LIS3DH ID: %02X", (uint8_t)whoami);
	if (whoami != 0x33) {
        NRF_LOG_INFO("LIS3DH is not found! whoami: 0x%x\r\n", whoami);
        vTaskDelay(1000);
        cnt++;
        if (cnt > 10)
            return 2;
        else {
            NRF_LOG_INFO("cnt: %d\r\n", cnt);
            goto retry;
        }
	}
		
	//设置ODR：100Hz
	if(LIS3DH_SetODR(LIS3DH_ODR_100Hz) == MEMS_SUCCESS)
        NRF_LOG_INFO("SET_ODR_OK");
	//设置工作模式：NORMAL
	if(LIS3DH_SetMode(LIS3DH_NORMAL) == MEMS_SUCCESS)
        NRF_LOG_INFO("SET_MODE_OK");
    //设置分辨率：+/-2G
	if(LIS3DH_SetFullScale(LIS3DH_FULLSCALE_2) == MEMS_SUCCESS)
        NRF_LOG_INFO("SET_FULLSCALE_OK");
    
	if(LIS3DH_SetAxis(LIS3DH_X_ENABLE | LIS3DH_Y_ENABLE | LIS3DH_Z_ENABLE) == MEMS_SUCCESS)
        NRF_LOG_INFO("SET_AXIS_OK");

    return 0;
}
/*******************************************************************************
* Function Name  : LIS3DH_GetStatusAUX
* Description    : Read the AUX status register
* Input          : Char to empty by status register buffer
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetStatusAUX(u8_t* val)
{

    if( !LIS3DH_ReadReg(LIS3DH_STATUS_AUX, val) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3DH_GetStatusAUXBIT
* Description    : Read the AUX status register BIT
* Input          : LIS3DH_STATUS_AUX_321OR, LIS3DH_STATUS_AUX_3OR, LIS3DH_STATUS_AUX_2OR, LIS3DH_STATUS_AUX_1OR,
                   LIS3DH_STATUS_AUX_321DA, LIS3DH_STATUS_AUX_3DA, LIS3DH_STATUS_AUX_2DA, LIS3DH_STATUS_AUX_1DA
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetStatusAUXBit(u8_t statusBIT, u8_t* val)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_STATUS_AUX, &value) )
        return MEMS_ERROR;

    if(statusBIT == LIS3DH_STATUS_AUX_321OR) {
        if(value &= LIS3DH_STATUS_AUX_321OR) {
            *val = MEMS_SET;
            return MEMS_SUCCESS;
        } else {
            *val = MEMS_RESET;
            return MEMS_SUCCESS;
        }
    }

    if(statusBIT == LIS3DH_STATUS_AUX_3OR) {
        if(value &= LIS3DH_STATUS_AUX_3OR) {
            *val = MEMS_SET;
            return MEMS_SUCCESS;
        } else {
            *val = MEMS_RESET;
            return MEMS_SUCCESS;
        }
    }

    if(statusBIT == LIS3DH_STATUS_AUX_2OR) {
        if(value &= LIS3DH_STATUS_AUX_2OR) {
            *val = MEMS_SET;
            return MEMS_SUCCESS;
        } else {
            *val = MEMS_RESET;
            return MEMS_SUCCESS;
        }
    }

    if(statusBIT == LIS3DH_STATUS_AUX_1OR) {
        if(value &= LIS3DH_STATUS_AUX_1OR) {
            *val = MEMS_SET;
            return MEMS_SUCCESS;
        } else {
            *val = MEMS_RESET;
            return MEMS_SUCCESS;
        }
    }

    if(statusBIT == LIS3DH_STATUS_AUX_321DA) {
        if(value &= LIS3DH_STATUS_AUX_321DA) {
            *val = MEMS_SET;
            return MEMS_SUCCESS;
        } else {
            *val = MEMS_RESET;
            return MEMS_SUCCESS;
        }
    }

    if(statusBIT == LIS3DH_STATUS_AUX_3DA) {
        if(value &= LIS3DH_STATUS_AUX_3DA) {
            *val = MEMS_SET;
            return MEMS_SUCCESS;
        } else {
            *val = MEMS_RESET;
            return MEMS_SUCCESS;
        }
    }

    if(statusBIT == LIS3DH_STATUS_AUX_2DA) {
        if(value &= LIS3DH_STATUS_AUX_2DA) {
            *val = MEMS_SET;
            return MEMS_SUCCESS;
        } else {
            *val = MEMS_RESET;
            return MEMS_SUCCESS;
        }
    }

    if(statusBIT == LIS3DH_STATUS_AUX_1DA) {
        if(value &= LIS3DH_STATUS_AUX_1DA) {
            *val = MEMS_SET;
            return MEMS_SUCCESS;
        } else {
            *val = MEMS_RESET;
            return MEMS_SUCCESS;
        }
    }
    return MEMS_ERROR;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetODR
* Description    : Sets LIS3DH Output Data Rate
* Input          : Output Data Rate
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetODR(LIS3DH_ODR_t ov)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG1, &value) )
        return MEMS_ERROR;

    value &= 0x0f;
    value |= ov<<LIS3DH_ODR_BIT;

    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG1, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetTemperature
* Description    : Sets LIS3DH Output Temperature
* Input          : MEMS_ENABLE, MEMS_DISABLE
* Output         : None
* Note           : For Read Temperature by LIS3DH_OUT_AUX_3, LIS3DH_SetADCAux and LIS3DH_SetBDU 
       functions must be ENABLE
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetTemperature(State_t state)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_TEMP_CFG_REG, &value) )
        return MEMS_ERROR;

    value &= 0xBF;
    value |= state<<LIS3DH_TEMP_EN;
    value |= state<<LIS3DH_ADC_PD;

    if( !LIS3DH_WriteReg(LIS3DH_TEMP_CFG_REG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetADCAux
* Description    : Sets LIS3DH Output ADC
* Input          : MEMS_ENABLE, MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetADCAux(State_t state)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_TEMP_CFG_REG, &value) )
        return MEMS_ERROR;

    value &= 0x7F;
    value |= state<<LIS3DH_ADC_PD;

    if( !LIS3DH_WriteReg(LIS3DH_TEMP_CFG_REG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_GetAuxRaw
* Description    : Read the Aux Values Output Registers
* Input          : Buffer to empty
* Output         : Aux Values Registers buffer
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetAuxRaw(LIS3DH_Aux123Raw_t* buff)
{
    u8_t valueL;
    u8_t valueH;

    if( !LIS3DH_ReadReg(LIS3DH_OUT_1_L, &valueL) )
        return MEMS_ERROR;

    if( !LIS3DH_ReadReg(LIS3DH_OUT_1_H, &valueH) )
        return MEMS_ERROR;

    buff->AUX_1 = (u16_t)( (valueH << 8) | valueL )/16;

    if( !LIS3DH_ReadReg(LIS3DH_OUT_2_L, &valueL) )
        return MEMS_ERROR;

    if( !LIS3DH_ReadReg(LIS3DH_OUT_2_H, &valueH) )
        return MEMS_ERROR;

    buff->AUX_2 = (u16_t)( (valueH << 8) | valueL )/16;

    if( !LIS3DH_ReadReg(LIS3DH_OUT_3_L, &valueL) )
        return MEMS_ERROR;

    if( !LIS3DH_ReadReg(LIS3DH_OUT_3_H, &valueH) )
        return MEMS_ERROR;

    buff->AUX_3 = (u16_t)( (valueH << 8) | valueL )/16;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_GetTempRaw
* Description    : Read the Temperature Values by AUX Output Registers OUT_3_H
* Input          : Buffer to empty
* Output         : Temperature Values Registers buffer
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetTempRaw(i8_t* buff)
{
    u8_t valueL;
    u8_t valueH;

    if( !LIS3DH_ReadReg(LIS3DH_OUT_3_L, &valueL) )
        return MEMS_ERROR;

    if( !LIS3DH_ReadReg(LIS3DH_OUT_3_H, &valueH) )
        return MEMS_ERROR;

    *buff = (i8_t)( valueH );

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetMode
* Description    : Sets LIS3DH Operating Mode
* Input          : Modality (LIS3DH_NORMAL, LIS3DH_LOW_POWER, LIS3DH_POWER_DOWN)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetMode(LIS3DH_Mode_t md)
{
    u8_t value;
    u8_t value2;
    static   u8_t ODR_old_value;

    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG1, &value) )
        return MEMS_ERROR;

    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG4, &value2) )
        return MEMS_ERROR;

    if((value & 0xF0)==0)
        value = value | (ODR_old_value & 0xF0); //if it comes from POWERDOWN

    switch(md) {

        case LIS3DH_POWER_DOWN:
            ODR_old_value = value;
            value &= 0x0F;
            break;

        case LIS3DH_NORMAL:
            value &= 0xF7;
            value |= (MEMS_RESET<<LIS3DH_LPEN);
            value2 &= 0xF7;
            value2 |= (MEMS_SET<<LIS3DH_HR);   //set HighResolution_BIT
            break;

        case LIS3DH_LOW_POWER:
            value &= 0xF7;
            value |=  (MEMS_SET<<LIS3DH_LPEN);
            value2 &= 0xF7;
            value2 |= (MEMS_RESET<<LIS3DH_HR); //reset HighResolution_BIT
            break;

        default:
            return MEMS_ERROR;
    }

    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG1, value) )
        return MEMS_ERROR;

    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG4, value2) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetAxis
* Description    : Enable/Disable LIS3DH Axis
* Input          : LIS3DH_X_ENABLE/DISABLE | LIS3DH_Y_ENABLE/DISABLE | LIS3DH_Z_ENABLE/DISABLE
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetAxis(LIS3DH_Axis_t axis)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG1, &value) )
        return MEMS_ERROR;
    value &= 0xF8;
    value |= (0x07 & axis);

    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG1, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetFullScale
* Description    : Sets the LIS3DH FullScale
* Input          : LIS3DH_FULLSCALE_2/LIS3DH_FULLSCALE_4/LIS3DH_FULLSCALE_8/LIS3DH_FULLSCALE_16
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetFullScale(LIS3DH_Fullscale_t fs)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG4, &value) )
        return MEMS_ERROR;

    value &= 0xCF;
    value |= (fs<<LIS3DH_FS);

    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG4, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetBDU
* Description    : Enable/Disable Block Data Update Functionality
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetBDU(State_t bdu)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG4, &value) )
        return MEMS_ERROR;

    value &= 0x7F;
    value |= (bdu<<LIS3DH_BDU);

    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG4, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetBLE
* Description    : Set Endianess (MSB/LSB)
* Input          : BLE_LSB / BLE_MSB
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetBLE(LIS3DH_Endianess_t ble)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG4, &value) )
        return MEMS_ERROR;

    value &= 0xBF;
    value |= (ble<<LIS3DH_BLE);

    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG4, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetSelfTest
* Description    : Set Self Test Modality
* Input          : LIS3DH_SELF_TEST_DISABLE/ST_0/ST_1
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetSelfTest(LIS3DH_SelfTest_t st)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG4, &value) )
        return MEMS_ERROR;

    value &= 0xF9;
    value |= (st<<LIS3DH_ST);

    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG4, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_HPFClick
* Description    : Enable/Disable High Pass Filter for click
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_HPFClickEnable(State_t hpfe)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG2, &value) )
        return MEMS_ERROR;

    value &= 0xFB;
    value |= (hpfe<<LIS3DH_HPCLICK);

    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG2, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_HPFAOI1
* Description    : Enable/Disable High Pass Filter for AOI on INT_1
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_HPFAOI1Enable(State_t hpfe)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG2, &value) )
        return MEMS_ERROR;

    value &= 0xFE;
    value |= (hpfe<<LIS3DH_HPIS1);

    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG2, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_HPFAOI2
* Description    : Enable/Disable High Pass Filter for AOI on INT_2
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_HPFAOI2Enable(State_t hpfe)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG2, &value) )
        return MEMS_ERROR;

    value &= 0xFD;
    value |= (hpfe<<LIS3DH_HPIS2);

    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG2, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetHPFMode
* Description    : Set High Pass Filter Modality
* Input          : LIS3DH_HPM_NORMAL_MODE_RES/LIS3DH_HPM_REF_SIGNAL/
       LIS3DH_HPM_NORMAL_MODE/LIS3DH_HPM_AUTORESET_INT
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetHPFMode(LIS3DH_HPFMode_t hpm)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG2, &value) )
        return MEMS_ERROR;

    value &= 0x3F;
    value |= (hpm<<LIS3DH_HPM);

    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG2, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetHPFCutOFF
* Description    : Set High Pass CUT OFF Freq
* Input          : HPFCF [0,3]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetHPFCutOFF(LIS3DH_HPFCutOffFreq_t hpf)
{
    u8_t value;

    if (hpf > 3)
        return MEMS_ERROR;

    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG2, &value) )
        return MEMS_ERROR;

    value &= 0xCF;
    value |= (hpf<<LIS3DH_HPCF);

    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG2, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;

}


/*******************************************************************************
* Function Name  : LIS3DH_SetFilterDataSel
* Description    : Set Filter Data Selection bypassed or sent to FIFO OUT register
* Input          : MEMS_SET, MEMS_RESET
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetFilterDataSel(State_t state)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG2, &value) )
        return MEMS_ERROR;

    value &= 0xF7;
    value |= (state<<LIS3DH_FDS);

    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG2, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;

}


/*******************************************************************************
* Function Name  : LIS3DH_SetInt1Pin
* Description    : Set Interrupt1 pin Function
* Input          :  LIS3DH_CLICK_ON_PIN_INT1_ENABLE/DISABLE    | LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE/DISABLE |              
                    LIS3DH_I1_INT2_ON_PIN_INT1_ENABLE/DISABLE  | LIS3DH_I1_DRDY1_ON_INT1_ENABLE/DISABLE    |              
                    LIS3DH_I1_DRDY2_ON_INT1_ENABLE/DISABLE     | LIS3DH_WTM_ON_INT1_ENABLE/DISABLE         |           
                    LIS3DH_INT1_OVERRUN_ENABLE/DISABLE  
* example        : SetInt1Pin(LIS3DH_CLICK_ON_PIN_INT1_ENABLE | LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE |              
                    LIS3DH_I1_INT2_ON_PIN_INT1_DISABLE | LIS3DH_I1_DRDY1_ON_INT1_ENABLE | LIS3DH_I1_DRDY2_ON_INT1_ENABLE |
                    LIS3DH_WTM_ON_INT1_DISABLE | LIS3DH_INT1_OVERRUN_DISABLE   ) 
* Note           : To enable Interrupt signals on INT1 Pad (You MUST use all input variable in the argument, as example)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetInt1Pin(LIS3DH_IntPinConf_t pinConf)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG3, &value) )
        return MEMS_ERROR;

    value &= 0x00;
    value |= pinConf;

    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG3, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetInt2Pin
* Description    : Set Interrupt2 pin Function
* Input          : LIS3DH_CLICK_ON_PIN_INT2_ENABLE/DISABLE   | LIS3DH_I2_INT1_ON_PIN_INT2_ENABLE/DISABLE |               
                   LIS3DH_I2_INT2_ON_PIN_INT2_ENABLE/DISABLE | LIS3DH_I2_BOOT_ON_INT2_ENABLE/DISABLE |                   
                   LIS3DH_INT_ACTIVE_HIGH/LOW
* example        : LIS3DH_SetInt2Pin(LIS3DH_CLICK_ON_PIN_INT2_ENABLE/DISABLE | LIS3DH_I2_INT1_ON_PIN_INT2_ENABLE/DISABLE |               
                   LIS3DH_I2_INT2_ON_PIN_INT2_ENABLE/DISABLE | LIS3DH_I2_BOOT_ON_INT2_ENABLE/DISABLE |                   
                   LIS3DH_INT_ACTIVE_HIGH/LOW)
* Note           : To enable Interrupt signals on INT2 Pad (You MUST use all input variable in the argument, as example)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetInt2Pin(LIS3DH_IntPinConf_t pinConf)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG6, &value) )
        return MEMS_ERROR;

    value &= 0x00;
    value |= pinConf;

    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG6, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetClickCFG
* Description    : Set Click Interrupt config Function
* Input          : LIS3DH_ZD_ENABLE/DISABLE | LIS3DH_ZS_ENABLE/DISABLE  | LIS3DH_YD_ENABLE/DISABLE  | 
                   LIS3DH_YS_ENABLE/DISABLE | LIS3DH_XD_ENABLE/DISABLE  | LIS3DH_XS_ENABLE/DISABLE 
* example        : LIS3DH_SetClickCFG( LIS3DH_ZD_ENABLE | LIS3DH_ZS_DISABLE | LIS3DH_YD_ENABLE | 
                               LIS3DH_YS_DISABLE | LIS3DH_XD_ENABLE | LIS3DH_XS_ENABLE)
* Note           : You MUST use all input variable in the argument, as example
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetClickCFG(u8_t status)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_CLICK_CFG, &value) )
        return MEMS_ERROR;

    value &= 0xC0;
    value |= status;

    if( !LIS3DH_WriteReg(LIS3DH_CLICK_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetClickTHS
* Description    : Set Click Interrupt threshold
* Input          : Click-click Threshold value [0-127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetClickTHS(u8_t ths)
{

    if(ths>127)
        return MEMS_ERROR;

    if( !LIS3DH_WriteReg(LIS3DH_CLICK_THS, ths) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetClickLIMIT
* Description    : Set Click Interrupt Time Limit
* Input          : Click-click Time Limit value [0-127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetClickLIMIT(u8_t t_limit)
{

    if(t_limit>127)
        return MEMS_ERROR;

    if( !LIS3DH_WriteReg(LIS3DH_TIME_LIMIT, t_limit) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetClickLATENCY
* Description    : Set Click Interrupt Time Latency
* Input          : Click-click Time Latency value [0-255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetClickLATENCY(u8_t t_latency)
{

    if( !LIS3DH_WriteReg(LIS3DH_TIME_LATENCY, t_latency) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetClickWINDOW
* Description    : Set Click Interrupt Time Window
* Input          : Click-click Time Window value [0-255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetClickWINDOW(u8_t t_window)
{

    if( !LIS3DH_WriteReg(LIS3DH_TIME_WINDOW, t_window) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_GetClickResponse
* Description    : Get Click Interrupt Response by CLICK_SRC REGISTER
* Input          : char to empty by Click Response Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetClickResponse(u8_t* res)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_CLICK_SRC, &value) )
        return MEMS_ERROR;

    value &= 0x7F;

    if((value & LIS3DH_IA)==0) {
        *res = LIS3DH_NO_CLICK;
        return MEMS_SUCCESS;
    } else {
        if (value & LIS3DH_DCLICK) {
            if (value & LIS3DH_CLICK_SIGN) {
                if (value & LIS3DH_CLICK_Z) {
                    *res = LIS3DH_DCLICK_Z_N;
                    return MEMS_SUCCESS;
                }
                if (value & LIS3DH_CLICK_Y) {
                    *res = LIS3DH_DCLICK_Y_N;
                    return MEMS_SUCCESS;
                }
                if (value & LIS3DH_CLICK_X) {
                    *res = LIS3DH_DCLICK_X_N;
                    return MEMS_SUCCESS;
                }
            } else {
                if (value & LIS3DH_CLICK_Z) {
                    *res = LIS3DH_DCLICK_Z_P;
                    return MEMS_SUCCESS;
                }
                if (value & LIS3DH_CLICK_Y) {
                    *res = LIS3DH_DCLICK_Y_P;
                    return MEMS_SUCCESS;
                }
                if (value & LIS3DH_CLICK_X) {
                    *res = LIS3DH_DCLICK_X_P;
                    return MEMS_SUCCESS;
                }
            }
        } else {
            if (value & LIS3DH_CLICK_SIGN) {
                if (value & LIS3DH_CLICK_Z) {
                    *res = LIS3DH_SCLICK_Z_N;
                    return MEMS_SUCCESS;
                }
                if (value & LIS3DH_CLICK_Y) {
                    *res = LIS3DH_SCLICK_Y_N;
                    return MEMS_SUCCESS;
                }
                if (value & LIS3DH_CLICK_X) {
                    *res = LIS3DH_SCLICK_X_N;
                    return MEMS_SUCCESS;
                }
            } else {
                if (value & LIS3DH_CLICK_Z) {
                    *res = LIS3DH_SCLICK_Z_P;
                    return MEMS_SUCCESS;
                }
                if (value & LIS3DH_CLICK_Y) {
                    *res = LIS3DH_SCLICK_Y_P;
                    return MEMS_SUCCESS;
                }
                if (value & LIS3DH_CLICK_X) {
                    *res = LIS3DH_SCLICK_X_P;
                    return MEMS_SUCCESS;
                }
            }
        }
    }
    return MEMS_ERROR;
}
/*******************************************************************************
* Function Name  : LIS3DH_Int1LatchEnable
* Description    : reboot sensor, reset mem registers function
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_RESET_MEM(void)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG5, &value) )
        return MEMS_ERROR;

    value &= 0x7F;
    value |= 1<<LIS3DH_BOOT;

    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG5, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_Int1LatchEnable
* Description    : Enable Interrupt 1 Latching function
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_Int1LatchEnable(State_t latch)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG5, &value) )
        return MEMS_ERROR;

    value &= 0xF7;
    value |= latch<<LIS3DH_LIR_INT1;

    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG5, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_ResetInt1Latch
* Description    : Reset Interrupt 1 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_ResetInt1Latch(void)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_INT1_SRC, &value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetIntConfiguration
* Description    : Interrupt 1 Configuration (without LIS3DH_6D_INT)
* Input          : LIS3DH_INT1_AND/OR | LIS3DH_INT1_ZHIE_ENABLE/DISABLE | LIS3DH_INT1_ZLIE_ENABLE/DISABLE...
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetIntConfiguration(LIS3DH_Int1Conf_t ic)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_INT1_CFG, &value) )
        return MEMS_ERROR;

    value &= 0x40;
    value |= ic;

    if( !LIS3DH_WriteReg(LIS3DH_INT1_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetIntMode
* Description    : Interrupt 1 Configuration mode (OR, 6D Movement, AND, 6D Position)
* Input          : LIS3DH_INT_MODE_OR, LIS3DH_INT_MODE_6D_MOVEMENT, LIS3DH_INT_MODE_AND, 
       LIS3DH_INT_MODE_6D_POSITION
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetIntMode(LIS3DH_Int1Mode_t int_mode)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_INT1_CFG, &value) )
        return MEMS_ERROR;

    value &= 0x3F;
    value |= (int_mode<<LIS3DH_INT_6D);

    if( !LIS3DH_WriteReg(LIS3DH_INT1_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetInt6D4DConfiguration
* Description    : 6D, 4D Interrupt Configuration
* Input          : LIS3DH_INT1_6D_ENABLE, LIS3DH_INT1_4D_ENABLE, LIS3DH_INT1_6D_4D_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetInt6D4DConfiguration(LIS3DH_INT_6D_4D_t ic)
{
    u8_t value;
    u8_t value2;

    if( !LIS3DH_ReadReg(LIS3DH_INT1_CFG, &value) )
        return MEMS_ERROR;
    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG5, &value2) )
        return MEMS_ERROR;

    if(ic == LIS3DH_INT1_6D_ENABLE) {
        value &= 0xBF;
        value |= (MEMS_ENABLE<<LIS3DH_INT_6D);
        value2 &= 0xFB;
        value2 |= (MEMS_DISABLE<<LIS3DH_D4D_INT1);
    }

    if(ic == LIS3DH_INT1_4D_ENABLE) {
        value &= 0xBF;
        value |= (MEMS_ENABLE<<LIS3DH_INT_6D);
        value2 &= 0xFB;
        value2 |= (MEMS_ENABLE<<LIS3DH_D4D_INT1);
    }

    if(ic == LIS3DH_INT1_6D_4D_DISABLE) {
        value &= 0xBF;
        value |= (MEMS_DISABLE<<LIS3DH_INT_6D);
        value2 &= 0xFB;
        value2 |= (MEMS_DISABLE<<LIS3DH_D4D_INT1);
    }

    if( !LIS3DH_WriteReg(LIS3DH_INT1_CFG, value) )
        return MEMS_ERROR;
    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG5, value2) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_Get6DPosition
* Description    : 6D, 4D Interrupt Position Detect
* Input          : Byte to empty by POSITION_6D_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_Get6DPosition(u8_t* val)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_INT1_SRC, &value) )
        return MEMS_ERROR;

    value &= 0x7F;

    switch (value) {
        case LIS3DH_UP_SX:
            *val = LIS3DH_UP_SX;
            break;
        case LIS3DH_UP_DX:
            *val = LIS3DH_UP_DX;
            break;
        case LIS3DH_DW_SX:
            *val = LIS3DH_DW_SX;
            break;
        case LIS3DH_DW_DX:
            *val = LIS3DH_DW_DX;
            break;
        case LIS3DH_TOP:
            *val = LIS3DH_TOP;
            break;
        case LIS3DH_BOTTOM:
            *val = LIS3DH_BOTTOM;
            break;
    }

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetInt1Threshold
* Description    : Sets Interrupt 1 Threshold
* Input          : Threshold = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetInt1Threshold(u8_t ths)
{
    if (ths > 127)
        return MEMS_ERROR;

    if( !LIS3DH_WriteReg(LIS3DH_INT1_THS, ths) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetInt1Duration
* Description    : Sets Interrupt 1 Duration
* Input          : Duration value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetInt1Duration(LIS3DH_Int1Conf_t id)
{

    if (id > 127)
        return MEMS_ERROR;

    if( !LIS3DH_WriteReg(LIS3DH_INT1_DURATION, id) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_FIFOModeEnable
* Description    : Sets Fifo Modality
* Input          : LIS3DH_FIFO_DISABLE, LIS3DH_FIFO_BYPASS_MODE, LIS3DH_FIFO_MODE, 
       LIS3DH_FIFO_STREAM_MODE, LIS3DH_FIFO_TRIGGER_MODE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_FIFOModeEnable(LIS3DH_FifoMode_t fm)
{
    u8_t value;

    if(fm == LIS3DH_FIFO_DISABLE) {
        if( !LIS3DH_ReadReg(LIS3DH_FIFO_CTRL_REG, &value) )
            return MEMS_ERROR;

        value &= 0x1F;
        value |= (LIS3DH_FIFO_BYPASS_MODE<<LIS3DH_FM);

        if( !LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG, value) )           //fifo mode bypass
            return MEMS_ERROR;
        if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG5, &value) )
            return MEMS_ERROR;

        value &= 0xBF;

        if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG5, value) )               //fifo disable
            return MEMS_ERROR;
    }

    if(fm == LIS3DH_FIFO_BYPASS_MODE)   {
        if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG5, &value) )
            return MEMS_ERROR;

        value &= 0xBF;
        value |= MEMS_SET<<LIS3DH_FIFO_EN;

        if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG5, value) )               //fifo enable
            return MEMS_ERROR;
        if( !LIS3DH_ReadReg(LIS3DH_FIFO_CTRL_REG, &value) )
            return MEMS_ERROR;

        value &= 0x1f;
        value |= (fm<<LIS3DH_FM);                     //fifo mode configuration

        if( !LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG, value) )
            return MEMS_ERROR;
    }

    if(fm == LIS3DH_FIFO_MODE)   {
        if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG5, &value) )
            return MEMS_ERROR;

        value &= 0xBF;
        value |= MEMS_SET<<LIS3DH_FIFO_EN;

        if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG5, value) )               //fifo enable
            return MEMS_ERROR;
        if( !LIS3DH_ReadReg(LIS3DH_FIFO_CTRL_REG, &value) )
            return MEMS_ERROR;

        value &= 0x1f;
        value |= (fm<<LIS3DH_FM);                      //fifo mode configuration

        if( !LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG, value) )
            return MEMS_ERROR;
    }

    if(fm == LIS3DH_FIFO_STREAM_MODE)   {
        if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG5, &value) )
            return MEMS_ERROR;

        value &= 0xBF;
        value |= MEMS_SET<<LIS3DH_FIFO_EN;

        if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG5, value) )               //fifo enable
            return MEMS_ERROR;
        if( !LIS3DH_ReadReg(LIS3DH_FIFO_CTRL_REG, &value) )
            return MEMS_ERROR;

        value &= 0x1f;
        value |= (fm<<LIS3DH_FM);                      //fifo mode configuration

        if( !LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG, value) )
            return MEMS_ERROR;
    }

    if(fm == LIS3DH_FIFO_TRIGGER_MODE)   {
        if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG5, &value) )
            return MEMS_ERROR;

        value &= 0xBF;
        value |= MEMS_SET<<LIS3DH_FIFO_EN;

        if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG5, value) )               //fifo enable
            return MEMS_ERROR;
        if( !LIS3DH_ReadReg(LIS3DH_FIFO_CTRL_REG, &value) )
            return MEMS_ERROR;

        value &= 0x1f;
        value |= (fm<<LIS3DH_FM);                      //fifo mode configuration

        if( !LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG, value) )
            return MEMS_ERROR;
    }

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetTriggerInt
* Description    : Trigger event liked to trigger signal INT1/INT2
* Input          : LIS3DH_TRIG_INT1/LIS3DH_TRIG_INT2
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetTriggerInt(LIS3DH_TrigInt_t tr)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_FIFO_CTRL_REG, &value) )
        return MEMS_ERROR;

    value &= 0xDF;
    value |= (tr<<LIS3DH_TR);

    if( !LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetWaterMark
* Description    : Sets Watermark Value
* Input          : Watermark = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetWaterMark(u8_t wtm)
{
    u8_t value;

    if(wtm > 31)
        return MEMS_ERROR;

    if( !LIS3DH_ReadReg(LIS3DH_FIFO_CTRL_REG, &value) )
        return MEMS_ERROR;

    value &= 0xE0;
    value |= wtm;

    if( !LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_GetStatusReg
* Description    : Read the status register
* Input          : char to empty by Status Reg Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetStatusReg(u8_t* val)
{
    if( !LIS3DH_ReadReg(LIS3DH_STATUS_REG, val) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_GetStatusBIT
* Description    : Read the status register BIT
* Input          : LIS3DH_STATUS_REG_ZYXOR, LIS3DH_STATUS_REG_ZOR, LIS3DH_STATUS_REG_YOR, LIS3DH_STATUS_REG_XOR,
                   LIS3DH_STATUS_REG_ZYXDA, LIS3DH_STATUS_REG_ZDA, LIS3DH_STATUS_REG_YDA, LIS3DH_STATUS_REG_XDA, 
       LIS3DH_DATAREADY_BIT
       val: Byte to be filled with the status bit 
* Output         : status register BIT
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetStatusBit(u8_t statusBIT, u8_t* val)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_STATUS_REG, &value) )
        return MEMS_ERROR;

    switch (statusBIT) {
        case LIS3DH_STATUS_REG_ZYXOR:
            if(value &= LIS3DH_STATUS_REG_ZYXOR) {
                *val = MEMS_SET;
                return MEMS_SUCCESS;
            } else {
                *val = MEMS_RESET;
                return MEMS_SUCCESS;
            }
        case LIS3DH_STATUS_REG_ZOR:
            if(value &= LIS3DH_STATUS_REG_ZOR) {
                *val = MEMS_SET;
                return MEMS_SUCCESS;
            } else {
                *val = MEMS_RESET;
                return MEMS_SUCCESS;
            }
        case LIS3DH_STATUS_REG_YOR:
            if(value &= LIS3DH_STATUS_REG_YOR) {
                *val = MEMS_SET;
                return MEMS_SUCCESS;
            } else {
                *val = MEMS_RESET;
                return MEMS_SUCCESS;
            }
        case LIS3DH_STATUS_REG_XOR:
            if(value &= LIS3DH_STATUS_REG_XOR) {
                *val = MEMS_SET;
                return MEMS_SUCCESS;
            } else {
                *val = MEMS_RESET;
                return MEMS_SUCCESS;
            }
        case LIS3DH_STATUS_REG_ZYXDA:
            if(value &= LIS3DH_STATUS_REG_ZYXDA) {
                *val = MEMS_SET;
                return MEMS_SUCCESS;
            } else {
                *val = MEMS_RESET;
                return MEMS_SUCCESS;
            }
        case LIS3DH_STATUS_REG_ZDA:
            if(value &= LIS3DH_STATUS_REG_ZDA) {
                *val = MEMS_SET;
                return MEMS_SUCCESS;
            } else {
                *val = MEMS_RESET;
                return MEMS_SUCCESS;
            }
        case LIS3DH_STATUS_REG_YDA:
            if(value &= LIS3DH_STATUS_REG_YDA) {
                *val = MEMS_SET;
                return MEMS_SUCCESS;
            } else {
                *val = MEMS_RESET;
                return MEMS_SUCCESS;
            }
        case LIS3DH_STATUS_REG_XDA:
            if(value &= LIS3DH_STATUS_REG_XDA) {
                *val = MEMS_SET;
                return MEMS_SUCCESS;
            } else {
                *val = MEMS_RESET;
                return MEMS_SUCCESS;
            }

    }
    return MEMS_ERROR;
}

/*******************************************************************************
* Function Name  : LIS3DH_GetAccAxesRaw
* Description    : Read the Acceleration Values Output Registers
* Input          : buffer to empity by AxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetAccAxesRaw(AxesRaw_t* buff)
{
	  int16_t value;
    u8_t *valueL = (u8_t *)(&value);
    u8_t *valueH = ((u8_t *)(&value)+1);
  
    if( !LIS3DH_ReadReg(LIS3DH_OUT_X_L, valueL) )
    return MEMS_ERROR;
  
    if( !LIS3DH_ReadReg(LIS3DH_OUT_X_H, valueH) )
    return MEMS_ERROR;
  
    buff->AXIS_X = value;
  
    if( !LIS3DH_ReadReg(LIS3DH_OUT_Y_L, valueL) )
    return MEMS_ERROR;
  
    if( !LIS3DH_ReadReg(LIS3DH_OUT_Y_H, valueH) )
    return MEMS_ERROR;
  
    buff->AXIS_Y = value;
  
    if( !LIS3DH_ReadReg(LIS3DH_OUT_Z_L, valueL) )
    return MEMS_ERROR;
  
    if( !LIS3DH_ReadReg(LIS3DH_OUT_Z_H, valueH) )
    return MEMS_ERROR;
  
    buff->AXIS_Z = value;
  
    return MEMS_SUCCESS;
	
}


/*******************************************************************************
* Function Name  : LIS3DH_GetInt1Src
* Description    : Reset Interrupt 1 Latching function
* Input          : Char to empty by Int1 source value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetInt1Src(u8_t* val)
{

    if( !LIS3DH_ReadReg(LIS3DH_INT1_SRC, val) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_GetInt1SrcBit
* Description    : Reset Interrupt 1 Latching function
* Input          : statusBIT: LIS3DH_INT_SRC_IA, LIS3DH_INT_SRC_ZH, LIS3DH_INT_SRC_ZL.....
*                  val: Byte to be filled with the status bit
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetInt1SrcBit(u8_t statusBIT, u8_t* val)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_INT1_SRC, &value) )
        return MEMS_ERROR;

    if(statusBIT == LIS3DH_INT1_SRC_IA) {
        if(value &= LIS3DH_INT1_SRC_IA) {
            *val = MEMS_SET;
            return MEMS_SUCCESS;
        } else {
            *val = MEMS_RESET;
            return MEMS_SUCCESS;
        }
    }

    if(statusBIT == LIS3DH_INT1_SRC_ZH) {
        if(value &= LIS3DH_INT1_SRC_ZH) {
            *val = MEMS_SET;
            return MEMS_SUCCESS;
        } else {
            *val = MEMS_RESET;
            return MEMS_SUCCESS;
        }
    }

    if(statusBIT == LIS3DH_INT1_SRC_ZL) {
        if(value &= LIS3DH_INT1_SRC_ZL) {
            *val = MEMS_SET;
            return MEMS_SUCCESS;
        } else {
            *val = MEMS_RESET;
            return MEMS_SUCCESS;
        }
    }

    if(statusBIT == LIS3DH_INT1_SRC_YH) {
        if(value &= LIS3DH_INT1_SRC_YH) {
            *val = MEMS_SET;
            return MEMS_SUCCESS;
        } else {
            *val = MEMS_RESET;
            return MEMS_SUCCESS;
        }
    }

    if(statusBIT == LIS3DH_INT1_SRC_YL) {
        if(value &= LIS3DH_INT1_SRC_YL) {
            *val = MEMS_SET;
            return MEMS_SUCCESS;
        } else {
            *val = MEMS_RESET;
            return MEMS_SUCCESS;
        }
    }
    if(statusBIT == LIS3DH_INT1_SRC_XH) {
        if(value &= LIS3DH_INT1_SRC_XH) {
            *val = MEMS_SET;
            return MEMS_SUCCESS;
        } else {
            *val = MEMS_RESET;
            return MEMS_SUCCESS;
        }
    }

    if(statusBIT == LIS3DH_INT1_SRC_XL) {
        if(value &= LIS3DH_INT1_SRC_XL) {
            *val = MEMS_SET;
            return MEMS_SUCCESS;
        } else {
            *val = MEMS_RESET;
            return MEMS_SUCCESS;
        }
    }
    return MEMS_ERROR;
}


/*******************************************************************************
* Function Name  : LIS3DH_GetFifoSourceReg
* Description    : Read Fifo source Register
* Input          : Byte to empty by FIFO source register value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetFifoSourceReg(u8_t* val)
{

    if( !LIS3DH_ReadReg(LIS3DH_FIFO_SRC_REG, val) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_GetFifoSourceBit
* Description    : Read Fifo WaterMark source bit
* Input          : statusBIT: LIS3DH_FIFO_SRC_WTM, LIS3DH_FIFO_SRC_OVRUN, LIS3DH_FIFO_SRC_EMPTY
*       val: Byte to fill  with the bit value
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetFifoSourceBit(u8_t statusBIT,  u8_t* val)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_FIFO_SRC_REG, &value) )
        return MEMS_ERROR;


    if(statusBIT == LIS3DH_FIFO_SRC_WTM) {
        if(value &= LIS3DH_FIFO_SRC_WTM) {
            *val = MEMS_SET;
            return MEMS_SUCCESS;
        } else {
            *val = MEMS_RESET;
            return MEMS_SUCCESS;
        }
    }

    if(statusBIT == LIS3DH_FIFO_SRC_OVRUN) {
        if(value &= LIS3DH_FIFO_SRC_OVRUN) {
            *val = MEMS_SET;
            return MEMS_SUCCESS;
        } else {
            *val = MEMS_RESET;
            return MEMS_SUCCESS;
        }
    }
    if(statusBIT == LIS3DH_FIFO_SRC_EMPTY) {
        if(value &= statusBIT == LIS3DH_FIFO_SRC_EMPTY) {
            *val = MEMS_SET;
            return MEMS_SUCCESS;
        } else {
            *val = MEMS_RESET;
            return MEMS_SUCCESS;
        }
    }
    return MEMS_ERROR;
}


/*******************************************************************************
* Function Name  : LIS3DH_GetFifoSourceFSS
* Description    : Read current number of unread samples stored in FIFO
* Input          : Byte to empty by FIFO unread sample value
* Output         : None
* Return         : Status [value of FSS]
*******************************************************************************/
status_t LIS3DH_GetFifoSourceFSS(u8_t* val)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_FIFO_SRC_REG, &value) )
        return MEMS_ERROR;

    value &= 0x1F;

    *val = value;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3DH_SetSPIInterface
* Description    : Set SPI mode: 3 Wire Interface OR 4 Wire Interface
* Input          : LIS3DH_SPI_3_WIRE, LIS3DH_SPI_4_WIRE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetSPIInterface(LIS3DH_SPIMode_t spi)
{
    u8_t value;

    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG4, &value) )
        return MEMS_ERROR;

    value &= 0xFE;
    value |= spi<<LIS3DH_SIM;

    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG4, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}
bool LIS3DH_enableFiFo(void)
{
    ASSERT(LIS3DH_SetTemperature(MEMS_ENABLE));

    /* Set data rate and power mode, and enable X/Y/Z */
    ASSERT(LIS3DH_WriteReg(LIS3DH_CTRL_REG1,
                           LIS3DH_CTRL_REG1_DATARATE_50HZ|    /* Normal mode, 10Hz */
                           LIS3DH_CTRL_REG1_XYZEN));           /* Enable X, Y and Z */
    // Settings for CTRL_REG2:
    // defaults
    ASSERT(LIS3DH_WriteReg(LIS3DH_CTRL_REG2, 0));
    // Settings for CTRL_REG3
    ASSERT(LIS3DH_WriteReg(LIS3DH_CTRL_REG3, 0));

    // Settings for CTRL_REG4:
    //   ----------------------------------------------------------------------------------------
    //CTRL_REG4| 7    6     5      4   3    2     1      0  |
    //   | BDU    BLE     FS1     FS2    HR    ST1   ST0   SIM|
    //    ----------------------------------------------------------------------------------------
    // FS1:FS2 -->g'range selection 00-->2g;01-->4g;10-->8g;11-->16g:
    // HR = 1  --> high resolution
    // SIM  -->if ==1 serial interface mode is selected(SPI)

    /* Settings for CTRL_REG4 Enable block update and set range to +/-2G */
    ASSERT(LIS3DH_WriteReg(LIS3DH_CTRL_REG4, 0x08));


    //  ASSERT(LIS3DH_WriteReg(LIS3DH_CTRL_REG5, 0x80));
    /* LIS3DH_CTRL_REG4_BLOCKDATAUPDATE   Enable block update */
    //  LIS3DH_CTRL_REG4_SCALE_2G));        /* +/-2G measurement range */
#ifdef TESTFIFO
    // Settings for CTRL_REG5:
    //   ----------------------------------------------------------------------------------------
    //CTRL_REG5| 7    6     5      4   3    2    1      0  |
    //    | BOOT   FIFO_EN     --   --      LIR_INT1    D4D_INT1      0   0  |
    //    ----------------------------------------------------------------------------------------
    ASSERT(LIS3DH_WriteReg( LIS3DH_CTRL_REG5 ,
                            BIT_6 |   /*Enable FIFO*/
                            BIT_3  /*lATCH INTERRUPT request on INT1_SRC register, with INT1_SRC register cleared by reading INT1_SRC itself*/

                          ));

    //---TESTING CLICK WITH CTRL_REG_6:
    // Settings for CTRL_REG6:
    //     ---------------------------------------------------------------------------------------------
    //CTRL_REG6| 7    | 6   | 5      | 4     | 3    | 2   | 1     | 0  |
    //    | I2_CLICKen  | I2_INT1   | 0   | BOOT_I1 | 0       |      | H_L-ACTIVE  |   -- |
    //   ---------------------------------------------------------------------------------------------

    ASSERT(LIS3DH_WriteReg(LIS3DH_CTRL_REG6 ,
                           BIT_7 | BIT_6)); /* enable tap and double tap interruppts on INT2 pin*/

    //   ----------------------------------------------------------------------------------------
    //CLICK_CFG| 7    6     5      4   3    2     1      0  |
    //   | --    --   ZD(DOUB) ZS(SING) YD     YS   XD     XS |
    //    ----------------------------------------------------------------------------------------
    ASSERT(LIS3DH_WriteReg( LIS3DH_CLICK_CFG ,
                            BIT_5));//Enable interrupt double CLICK-CLICK on Z axis  enable interrupt request on measured accel. value higher than preset threshold

    //CLICK_THS
    //Threshold for click detection (arbitary value) 1LSb = fullscale/128

    ASSERT(LIS3DH_WriteReg( LIS3DH_CLICK_THS , 0x03 ));

    //TIME_LIMIT
    //Time limit a short time frame where in which click is recognised when
    //1.corresponding value in that axis exceeds threshold and
    // 2.comes below threshold.
    // To get detected both should happen in this time limit.
    ASSERT(LIS3DH_WriteReg( LIS3DH_TIME_LIMIT , 0x33 ));

    ASSERT(LIS3DH_WriteReg( LIS3DH_TIME_LATENCY , 0xFF ));



    /* Enable FIFO */
    ASSERT(LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG,
                           BIT_6 /* FIFO MODE, trigger signal on INT1 */
                          ));

    /* Enable interrupt */
    ASSERT(LIS3DH_WriteReg(LIS3DH_INT1_CFG,
                           BIT_6 /* FIFO MODE, trigger signal on INT1 */
                          ));
#endif
    // Settings for INT1_CFG
    // defaults

    // Settings for INT1_THS
    // defaults

    // Settings for INT1_DUR
    // Defaults

    return true;
}
/**************************************************************************/
/*!
    @brief  Initialises the SPI block
    1+196 bytes
  197*8 = 1576bits
  1576 /(8*1024*1024)=0.188ms
 
*/
/**************************************************************************/
#define TESTFIFO 1
#if 0
bool LIS3DH_test()
{
    uint8_t index;
    uint8_t values[256];
    uint32_t pg_size;
    //ASSERT(LIS3DH_WriteReg(LIS3DH_CTRL_REG2,0));
    while(1) {
        //ASSERT(LIS3DH_WriteReg(LIS3DH_CTRL_REG2,ii));
        for(index = 0;index <20; index ++) {
            ASSERT(LIS3DH_ReadReg(LIS3DH_CTRL_REG1+index,values+index));
            //simple_uart_put(values[index]);
        }
        for(pg_size = 0xfffff; pg_size > 0; pg_size --)
            ;
        /*
         for(index = 0;index <6; index ++)
          {
            ASSERT(LIS3DH_ReadReg(LIS3DH_OUT_X_L+index,acc+index));  
            for(pg_size = 0xffff; pg_size > 0; pg_size --);   
          }
          simple_uart_put(0x55);
          simple_uart_put(acc[1]);
          simple_uart_put(acc[3]);
          simple_uart_put(acc[5]);
           */

    }
    return true;

}
#endif
/******************* (C) COPYRIGHT 2013 Bidu *****END OF FILE****/
