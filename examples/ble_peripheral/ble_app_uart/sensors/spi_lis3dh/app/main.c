/****************************************Copyright (c)************************************************
**                                      [艾克姆科技]
**                                        IIKMSIK 
**                            官方店铺：https://acmemcu.taobao.com
**                            官方论坛：http://www.e930bbs.com
**                                   
**--------------File Info-----------------------------------------------------------------------------
** File name:			     main.c
** Last modified Date:          
** Last Version:		   
** Descriptions:		   使用的SDK版本-SDK_15.2
**						
**----------------------------------------------------------------------------------------------------
** Created by:			
** Created date:		2019-8-24
** Version:			    1.0
** Descriptions:		SPI读取LIS3DH加速度数据
**---------------------------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "boards.h"

#include "app_uart.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_drv_spi.h"
#include "lis3dh_drive.h"
/* 试验需要用到IK-52832DK开发板中的指示灯D1和UART，占用的nRF52832引脚资源如下
P0.17：输出：驱动指示灯D1
P0.18：输出：驱动指示灯D2
P0.19：输出：驱动指示灯D3
P0.20：输出：驱动指示灯D4

P0.06：输出：串口TXD
P0.08：输入：串口RXD

本实验需要接入LIS3DSH模块
CS   :P0.25
CLK  :P0.22
MISO :P0.24
MOSI :P0.23
需要用跳线帽短接P0.17 P0.18 P0.19 P0.20 P0.06 P0.08
*/

#define UART_TX_BUF_SIZE 256       //串口发送缓存大小（字节数）
#define UART_RX_BUF_SIZE 256       //串口接收缓存大小（字节数）


//串口事件回调函数，该函数中判断事件类型并进行处理
void uart_error_handle(app_uart_evt_t * p_event)
{
    //通讯错误事件
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    //FIFO错误事件
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}
//串口初始化配置：波特率115200bps，关闭硬件流控
void uart_config(void)
{
  uint32_t err_code;
	
	//定义串口通讯参数配置结构体并初始化
  const app_uart_comm_params_t comm_params =
  {
    RX_PIN_NUMBER,//定义uart接收引脚
    TX_PIN_NUMBER,//定义uart发送引脚
    RTS_PIN_NUMBER,//定义uart RTS引脚，注意流控关闭后虽然定义了RTS和CTS引脚，但是不起作用
    CTS_PIN_NUMBER,//定义uart CTS引脚
    APP_UART_FLOW_CONTROL_DISABLED,//关闭uart硬件流控
    false,//禁止奇偶检验
    NRF_UART_BAUDRATE_115200//uart波特率设置为115200bps
  };
  //初始化串口，注册串口事件回调函数
  APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

  APP_ERROR_CHECK(err_code);		
}
/***************************************************************************
* 描  述 : main函数 
* 入  参 : 无 
* 返回值 : int 类型
**************************************************************************/
int main(void)
{
	//定义一个结构体变量，用于保存从LIS3DH中读取的加速度数据
	AxesRaw_t data;
	
	//配置用于驱动LED指示灯D1 D2 D3 D4的引脚脚，即配置P0.13~P0.16为输出，并将LED的初始状态设置为熄灭 
	bsp_board_init(BSP_INIT_LEDS); 
	//初始化串口
  uart_config();
	
	//初始化SPI和LIS3DH
	LIS3DH_Init();
 		
  while(true)
  {
		//读取LIS3DH加速度数据，读取成功后，通过串口打印出数据
		if(LIS3DH_GetAccAxesRaw(&data))
	  {
		  printf("X=%6d Y=%6d Z=%6d \r\n", data.AXIS_X, data.AXIS_Y, data.AXIS_Z);
			nrf_gpio_pin_toggle(LED_1);	
	  }
		//延时300ms，方便观察读取的数据
		nrf_delay_ms(300);	
  }
}
/********************************************END FILE**************************************/
