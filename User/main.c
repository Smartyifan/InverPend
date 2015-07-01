/**
  ******************************************************************************
  * @file    STM32工程/工程模版/User
  * @author  贾一帆
  * @version V3.5.0
  * @date    2014-10-4
  * @brief   Main program body
  ******************************************************************************
  * @attention
  * No attention
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"


/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */
#include "sys/sys.h"  
#include "delay/delay.h"
#include "usart/usart.h"

#include "Encoder/Encoder.h"
#include "PID/PID.h"
#include "TIM/timer.h"
#include "LED/led.h"
#include "RoboModule/robomodule.h"
#include "EXTI/exti.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


void SetParam(void){
	
}

/**
  *@brief   Initial
  *@param   None;
  *@retval  None
  */
void Initial()
{
	SetParam();

    delay_init();				//延时初始化
    uart_init(115200);			//串口初始化
	
	LEDInit();					//最小系统板上的LED初始化
	EXTIInit();					//最小系统板按键中断的初始化
	RoboInit(ENTER_LOCATION_MODE);	//初始化RoboModule并进入位置模式
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
    Initial();
    while(1)
    {
		/*flash LED -----------------------------------------*/
		RLED= 0;			//RLED 灭
		delay_ms(1000);		//延时1s																	
		
		RLED = 1;			//RLED亮
		delay_ms(1000);		//延时1s
    }
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
