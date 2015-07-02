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
	/* 摆杆PID结构体参数初始化 ----------*/
	PIDParamInit(&PendPID);	//数据初始化
	PendPID.Kp = 0.01;			//给参数赋值
	PendPID.Ki = 0;
	PendPID.Kd = 0;
	PendPID.PIDout_H = 30;		//PIDout高阈值设置
	
	/* 摆杆编码器参数初始化 ------------*/
	PendBarEncoder.Timer = TIM3;
	PendBarEncoder.Line = 400;
}

/**
  *@brief   Initial
  *@param   None;
  *@retval  None
  */
void Initial(void)
{
	SetParam();

    delay_init();				//延时初始化
    uart_init(115200);			//串口初始化
	LEDInit();					//最小系统板上的LED初始化

	Timer2Init(2000,720);			//72 000 000/(2 000*720) = 50 Hz
	
	EXTIInit();						//按键中断初始化
	EncoderInit(&PendBarEncoder);	//摆杆编码器初始化
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
		/*flash LED 指示程序运行---------------------------------*/
		RLED= 0;			//RLED 灭
		delay_ms(500);		//延时1s	
		
		RLED = 1;			//RLED亮
		delay_ms(500);		//延时1s
    }
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
