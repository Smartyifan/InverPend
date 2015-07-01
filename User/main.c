/**
  ******************************************************************************
  * @file    STM32����/����ģ��/User
  * @author  ��һ��
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

    delay_init();				//��ʱ��ʼ��
    uart_init(115200);			//���ڳ�ʼ��
	
	LEDInit();					//��Сϵͳ���ϵ�LED��ʼ��
	EXTIInit();					//��Сϵͳ�尴���жϵĳ�ʼ��
	RoboInit(ENTER_LOCATION_MODE);	//��ʼ��RoboModule������λ��ģʽ
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
		RLED= 0;			//RLED ��
		delay_ms(1000);		//��ʱ1s																	
		
		RLED = 1;			//RLED��
		delay_ms(1000);		//��ʱ1s
    }
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
