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
    /*PendBarEncoder---------*/
    PendBarEncoder.Timer = TIM3;    //TIM3  PA.6    PA.7
    PendBarEncoder.Line = 400;
    PendBarEncoder.sample_ms = 10;

	/*Pending PID*/
    PIDParamInit(&PendPID);
	PendPID.Kp = 0.03;
    PendPID.Kd = 0;

}

/**
  *@brief   Initial
  *@param   None;
  *@retval  None
  */
void Initial()
{
	SetParam();
	NVIC_Configuration();		//�жϳ�ʼ��

    delay_init();				//��ʱ��ʼ��
    uart_init(115200);			//���ڳ�ʼ��
	
	EncoderInit(&PendBarEncoder);	//��������ʼ��
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
		/*print position of PendBar --------------------------*/
// 		EncoderUpdata(&PendBarEncoder);
		PendBarEncoder.Position = TIM_GetCounter(TIM3);    //���Ƕ� 0~4000

		printf("%d\n",PendBarEncoder.Position);
		delay_ms(1000);

		/*flash LED -----------------------------------------*/
// 		RLED= 0;
// 		delay_ms(1000);
// 		RLED = 1;
// 		delay_ms(1000);
    }
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
