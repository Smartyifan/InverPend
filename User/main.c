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
	/* �ڸ�PID�ṹ�������ʼ�� ----------*/
	PIDParamInit(&PendPID);	//���ݳ�ʼ��
	PendPID.Kp = 0.01;			//��������ֵ
	PendPID.Ki = 0;
	PendPID.Kd = 0;
	PendPID.PIDout_H = 30;		//PIDout����ֵ����
	
	/* �ڸ˱�����������ʼ�� ------------*/
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

    delay_init();				//��ʱ��ʼ��
    uart_init(115200);			//���ڳ�ʼ��
	LEDInit();					//��Сϵͳ���ϵ�LED��ʼ��

	Timer2Init(2000,720);			//72 000 000/(2 000*720) = 50 Hz
	
	EXTIInit();						//�����жϳ�ʼ��
	EncoderInit(&PendBarEncoder);	//�ڸ˱�������ʼ��
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
		/*flash LED ָʾ��������---------------------------------*/
		RLED= 0;			//RLED ��
		delay_ms(500);		//��ʱ1s	
		
		RLED = 1;			//RLED��
		delay_ms(500);		//��ʱ1s
    }
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
