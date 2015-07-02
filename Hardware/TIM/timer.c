/**
  ******************************************************************************
  * @file    Hardware/TIM
  * @author  ��һ��
  * @version V3.5.0
  * @date    2014-10-4
  * @brief   ��ʱ���ж�
  ******************************************************************************
  * @attention
  * TIM2�ж�
  *	2015-07-02 21:45:10
  * �Ȳ�ʹ���жϸ�����TIM2������Ῠ�ڷ���UART2���ݵ���ѭ���У����ǽ�RoboSendData
  *	��������ѭ���ĳɾ���ʱ��
  ******************************************************************************
  */  
  
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */
#include "TIM/timer.h"
#include "PID/PID.h"
#include "Encoder/Encoder.h"
#include "usart/usart.h"
#include "LED/led.h"
#include "RoboModule/robomodule.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
  *@brief   Timer2Init      ��ʱ��1��ʼ��	
  *@param1  arr:�Զ���װ��ֵ
  *@param2  psc:ʱ��Ԥ��Ƶ��
  *@retval  None
  *@attention   Tout=(arr*psc)/Tclk(us)
  */
void Timer2Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʱ��ʹ��
	
	//��ʱ��TIM3��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr-1; 					//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc-1; 				//����������ΪTIM2ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM2���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 			//����ָ���Ĳ�����ʼ��TIM2��ʱ�������λ
 
	

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  			//TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  	//��ռ���ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  		//�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 			//IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  							//��ʼ��NVIC�Ĵ���


	/* ��ʹ��TIM2���ڰ����ж���ʹ�� ----------------------------------------*/
	TIM_Cmd(TIM2, DISABLE);  						//��ʹ��TIM2					 

	TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE ); 		//ʹ��ָ����TIM2�ж�,��������ж�

}
 
/****
    *@brief     TIM2_IRQHandler	��ʱ��2�жϷ�����
    *@param     None
    *@retval    None
    */
void TIM2_IRQHandler(void)
{
	double error;		//��ǰƫ����
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //���TIM2�����жϷ������
		{
			/* �жϴ������� -------------------------------------------------------------------------------------------*/
			EncoderUpdata(&PendBarEncoder);		//��ȡ��������ֵ��ðڸ�λ��
			
			if(PendBarEncoder.Position >= 600 && PendBarEncoder.Position <= 1000){	//��λ�����ڵ��ڷ�Χ��
				error = 800 - PendBarEncoder.Position;								//����ƫ��
				PIDCalculater(&PendPID,error);										//����ƫ�����PIDout
				printf("%d\t%f\t%d\n",PendBarEncoder.Position,PendPID.PIDout,(short)PendPID.PIDout);	//��ӡ��ǰ�ڸ�λ��\PIDout\���뵽���������PIDout					

				PIDControl(&PendPID);												//����PIDout����Robomoudule�������				
			}
			else{ 							//���ڵ��ڷ�Χ֮��
				PIDParamInit(&PendPID);		//PIDout=0
				PIDControl(&PendPID);		//���ֹͣ
			}
			/* END ---------------------------------------------------------------------------------------------------*/
			
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //���TIMx�����жϱ�־ 
		}
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
