/**
  ******************************************************************************
  * @file    E:\GitRepo\InverPend\Hardware\EXTI\exti.c
  * @author  ��һ��
  * @version V3.5.0
  * @date    2015-07-02 19:42:16
  * @brief   �����жϳ���
  ******************************************************************************
  * @attention
  * PA.0 �½����ж�
  ******************************************************************************
  */  
  
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */
#include "delay/delay.h"  
  
#include "KEY/key.h"
#include "EXTI/exti.h"
#include "RoboModule/robomodule.h"
#include "LED/led.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
  *@brief   EXTIInit    �ⲿ�жϳ�ʼ��	
  *@param   None
  *@retval  None
  *@attention ʹ��֮ǰҪ����Ӧ�����ʼ�� �˴�Ϊ����
  */
void EXTIInit(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;    //��ʼ���ṹ�嶨��
    NVIC_InitTypeDef NVIC_InitStructure;
    
    KeyInit();      //������ʼ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); //ʹ�����Ÿ���ʱ��
    
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);     	//PA.0 KEY0
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;                     	//0���ж�
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;             //�ж��¼�
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;         //�½��ش���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                       //ʹ��
    EXTI_Init(&EXTI_InitStructure);                                 //��ʼ���ж�
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;            	//0���ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;       //�����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);                                 //���ȼ���ʼ��
}
 
/**
  *@brief   EXTI15_10_IRQHandler        13���жϷ�����
  *@param   None
  *@retval  None
  */
void EXTI0_IRQHandler(void)
{
    delay_ms(100);
    if(KEY0 == 0){
		/* �жϴ����� ----------------------------------------------------------------------------*/
		GLED = 0;							//GLED ���� 
		RoboInit(ENTER_SPEED_MODE);			//�����ٶȻ�����ģʽ 		
		TIM_Cmd(TIM2, ENABLE);  						//ʹ��TIM2					 
		TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); 		//ʹ��ָ����TIM2�ж�,��������ж�
		while(KEY0 == 0);					//�ȴ�KEY0�ɿ�
		GLED = 1;							//GLED Ϩ��
		/* END -------------------------------------------------------------------------------------*/
	}
    EXTI_ClearITPendingBit(EXTI_Line0);    //���0���ж�
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
