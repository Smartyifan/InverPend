/**
  ******************************************************************************
  * @file    EXTI/exti
  * @author  ��һ��
  * @version V3.5.0
  * @date    Date
  * @brief   Blank
  ******************************************************************************
  * @attention
  * 
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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;       //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;       //�����ȼ�
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
    delay_ms(200);
    if(KEY0 == 0){
		GLED = !GLED;
	    RoboModule_Driver_Location_Mode_Set(2500,1000,500);		//�����500��λ��
		
		/* ��λ -----------------------------------------------------*/
// 		RoboModule_Driver_Reset();
// 		delay_ms(1500);					//�ȴ�1.5s��
// 		
// 		/* ģʽѡ�� -------------------------------------------------*/
// 		RoboModule_Driver_Mode_Chioce(ENTER_SPEED_MODE);
// 		delay_ms(600);					//�ȴ�600ms
// 		
// 		/* Speed = 0 ------------------------------------------------*/
// 		RoboModule_Driver_Speed_Mode_Set(4000,0);
	}
    EXTI_ClearITPendingBit(EXTI_Line13);    //���13���ж�
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
