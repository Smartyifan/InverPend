/**
  ******************************************************************************
  * @file    Hardware/TIM
  * @author  贾一帆
  * @version V3.5.0
  * @date    2014-10-4
  * @brief   定时器中断
  ******************************************************************************
  * @attention
  * TIM3
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
  *@brief   Timer3Init      定时器3初始化	
  *@param1  arr:自动重装载值
  *@param2  psc:时钟预分频数
  *@retval  None
  *@attention   Tout=(arr*psc)/Tclk(us)
  */
void Timer1Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef         NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);           
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;           
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;     
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           
    NVIC_Init(&NVIC_InitStructure); 

    TIM_TimeBaseStructure.TIM_Period = psc-1; 
    TIM_TimeBaseStructure.TIM_Prescaler =arr-1;  
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 
    TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM1, ENABLE);  
}
 
/****
    *@brief     TIM1_IRQHandler
    *@param     None
    *@retval    None
    */
void TIM1_UP_IRQHandler(void)
{
    if ( TIM_GetITStatus(TIM1 , TIM_IT_Update) != RESET ) 
    { 		
        TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);    
    }    
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
