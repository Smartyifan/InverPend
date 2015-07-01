/**
  ******************************************************************************
  * @file    EXTI/exti
  * @author  贾一帆
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
  *@brief   EXTIInit    外部中断初始化	
  *@param   None
  *@retval  None
  *@attention 使用之前要对相应外设初始化 此处为按键
  */
void EXTIInit(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;    //初始化结构体定义
    NVIC_InitTypeDef NVIC_InitStructure;
    
    KeyInit();      //按键初始化
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); //使能引脚复用时钟
    
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);     	//PA.0 KEY0
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;                     	//0线中断
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;             //中断事件
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;         //下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                       //使能
    EXTI_Init(&EXTI_InitStructure);                                 //初始化中断
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;            	//0线中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;       //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;       //子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);                                 //优先级初始化
}
 
/**
  *@brief   EXTI15_10_IRQHandler        13线中断服务函数
  *@param   None
  *@retval  None
  */
void EXTI0_IRQHandler(void)
{
    delay_ms(200);
    if(KEY0 == 0){
		GLED = !GLED;
	    RoboModule_Driver_Location_Mode_Set(2500,1000,500);		//电机到500线位置
		
		/* 复位 -----------------------------------------------------*/
// 		RoboModule_Driver_Reset();
// 		delay_ms(1500);					//等待1.5s钟
// 		
// 		/* 模式选择 -------------------------------------------------*/
// 		RoboModule_Driver_Mode_Chioce(ENTER_SPEED_MODE);
// 		delay_ms(600);					//等待600ms
// 		
// 		/* Speed = 0 ------------------------------------------------*/
// 		RoboModule_Driver_Speed_Mode_Set(4000,0);
	}
    EXTI_ClearITPendingBit(EXTI_Line13);    //清除13线中断
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
