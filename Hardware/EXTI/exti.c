/**
  ******************************************************************************
  * @file    E:\GitRepo\InverPend\Hardware\EXTI\exti.c
  * @author  贾一帆
  * @version V3.5.0
  * @date    2015-07-02 19:42:16
  * @brief   按键中断程序
  ******************************************************************************
  * @attention
  * PA.0 下降沿中断
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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;       //子优先级
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
    delay_ms(100);
    if(KEY0 == 0){
		/* 中断处理部分 ----------------------------------------------------------------------------*/
		GLED = 0;							//GLED 点亮 
		RoboInit(ENTER_SPEED_MODE);			//进入速度环控制模式 		
		TIM_Cmd(TIM2, ENABLE);  						//使能TIM2					 
		TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); 		//使能指定的TIM2中断,允许更新中断
		while(KEY0 == 0);					//等待KEY0松开
		GLED = 1;							//GLED 熄灭
		/* END -------------------------------------------------------------------------------------*/
	}
    EXTI_ClearITPendingBit(EXTI_Line0);    //清除0线中断
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
