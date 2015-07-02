/**
  ******************************************************************************
  * @file    Hardware/TIM
  * @author  贾一帆
  * @version V3.5.0
  * @date    2014-10-4
  * @brief   定时器中断
  ******************************************************************************
  * @attention
  * TIM2中断
  *	2015-07-02 21:45:10
  * 先不使能中断更新与TIM2，否则会卡在发送UART2数据的死循环中，考虑将RoboSendData
  *	函数的死循环改成具有时限
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
  *@brief   Timer2Init      定时器1初始化	
  *@param1  arr:自动重装载值
  *@param2  psc:时钟预分频数
  *@retval  None
  *@attention   Tout=(arr*psc)/Tclk(us)
  */
void Timer2Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能
	
	//定时器TIM3初始化
	TIM_TimeBaseStructure.TIM_Period = arr-1; 					//设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc-1; 				//设置用来作为TIM2时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM2向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 			//根据指定的参数初始化TIM2的时间基数单位
 
	

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  			//TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  	//先占优先级1级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  		//从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 			//IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  							//初始化NVIC寄存器


	/* 不使能TIM2，在按键中断中使能 ----------------------------------------*/
	TIM_Cmd(TIM2, DISABLE);  						//不使能TIM2					 

	TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE ); 		//使能指定的TIM2中断,允许更新中断

}
 
/****
    *@brief     TIM2_IRQHandler	定时器2中断服务函数
    *@param     None
    *@retval    None
    */
void TIM2_IRQHandler(void)
{
	double error;		//当前偏差量
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //检查TIM2更新中断发生与否
		{
			/* 中断处理内容 -------------------------------------------------------------------------------------------*/
			EncoderUpdata(&PendBarEncoder);		//读取编码器数值获得摆杆位置
			
			if(PendBarEncoder.Position >= 600 && PendBarEncoder.Position <= 1000){	//若位置落在调节范围内
				error = 800 - PendBarEncoder.Position;								//计算偏差
				PIDCalculater(&PendPID,error);										//根据偏差计算PIDout
				printf("%d\t%f\t%d\n",PendBarEncoder.Position,PendPID.PIDout,(short)PendPID.PIDout);	//打印当前摆杆位置\PIDout\输入到电机驱动的PIDout					

				PIDControl(&PendPID);												//根据PIDout控制Robomoudule电机驱动				
			}
			else{ 							//若在调节范围之外
				PIDParamInit(&PendPID);		//PIDout=0
				PIDControl(&PendPID);		//电机停止
			}
			/* END ---------------------------------------------------------------------------------------------------*/
			
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIMx更新中断标志 
		}
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
