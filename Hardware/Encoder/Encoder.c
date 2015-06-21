/**
  ******************************************************************************
  * @file    路径
  * @author  贾一帆
  * @version V3.5.0
  * @date    Date
  * @brief   Blank
  ******************************************************************************
  * @attention
  * 使用TIM3中断
  ******************************************************************************
  */  
  
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

#include "Encoder/Encoder.h"
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
EncoderStruct PendBarEncoder;   //摆杆位置编码器
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
void EncoderGPIOInit(EncoderStruct * Encoder);
void EncoderNVICInit(EncoderStruct * Encoder);
void EncoderTIMInit(EncoderStruct * Encoder);

/**
  *@brief   EncoderInit
  *@param   None;
  *@retval  None
  */
void EncoderInit(EncoderStruct * Encoder){
    EncoderGPIOInit(Encoder);
    EncoderTIMInit(Encoder);
    
//     EncoderNVICInit(Encoder);
    
    Encoder->Dir = ENC_CW;
    Encoder->Palstance = 0;
    Encoder->Position = 0;
    Encoder->LastPosition = 0;
    Encoder->LastPalstance = 0;
    Encoder->AngAcce = 0;
    Encoder->Turns = 0;
    Encoder->FindMechOrig = FALSE;
    
    TIM_Cmd(Encoder->Timer, ENABLE);  
}
 
/**
  *@brief   EncoderGPIOInit
  *@param   None;
  *@retval  None
  */
void EncoderGPIOInit(EncoderStruct * Encoder){
    GPIO_InitTypeDef GPIO_InitStructure;

    if(Encoder->Timer == TIM2){
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        Encoder->GPIOBase = GPIOA;
        Encoder->TI1 = GPIO_Pin_0;
        Encoder->TI2 = GPIO_Pin_1;
    }else if(Encoder->Timer == TIM3){
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        Encoder->GPIOBase = GPIOA;
        Encoder->TI1 = GPIO_Pin_6;
        Encoder->TI2 = GPIO_Pin_7;
    }else if(Encoder->Timer == TIM4){
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        Encoder->GPIOBase = GPIOB;
        Encoder->TI1 = GPIO_Pin_6;
        Encoder->TI2 = GPIO_Pin_7;
    }
    
    //设置为上拉输入模式
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = Encoder->TI1 | Encoder->TI2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		
    GPIO_Init(Encoder->GPIOBase, &GPIO_InitStructure);
}

/**
  *@brief   EncoderNVICInit
  *@param   None;
  *@retval  None
  */
void EncoderNVICInit(EncoderStruct * Encoder){
    NVIC_InitTypeDef NVIC_InitStructure;
    
    if(Encoder->Timer == TIM2){
        NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    }else if(Encoder->Timer == TIM3){
        NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    }else if(Encoder->Timer == TIM4){
        NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    }
    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = Encoder->TIMx_PRE_EMPTION_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = Encoder->TIMx_SUB_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    TIM_ClearFlag(Encoder->Timer, TIM_FLAG_Update);
    TIM_ITConfig(Encoder->Timer, TIM_IT_Update, ENABLE);
}

/**
  *@brief   EncoderTIMInit
  *@param   None;
  *@retval  None
  */
void EncoderTIMInit(EncoderStruct * Encoder){
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    if(Encoder->Timer == TIM2){
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    }else if(Encoder->Timer == TIM3){
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    }else if(Encoder->Timer == TIM4){
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    }
    
    TIM_DeInit(Encoder->Timer);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
    TIM_TimeBaseStructure.TIM_Period = 4*Encoder->Line-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
    TIM_TimeBaseInit(Encoder->Timer, &TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(Encoder->Timer, TIM_EncoderMode_TI12, 
                             TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInit(Encoder->Timer, &TIM_ICInitStructure);
    
    Encoder->Timer->CNT = 0;
}

/**
  *@brief   EncoderUpdata
  *@param   None;
  *@retval  None
  */
void EncoderUpdata(EncoderStruct * Encoder){
    Encoder->Dir = 0x0010 & Encoder->Timer->CR1;            //读方向
    Encoder->Position = TIM_GetCounter(Encoder->Timer);    //读角度 0~1600
}
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
