/**
  ******************************************************************************
  * @file    
  * @author  贾一帆
  * @version V3.5.0
  * @date    2014-10-4
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */  
  
#ifndef _ENCODER_H
#define _ENCODER_H
/* Includes ------------------------------------------------------------------*/

#include "sys/sys.h"
/* Define --------------------------------------------------------------------*/
#define ENC_CW      0x0010       //顺时针方向
#define ENC_AntiCW  0x0000       //逆时针方向

typedef struct EncoderStruct{
    TIM_TypeDef * Timer;
    u8 TIMx_PRE_EMPTION_PRIORITY;   //中断优先级
    u8 TIMx_SUB_PRIORITY;           //子优先级
    u16 Line;           //线数
    u16 sample_ms;      //采样间隔  (ms)

    GPIO_TypeDef * GPIOBase;
    u16 TI1;
    u16 TI2;
    
    bool FindMechOrig;  //找到机械零点的标志位
    u8 Dir;             //方向
    s32 Turns;          //圈数
    double AngAcce;      //角加速度
    double Palstance;    //角速度
    double LastPalstance;//上一次的角速度
    u16 Position;     //位置
    double LastPosition; //上一次的位置
}EncoderStruct;
/*extern function-------------------------------------------------------------*/
extern EncoderStruct PendBarEncoder;   //摆杆位置编码器

extern void EncoderInit(EncoderStruct * Encoder);
extern void EncoderUpdata(EncoderStruct * Encoder);
#endif
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
