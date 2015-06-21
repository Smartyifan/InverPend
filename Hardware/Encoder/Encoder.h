/**
  ******************************************************************************
  * @file    
  * @author  ��һ��
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
#define ENC_CW      0x0010       //˳ʱ�뷽��
#define ENC_AntiCW  0x0000       //��ʱ�뷽��

typedef struct EncoderStruct{
    TIM_TypeDef * Timer;
    u8 TIMx_PRE_EMPTION_PRIORITY;   //�ж����ȼ�
    u8 TIMx_SUB_PRIORITY;           //�����ȼ�
    u16 Line;           //����
    u16 sample_ms;      //�������  (ms)

    GPIO_TypeDef * GPIOBase;
    u16 TI1;
    u16 TI2;
    
    bool FindMechOrig;  //�ҵ���е���ı�־λ
    u8 Dir;             //����
    s32 Turns;          //Ȧ��
    double AngAcce;      //�Ǽ��ٶ�
    double Palstance;    //���ٶ�
    double LastPalstance;//��һ�εĽ��ٶ�
    u16 Position;     //λ��
    double LastPosition; //��һ�ε�λ��
}EncoderStruct;
/*extern function-------------------------------------------------------------*/
extern EncoderStruct PendBarEncoder;   //�ڸ�λ�ñ�����

extern void EncoderInit(EncoderStruct * Encoder);
extern void EncoderUpdata(EncoderStruct * Encoder);
#endif
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
