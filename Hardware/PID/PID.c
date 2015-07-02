/**
  ******************************************************************************
  * @file    E:\GitRepo\InverPend\Hardware\PID\PID.c
  * @author  ��һ��
  * @version V3.5.0
  * @date    2015-07-02 16:01:43
  * @brief   ͨ������ʽPID����
  ******************************************************************************
  * @attention
  *	ʹ�ò��裺
  *		1.������Ӧ�ṹ�岢ʹ��PIDParamInit��ʼ����ز���
  *		2.��ʹ��ǰ�ȸ�Kp\Ki\Kd��ֵ
  *		3.ͨ��PIDCalculater�õ�PIDout���
  ******************************************************************************
  */  
  
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdlib.h>
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */
#include "PID/PID.h"
#include "usart/usart.h"
#include "RoboModule/robomodule.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
PIDStruct PendPID;		//�ڸ�PID���ƽṹ��
/* Private function prototypes -----------------------------------------------*/
void PIDGetError(PIDStruct * PID,double error);
/* Private functions ---------------------------------------------------------*/
/**
  *@brief   PIDCalculatre
  *@param   None;
  *@retval  None
  */
void PIDParamInit(PIDStruct * PID){
	
	/* PIDout Init------------*/
    PID->PIDout = 0;	//PIDout��ʼ��Ϊ0	
    
	/* errors Init------------*/
    PID->error[0] = 0;	//���ƫ������
	PID->error[1] = 0;
    PID->error[2] = 0;
    
}

/**
  *@brief   PIDCalculater   ����ʽPID����
  *@param   PIDStruct * PID	ָ��PID�ṹ���ָ��
  *@retval  None
  */
void PIDCalculater(PIDStruct * PID,double error){  
	
	/* �õ�ƫ���� -----------------------------------------------------*/
    PIDGetError(PID,error);		//���ƫ�����ƫ��������
	
	/* ����PID��� ----------------------------------------------------*/
    PID->Pout = PID->Kp*(PID->error[0] - PID->error[1]);	//����Pout
    
    PID->Iout = PID->Ki*PID->error[0];						//����Iout
   
    PID->Dout = PID->Kd*(PID->error[0] + PID->error[2] -2*PID->error[1]);	//����Dout
	
    PID->PIDout += PID->Pout + PID->Iout + PID->Dout;		//�õ�PIDout

    /* �ߵ���ֵ���� --------------------------------------------------*/
    if(abs(PID->PIDout) > PID->PIDout_H)PID->PIDout = PID->PIDout_H;		//PIDout���ֵ����
//     if(PID->PIDout < PID->PIDout_L)PID->PIDout = PID->PIDout_L;
}

/**
  *@brief   PIDGetError
  *@param   PIDStruct * PID	ָ��PID�ṹ���ָ��
  *			double error	��ǰƫ����			
  *@retval  None
  */
void PIDGetError(PIDStruct * PID,double error){
	PID->error[2] = PID->error[1];	//������λ
	PID->error[1] = PID->error[0];
    PID->error[0] = error;
}

/**
  *@brief   PIDControl	����PIDout�����ⲿ�豸�����ݸ���ʵ�������д
  *@param   PIDStruct * PID	ָ��PID�ṹ���ָ��	
  *@retval    None
  */
void PIDControl(PIDStruct * PID){
	RoboModule_Driver_Speed_Mode_Set(4000,(short)PID->PIDout);		//����PIDout����Robomodule�������
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
