/**
  ******************************************************************************
  * @file    E:\GitRepo\InverPend\Hardware\PID\PID.c
  * @author  贾一帆
  * @version V3.5.0
  * @date    2015-07-02 16:01:43
  * @brief   通用增量式PID计算
  ******************************************************************************
  * @attention
  *	使用步骤：
  *		1.定义相应结构体并使用PIDParamInit初始化相关参数
  *		2.在使用前先给Kp\Ki\Kd赋值
  *		3.通过PIDCalculater得到PIDout输出
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
PIDStruct PendPID;		//摆杆PID控制结构体
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
    PID->PIDout = 0;	//PIDout初始化为0	
    
	/* errors Init------------*/
    PID->error[0] = 0;	//清空偏差数组
	PID->error[1] = 0;
    PID->error[2] = 0;
    
}

/**
  *@brief   PIDCalculater   增量式PID计算
  *@param   PIDStruct * PID	指向PID结构体的指针
  *@retval  None
  */
void PIDCalculater(PIDStruct * PID,double error){  
	
	/* 得到偏差量 -----------------------------------------------------*/
    PIDGetError(PID,error);		//获得偏差并更新偏差量数组
	
	/* 计算PID输出 ----------------------------------------------------*/
    PID->Pout = PID->Kp*(PID->error[0] - PID->error[1]);	//计算Pout
    
    PID->Iout = PID->Ki*PID->error[0];						//计算Iout
   
    PID->Dout = PID->Kd*(PID->error[0] + PID->error[2] -2*PID->error[1]);	//计算Dout
	
    PID->PIDout += PID->Pout + PID->Iout + PID->Dout;		//得到PIDout

    /* 高低阈值限制 --------------------------------------------------*/
    if(abs(PID->PIDout) > PID->PIDout_H)PID->PIDout = PID->PIDout_H;		//PIDout最高值限制
//     if(PID->PIDout < PID->PIDout_L)PID->PIDout = PID->PIDout_L;
}

/**
  *@brief   PIDGetError
  *@param   PIDStruct * PID	指向PID结构体的指针
  *			double error	当前偏差量			
  *@retval  None
  */
void PIDGetError(PIDStruct * PID,double error){
	PID->error[2] = PID->error[1];	//数组移位
	PID->error[1] = PID->error[0];
    PID->error[0] = error;
}

/**
  *@brief   PIDControl	根据PIDout控制外部设备，内容根据实际情况书写
  *@param   PIDStruct * PID	指向PID结构体的指针	
  *@retval    None
  */
void PIDControl(PIDStruct * PID){
	RoboModule_Driver_Speed_Mode_Set(4000,(short)PID->PIDout);		//根据PIDout控制Robomodule电机驱动
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
