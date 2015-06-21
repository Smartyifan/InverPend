/**
  ******************************************************************************
  * @file    路径
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
#include "PID/PID.h"
#include "usart/usart.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
PIDStruct MotorSpeedPID; //位置式PID
PIDStruct PendPID;
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
  *@brief   PIDCalculatre
  *@param   None;
  *@retval  None
  */
void PIDParamInit(PIDStruct * PID){
    PID->PIDout = 0;
    
    PID->error[0] = 0;
	PID->error[1] = 0;
    PID->error[2] = 0;
	
    
	PID->Kp = 0;  //PID 参数     
	PID->Ti = 0;  
    
	PID->Td = 0;
    PID->T = 0;   
	
	PID->Kp = 0; 
    PID->Ki = 0;
    PID->Kd = 0;
}

/**
  *@brief   PIDCalculatre   位置式PID计算
  *@param   None;
  *@retval  
  */
void PIDCalculatre(PIDStruct * PID){  
    
    PID->Pout = PID->Kp*(PID->error[2] - PID->error[1]);
    
    PID->Iout = PID->Ki*PID->error[2];
   
    PID->Dout = PID->Kd*(PID->error[2] + PID->error[0] -2*PID->error[1]);
	
    PID->PIDout += PID->Pout + PID->Iout - PID->Dout;

    //高低阈值限制
//     if(PID->PIDout > PID->PIDout_H)PID->PIDout = PID->PIDout_H;
//     if(PID->PIDout < PID->PIDout_L)PID->PIDout = PID->PIDout_L;
}
 
/**
  *@brief   PIDGetError
  *@param   None;
  *@retval  None
  */
void PIDGetError(PIDStruct * PID,double error){
	PID->error[0] = PID->error[1];
	PID->error[1] = PID->error[2];
    PID->error[2] = error;
}
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
