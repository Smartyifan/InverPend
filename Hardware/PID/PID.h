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
  
#ifndef _PID_H
#define _PID_H
/* Includes ------------------------------------------------------------------*/
#include "sys/sys.h"
/* Define --------------------------------------------------------------------*/
#define I_Integral 20   //I的积分域

typedef struct PIDStruct{
    double Kp;  //PID 参数 
    double Ti;  
    double Td;
    double T;   
    
    double Ki;
    double Kd;
    
    double error[3];      
        
    double Pout;
    double Iout;
    double Dout;
    
    double PIDout;
    double PIDout_H;
    double PIDout_L;
}PIDStruct;
/*extern function-------------------------------------------------------------*/
extern PIDStruct PendPID;       //摆杆位置式PID

extern void PIDParamInit(PIDStruct * PID);
extern void PIDGetError(PIDStruct * PID,double error);
extern void PIDCalculatre(PIDStruct * PID);

#endif
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
