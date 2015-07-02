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

typedef struct PIDStruct{          
	double Kp;			//计算参数
    double Ki;
    double Kd;
    
    double error[3];    //存放偏差量的数组  
        
    double Pout;
    double Iout;
    double Dout;
    
    double PIDout;		//PIDout
    double PIDout_H;	//PIDout阈值限制
    double PIDout_L;
}PIDStruct;
/*extern function-------------------------------------------------------------*/
extern PIDStruct PendPID;       //摆杆位置式PID

extern void PIDParamInit(PIDStruct * PID);
extern void PIDCalculater(PIDStruct * PID,double error);
extern void PIDControl(PIDStruct * PID);
#endif
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
