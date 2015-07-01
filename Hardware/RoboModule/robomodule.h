/**
  ******************************************************************************
  * @file    
  * @author  贾一帆
  * @version V3.5.0
  * @date    2015-6-30
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */  
  
#ifndef _ROBOMODULE_H
#define _ROBOMODULE_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Define --------------------------------------------------------------------*/
#define Robo_ID_DRIVER_RESET_ALL      0x00 //广播
#define Robo_ID_DRIVER_RESET_NUM01    0x10
#define Robo_ID_DRIVER_RESET_NUM02    0x20
#define Robo_ID_DRIVER_RESET_NUM03    0x30
#define Robo_ID_DRIVER_RESET_NUM04    0x40
#define Robo_ID_DRIVER_RESET_NUM05    0x50
#define Robo_ID_DRIVER_RESET_NUM06    0x60
#define Robo_ID_DRIVER_RESET_NUM07    0x70
#define Robo_ID_DRIVER_RESET_NUM08    0x80
#define Robo_ID_DRIVER_RESET_NUM09    0x90
#define Robo_ID_DRIVER_RESET_NUM10    0xA0
#define Robo_ID_DRIVER_RESET_NUM11    0xB0
#define Robo_ID_DRIVER_RESET_NUM12    0xC0
#define Robo_ID_DRIVER_RESET_NUM13    0xD0
#define Robo_ID_DRIVER_RESET_NUM14    0xE0
#define Robo_ID_DRIVER_RESET_NUM15    0xF0

#define Robo_ID_MODE_CHOICE_ALL       0x01 //广播
#define Robo_ID_MODE_CHOICE_NUM01     0x11
#define Robo_ID_MODE_CHOICE_NUM02     0x21
#define Robo_ID_MODE_CHOICE_NUM03     0x31
#define Robo_ID_MODE_CHOICE_NUM04     0x41
#define Robo_ID_MODE_CHOICE_NUM05     0x51
#define Robo_ID_MODE_CHOICE_NUM06     0x61
#define Robo_ID_MODE_CHOICE_NUM07     0x71
#define Robo_ID_MODE_CHOICE_NUM08     0x81
#define Robo_ID_MODE_CHOICE_NUM09     0x91
#define Robo_ID_MODE_CHOICE_NUM10     0xA1
#define Robo_ID_MODE_CHOICE_NUM11     0xB1
#define Robo_ID_MODE_CHOICE_NUM12     0xC1
#define Robo_ID_MODE_CHOICE_NUM13     0xD1
#define Robo_ID_MODE_CHOICE_NUM14     0xE1
#define Robo_ID_MODE_CHOICE_NUM15     0xF1

#define Robo_ID_PWM_MODE_ALL          0x02 //广播
#define Robo_ID_PWM_MODE_NUM01        0x12
#define Robo_ID_PWM_MODE_NUM02        0x22
#define Robo_ID_PWM_MODE_NUM03        0x32
#define Robo_ID_PWM_MODE_NUM04        0x42
#define Robo_ID_PWM_MODE_NUM05        0x52
#define Robo_ID_PWM_MODE_NUM06        0x62
#define Robo_ID_PWM_MODE_NUM07        0x72
#define Robo_ID_PWM_MODE_NUM08        0x82
#define Robo_ID_PWM_MODE_NUM09        0x92
#define Robo_ID_PWM_MODE_NUM10        0xA2
#define Robo_ID_PWM_MODE_NUM11        0xB2
#define Robo_ID_PWM_MODE_NUM12        0xC2
#define Robo_ID_PWM_MODE_NUM13        0xD2
#define Robo_ID_PWM_MODE_NUM14        0xE2
#define Robo_ID_PWM_MODE_NUM15        0xF2

#define Robo_ID_CURRENT_MODE_ALL      0x03
#define Robo_ID_CURRENT_MODE_NUM01    0x13
#define Robo_ID_CURRENT_MODE_NUM02    0x23
#define Robo_ID_CURRENT_MODE_NUM03    0x33
#define Robo_ID_CURRENT_MODE_NUM04    0x43
#define Robo_ID_CURRENT_MODE_NUM05    0x53
#define Robo_ID_CURRENT_MODE_NUM06    0x63
#define Robo_ID_CURRENT_MODE_NUM07    0x73
#define Robo_ID_CURRENT_MODE_NUM08    0x83
#define Robo_ID_CURRENT_MODE_NUM09    0x93
#define Robo_ID_CURRENT_MODE_NUM10    0xA3
#define Robo_ID_CURRENT_MODE_NUM11    0xB3
#define Robo_ID_CURRENT_MODE_NUM12    0xC3
#define Robo_ID_CURRENT_MODE_NUM13    0xD3
#define Robo_ID_CURRENT_MODE_NUM14    0xE3
#define Robo_ID_CURRENT_MODE_NUM15    0xF3

#define Robo_ID_SPEED_MODE_ALL        0x04 //广播
#define Robo_ID_SPEED_MODE_NUM01      0x14
#define Robo_ID_SPEED_MODE_NUM02      0x24
#define Robo_ID_SPEED_MODE_NUM03      0x34
#define Robo_ID_SPEED_MODE_NUM04      0x44
#define Robo_ID_SPEED_MODE_NUM05      0x54
#define Robo_ID_SPEED_MODE_NUM06      0x64
#define Robo_ID_SPEED_MODE_NUM07      0x74
#define Robo_ID_SPEED_MODE_NUM08      0x84
#define Robo_ID_SPEED_MODE_NUM09      0x94
#define Robo_ID_SPEED_MODE_NUM10      0xA4
#define Robo_ID_SPEED_MODE_NUM11      0xB4
#define Robo_ID_SPEED_MODE_NUM12      0xC4
#define Robo_ID_SPEED_MODE_NUM13      0xD4
#define Robo_ID_SPEED_MODE_NUM14      0xE4
#define Robo_ID_SPEED_MODE_NUM15      0xF4

#define Robo_ID_LOCATION_MODE_ALL     0x05 //广播
#define Robo_ID_LOCATION_MODE_NUM01   0x15
#define Robo_ID_LOCATION_MODE_NUM02   0x25
#define Robo_ID_LOCATION_MODE_NUM03   0x35
#define Robo_ID_LOCATION_MODE_NUM04   0x45
#define Robo_ID_LOCATION_MODE_NUM05   0x55
#define Robo_ID_LOCATION_MODE_NUM06   0x65
#define Robo_ID_LOCATION_MODE_NUM07   0x75
#define Robo_ID_LOCATION_MODE_NUM08   0x85
#define Robo_ID_LOCATION_MODE_NUM09   0x95
#define Robo_ID_LOCATION_MODE_NUM10   0xA5
#define Robo_ID_LOCATION_MODE_NUM11   0xB5
#define Robo_ID_LOCATION_MODE_NUM12   0xC5
#define Robo_ID_LOCATION_MODE_NUM13   0xD5
#define Robo_ID_LOCATION_MODE_NUM14   0xE5
#define Robo_ID_LOCATION_MODE_NUM15   0xF5

#define ENTER_PWM_MODE               0x56
#define ENTER_CURRENT_MODE           0x57
#define ENTER_SPEED_MODE             0x58
#define ENTER_LOCATION_MODE          0x59
/*extern function-------------------------------------------------------------*/
extern void RoboInit(u8 RoboMode);
extern void RoboModule_Driver_Reset(void);
extern void RoboModule_Driver_Mode_Chioce(unsigned char ENTER_X_MODE);
extern void RoboModule_Driver_PWM_Mode_Set(short PWM_Value);
extern void RoboModule_Driver_Current_Mode_Set(short Current_Value);
extern void RoboModule_Driver_Speed_Mode_Set(short PWM_Value,short Speed_Value);
extern void RoboModule_Driver_Location_Mode_Set(short PWM_Value,short Speed_Value,int Location_Value);

		 				    
#endif
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
