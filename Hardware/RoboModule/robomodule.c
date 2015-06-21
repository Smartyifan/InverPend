/**
  ******************************************************************************
  * @file    ·��
  * @author  ��һ��
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
#include "usart/usart.h"
#include "delay/delay.h"
#include "RoboModule/robomodule.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void UART2Init(u32 bound);
void RoboInit(void);
void RoboSendData(u8 *Data);
void RoboModule_Driver_Reset(void);
void RoboModule_Driver_Mode_Chioce(unsigned char ENTER_X_MODE);
void RoboModule_Driver_PWM_Mode_Set(short PWM_Value);
void RoboModule_Driver_Current_Mode_Set(short Current_Value);
void RoboModule_Driver_Speed_Mode_Set(short PWM_Value,short Speed_Value);
void RoboModule_Driver_Location_Mode_Set(short PWM_Value,short Speed_Value,int Location_Value);
/* Private functions ---------------------------------------------------------*/
/**
  *@brief   UART2Init	
  *@param   None;
  *@retval  None
  */
void UART2Init(u32 bound){
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
// 	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART2��GPIOAʱ��
 	USART_DeInit(USART2);  //��λ����2
	
	/* GPIO initiate ---------------------------------------------------------*/
	//USART2_TX   PA.2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ��PA9
   
    //USART2_RX	  PA.3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);  //��ʼ��PA10
  
	/* USART initiate --------------------------------------------------------*/
   //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART2, &USART_InitStructure); //��ʼ������
	
    USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ��� 
}

/**
  *@brief   RoboInit	
  *@param   None;
  *@retval  None
  */
void RoboInit(void){
	
	UART2Init(115200);		//��ʼ������2
	/* ��λ -----------------------------------------------------*/
	RoboModule_Driver_Reset();
	delay_ms(1500);					//�ȴ�1.5s��
	
	/* ģʽѡ�� -------------------------------------------------*/
	RoboModule_Driver_Mode_Chioce(ENTER_LOCATION_MODE);
	delay_ms(600);					//�ȴ�600ms
}

/**
  *@brief   RoboSendData	
  *@param   None;
  *@retval  None
  */
void RoboSendData(u8 *Data){
	u8 i;
	//����ͨ������
	for(i=0;i<10;i++)USART_SendData(USART2,*(Data+i));
}


/*************************************************************************
                      RoboModule_Driver_Reset
������������������λ
�����������
*************************************************************************/
void RoboModule_Driver_Reset(void){
	u8 Data[10];
    
    Data[0] = 0x55;
    Data[1] = 0x55;
    Data[2] = 0xFF;
    Data[3] = 0xFF;
    Data[4] = 0xFF;
    Data[5] = 0xFF;
    Data[6] = 0xFF;
    Data[7] = 0xFF;
    Data[8] = 0xFF;
    Data[9] = 0xFF;
	
    RoboSendData(Data);
}


/*************************************************************************
                      RoboModule_Driver_Mode_Chioce
�����������ùҽ���CAN�����ϵ�ĳ������������ĳ��ģʽ
���������CAN_ID
���������ENTER_X_MODE
*************************************************************************/
void RoboModule_Driver_Mode_Chioce(unsigned char ENTER_X_MODE)
{    
 	u8 Data[10];
   
    if((ENTER_X_MODE != ENTER_PWM_MODE)&&
       (ENTER_X_MODE != ENTER_CURRENT_MODE)&&
       (ENTER_X_MODE != ENTER_SPEED_MODE)&&
       (ENTER_X_MODE != ENTER_LOCATION_MODE))
    {
        ENTER_X_MODE = ENTER_PWM_MODE;
    }
    
    Data[0] = 0x55;
    Data[1] = ENTER_X_MODE;
    Data[2] = 0xFF;
    Data[3] = 0xFF;
    Data[4] = 0xFF;
    Data[5] = 0xFF;
    Data[6] = 0xFF;
    Data[7] = 0xFF;
    Data[8] = 0xFF;
    Data[9] = 0xFF;
    
    RoboSendData(Data);
}

/*************************************************************************
                      RoboModule_Driver_PWM_Mode_Set
�������������ҽ���CAN�����ϵ�ĳ����������PWMģʽ�¸�ֵ
���������CAN_ID
���������PWM_Value
*************************************************************************/
void RoboModule_Driver_PWM_Mode_Set(short PWM_Value)
{
	u8 Data[10];

    if(PWM_Value > 5000)
    {
        PWM_Value = 5000;
    }
    else if(PWM_Value < -5000)
    {
        PWM_Value = -5000;
    }
    
    Data[0] = 0x55;
    Data[1] = 0x56;
    Data[2] = (unsigned char)((PWM_Value>>8)&0xff);;
    Data[3] = (unsigned char)(PWM_Value&0xff);
    Data[4] = 0xFF;
    Data[5] = 0xFF;
    Data[6] = 0xFF;
    Data[7] = 0xFF;
    Data[8] = 0xFF;
    Data[9] = 0xFF;

	RoboSendData(Data);
}

/*************************************************************************
                      RoboModule_Driver_Current_Mode_Set
�������������ҽ���CAN�����ϵ�ĳ����������Currentģʽ�¸�ֵ
���������CAN_ID
���������Current_Value
*************************************************************************/
void RoboModule_Driver_Current_Mode_Set(short Current_Value)
{
	u8 Data[10];

    if(Current_Value > 2000)
    {
        Current_Value = 2000;
    }
    else if(Current_Value < -2000)
    {
        Current_Value = -2000;
    }

    Data[0] = 0x55;
    Data[1] = 0x57;
    Data[2] = (unsigned char)((Current_Value>>8)&0xff);
    Data[3] = (unsigned char)(Current_Value&0xff);
    Data[4] = 0xFF;
    Data[5] = 0xFF;
    Data[6] = 0xFF;
    Data[7] = 0xFF;
    Data[8] = 0xFF;
    Data[9] = 0xFF;
    
	RoboSendData(Data);
}

/*************************************************************************
                      RoboModule_Driver_Speed_Mode_Set
�������������ҽ���CAN�����ϵ�ĳ����������Speedģʽ�¸�ֵ
���������CAN_ID
���������PWM_Value 
          Speed_Value
*************************************************************************/
void RoboModule_Driver_Speed_Mode_Set(short PWM_Value,short Speed_Value)
{
	u8 Data[10];

    if(PWM_Value > 5000)
    {
        PWM_Value = 5000;
    }
    else if(PWM_Value < -5000)
    {
        PWM_Value = -5000;
    }

    if(Speed_Value > 1000)
    {
        Speed_Value = 1000;
    }
    else if(Speed_Value < -1000)
    {
        Speed_Value = -1000;
    }

    Data[0] = 0x55;
    Data[1] = 0x58;
    Data[2] = (unsigned char)((PWM_Value>>8)&0xff);
    Data[3] = (unsigned char)(PWM_Value&0xff);
    Data[4] = (unsigned char)((Speed_Value>>8)&0xff);
    Data[5] = (unsigned char)(Speed_Value&0xff);
    Data[6] = 0xFF;
    Data[7] = 0xFF;
    Data[8] = 0xFF;
    Data[9] = 0xFF;
    
	RoboSendData(Data);
}

/*************************************************************************
                      RoboModule_Driver_Location_Mode_Set
�������������ҽ���CAN�����ϵ�ĳ����������Locationģʽ�¸�ֵ
���������CAN_ID
���������PWM_Value 
          Speed_Value 
          Location_Mode
*************************************************************************/
void RoboModule_Driver_Location_Mode_Set(short PWM_Value,short Speed_Value,int Location_Value)
{
	u8 Data[10];

    if(PWM_Value > 5000)
    {
        PWM_Value = 5000;
    }
    else if(PWM_Value < -5000)
    {
        PWM_Value = -5000;
    }

    if(Speed_Value > 1000)
    {
        Speed_Value = 1000;
    }
    else if(Speed_Value < -1000)
    {
        Speed_Value = -1000;
    }
    
    if(Location_Value > 5000000)
    {
        Location_Value = 5000000;
    }
    else if(Location_Value < -5000000)
    {
        Location_Value = -5000000;
    }

    Data[0] = 0x55;
    Data[1] = 0x59;
    Data[2] = (unsigned char)((PWM_Value>>8)&0xff);
    Data[3] = (unsigned char)(PWM_Value&0xff);
    Data[4] = (unsigned char)((Speed_Value>>8)&0xff);
    Data[5] = (unsigned char)(Speed_Value&0xff);
    Data[6] = (unsigned char)((Location_Value>>24)&0xff);
    Data[7] = (unsigned char)((Location_Value>>16)&0xff);
    Data[8] = (unsigned char)((Location_Value>>8)&0xff);
    Data[9] = (unsigned char)(Location_Value&0xff);
    
	RoboSendData(Data);
}


/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
