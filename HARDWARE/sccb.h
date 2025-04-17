#ifndef __SCCB_H
#define __SCCB_H
#include "stm32f4xx.h"
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//OVϵ������ͷ SCCB ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/14
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

/*************************��Ҫ�޸ĵĵط�*************************/
#define OV7670_XCLK_RCC		RCC_AHB1Periph_GPIOA
#define OV7670_XCLK_Pin		GPIO_Pin_8
#define OV7670_XCLK_GPIO	GPIOA
#define STM32_MCO1_DIV		RCC_MCO1Div_4

#define OV7670_SCCB_RCC		RCC_AHB1Periph_GPIOB
#define OV7670_SCCB_Pin		GPIO_Pin_4|GPIO_Pin_3
#define OV7670_SCCB_GPIO	GPIOB


//IO��������
//#define SCCB_SDA_IN()  		{GPIOF->MODER&=~(0x0003<<18);GPIOF->MODER|=(0x0000<<18);}	//PF9 ���� �Ĵ�������2*9=18λ
//#define SCCB_SDA_OUT() 		{GPIOF->MODER&=~(0x0003<<18);GPIOF->MODER|=(0x0001<<18);} 	//PF9 ���


// PB3 ����ģʽ��GPIOB_MODER����Ϊ00��
#define SCCB_SDA_IN()  do { \
    GPIOB->MODER &= ~(0x3 << (3*2)); \
    GPIOB->MODER |=  (0x0 << (3*2)); \
} while (0) //PB3 ����

// PB3 ���ģʽ��GPIOB_MODER����Ϊ01��
#define SCCB_SDA_OUT() do { \
    GPIOB->MODER &= ~(0x3 << (3*2)); \
    GPIOB->MODER |=  (0x1 << (3*2)); \
} while (0) //PB3 ���

//IO��������	 
#define SCCB_SCL_H    		GPIO_SetBits(GPIOB,GPIO_Pin_4)	 		//SCL
#define SCCB_SCL_L    		GPIO_ResetBits(GPIOB,GPIO_Pin_4)	 	//SCL

#define SCCB_SDA_H    		GPIO_SetBits(GPIOB,GPIO_Pin_3) 			//SDA	 
#define SCCB_SDA_L    		GPIO_ResetBits(GPIOB,GPIO_Pin_3) 		//SDA	

#define SCCB_READ_SDA   	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3) //����SDA  
/*************************��Ҫ�޸ĵĵط�*************************/

#define SCCB_ID   			0x42  			//OV7670��ID

///////////////////////////////////////////
void SCCB_Init(void);
void SCCB_Start(void);
void SCCB_Stop(void);
void SCCB_No_Ack(void);
uint8_t SCCB_WR_Byte(uint8_t dat);
uint8_t SCCB_RD_Byte(void);
uint8_t SCCB_WR_Reg(uint8_t reg, uint8_t data);
uint8_t SCCB_RD_Reg(uint8_t reg);
#endif


