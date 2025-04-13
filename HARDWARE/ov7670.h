#ifndef _OV7670_H
#define _OV7670_H

#include "stm32f4xx.h"


/*************************��Ҫ�޸ĵĵط�*************************/
#define OV7670_RST_PW_RCC		RCC_AHB1Periph_GPIOA
#define OV7670_RST_PW_Pin		GPIO_Pin_15|GPIO_Pin_4 	//GPIO_Pin_15(res)  GPIO_Pin_4(pwdn)
#define OV7670_RST_PW_GPIO		GPIOA


#define OV7670_PWDN_H  			GPIO_SetBits(GPIOA,GPIO_Pin_4)			//POWER DOWN�����ź� 
#define OV7670_PWDN_L  			GPIO_ResetBits(GPIOA,GPIO_Pin_4)		//POWER DOWN�����ź� 

#define OV7670_RST_H  			GPIO_SetBits(GPIOA,GPIO_Pin_15)			//��λ�����ź� 
#define OV7670_RST_L  			GPIO_ResetBits(GPIOA,GPIO_Pin_15)		//��λ�����ź� 

//320*240�ü���ض���
#define PIC_START_X				0		//��ʼ����x
#define PIC_START_Y				0		//��ʼ����y
#define PIC_WIDTH				320		//��Ƭ����
#define PIC_HEIGHT				200		//��Ƭ�߶�

extern uint16_t camera_buffer[PIC_WIDTH*PIC_HEIGHT];
/*************************��Ҫ�޸ĵĵط�*************************/

////////////////////////////////////////////////////////////////////////////////// 
#define OV7670_MID				0X7FA2    
#define OV7670_PID				0X7673
/////////////////////////////////////////
	    				 
uint8_t   OV7670_Init(void);		  	   		 
void OV7670_Light_Mode(uint8_t mode);
void OV7670_Color_Saturation(uint8_t sat);
void OV7670_Brightness(uint8_t bright);
void OV7670_Contrast(uint8_t contrast);
void OV7670_Special_Effects(uint8_t eft);
void OV7670_Window_Set(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height);
void set_cif(void);

#endif
