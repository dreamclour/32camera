#include "sccb.h"
#include "delay.h"
#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_gpio.h" 
#include "stm32f4xx_rcc.h" 
//////////////////////////////////////////////////////////////////////////////////	 
									  
////////////////////////////////////////////////////////////////////////////////// 

//��ʼ��SCCB�ӿ� 
void SCCB_Init(void)
{				
  	GPIO_InitTypeDef  GPIO_InitStructure;

  	RCC_AHB1PeriphClockCmd(OV7670_SCCB_RCC, ENABLE);
  	RCC_AHB1PeriphClockCmd(OV7670_XCLK_RCC, ENABLE);

	//STM32F4ʱ�����XCLK
  	GPIO_InitStructure.GPIO_Pin = OV7670_XCLK_Pin;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  	GPIO_Init(OV7670_XCLK_GPIO, &GPIO_InitStructure);
	
  	RCC_MCO1Config(RCC_MCO1Source_HSI,STM32_MCO1_DIV);
	
  	//GPIOF9,F10��ʼ������
  	GPIO_InitStructure.GPIO_Pin = OV7670_SCCB_Pin;//
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  //
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  	GPIO_Init(OV7670_SCCB_GPIO, &GPIO_InitStructure);//��ʼ��
 
	GPIO_SetBits(OV7670_SCCB_GPIO,OV7670_SCCB_Pin);
	SCCB_SDA_OUT();	   
}			 

//SCCB��ʼ�ź�
//��ʱ��Ϊ�ߵ�ʱ��,�����ߵĸߵ���,ΪSCCB��ʼ�ź�
//�ڼ���״̬��,SDA��SCL��Ϊ�͵�ƽ
void SCCB_Start(void)
{
    SCCB_SDA_H;     //�����߸ߵ�ƽ	 
    SCCB_SCL_H;	    //��ʱ���߸ߵ�ʱ���������ɸ�����
    delay_us(10);  
    SCCB_SDA_L;
    delay_us(10);	 
    SCCB_SCL_L;	    //�����߻ָ��͵�ƽ��������������Ҫ	
	delay_us(10);
}

//SCCBֹͣ�ź�
//��ʱ��Ϊ�ߵ�ʱ��,�����ߵĵ͵���,ΪSCCBֹͣ�ź�
//����״����,SDA,SCL��Ϊ�ߵ�ƽ
void SCCB_Stop(void)
{
	SCCB_SCL_L;
    SCCB_SDA_L;
    delay_us(10);	 
    SCCB_SCL_H;	
    delay_us(10); 
    SCCB_SDA_H;	
    delay_us(10);
}  
//����NA�ź�
void SCCB_No_Ack(void)
{
	//delay_us(10);
	SCCB_SDA_H;	
	SCCB_SCL_H;	
	delay_us(10);
	SCCB_SCL_L;	
	delay_us(10);
	SCCB_SDA_L;	
	delay_us(10);
}
//SCCB,д��һ���ֽ�
//����ֵ:0,�ɹ�;1,ʧ��. 
uint8_t SCCB_WR_Byte(uint8_t dat)
{
	uint8_t j,res;	 
	for(j=0;j<8;j++) //ѭ��8�η�������
	{
		if(dat&0x80)SCCB_SDA_H;	
		else SCCB_SDA_L;
		dat<<=1;
		SCCB_SCL_H;	
		delay_us(10);
		SCCB_SCL_L;
		delay_us(10);		
	}			 
	SCCB_SDA_IN();		//����SDAΪ���� 
	delay_us(10);
	SCCB_SCL_H;			//���յھ�λ,���ж��Ƿ��ͳɹ�
	delay_us(10);
	if(SCCB_READ_SDA)res=1;  //SDA=1����ʧ�ܣ�����1
	else res=0;         //SDA=0���ͳɹ�������0
	SCCB_SCL_L;		 
	SCCB_SDA_OUT();		//����SDAΪ���    
	return res;  
}	 
//SCCB ��ȡһ���ֽ�
//��SCL��������,��������
//����ֵ:����������
uint8_t SCCB_RD_Byte(void)
{
	uint8_t temp=0,j;    
	SCCB_SDA_IN();		//����SDAΪ����  
	for(j=8;j>0;j--) 	//ѭ��8�ν�������
	{		     	  
		delay_us(10);
		SCCB_SCL_H;
		temp=temp<<1;
		if(SCCB_READ_SDA)temp++;   
		delay_us(10);
		SCCB_SCL_L;
	}	
	SCCB_SDA_OUT();		//����SDAΪ���
	SCCB_No_Ack();
	return temp;
} 							    
//д�Ĵ���
//����ֵ:0,�ɹ�;1,ʧ��.
uint8_t SCCB_WR_Reg(uint8_t reg,uint8_t data)
{
	uint8_t res=0;
	SCCB_Start(); 					//����SCCB����
	if(SCCB_WR_Byte(SCCB_ID))res=1;	//д����ID	  
  	if(SCCB_WR_Byte(reg))res=1;		//д�Ĵ�����ַ	  
  	if(SCCB_WR_Byte(data))res=1; 	//д����	 
  	SCCB_Stop();	  
  	return	res;
}		  					    
//���Ĵ���
//����ֵ:�����ļĴ���ֵ
uint8_t SCCB_RD_Reg(uint8_t reg)
{
	uint8_t val=0;
	SCCB_Start(); 				//����SCCB����
	SCCB_WR_Byte(SCCB_ID);		//д����ID	  	 
  	SCCB_WR_Byte(reg);			//д�Ĵ�����ַ	  	  
	SCCB_Stop();   
	delay_us(100);	   
	//���üĴ�����ַ�󣬲��Ƕ�
	SCCB_Start();
	SCCB_WR_Byte(SCCB_ID|0X01);	//���Ͷ�����	  
  	val=SCCB_RD_Byte();		 	//��ȡ����
  	SCCB_Stop();
  	return val;
}


