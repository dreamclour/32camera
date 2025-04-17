#include "sccb.h"
#include "delay.h"
#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_gpio.h" 
#include "stm32f4xx_rcc.h" 
//////////////////////////////////////////////////////////////////////////////////	 
									  
////////////////////////////////////////////////////////////////////////////////// 

//初始化SCCB接口 
void SCCB_Init(void)
{				
  	GPIO_InitTypeDef  GPIO_InitStructure;

  	RCC_AHB1PeriphClockCmd(OV7670_SCCB_RCC, ENABLE);
  	RCC_AHB1PeriphClockCmd(OV7670_XCLK_RCC, ENABLE);

	//STM32F4时钟输出XCLK
  	GPIO_InitStructure.GPIO_Pin = OV7670_XCLK_Pin;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  	GPIO_Init(OV7670_XCLK_GPIO, &GPIO_InitStructure);
	
  	RCC_MCO1Config(RCC_MCO1Source_HSI,STM32_MCO1_DIV);
	
  	//GPIOF9,F10初始化设置
  	GPIO_InitStructure.GPIO_Pin = OV7670_SCCB_Pin;//
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  //
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  	GPIO_Init(OV7670_SCCB_GPIO, &GPIO_InitStructure);//初始化
 
	GPIO_SetBits(OV7670_SCCB_GPIO,OV7670_SCCB_Pin);
	SCCB_SDA_OUT();	   
}			 

//SCCB起始信号
//当时钟为高的时候,数据线的高到低,为SCCB起始信号
//在激活状态下,SDA和SCL均为低电平
void SCCB_Start(void)
{
    SCCB_SDA_H;     //数据线高电平	 
    SCCB_SCL_H;	    //在时钟线高的时候数据线由高至低
    delay_us(10);  
    SCCB_SDA_L;
    delay_us(10);	 
    SCCB_SCL_L;	    //数据线恢复低电平，单操作函数必要	
	delay_us(10);
}

//SCCB停止信号
//当时钟为高的时候,数据线的低到高,为SCCB停止信号
//空闲状况下,SDA,SCL均为高电平
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
//产生NA信号
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
//SCCB,写入一个字节
//返回值:0,成功;1,失败. 
uint8_t SCCB_WR_Byte(uint8_t dat)
{
	uint8_t j,res;	 
	for(j=0;j<8;j++) //循环8次发送数据
	{
		if(dat&0x80)SCCB_SDA_H;	
		else SCCB_SDA_L;
		dat<<=1;
		SCCB_SCL_H;	
		delay_us(10);
		SCCB_SCL_L;
		delay_us(10);		
	}			 
	SCCB_SDA_IN();		//设置SDA为输入 
	delay_us(10);
	SCCB_SCL_H;			//接收第九位,以判断是否发送成功
	delay_us(10);
	if(SCCB_READ_SDA)res=1;  //SDA=1发送失败，返回1
	else res=0;         //SDA=0发送成功，返回0
	SCCB_SCL_L;		 
	SCCB_SDA_OUT();		//设置SDA为输出    
	return res;  
}	 
//SCCB 读取一个字节
//在SCL的上升沿,数据锁存
//返回值:读到的数据
uint8_t SCCB_RD_Byte(void)
{
	uint8_t temp=0,j;    
	SCCB_SDA_IN();		//设置SDA为输入  
	for(j=8;j>0;j--) 	//循环8次接收数据
	{		     	  
		delay_us(10);
		SCCB_SCL_H;
		temp=temp<<1;
		if(SCCB_READ_SDA)temp++;   
		delay_us(10);
		SCCB_SCL_L;
	}	
	SCCB_SDA_OUT();		//设置SDA为输出
	SCCB_No_Ack();
	return temp;
} 							    
//写寄存器
//返回值:0,成功;1,失败.
uint8_t SCCB_WR_Reg(uint8_t reg,uint8_t data)
{
	uint8_t res=0;
	SCCB_Start(); 					//启动SCCB传输
	if(SCCB_WR_Byte(SCCB_ID))res=1;	//写器件ID	  
  	if(SCCB_WR_Byte(reg))res=1;		//写寄存器地址	  
  	if(SCCB_WR_Byte(data))res=1; 	//写数据	 
  	SCCB_Stop();	  
  	return	res;
}		  					    
//读寄存器
//返回值:读到的寄存器值
uint8_t SCCB_RD_Reg(uint8_t reg)
{
	uint8_t val=0;
	SCCB_Start(); 				//启动SCCB传输
	SCCB_WR_Byte(SCCB_ID);		//写器件ID	  	 
  	SCCB_WR_Byte(reg);			//写寄存器地址	  	  
	SCCB_Stop();   
	delay_us(100);	   
	//设置寄存器地址后，才是读
	SCCB_Start();
	SCCB_WR_Byte(SCCB_ID|0X01);	//发送读命令	  
  	val=SCCB_RD_Byte();		 	//读取数据
  	SCCB_Stop();
  	return val;
}


