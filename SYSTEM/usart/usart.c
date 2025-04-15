#include "sys.h"
#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////
// 如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h" //ucos 使用
#endif
//////////////////////////////////////////////////////////////////////////////////
// 本程序只供学习使用，未经作者许可，不得用于其它任何用途
// ALIENTEK STM32F4探索者开发板
// 串口1初始化
// 正点原子@ALIENTEK
// 技术论坛:www.openedv.com
// 修改日期:2014/6/10
// 版本：V1.5
// 版权所有，盗版必究。
// Copyright(C) 广州市星翼电子科技有限公司 2009-2019
// All rights reserved
//********************************************************************************
// V1.3修改说明
// 支持适应不同频率下的串口波特率设置.
// 加入了对printf的支持
// 增加了串口接收命令功能.
// 修正了printf第一个字符丢失的bug
// V1.4修改说明
// 1,修改串口初始化IO的bug
// 2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
// 3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
// 4,修改了EN_USART1_RX的使能方式
// V1.5修改说明
// 1,增加了对UCOSII的支持
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// 加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
// 标准库需要的支持函数
struct __FILE
{
	int handle;
};

FILE __stdout;
// 定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
	x = x;
}
//__use_no_semihosting was requested, but _ttywrch was 
void _ttywrch(int ch)
{
     ch = ch;
}
/*
// 重定义fputc函数
int fputc(int ch, FILE *f)
{
	while ((USART1->SR & 0X40) == 0)
		; // 循环发送,直到发送完毕
	USART1->DR = (uint8_t)ch;
	return ch;
}
*/

///重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
    //  发送一个字节数据到串口 
    USART_SendData(USART1, (uint8_t) ch);
		
	//  等待发送完毕 
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		
	
	return (ch);
}
//重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
	//  等待串口输入数据 
	// while (USART_GetFlagStatus(USART, USART_FLAG_RXNE) == RESET);

	// return (int)USART_ReceiveData(USART);
	while (SEGGER_RTT_HasKey() == 0){}
   	return SEGGER_RTT_GetKey();
}

#endif



#if EN_USART1_RX // 如果使能了接收
// 串口1中断服务程序
// 注意,读取USARTx->SR能避免莫名其妙的错误
uint8_t USART_RX_BUF[USART_REC_LEN]; // 接收缓冲,最大USART_REC_LEN个字节. (USMART在使用)

// 接收状态
// bit15，	接收完成标志
// bit14，	接收到0x0d
// bit13~0，	接收到的有效字节数目
uint16_t USART_RX_STA = 0;	//接收状态标记    (USMART在使用)

volatile uint16_t com1_rx_len = 0;    //接收帧数据的长度
volatile uint8_t com1_recv_end_flag = 0;//帧数据接收完成标志
uint8_t com1_rx_buffer[1024]={0};   //接收数据缓存
uint8_t DMA_USART1_TX_BUF[1024]={0};//DMA发送缓存

// 封装RTT格式化输出
int RTT_printf(unsigned BufferIndex, const char *sFormat, ...)
{
	int r;
	va_list ParamList;

	SEGGER_RTT_SetTerminal(BufferIndex); // 指定输出终端
	
	va_start(ParamList, sFormat);
	r = SEGGER_RTT_vprintf(0, sFormat, &ParamList);
	va_end(ParamList);
	return r;
}

//串口1初始化
void uart1_Init(uint32_t bound)
{
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    USART_DeInit(USART1);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 		//使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);		//使能USART1时钟
    //串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); 	//GPIOA9复用为USART1
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);	 //GPIOA10复用为USART1
    //USART1端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_9; 	//GPIOA10与GPIOA9
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;				//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 				//推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 				//上拉
    GPIO_Init(GPIOA,&GPIO_InitStructure); 						//初始化PA9，PA10
    //USART1 初始化设置
    USART_InitStructure.USART_BaudRate = bound;					//波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;		//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;			//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件数据流控制
    USART_InitStructure.USART_Mode =  USART_Mode_Rx | USART_Mode_Tx;				//收发模式
    USART_Init(USART1, &USART_InitStructure); 	//初始化串口1

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;       //串口1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2; //抢占优先级2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;	    //子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);							//根据指定的参数初始化VIC寄存器、
    
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn; //嵌套通道为DMA2_Stream5_IRQn
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //抢占优先级为 2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //响应优先级为 2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //通道中断使能
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn ;//串口1发送中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   //抢占优先级2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 3;   //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE; //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);  	//开启串口空闲中断
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);  	// 开启串口DMA接收
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  	// 开启串口DMA接收
    /* 配置DMA2_Stream5串口DMA接收*/
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  					// 开启DMA时钟
    DMA_DeInit(DMA2_Stream5);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4; 							//通道选择
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;		//DMA外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)com1_rx_buffer;		//DMA 存储器0地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;   				//存储器到外设模式
    DMA_InitStructure.DMA_BufferSize = USART_MAX_LEN;						//数据传输量
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//外设非增量模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//存储器增量模式
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据长度:8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//存储器数据长度:8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							//使用普通模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;				    //高等优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //不开启FIFO模式
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           //FIFO阈值
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//外设突发单次传输
    DMA_Init(DMA2_Stream5, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream5, ENABLE); //使能DMA2_Stream5通道

    DMA_DeInit(DMA2_Stream7);    //初始化DMA Stream
    while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);//等待DMA可配置
    /* 配置DMA2 Stream7，USART1发送 */
    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;               //通道选择
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;            //DMA外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)DMA_USART1_TX_BUF;      //DMA 存储器0地址
    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;  //存储器到外设模式
    DMA_InitStructure.DMA_BufferSize         = DMA_USART1_TX_BUF_LEN;       //数据传输量
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;   //外设非增量模式
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;        //存储器增量模式
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据长度:8位
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;     //存储器数据长度:8位
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;             //使用普通模式
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;         //中等优先级
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;      //存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;  //外设突发单次传输
    DMA_Init(DMA2_Stream7, &DMA_InitStructure);                             //初始化DMA Stream7

    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);							//DMA2传输完成中断
    DMA_Cmd(DMA2_Stream7, DISABLE);											//不使能
    USART_Cmd(USART1, ENABLE);  //使能串口1
}

//IDLE空闲中断服务函数
void USART1_IRQHandler(void)  												//串口1中断服务程序
{
    if(USART_GetITStatus(USART1,USART_IT_IDLE)!=RESET) 	//空闲中断触发
    {
    	com1_recv_end_flag = 1;  	// 接受完成标志位置1

    	DMA_Cmd(DMA2_Stream5, DISABLE);  					   /* 暂时关闭dma，数据尚未处理 */
    	com1_rx_len = USART_MAX_LEN - DMA_GetCurrDataCounter(DMA2_Stream5);/* 获取接收到的数据长度 单位为字节*/
    	DMA_ClearFlag(DMA2_Stream5,DMA_FLAG_TCIF5);  		/* 清DMA标志位 */
    	DMA_SetCurrDataCounter(DMA2_Stream5,USART_MAX_LEN);	/* 重新赋值计数值，必须大于等于最大可能接收到的数据帧数目 */
    	DMA_Cmd(DMA2_Stream5, ENABLE);      				/*打开DMA*/
    	USART_ReceiveData(USART1);   						//清除空闲中断标志位（接收函数有清标志位的作用）
    }

  	if(USART_GetFlagStatus(USART1,USART_IT_TXE)==RESET)	//串口发送完成
  	{
    	USART_ITConfig(USART1,USART_IT_TC,DISABLE);
 	}
}

//DMA接收
void DMA2_Stream7_IRQHandler(void)
{
	//清除标志
	if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//等待DMA2_Steam7传输完成
	{
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7); //清除DMA2_Steam7传输完成标志
   		DMA_Cmd(DMA2_Stream7,DISABLE);				//关闭使能
    	USART_ITConfig(USART1,USART_IT_TC,ENABLE);  //打开串口发送完成中断
	}
}

//DMA发送
void DMA_USART1_Send(uint8_t *data,uint16_t size)//函数名称可自定义
{
	memcpy(DMA_USART1_TX_BUF,data,size);				//复制数据到DMA发送缓存区
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);	//确保DMA可以被设置
	DMA_SetCurrDataCounter(DMA2_Stream7,size);			//设置数据传输长度
	DMA_Cmd(DMA2_Stream7,ENABLE);						//打开DMA数据流，开始发送
}



/*
//正点原子源代码
// 初始化IO 串口1
// bound:波特率
void uart_init(u32 bound)
{
	// GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // 使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // 使能USART1时钟

	// 串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  // GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); // GPIOA10复用为USART1

	// USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; // GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			// 复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// 速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			// 推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			// 上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);					// 初始化PA9，PA10

	// USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;										// 波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								// 无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 收发模式
	USART_Init(USART1, &USART_InitStructure);										// 初始化串口1

	USART_Cmd(USART1, ENABLE); // 使能串口1

	// USART_ClearFlag(USART1, USART_FLAG_TC);

#if EN_USART1_RX
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 开启相关中断

	// Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		  // 串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  // 子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  // 根据指定的参数初始化VIC寄存器、

#endif
}

void USART1_IRQHandler(void) // 串口1中断服务程序
{
	u8 Res;
#if SYSTEM_SUPPORT_OS // 如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();
#endif
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) // 接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res = USART_ReceiveData(USART1); //(USART1->DR);	//读取接收到的数据

		if ((USART_RX_STA & 0x8000) == 0) // 接收未完成
		{
			if (USART_RX_STA & 0x4000) // 接收到了0x0d
			{
				if (Res != 0x0a)
					USART_RX_STA = 0; // 接收错误,重新开始
				else
					USART_RX_STA |= 0x8000; // 接收完成了
			}
			else // 还没收到0X0D
			{
				if (Res == 0x0d)
					USART_RX_STA |= 0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA & 0X3FFF] = Res;
					USART_RX_STA++;
					if (USART_RX_STA > (USART_REC_LEN - 1))
						USART_RX_STA = 0; // 接收数据错误,重新开始接收
				}
			}
		}
	}
#if SYSTEM_SUPPORT_OS // 如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();
#endif
}
*/
#endif
