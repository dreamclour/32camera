#include "sys.h"
#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////
// ���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h" //ucos ʹ��
#endif
//////////////////////////////////////////////////////////////////////////////////
// ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
// ALIENTEK STM32F4̽���߿�����
// ����1��ʼ��
// ����ԭ��@ALIENTEK
// ������̳:www.openedv.com
// �޸�����:2014/6/10
// �汾��V1.5
// ��Ȩ���У�����ؾ���
// Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
// All rights reserved
//********************************************************************************
// V1.3�޸�˵��
// ֧����Ӧ��ͬƵ���µĴ��ڲ���������.
// �����˶�printf��֧��
// �����˴��ڽ��������.
// ������printf��һ���ַ���ʧ��bug
// V1.4�޸�˵��
// 1,�޸Ĵ��ڳ�ʼ��IO��bug
// 2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
// 3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
// 4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
// V1.5�޸�˵��
// 1,�����˶�UCOSII��֧��
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// �������´���,֧��printf����,������Ҫѡ��use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
// ��׼����Ҫ��֧�ֺ���
struct __FILE
{
	int handle;
};

FILE __stdout;
// ����_sys_exit()�Ա���ʹ�ð�����ģʽ
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
// �ض���fputc����
int fputc(int ch, FILE *f)
{
	while ((USART1->SR & 0X40) == 0)
		; // ѭ������,ֱ���������
	USART1->DR = (uint8_t)ch;
	return ch;
}
*/

///�ض���c�⺯��printf�����ڣ��ض�����ʹ��printf����
int fputc(int ch, FILE *f)
{
    //  ����һ���ֽ����ݵ����� 
    USART_SendData(USART1, (uint8_t) ch);
		
	//  �ȴ�������� 
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		
	
	return (ch);
}
//�ض���c�⺯��scanf�����ڣ���д����ʹ��scanf��getchar�Ⱥ���
int fgetc(FILE *f)
{
	//  �ȴ������������� 
	// while (USART_GetFlagStatus(USART, USART_FLAG_RXNE) == RESET);

	// return (int)USART_ReceiveData(USART);
	while (SEGGER_RTT_HasKey() == 0){}
   	return SEGGER_RTT_GetKey();
}

#endif



#if EN_USART1_RX // ���ʹ���˽���
// ����1�жϷ������
// ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���
uint8_t USART_RX_BUF[USART_REC_LEN]; // ���ջ���,���USART_REC_LEN���ֽ�. (USMART��ʹ��)

// ����״̬
// bit15��	������ɱ�־
// bit14��	���յ�0x0d
// bit13~0��	���յ�����Ч�ֽ���Ŀ
uint16_t USART_RX_STA = 0;	//����״̬���    (USMART��ʹ��)

volatile uint16_t com1_rx_len = 0;    //����֡���ݵĳ���
volatile uint8_t com1_recv_end_flag = 0;//֡���ݽ�����ɱ�־
uint8_t com1_rx_buffer[1024]={0};   //�������ݻ���
uint8_t DMA_USART1_TX_BUF[1024]={0};//DMA���ͻ���

// ��װRTT��ʽ�����
int RTT_printf(unsigned BufferIndex, const char *sFormat, ...)
{
	int r;
	va_list ParamList;

	SEGGER_RTT_SetTerminal(BufferIndex); // ָ������ն�
	
	va_start(ParamList, sFormat);
	r = SEGGER_RTT_vprintf(0, sFormat, &ParamList);
	va_end(ParamList);
	return r;
}

//����1��ʼ��
void uart1_Init(uint32_t bound)
{
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    USART_DeInit(USART1);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 		//ʹ��GPIOAʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);		//ʹ��USART1ʱ��
    //����1��Ӧ���Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); 	//GPIOA9����ΪUSART1
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);	 //GPIOA10����ΪUSART1
    //USART1�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_9; 	//GPIOA10��GPIOA9
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;				//���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//�ٶ�100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 				//���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 				//����
    GPIO_Init(GPIOA,&GPIO_InitStructure); 						//��ʼ��PA9��PA10
    //USART1 ��ʼ������
    USART_InitStructure.USART_BaudRate = bound;					//����������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;		//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;			//����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//��Ӳ������������
    USART_InitStructure.USART_Mode =  USART_Mode_Rx | USART_Mode_Tx;				//�շ�ģʽ
    USART_Init(USART1, &USART_InitStructure); 	//��ʼ������1

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;       //����1�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2; //��ռ���ȼ�2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;	    //�����ȼ�1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);							//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
    
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn; //Ƕ��ͨ��ΪDMA2_Stream5_IRQn
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //��ռ���ȼ�Ϊ 2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //��Ӧ���ȼ�Ϊ 2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ͨ���ж�ʹ��
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn ;//����1�����ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   //��ռ���ȼ�2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 3;   //�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE; //IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);  	//�������ڿ����ж�
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);  	// ��������DMA����
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  	// ��������DMA����
    /* ����DMA2_Stream5����DMA����*/
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  					// ����DMAʱ��
    DMA_DeInit(DMA2_Stream5);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4; 							//ͨ��ѡ��
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;		//DMA�����ַ
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)com1_rx_buffer;		//DMA �洢��0��ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;   				//�洢��������ģʽ
    DMA_InitStructure.DMA_BufferSize = USART_MAX_LEN;						//���ݴ�����
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//���������ģʽ
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//�洢������ģʽ
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //�������ݳ���:8λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//�洢�����ݳ���:8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							//ʹ����ͨģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;				    //�ߵ����ȼ�
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //������FIFOģʽ
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           //FIFO��ֵ
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//�洢��ͻ�����δ���
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//����ͻ�����δ���
    DMA_Init(DMA2_Stream5, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream5, ENABLE); //ʹ��DMA2_Stream5ͨ��

    DMA_DeInit(DMA2_Stream7);    //��ʼ��DMA Stream
    while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);//�ȴ�DMA������
    /* ����DMA2 Stream7��USART1���� */
    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;               //ͨ��ѡ��
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;            //DMA�����ַ
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)DMA_USART1_TX_BUF;      //DMA �洢��0��ַ
    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;  //�洢��������ģʽ
    DMA_InitStructure.DMA_BufferSize         = DMA_USART1_TX_BUF_LEN;       //���ݴ�����
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;   //���������ģʽ
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;        //�洢������ģʽ
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //�������ݳ���:8λ
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;     //�洢�����ݳ���:8λ
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;             //ʹ����ͨģʽ
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;         //�е����ȼ�
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;      //�洢��ͻ�����δ���
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;  //����ͻ�����δ���
    DMA_Init(DMA2_Stream7, &DMA_InitStructure);                             //��ʼ��DMA Stream7

    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);							//DMA2��������ж�
    DMA_Cmd(DMA2_Stream7, DISABLE);											//��ʹ��
    USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1
}

//IDLE�����жϷ�����
void USART1_IRQHandler(void)  												//����1�жϷ������
{
    if(USART_GetITStatus(USART1,USART_IT_IDLE)!=RESET) 	//�����жϴ���
    {
    	com1_recv_end_flag = 1;  	// ������ɱ�־λ��1

    	DMA_Cmd(DMA2_Stream5, DISABLE);  					   /* ��ʱ�ر�dma��������δ���� */
    	com1_rx_len = USART_MAX_LEN - DMA_GetCurrDataCounter(DMA2_Stream5);/* ��ȡ���յ������ݳ��� ��λΪ�ֽ�*/
    	DMA_ClearFlag(DMA2_Stream5,DMA_FLAG_TCIF5);  		/* ��DMA��־λ */
    	DMA_SetCurrDataCounter(DMA2_Stream5,USART_MAX_LEN);	/* ���¸�ֵ����ֵ��������ڵ��������ܽ��յ�������֡��Ŀ */
    	DMA_Cmd(DMA2_Stream5, ENABLE);      				/*��DMA*/
    	USART_ReceiveData(USART1);   						//��������жϱ�־λ�����պ��������־λ�����ã�
    }

  	if(USART_GetFlagStatus(USART1,USART_IT_TXE)==RESET)	//���ڷ������
  	{
    	USART_ITConfig(USART1,USART_IT_TC,DISABLE);
 	}
}

//DMA����
void DMA2_Stream7_IRQHandler(void)
{
	//�����־
	if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//�ȴ�DMA2_Steam7�������
	{
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7); //���DMA2_Steam7������ɱ�־
   		DMA_Cmd(DMA2_Stream7,DISABLE);				//�ر�ʹ��
    	USART_ITConfig(USART1,USART_IT_TC,ENABLE);  //�򿪴��ڷ�������ж�
	}
}

//DMA����
void DMA_USART1_Send(uint8_t *data,uint16_t size)//�������ƿ��Զ���
{
	memcpy(DMA_USART1_TX_BUF,data,size);				//�������ݵ�DMA���ͻ�����
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);	//ȷ��DMA���Ա�����
	DMA_SetCurrDataCounter(DMA2_Stream7,size);			//�������ݴ��䳤��
	DMA_Cmd(DMA2_Stream7,ENABLE);						//��DMA����������ʼ����
}



/*
//����ԭ��Դ����
// ��ʼ��IO ����1
// bound:������
void uart_init(u32 bound)
{
	// GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // ʹ��USART1ʱ��

	// ����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  // GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); // GPIOA10����ΪUSART1

	// USART1�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; // GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			// ���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// �ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			// ���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			// ����
	GPIO_Init(GPIOA, &GPIO_InitStructure);					// ��ʼ��PA9��PA10

	// USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;										// ����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// �ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;								// ����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// �շ�ģʽ
	USART_Init(USART1, &USART_InitStructure);										// ��ʼ������1

	USART_Cmd(USART1, ENABLE); // ʹ�ܴ���1

	// USART_ClearFlag(USART1, USART_FLAG_TC);

#if EN_USART1_RX
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // ��������ж�

	// Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		  // ����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // ��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  // �����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);							  // ����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif
}

void USART1_IRQHandler(void) // ����1�жϷ������
{
	u8 Res;
#if SYSTEM_SUPPORT_OS // ���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();
#endif
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) // �����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res = USART_ReceiveData(USART1); //(USART1->DR);	//��ȡ���յ�������

		if ((USART_RX_STA & 0x8000) == 0) // ����δ���
		{
			if (USART_RX_STA & 0x4000) // ���յ���0x0d
			{
				if (Res != 0x0a)
					USART_RX_STA = 0; // ���մ���,���¿�ʼ
				else
					USART_RX_STA |= 0x8000; // ���������
			}
			else // ��û�յ�0X0D
			{
				if (Res == 0x0d)
					USART_RX_STA |= 0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA & 0X3FFF] = Res;
					USART_RX_STA++;
					if (USART_RX_STA > (USART_REC_LEN - 1))
						USART_RX_STA = 0; // �������ݴ���,���¿�ʼ����
				}
			}
		}
	}
#if SYSTEM_SUPPORT_OS // ���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();
#endif
}
*/
#endif
