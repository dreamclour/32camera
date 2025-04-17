#ifndef __USART_H
#define __USART_H
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_conf.h"
#include "SEGGER_RTT.h"
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////
// ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
// Mini STM32������
// ����1��ʼ��
// ����ԭ��@ALIENTEK
// ������̳:www.openedv.csom
// �޸�����:2011/6/14
// �汾��V1.4
// ��Ȩ���У�����ؾ���
// Copyright(C) ����ԭ�� 2009-2019
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
//////////////////////////////////////////////////////////////////////////////////
#define USART_REC_LEN 256 // �����������ֽ��� 256
#define EN_USART1_RX 1    // ʹ�ܣ�1��/��ֹ��0������1����

#define USART_MAX_LEN 1024
#define DMA_USART1_TX_BUF_LEN 1024

extern uint8_t  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern uint16_t USART_RX_STA;         		//����״̬���	

// ��װRTT��ʽ�����
int RTT_printf(unsigned BufferIndex, const char *sFormat, ...);

// ����봮���жϽ��գ��벻Ҫע�����º궨��
void uart1_Init(uint32_t bound);

//DMA����
void DMA_USART1_Send(uint8_t *data,uint16_t size);


#endif
