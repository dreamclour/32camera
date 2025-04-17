#include "main.h"
#include "sccb.h"
#include "ov7670.h"
#include "dcmi.h"
#include "stm32f4xx_dma.h"                  // Device header



// ���֡ͬ��ͷβ��У��
#define FRAME_HEADER 0xFE
#define FRAME_FOOTER 0xFF

void SendCameraFrame(uint16_t *buffer, uint16_t width, uint16_t height);
uint8_t len=10;
		
int main(void)
{

	int ov_rev_ok  = 0;
	//SCCB_Init();
	
	//OV7670_RST_PW_Init();
    /***********************��ʼ������***********************/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // ����ϵͳ�ж����ȼ�����2
    delay_init(168);                                // ��ʱ��ʼ��
    uart1_Init(1000000);//115200   230400  57600    460800   1000000    ����3000000�Ͳ�����               // ����1��ʼ��
    SEGGER_RTT_Init();  //��ʼ�� RTT ���ƿ顣
    usmart_init(168);   //usmart��ʼ��
    /***********************��ʼ�����***********************/
		OV7670_Init();
		//printf("%d",len);
	
	//OV7670_Special_Effects(0);
	//OV7670_Light_Mode(4);
		
    while (1)
    {
		if(ov_rev_ok == 0)
		{
			DCMI_Start();
			delay_ms(10);
			DCMI_Stop();
			delay_ms(50);
			ov_rev_ok = 1;
		}
		
		if(ov_rev_ok) {
			SendCameraFrame(camera_buffer, PIC_WIDTH, PIC_HEIGHT);
			//printf("Frame Sent\r\n");
			delay_ms(100); // �����ն˴���ʱ��
			delay_ms(13500); // Ԥ��13.5�봫��ʱ�� 13500
			ov_rev_ok = 0;
		}
			
    }
		
}



//// ��Э������ݷ���
//void SendCameraFrame(uint16_t *buffer, uint16_t width, uint16_t height) {
//    uint32_t payload_size = width * height * 2; // RGB565������
//    uint8_t checksum = 0;
//    
//    // ����֡ͷ
//    uint8_t header[] = {FRAME_HEADER, FRAME_HEADER};
//    //USART1_SendBlocking(header, sizeof(header));
//	DMA_USART1_Send(header,sizeof(header));
//    
//    // ���ͳ�����Ϣ��С�˸�ʽ��
//    uint8_t len_bytes[4] = {
//        payload_size & 0xFF,
//        (payload_size >> 8) & 0xFF,
//        (payload_size >> 16) & 0xFF,
//        (payload_size >> 24) & 0xFF
//    };
//    //USART1_SendBlocking(len_bytes, 4);
//	DMA_USART1_Send(len_bytes,4);
//    
//    // ������Ч�غ�
//    uint8_t *p = (uint8_t*)buffer;
//    for(uint32_t i=0; i<payload_size; i++) {
//        checksum ^= p[i]; // �����У��
//        //USART1_SendBlocking(&p[i], 1);
//		DMA_USART1_Send(&p[i],1);
//    }
//    
//    // ����У���
//    //USART1_SendBlocking(&checksum, 1);
//	DMA_USART1_Send(&checksum, 1);
//    
//    // ����֡β
//    uint8_t footer[] = {FRAME_FOOTER, FRAME_FOOTER};
//    //USART1_SendBlocking(footer, sizeof(footer));
//	DMA_USART1_Send(footer, sizeof(footer));
//}

void SendCameraFrame(uint16_t *buffer, uint16_t width, uint16_t height) {
    uint32_t payload_size = width * height * 2;
    uint8_t checksum = 0;

    // ����֡ͷ
    uint8_t header[] = {FRAME_HEADER, FRAME_HEADER};
    //DMA_USART1_Send(header, sizeof(header));

    // ���ͳ��ȣ�С�ˣ�
    uint8_t len_bytes[4] = {
        (uint8_t)(payload_size),
        (uint8_t)(payload_size >> 8),
        (uint8_t)(payload_size >> 16),
        (uint8_t)(payload_size >> 24)
    };

    //DMA_USART1_Send(len_bytes, 4);
		//DMA_USART1_Send(len, 2);

    // ����Payload�������ֽ���
    uint8_t *p = (uint8_t*)buffer;
    for (uint32_t i = 0; i < payload_size; i += 2) {
        // ������Ҫ���ֽ���ǰ
        uint8_t high_byte = p[i + 1];
        uint8_t low_byte = p[i];
        DMA_USART1_Send(&high_byte, 1);
        DMA_USART1_Send(&low_byte, 1);
				
				//printf("%d",low_byte);
        checksum ^= high_byte;
        checksum ^= low_byte;
    }

    // ����У��ͼ�֡β
    //DMA_USART1_Send(&checksum, 1);
    uint8_t footer[] = {FRAME_FOOTER, FRAME_FOOTER};
    //DMA_USART1_Send(footer, sizeof(footer));
}
