#include "main.h"
#include "sccb.h"
#include "ov7670.h"
#include "dcmi.h"
#include "stm32f4xx_dma.h"                  // Device header



// 添加帧同步头尾和校验
#define FRAME_HEADER 0xFE
#define FRAME_FOOTER 0xFF

void SendCameraFrame(uint16_t *buffer, uint16_t width, uint16_t height);
uint8_t len=10;
		
int main(void)
{

	int ov_rev_ok  = 0;
	//SCCB_Init();
	
	//OV7670_RST_PW_Init();
    /***********************初始化代码***********************/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 设置系统中断优先级分组2
    delay_init(168);                                // 延时初始化
    uart1_Init(1000000);//115200   230400  57600    460800   1000000    超过3000000就不行了               // 串口1初始化
    SEGGER_RTT_Init();  //初始化 RTT 控制块。
    usmart_init(168);   //usmart初始化
    /***********************初始化完成***********************/
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
			delay_ms(100); // 给接收端处理时间
			delay_ms(13500); // 预留13.5秒传输时间 13500
			ov_rev_ok = 0;
		}
			
    }
		
}



//// 带协议的数据发送
//void SendCameraFrame(uint16_t *buffer, uint16_t width, uint16_t height) {
//    uint32_t payload_size = width * height * 2; // RGB565数据量
//    uint8_t checksum = 0;
//    
//    // 发送帧头
//    uint8_t header[] = {FRAME_HEADER, FRAME_HEADER};
//    //USART1_SendBlocking(header, sizeof(header));
//	DMA_USART1_Send(header,sizeof(header));
//    
//    // 发送长度信息（小端格式）
//    uint8_t len_bytes[4] = {
//        payload_size & 0xFF,
//        (payload_size >> 8) & 0xFF,
//        (payload_size >> 16) & 0xFF,
//        (payload_size >> 24) & 0xFF
//    };
//    //USART1_SendBlocking(len_bytes, 4);
//	DMA_USART1_Send(len_bytes,4);
//    
//    // 发送有效载荷
//    uint8_t *p = (uint8_t*)buffer;
//    for(uint32_t i=0; i<payload_size; i++) {
//        checksum ^= p[i]; // 简单异或校验
//        //USART1_SendBlocking(&p[i], 1);
//		DMA_USART1_Send(&p[i],1);
//    }
//    
//    // 发送校验和
//    //USART1_SendBlocking(&checksum, 1);
//	DMA_USART1_Send(&checksum, 1);
//    
//    // 发送帧尾
//    uint8_t footer[] = {FRAME_FOOTER, FRAME_FOOTER};
//    //USART1_SendBlocking(footer, sizeof(footer));
//	DMA_USART1_Send(footer, sizeof(footer));
//}

void SendCameraFrame(uint16_t *buffer, uint16_t width, uint16_t height) {
    uint32_t payload_size = width * height * 2;
    uint8_t checksum = 0;

    // 发送帧头
    uint8_t header[] = {FRAME_HEADER, FRAME_HEADER};
    //DMA_USART1_Send(header, sizeof(header));

    // 发送长度（小端）
    uint8_t len_bytes[4] = {
        (uint8_t)(payload_size),
        (uint8_t)(payload_size >> 8),
        (uint8_t)(payload_size >> 16),
        (uint8_t)(payload_size >> 24)
    };

    //DMA_USART1_Send(len_bytes, 4);
		//DMA_USART1_Send(len, 2);

    // 发送Payload（调整字节序）
    uint8_t *p = (uint8_t*)buffer;
    for (uint32_t i = 0; i < payload_size; i += 2) {
        // 假设需要高字节在前
        uint8_t high_byte = p[i + 1];
        uint8_t low_byte = p[i];
        DMA_USART1_Send(&high_byte, 1);
        DMA_USART1_Send(&low_byte, 1);
				
				//printf("%d",low_byte);
        checksum ^= high_byte;
        checksum ^= low_byte;
    }

    // 发送校验和及帧尾
    //DMA_USART1_Send(&checksum, 1);
    uint8_t footer[] = {FRAME_FOOTER, FRAME_FOOTER};
    //DMA_USART1_Send(footer, sizeof(footer));
}
