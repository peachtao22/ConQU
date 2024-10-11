#include "DJ_d16.h"

Yaokong yaokong;


uint8_t RxBuf[32] = {0};

void DJ16_Init(UART_HandleTypeDef *huart)
{
  __HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
  HAL_UARTEx_ReceiveToIdle_DMA(huart,RxBuf,18);
};

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t size)
{
  if(huart -> Instance == USART3)
  {  
//		memcpy(RxBuf_S, RxBuf, sizeof(RxBuf));
//     HAL_UART_DMAStop(huart);
//     yaokong.ch0 = (RxBuf_S[0]|(RxBuf_S[1]<<8))&0x07ff;
//     yaokong.ch1 = ((RxBuf_S[1]>>3)|(RxBuf_S[2]<<5))&0x07ff;
//     yaokong.ch2 = ((RxBuf_S[2]>>6)|(RxBuf_S[3]<<2)|(RxBuf_S[4]<<10))&0x07ff;
//     yaokong.ch3 = ((RxBuf_S[4]>>1)|(RxBuf_S[5]<<7))&0x07ff;

//     yaokong.s0 =((RxBuf_S[5]>>4)&0x000C) >> 2;
//     yaokong.s1 =((RxBuf_S[5]>>4)&0x0003);
	 yaokong.ch0 = (RxBuf[0]|(RxBuf[1]<<8))&0x07ff;
     yaokong.ch1 = ((RxBuf[1]>>3)|(RxBuf[2]<<5))&0x07ff;
     yaokong.ch2 = ((RxBuf[2]>>6)|(RxBuf[3]<<2)|(RxBuf[4]<<10))&0x07ff;
     yaokong.ch3 = ((RxBuf[4]>>1)|(RxBuf[5]<<7))&0x07ff;

     yaokong.s0 =((RxBuf[5]>>4)&0x000C) >> 2;
     yaokong.s1 =((RxBuf[5]>>4)&0x0003);
     HAL_UARTEx_ReceiveToIdle_DMA(&huart3,RxBuf,18);
  };
};