#ifndef __DJ_D16_H_
#define __DJ_D16_H_
#include "main.h"
#include "usart.h"
#include <string.h>


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t size);
void DJ16_Init(UART_HandleTypeDef *huart);

typedef struct 
{
  uint16_t ch0;
  uint16_t ch1;
  uint16_t ch2;
  uint16_t ch3;
  uint8_t s0;
  uint8_t s1;
  /* data */
}Yaokong;
extern Yaokong yaokong;


extern uint8_t RxBuf[32];



#endif
