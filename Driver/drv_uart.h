/**
 * @file drv_uart.h
 * @author zrw
 * @brief 
 * @version 0.1
 * @date 2019-03-05
 * 
 * @copyright Copyright (c) 2019
 * 
 */


#ifndef __DRV_UART_H__
#define __DRV_UART_H__

#include "HeaderInclude.h"


#define _DEBUG_USART_NUM(a) USART##a
#define DEBUG_USART_NUM(a) _DEBUG_USART_NUM(a)
#define DEBUG_PORT      DEBUG_USART_NUM(DEBUG_UART)
//#define DEBUG

#ifdef DEBUG
#define debug(arg...)   printf(##arg)
#else
#define debug(arg...)   
#endif


typedef struct
{
  unsigned char* rx;
  unsigned char* tx;
  unsigned short txIndex;
  unsigned short txLen;
  void (*_uartIrq)(unsigned char ch);
  void (*_uartRecOverIrq)(unsigned char* p,unsigned short len);
}UART;



void Uart_Init(uint8_t id, uint32_t UartBaudRate );
void Uart_Send(uint8_t id, uint8_t *TxBuffer, uint8_t Length );
void Uart_Attach(uint8_t id, void (*_uartIrq)(unsigned char ch));
void Uart_Attach_Rec_Over(uint8_t id, void (*_uartRecOverIrq)(unsigned char* p,unsigned short len));
void myprintf(const char *format, ...);

#endif



