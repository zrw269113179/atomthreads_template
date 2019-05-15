#ifndef	__HEADERINCLUDE_H__
#define __HEADERINCLUDE_H__

#include <atom.h>
#include <atommutex.h>
#include <atomtimer.h>
#include "atomport-private.h"

#include "stm8l15x.h"
#include "drv_uart.h"
#include "drv_pin.h"
#include <string.h>
#include <stdio.h>

#define DEBUG_UART      3
/**
 * @brief 同时仅1个使用uart，否则需要自己配置
 * 
 */
//#define UART1_DMA
//#define UART2_DMA
#define UART3_DMA

#if defined(UART1_DMA)

#define UART1_TX_LEN    20
#define UART1_RX_LEN    20

#elif defined(UART2_DMA)

#define UART2_TX_LEN    20
#define UART2_RX_LEN    20

#elif defined(UART3_DMA)

#define UART3_TX_LEN    20
#define UART3_RX_LEN    20

#endif

/**********通用**************/
#define BIT_SET(X,Y)    (X |= (1<<Y))
#define BIT_GET(X,Y)    (X & (1<<Y))
#define BIT_CLR(X,Y)    (X &= (~(1<<Y)))


#define LOG_NULL    0
#define LOG_DEBUG   5
#define LOG_INFO    4
#define LOG_WARN    3
#define LOG_ERROR   2
#define LOG_FATAL   1

#ifdef DEBUG_UART
#define LOG_LEVEL   LOG_WARN 
#else
#define LOG_LEVEL   LOG_NULL 
#endif


#if(LOG_LEVEL >= LOG_DEBUG)
#define LOG_D(format, args...) printf("Debug:"format"\n", ##args)
#else
#define LOG_D(format, args...)
#endif
#if(LOG_LEVEL >= LOG_INFO)
#define LOG_I(format, args...) printf("Info:"format"\n", ##args)
#else
#define LOG_I(format, args...)
#endif
#if(LOG_LEVEL >= LOG_WARN)
#define LOG_W(format, args...) printf("Warn:"format"\n", ##args)
#else
#define LOG_W(format, args...)
#endif
#if(LOG_LEVEL >= LOG_ERROR)
#define LOG_E(format, args...) printf("Error:File: "__FILE__", Line: %05d:" format"\n", ##args)
#else
#define LOG_E(format, args...)
#endif
#if(LOG_LEVEL >= LOG_FATAL)
#define LOG_F(format, args...) printf("Fatal:File: "__FILE__", Line: %05d:" format"\n", ##args)
#else
#define LOG_F(format, args...)
#endif

#endif