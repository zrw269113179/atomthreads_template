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

#endif