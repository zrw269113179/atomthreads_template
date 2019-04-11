/**
 * @file drv_uart.c
 * @author zrw
 * @brief uart串口驱动程序
 * @version 0.1
 * @date 2019-03-05
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "HeaderInclude.h"

#ifdef _RAISONANCE_
#define PUTCHAR_PROTOTYPE int putchar (char c)
#define GETCHAR_PROTOTYPE int getchar (void)
#elif defined (_COSMIC_)
#define PUTCHAR_PROTOTYPE char putchar (char c)
#define GETCHAR_PROTOTYPE char getchar (void)
#else /* _IAR_ */
#define PUTCHAR_PROTOTYPE int putchar (int c)
#define GETCHAR_PROTOTYPE int getchar (void)
#endif /* _RAISONANCE_ */

#if DEBUG_UART == 1 || DEBUG_UART == 2 || DEBUG_UART == 3
static ATOM_MUTEX uart_mutex;
#endif

static UART _uart[3];

#if defined(UART1_DMA)

unsigned char uart1_tx[UART1_TX_LEN];
unsigned char uart1_rx[UART1_RX_LEN];

#elif defined(UART2_DMA)
unsigned char uart2_tx[UART2_TX_LEN];
unsigned char uart2_rx[UART2_RX_LEN];

#elif defined(UART3_DMA)

unsigned char uart3_tx[UART3_TX_LEN];
unsigned char uart3_rx[UART3_RX_LEN];

#endif

#define UART1_DR_ADDRESS                 (uint16_t)0x5231
#define UART2_DR_ADDRESS                 (uint16_t)0x53E1
#define UART3_DR_ADDRESS                 (uint16_t)0x53F1


/**
  * @brief :串口初始化
  * @param :无
  * @note  :无
  * @retval:无
  */
void Uart_Init(uint8_t id, uint32_t UartBaudRate)
{
    switch (id)
    {

    case 1:
        //串口引脚配置 TX推挽输出 RX上拉输入
        GPIO_Init(GPIOC, GPIO_Pin_3, GPIO_Mode_Out_PP_High_Slow);
        GPIO_Init(GPIOC, GPIO_Pin_2, GPIO_Mode_In_PU_No_IT);

        //USART外设配置
        CLK_PeripheralClockConfig(CLK_Peripheral_USART1, ENABLE); //使能串口时钟
        USART_DeInit(USART1);                                     //串口复位
        //串口初始化 8位数据 1个停止位 无校验 发送接收 波特率可变
        USART_Init(USART1, UartBaudRate, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, (USART_Mode_TypeDef)(USART_Mode_Tx | USART_Mode_Rx));

#ifdef UART1_DMA
        USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
        CLK_PeripheralClockConfig(CLK_Peripheral_DMA1, ENABLE);
        _uart[0].tx = uart1_tx;
        _uart[0].rx = uart1_rx;

        DMA_DeInit(DMA1_Channel1);
        DMA_DeInit(DMA1_Channel2);
        DMA_Init(DMA1_Channel1, (uint16_t)_uart[0].tx, UART1_DR_ADDRESS,    UART1_TX_LEN,  DMA_DIR_MemoryToPeripheral, DMA_Mode_Normal,    DMA_MemoryIncMode_Inc, DMA_Priority_High, DMA_MemoryDataSize_Byte);
        DMA_Init(DMA1_Channel2, (uint16_t)_uart[0].rx, UART1_DR_ADDRESS,    UART1_RX_LEN,  DMA_DIR_PeripheralToMemory, DMA_Mode_Normal,    DMA_MemoryIncMode_Inc, DMA_Priority_High, DMA_MemoryDataSize_Byte);
        USART_DMACmd(USART1, USART_DMAReq_RX, ENABLE);
        USART_DMACmd(USART1, USART_DMAReq_TX, ENABLE);

        DMA_Cmd(DMA1_Channel1, ENABLE);
        DMA_Cmd(DMA1_Channel2, ENABLE);
#else
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);
#endif
        USART_Cmd(USART1, ENABLE); //使能串口

        break;

    case 2:

        //串口引脚配置 TX推挽输出 RX上拉输入
        GPIO_Init(GPIOE, GPIO_Pin_4, GPIO_Mode_Out_PP_High_Slow);
        GPIO_Init(GPIOE, GPIO_Pin_3, GPIO_Mode_In_PU_No_IT);

        //USART外设配置
        CLK_PeripheralClockConfig(CLK_Peripheral_USART2, ENABLE); //使能串口时钟
        USART_DeInit(USART2);                                     //串口复位
        //串口初始化 8位数据 1个停止位 无校验 发送接收 波特率可变
        USART_Init(USART2, UartBaudRate, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, (USART_Mode_TypeDef)(USART_Mode_Tx | USART_Mode_Rx));
#ifdef UART2_DMA
        USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
        CLK_PeripheralClockConfig(CLK_Peripheral_DMA1, ENABLE);
        _uart[1].tx = uart2_tx;
        _uart[1].rx = uart2_rx;

        DMA_DeInit(DMA1_Channel0);
        DMA_DeInit(DMA1_Channel4);
        DMA_Init(DMA1_Channel0, (uint16_t)_uart[1].tx, UART2_DR_ADDRESS,    UART2_TX_LEN,  DMA_DIR_MemoryToPeripheral, DMA_Mode_Normal,    DMA_MemoryIncMode_Inc, DMA_Priority_High, DMA_MemoryDataSize_Byte);
        DMA_Init(DMA1_Channel4, (uint16_t)_uart[1].rx, UART2_DR_ADDRESS,    UART2_RX_LEN,  DMA_DIR_PeripheralToMemory, DMA_Mode_Normal,    DMA_MemoryIncMode_Inc, DMA_Priority_High, DMA_MemoryDataSize_Byte);
        USART_DMACmd(USART2, USART_DMAReq_RX, ENABLE);
        USART_DMACmd(USART2, USART_DMAReq_TX, ENABLE);

        DMA_Cmd(DMA1_Channel0, ENABLE);
        DMA_Cmd(DMA1_Channel4, ENABLE);
#else
        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
        USART_ITConfig(USART2, USART_IT_TC, DISABLE);
#endif


        USART_Cmd(USART2, ENABLE); //使能串口
        break;

    case 3:
        //串口引脚配置 TX推挽输出 RX上拉输入
        GPIO_Init(GPIOG, GPIO_Pin_1, GPIO_Mode_Out_PP_High_Slow);
        GPIO_Init(GPIOG, GPIO_Pin_0, GPIO_Mode_In_PU_No_IT);

        //USART外设配置
        CLK_PeripheralClockConfig(CLK_Peripheral_USART3, ENABLE); //使能串口时钟
        USART_DeInit(USART3);                                     //串口复位
        //串口初始化 8位数据 1个停止位 无校验 发送接收 波特率可变
        USART_Init(USART3, UartBaudRate, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, (USART_Mode_TypeDef)(USART_Mode_Tx | USART_Mode_Rx));
#ifdef UART3_DMA
        USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
        CLK_PeripheralClockConfig(CLK_Peripheral_DMA1, ENABLE);
        _uart[2].tx = uart3_tx;
        _uart[2].rx = uart3_rx;

        DMA_DeInit(DMA1_Channel1);
        DMA_DeInit(DMA1_Channel2);
        DMA_Init(DMA1_Channel1, (uint16_t)_uart[2].tx, UART3_DR_ADDRESS,    UART3_TX_LEN,  DMA_DIR_MemoryToPeripheral, DMA_Mode_Normal,    DMA_MemoryIncMode_Inc, DMA_Priority_Low, DMA_MemoryDataSize_Byte);
        DMA_Init(DMA1_Channel2, (uint16_t)_uart[2].rx, UART3_DR_ADDRESS,    UART3_RX_LEN,  DMA_DIR_PeripheralToMemory, DMA_Mode_Normal,    DMA_MemoryIncMode_Inc, DMA_Priority_High, DMA_MemoryDataSize_Byte);
        USART_DMACmd(USART3, USART_DMAReq_RX, ENABLE);
        USART_DMACmd(USART3, USART_DMAReq_TX, ENABLE);

        DMA_Cmd(DMA1_Channel1, ENABLE);
        DMA_Cmd(DMA1_Channel2, ENABLE);
#else
        USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
        USART_ITConfig(USART3, USART_IT_TC, DISABLE);
#endif


        USART_Cmd(USART3, ENABLE); //使能串口
        break;

    default:
        break;
    }
#if DEBUG_UART == 1 || DEBUG_UART == 2 || DEBUG_UART == 3
    atomMutexCreate(&uart_mutex);
#endif

}


/**
 * \b uart_putchar
 *
 * Write a char out via UART1
 *
 * @param[in] c Character to send
 *
 * @return Character sent
 */
char uart_putchar (char c)
{
    /* Block on private access to the UART */
    if (atomMutexGet(&uart_mutex, 0) == ATOM_OK)
    {
        /* Convert \n to \r\n */
        if (c == '\n')
        {
            putchar('\r');

        }

        /* Write a character to the USART1 */
        USART_SendData8(DEBUG_PORT, c);
        /* Loop until the end of transmission */
        while (USART_GetFlagStatus(DEBUG_PORT, USART_FLAG_TC) == RESET);


        /* Return mutex access */
        atomMutexPut(&uart_mutex);

    }

    return (c);
}
/* COSMIC: Requires putchar() routine to override stdio */
#if defined(__CSMC__)
/**
 * \b putchar
 *
 * Retarget putchar() to use UART1
 *
 * @param[in] c Character to send
 *
 * @return Character sent
 */
char putchar (char c)
{
    return (uart_putchar(c));
}
#endif /* __CSMC__ */


/* RAISONANCE: Requires putchar() routine to override stdio */
#if defined(__RCSTM8__)
/**
 * \b putchar
 *
 * Retarget putchar() to use UART1
 *
 * @param[in] c Character to send
 *
 * @return 1 on success
 */
int putchar (char c)
{
    uart_putchar(c);
    return (1);
}
#endif /* __RCSTM8__ */


/* IAR: Requires __write() routine to override stdio */
#if defined(__IAR_SYSTEMS_ICC__)
/**
 * \b __write
 *
 * Override for IAR stream output
 *
 * @param[in] handle Stdio handle. -1 to flush.
 * @param[in] buf Pointer to buffer to be written
 * @param[in] bufSize Number of characters to be written
 *
 * @return Number of characters sent
 */
size_t __write(int handle, const unsigned char *buf, size_t bufSize)
{
    size_t chars_written = 0;

    /* Ignore flushes */
    if (handle == -1)
    {
        chars_written = (size_t)0;
    }
    /* Only allow stdout/stderr output */
    else if ((handle != 1) && (handle != 2))
    {
        chars_written = (size_t) -1;
    }
    /* Parameters OK, call the low-level character output routine */
    else
    {
        while (chars_written < bufSize)
        {
            uart_putchar (buf[chars_written]);
            chars_written++;
        }
    }

    return (chars_written);
}
#endif /* __IAR_SYSTEMS_ICC__ */


/**
  * @brief :串口发送数据
  * @param :
  *         @TxBuffer:发送数据首地址
  *         @Length:数据长度
  * @note  :无
  * @retval:无
  */
void Uart_Send(uint8_t id, uint8_t *TxBuffer, uint8_t Length)
{

    _uart[id - 1].txLen = Length;

    _uart[id - 1].txIndex = 0;
#ifndef UART1_DMA
    if (id == 1)
    {
        _uart[id - 1].tx = TxBuffer;
        USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

        USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    }
#else
    if (id == 1)
    {
        memcpy(uart1_tx, TxBuffer, Length);
        DMA_Cmd(DMA1_Channel1, DISABLE);
        DMA_SetCurrDataCounter(DMA1_Channel1, Length);
        DMA_Cmd(DMA1_Channel1, ENABLE);
    }
#endif
#ifndef UART2_DMA
    else if (id == 2)
    {
        _uart[id - 1].tx = TxBuffer;
        USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);

        USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    }
#else
    else if (id == 2)
    {
        memcpy(uart2_tx, TxBuffer, Length);
        DMA_Cmd(DMA1_Channel0, DISABLE);
        DMA_SetCurrDataCounter(DMA1_Channel0, Length);
        DMA_Cmd(DMA1_Channel0, ENABLE);
    }
#endif
#ifndef UART3_DMA
    else if (id == 3)
    {
        _uart[id - 1].tx = TxBuffer;
        USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);

        USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
    }
#else
    else if (id == 3)
    {
        memcpy(uart3_tx, TxBuffer, Length);
        DMA_Cmd(DMA1_Channel1, DISABLE);
        DMA_SetCurrDataCounter(DMA1_Channel1, Length);
        DMA_Cmd(DMA1_Channel1, ENABLE);
    }
#endif
}
/**
 * @brief 绑定中断处理函数
 *
 * @param id 串口号
 * @param _uartIrq 中断处理函数
 * @note 仅在不使用dma的串口中调用
 */
void Uart_Attach(uint8_t id, void (*_uartIrq)(unsigned char ch))
{
    #ifndef UART1_DMA
    if (id == 1)
    {
        _uart[0]._uartIrq = _uartIrq;
    }
    #endif
    #ifndef UART2_DMA
    if (id == 2)
    {
        _uart[1]._uartIrq = _uartIrq;
    }
    #endif
    #ifndef UART3_DMA
    if (id == 3)
    {
        _uart[2]._uartIrq = _uartIrq;
    }
    #endif
}
/**
 * @brief 绑定串口接收完毕处理函数
 *
 * @param id 串口号
 * @param _uartRecOverIrq 中断处理函数
 * @note 仅在使用dma的串口中调用
 */
void Uart_Attach_Rec_Over(uint8_t id, void (*_uartRecOverIrq)(unsigned char *p, unsigned short len))
{
    #ifdef UART1_DMA 
    if (id == 1)
    {
        _uart[0]._uartRecOverIrq = _uartRecOverIrq;
    }
    #endif
    #ifdef UART2_DMA
    if (id == 2)
    {
        _uart[1]._uartRecOverIrq = _uartRecOverIrq;
    }
    #endif
    #ifdef UART3_DMA
    if (id == 3)
    {
        _uart[2]._uartRecOverIrq = _uartRecOverIrq;
    }
    #endif
}
/**
  * @brief USART1 TX / TIM5 Update/Overflow/Trigger/Break Interrupt  routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(USART1_TX_TIM5_UPD_OVF_TRG_BRK_IRQHandler, 27)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
#ifndef UART1_DMA
    if (SET == USART_GetITStatus(USART1, USART_IT_TXE))
    {
        if (_uart[0].txIndex >= _uart[0].txLen)
        {
            _uart[0].txIndex = 0;
            USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        }
        else
        {
            USART_SendData8(USART1, _uart[0].tx[_uart[0].txIndex]);
            _uart[0].txIndex++;
        }
        USART_ClearITPendingBit(USART1, USART_IT_TXE);
    }
#endif
}

/**
  * @brief USART1 RX / Timer5 Capture/Compare Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(USART1_RX_TIM5_CC_IRQHandler, 28)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
    if (SET == USART_GetITStatus(USART1, USART_IT_RXNE))
    {
        if (_uart[0]._uartIrq)
        {
            _uart[0]._uartIrq(USART1->DR);
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
#ifdef UART1_DMA
    else if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) //IDLE interrupt
    {
        unsigned char DataLen;
        USART1->SR;//先读SR
        USART1->DR;//再读DR
        DataLen = UART1_RX_LEN - DMA_GetCurrDataCounter(DMA1_Channel2);
        //printf("date len:%d", DataLen);
        if(_uart[0]._uartRecOverIrq)
        {
            _uart[0]._uartRecOverIrq(_uart[0].rx,DataLen);
        }
        DMA_Cmd(DMA1_Channel2, DISABLE);
        DMA_SetCurrDataCounter(DMA1_Channel2, UART1_RX_LEN);
        DMA_Cmd(DMA1_Channel2, ENABLE);
    }
#endif
}

/**
  * @brief TIM2 Update/Overflow/Trigger/Break /USART2 TX Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM2_UPD_OVF_TRG_BRK_USART2_TX_IRQHandler, 19)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
#ifndef UART2_DMA
    if (SET == USART_GetITStatus(USART2, USART_IT_TXE))
    {
        if (_uart[1].txIndex >= _uart[1].txLen)
        {
            _uart[1].txIndex = 0;
            USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        }
        else
        {
            USART_SendData8(USART2, _uart[1].tx[_uart[1].txIndex]);
            _uart[1].txIndex++;
        }
        USART_ClearITPendingBit(USART2, USART_IT_TXE);
    }
#endif
}
/**
  * @brief Timer2 Capture/Compare / USART2 RX Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM2_CC_USART2_RX_IRQHandler, 20)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
    if (SET == USART_GetITStatus(USART2, USART_IT_RXNE))
    {
        if (_uart[1]._uartIrq)
        {
            _uart[1]._uartIrq(USART2->DR);
        }

        USART_SendData8(USART3, USART2->DR);

        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
#ifdef UART2_DMA
    else if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) //IDLE interrupt
    {
        unsigned char DataLen;
        USART2->SR;//先读SR
        USART2->DR;//再读DR
        DataLen = UART2_RX_LEN - DMA_GetCurrDataCounter(DMA1_Channel4);
        //printf("date len:%d", DataLen);
        if(_uart[1]._uartRecOverIrq)
        {
            _uart[1]._uartRecOverIrq(_uart[1].rx,DataLen);
        }
        DMA_Cmd(DMA1_Channel4, DISABLE);
        DMA_SetCurrDataCounter(DMA1_Channel4, UART2_RX_LEN);
        DMA_Cmd(DMA1_Channel4, ENABLE);
    }
#endif
}

/**
  * @brief Timer3 Update/Overflow/Trigger/Break Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM3_UPD_OVF_TRG_BRK_USART3_TX_IRQHandler, 21)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
#ifndef UART3_DMA
    if (SET == USART_GetITStatus(USART3, USART_IT_TXE))
    {

        if (_uart[2].txIndex >= _uart[2].txLen)
        {
            _uart[2].txIndex = 0;
            USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

            USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
        }
        else
        {
            USART_SendData8(USART3, _uart[2].tx[_uart[2].txIndex]);
            _uart[2].txIndex++;
        }

        USART_ClearITPendingBit(USART3, USART_IT_TXE);
    }
#endif
}

/**
  * @brief Timer3 Capture/Compare /USART3 RX Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM3_CC_USART3_RX_IRQHandler, 22)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
    if (SET == USART_GetITStatus(USART3, USART_IT_RXNE))
    {
        /*if (_uart[2]._uartIrq)
        {
            _uart[2]._uartIrq(USART3->DR);
        }*/
        USART_SendData8(USART2, USART3->DR);

        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
#ifdef UART3_DMA
    else if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) //IDLE interrupt
    {
        unsigned char DataLen;
        USART3->SR;//先读SR
        USART3->DR;//再读DR
        DataLen = UART3_RX_LEN - DMA_GetCurrDataCounter(DMA1_Channel2);
        //printf("date len:%d", DataLen);
        //Uart_Send(3,_uart[2].rx,DataLen);
        if(_uart[2]._uartRecOverIrq)
        {
            _uart[2]._uartRecOverIrq(_uart[2].rx,DataLen);
        }
        DMA_Cmd(DMA1_Channel2, DISABLE);
        DMA_SetCurrDataCounter(DMA1_Channel2, UART3_RX_LEN);
        DMA_Cmd(DMA1_Channel2, ENABLE);
    }
#endif
}
