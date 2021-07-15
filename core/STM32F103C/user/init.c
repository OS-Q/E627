/******************************************************************************
****版本：0.9.0
****平台：STM32F103C
****日期：2021-07-12
****作者：Qitas
****版权：OS-Q
*******************************************************************************/
#include "init.h"

lwrb_t usart_tx_dma_ringbuff;
lwrb_t usart_rx_dma_ringbuff;
size_t usart_tx_dma_current_len;
uint8_t usart_rx_dma_buffer[64];
uint8_t usart_tx_dma_lwrb_data[128];
uint8_t usart_rx_dma_lwrb_data[128];
/*******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
********************************************************************************/
void hal_init(void)
{
    lwrb_init(&usart_tx_dma_ringbuff, usart_tx_dma_lwrb_data, sizeof(usart_tx_dma_lwrb_data));
    lwrb_init(&usart_rx_dma_ringbuff, usart_rx_dma_lwrb_data, sizeof(usart_rx_dma_lwrb_data));
    /* USART RX DMA Init */
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&USART1->DR);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)usart_rx_dma_buffer);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, ARRAY_LEN(usart_rx_dma_buffer));

    /* USART TX DMA Init */
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)&USART1->DR);

    // LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&USART1->DR);
    // LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)usart_rx_dma_buffer);
    // LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, ARRAY_LEN(usart_rx_dma_buffer));
    // LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)&USART1->DR);

    /* Enable HT & TC interrupts */
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_5);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);

    LL_USART_EnableDMAReq_RX(USART1);
    LL_USART_EnableDMAReq_TX(USART1);
    LL_USART_EnableIT_IDLE(USART1);

    /* Enable USART and DMA */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
}
/*******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
********************************************************************************/
uint8_t usart_start_tx_dma_transfer(void)
{
    uint32_t old_primask;
    uint8_t started = 0;
    if (usart_tx_dma_current_len > 0) {
        return 0;
    }
    old_primask = __get_PRIMASK();
    __disable_irq();

    /* data to send */
    if (usart_tx_dma_current_len == 0
            && (usart_tx_dma_current_len = lwrb_get_linear_block_read_length(&usart_tx_dma_ringbuff)) > 0) {
        /* Disable channel if enabled */
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);

        LL_DMA_ClearFlag_TC4(DMA1);
        LL_DMA_ClearFlag_HT4(DMA1);
        LL_DMA_ClearFlag_GI4(DMA1);
        LL_DMA_ClearFlag_TE4(DMA1);

        /* Start DMA transfer */
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, usart_tx_dma_current_len);
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)lwrb_get_linear_block_read_address(&usart_tx_dma_ringbuff));

        /* Start new transfer */
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
        started = 1;
    }
    __set_PRIMASK(old_primask);
    return started;
}
/*******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
********************************************************************************/
void usart_process_data(const void* data, size_t len)
{
    // const uint8_t* d = data;
    // for (; len > 0; --len, ++d) {
    //     LL_USART_TransmitData8(USART1, *d);
    //     while (!LL_USART_IsActiveFlag_TXE(USART1)) {}
    // }
    // while (!LL_USART_IsActiveFlag_TC(USART1)) {}
    lwrb_write(&usart_tx_dma_ringbuff, data, len);
}

void usart_send_string(const char* str)
{
    usart_process_data(str, strlen(str));
    usart_start_tx_dma_transfer();
}


/*******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
********************************************************************************/

void usart_rx_check(void)
{
    static size_t old_pos;
    size_t pos;
    /* Calculate current position in buffer */
    pos = ARRAY_LEN(usart_rx_dma_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5);
    if (pos != old_pos) {                       /* Check change in received data */
        if (pos > old_pos) {                    /* Current position is over previous one */
            usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
        }
        else {
            /* We are in "overflow" mode */
            usart_process_data(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);
            /* Check and continue with beginning of buffer */
            if (pos > 0) {
                usart_process_data(&usart_rx_dma_buffer[0], pos);
            }
        }
        old_pos = pos;
    }
}


/*******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
********************************************************************************/
void DMA_Handler(void)
{
    if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_CHANNEL_5) && LL_DMA_IsActiveFlag_HT5(DMA1)) {
        LL_DMA_ClearFlag_HT5(DMA1);             /* Clear half-transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
    }
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_5) && LL_DMA_IsActiveFlag_TC5(DMA1)) {
        LL_DMA_ClearFlag_TC5(DMA1);             /* Clear transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
    }
}
/*******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
********************************************************************************/
void DMA4_Handler(void)
{
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_4) && LL_DMA_IsActiveFlag_TC4(DMA1)) {
        LL_DMA_ClearFlag_TC4(DMA1);             /* Clear transfer complete flag */
        lwrb_skip(&usart_tx_dma_ringbuff, usart_tx_dma_current_len);/* Skip sent data, mark as read */
        usart_tx_dma_current_len = 0;           /* Clear length variable */
        usart_start_tx_dma_transfer();          /* Start sending more data */
    }
}
/*******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
********************************************************************************/
void UART_Handler(void)
{
    if (LL_USART_IsEnabledIT_IDLE(USART1) && LL_USART_IsActiveFlag_IDLE(USART1)) {
        LL_USART_ClearFlag_IDLE(USART1);        /* Clear IDLE line flag */
        usart_rx_check();                       /* Check for data to process */
    }
}
