
#include "init.h"
#include "dma.h"
#include "usart.h"

lwrb_t usart_tx_dma_ringbuff;
size_t usart_tx_dma_current_len;
uint8_t usart_rx_dma_buffer[64];
uint8_t usart_tx_dma_lwrb_data[128];
/*******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
********************************************************************************/
void hal_init(void)
{

    lwrb_init(&usart_tx_dma_ringbuff, usart_tx_dma_lwrb_data, sizeof(usart_tx_dma_lwrb_data));

    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    /*
     * USART2 GPIO Configuration
     *
     * PA2   ------> USART2_TX
     * PA3   ------> USART2_RX
     */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 RX DMA Init */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_USART2_RX);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&USART2->RDR);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)usart_rx_dma_buffer);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, ARRAY_LEN(usart_rx_dma_buffer));

    /* USART2 TX DMA Init */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMAMUX_REQ_USART2_TX);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&USART2->TDR);

    /* Enable DMA RX HT & TC interrupts */
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
    /* Enable DMA TX TC interrupts */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);

    /* DMA interrupt init */
    NVIC_SetPriority(DMA1_Channel2_3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

    /* Configure USART2 */
    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART2, &USART_InitStruct);
    LL_USART_SetTXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_DisableFIFO(USART2);
    LL_USART_ConfigAsyncMode(USART2);
    LL_USART_EnableDMAReq_RX(USART2);
    LL_USART_EnableDMAReq_TX(USART2);
    LL_USART_EnableIT_IDLE(USART2);

    /* USART interrupt, same priority as DMA channel */
    NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART2_IRQn);

    /* Enable USART and DMA RX */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
    LL_USART_Enable(USART2);
}
/*******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
********************************************************************************/
void usart_process_data(const void* data, size_t len)
{
    lwrb_write(&usart_tx_dma_ringbuff, data, len);
    /* Start DMA transfer if not already */
    usart_start_tx_dma_transfer();
}

void usart_send_string(const char* str)
{
    usart_process_data(str, strlen(str));
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

    /* Pre-check if transfer active to avoid interrupt disable */
    if (usart_tx_dma_current_len > 0) {
        return 0;
    }

    /* Check if DMA is active */
    /* Must be set to 0 */
    old_primask = __get_PRIMASK();
    __disable_irq();

    /* data to send */
    if (usart_tx_dma_current_len == 0
            && (usart_tx_dma_current_len = lwrb_get_linear_block_read_length(&usart_tx_dma_ringbuff)) > 0) {
        /* Disable channel if enabled */
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);

        /* Clear all flags */
        LL_DMA_ClearFlag_TC3(DMA1);
        LL_DMA_ClearFlag_HT3(DMA1);
        LL_DMA_ClearFlag_GI3(DMA1);
        LL_DMA_ClearFlag_TE3(DMA1);

        /* Start DMA transfer */
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, usart_tx_dma_current_len);
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)lwrb_get_linear_block_read_address(&usart_tx_dma_ringbuff));

        /* Start new transfer */
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
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

void usart_rx_check(void)
{
    static size_t old_pos;
    size_t pos;

    /* Calculate current position in buffer */
    pos = ARRAY_LEN(usart_rx_dma_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_2);
    if (pos != old_pos) {                       /* Check change in received data */
        if (pos > old_pos) {                    /* Current position is over previous one */
            /* We are in "linear" mode */
            /* Process data directly by subtracting "pointers" */
            usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
        } else {
            /* We are in "overflow" mode */
            /* First process data to the end of buffer */
            usart_process_data(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);
            /* Check and continue with beginning of buffer */
            if (pos > 0) {
                usart_process_data(&usart_rx_dma_buffer[0], pos);
            }
        }
        old_pos = pos;                          /* Save current position as old */
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
    if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_CHANNEL_2) && LL_DMA_IsActiveFlag_HT2(DMA1)) {
        LL_DMA_ClearFlag_HT2(DMA1);             /* Clear half-transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
    }

    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_2) && LL_DMA_IsActiveFlag_TC2(DMA1)) {
        LL_DMA_ClearFlag_TC2(DMA1);             /* Clear transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
    }

    /* Events for DMA Channel 3 = USART DMA TX */
    /* Check transfer complete */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_3) && LL_DMA_IsActiveFlag_TC3(DMA1)) {
        LL_DMA_ClearFlag_TC3(DMA1);             /* Clear transfer complete flag */
        lwrb_skip(&usart_tx_dma_ringbuff, usart_tx_dma_current_len);/* Skip sent data, mark as read */
        usart_tx_dma_current_len = 0;           /* Clear length variable */
        usart_start_tx_dma_transfer();          /* Start sending more data */
    }
}

void UART_Handler(void)
{
    if (LL_USART_IsEnabledIT_IDLE(USART2) && LL_USART_IsActiveFlag_IDLE(USART2)) {
        LL_USART_ClearFlag_IDLE(USART2);        /* Clear IDLE line flag */
        usart_rx_check();                       /* Check for data to process */
    }
}
