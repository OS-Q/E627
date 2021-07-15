#ifndef _UART_DMA_H_
#define _UART_DMA_H_
#include "main.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

void hal_init(void);
void usart_process_data(const void* data, size_t len);
void usart_send_string(const char* str);
void DMA_Handler(void);
void UART_Handler(void);

#endif
