#ifndef _UART_DMA_H_
#define _UART_DMA_H_
#include "main.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

void LL_Init(void);
void usart_init(void);
void usart_process_data(const void* data, size_t len);
void usart_send_string(const char* str);

#endif
