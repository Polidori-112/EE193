#ifndef __STM32L476G_DISCOVERY_UART_H
#define __STM32L476G_DISCOVERY_UART_H

#include "stm32l476xx.h"

void UART2_Init(void);
void USART_Write(USART_TypeDef *USARTx, const uint8_t *buffer);
uint8_t USART_Read(USART_TypeDef * USARTx);

#endif /* __STM32L476G_DISCOVERY_UART_H */
