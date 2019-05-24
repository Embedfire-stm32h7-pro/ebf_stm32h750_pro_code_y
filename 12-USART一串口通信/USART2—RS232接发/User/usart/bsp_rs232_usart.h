#ifndef __RS232_USART_H
#define	__RS232_USART_H

#include "stm32h7xx.h"
#include <stdio.h>



//���Ŷ���
/*******************************************************/
#define RS232_USART                             USART2
#define RS232_USART_CLK_ENABLE()                __USART2_CLK_ENABLE();

#define RCC_PERIPHCLK_UARTx               			RCC_PERIPHCLK_USART2
#define RCC_UARTxCLKSOURCE_SYSCLK         			RCC_USART234578CLKSOURCE_D2PCLK1

#define RS232_USART_RX_GPIO_PORT                GPIOD
#define RS232_USART_RX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOD_CLK_ENABLE()
#define RS232_USART_RX_PIN                      GPIO_PIN_6
#define RS232_USART_RX_AF                       GPIO_AF7_USART2


#define RS232_USART_TX_GPIO_PORT                GPIOD
#define RS232_USART_TX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOD_CLK_ENABLE()
#define RS232_USART_TX_PIN                      GPIO_PIN_5
#define RS232_USART_TX_AF                       GPIO_AF7_USART2

#define RS232_USART_IRQHandler                  USART2_IRQHandler
#define RS232_USART_IRQ                 				USART2_IRQn
/************************************************************/


//���ڲ�����
#define RS232_USART_BAUDRATE                    115200

void Usart_SendString(uint8_t *str);
void RS232_USART_Config(void);

extern UART_HandleTypeDef UartHandle;
#endif /* __USART1_H */
