#ifndef __ADC_H
#define	__ADC_H

#include "stm32h7xx.h"


//���Ŷ���
#define RHEOSTAT_ADC_PIN                            GPIO_PIN_3                 
#define RHEOSTAT_ADC_GPIO_PORT                      GPIOC                     
#define RHEOSTAT_ADC_GPIO_CLK_ENABLE()              __GPIOC_CLK_ENABLE()

// ADC ��ź궨��
#define RHEOSTAT_ADC                                ADC3
#define RHEOSTAT_ADC_CLK_ENABLE()                   __ADC3_CLK_ENABLE()
#define RHEOSTAT_ADC_CHANNEL                        ADC_CHANNEL_1
//DMAʱ��ʹ��
#define RHEOSTAT_ADC_DMA_CLK_ENABLE()               __HAL_RCC_BDMA_CLK_ENABLE();
#define RHEOSTAT_ADC_DMA_Base                       BDMA_Channel1
#define RHEOSTAT_ADC_DMA_Request                    BDMA_REQUEST_ADC3
//DMA�жϷ�����
#define RHEOSTAT_ADC_DMA_IRQHandler                 DMA1_Stream1_IRQHandler

#define Rheostat_ADC12_IRQ                          ADC_IRQn

void ADC_Init(void);

uint16_t ADC_GetValue(void);
void Rheostat_ADC_NVIC_Config(void);
#endif /* __ADC_H */
