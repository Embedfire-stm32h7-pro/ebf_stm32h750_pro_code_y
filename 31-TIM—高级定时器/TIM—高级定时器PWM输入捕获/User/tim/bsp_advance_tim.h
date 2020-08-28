#ifndef __BASIC_TIM_H
#define	__BASIC_TIM_H

#include "stm32h7xx.h"

//ͨ�ö�ʱ���궨��
#define GENERAL_TIM           		        TIM2
#define GENERAL_TIM_CLK_ENABLE()       		__TIM2_CLK_ENABLE()
#define GENERAL_TIM_CH                    TIM_CHANNEL_1
//ͨ�ö�ʱ��TIMͨ��CH1�궨��
#define GENERAL_TIM_CHx_CLK()            __GPIOA_CLK_ENABLE()
#define GENERAL_TIM_CHx_PORT             GPIOA 
#define GENERAL_TIM_CHx_PIN              GPIO_PIN_5



//�߼���ʱ���궨��
#define ADVANCE_TIM           		        TIM8
#define ADVANCE_TIM_CLK_ENABLE()       		__TIM8_CLK_ENABLE()
#define ADVANCE_TIM_CH                    TIM_CHANNEL_1
//TIMͨ��CH1�����ź궨��
#define ADVANCE_TIM_CHx_CLK()             __GPIOC_CLK_ENABLE()
#define ADVANCE_TIM_CHx_PORT              GPIOC
#define ADVANCE_TIM_CHx_PIN               GPIO_PIN_6

#define ADVANCE_TIM_IC_CH1                TIM_CHANNEL_1
#define ADVANCE_TIM_IC_CH2                TIM_CHANNEL_2

#define ADVANCE_TIM_IRQ                   TIM8_CC_IRQn
#define ADVANCE_TIM_IRQHandler            TIM8_CC_IRQHandler

void TIM_Advance_Init(void);

#endif /* __BASIC_TIM_H */

