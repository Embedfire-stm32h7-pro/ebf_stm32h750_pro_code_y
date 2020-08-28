#ifndef __BASIC_TIM_H
#define	__BASIC_TIM_H

#include "stm32h7xx.h"



//�߼���ʱ���궨��
#define ADVANCE_TIM           		        TIM8
#define ADVANCE_TIM_CLK_ENABLE()       		__TIM8_CLK_ENABLE()
#define ADVANCE_TIM_CH                    TIM_CHANNEL_1
//TIMͨ��CH1�����ź궨��
#define ADVANCE_TIM_CHx_CLK()             __GPIOC_CLK_ENABLE()
#define ADVANCE_TIM_CHx_PORT              GPIOC
#define ADVANCE_TIM_CHx_PIN               GPIO_PIN_6
//TIMͨ��CH1N�����ź궨��
#define ADVANCE_TIM_CHxN_CLK()            __GPIOA_CLK_ENABLE()
#define ADVANCE_TIM_CHxN_PORT             GPIOA 
#define ADVANCE_TIM_CHxN_PIN              GPIO_PIN_5
//TIM����BKIN�궨��
#define ADVANCE_TIM_BKIN_CLK()            __GPIOA_CLK_ENABLE()
#define ADVANCE_TIM_BKIN_PORT             GPIOA 
#define ADVANCE_TIM_BKIN_PIN              GPIO_PIN_6

void TIM_Advance_Init(void);

#endif /* __BASIC_TIM_H */

