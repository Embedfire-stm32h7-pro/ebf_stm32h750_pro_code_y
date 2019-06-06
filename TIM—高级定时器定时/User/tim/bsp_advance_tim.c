/**
  ******************************************************************************
  * @file    bsp_advance_tim.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   �߼����ƶ�ʱ����ʱ����
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� STM32H743 ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./tim/bsp_advance_tim.h"
#include "./led/bsp_led.h"

TIM_HandleTypeDef TIM_TimeBaseStructure;
 /**
  * @brief  �߼����ƶ�ʱ�� TIMx,x[1,8]�ж����ȼ�����
  * @param  ��
  * @retval ��
  */
static void TIMx_NVIC_Configuration(void)
{	
	//������ռ���ȼ��������ȼ�
	HAL_NVIC_SetPriority(ADVANCE_TIM_IRQn, 0, 3);
	// �����ж���Դ
	HAL_NVIC_EnableIRQ(ADVANCE_TIM_IRQn);
}

/*
 * ע�⣺TIM_TimeBaseInitTypeDef�ṹ��������5����Ա��TIM6��TIM7�ļĴ�������ֻ��
 * TIM_Prescaler��TIM_Period������ʹ��TIM6��TIM7��ʱ��ֻ���ʼ����������Ա���ɣ�
 * ����������Ա��ͨ�ö�ʱ���͸߼���ʱ������.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         ����
 * TIM_CounterMode			 TIMx,x[6,7]û�У��������У�������ʱ����
 * TIM_Period            ����
 * TIM_ClockDivision     TIMx,x[6,7]û�У���������(������ʱ��)
 * TIM_RepetitionCounter TIMx,x[1,8]����(�߼���ʱ��)
 *-----------------------------------------------------------------------------
 */
static void TIM_Mode_Config(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	
	// ����TIMx_CLK,x[1,8] 
	ADVANCE_TIM_CLK_ENABLE(); 
	
	TIM_TimeBaseStructure.Instance = ADVANCE_TIM;
	/* �ۼ� TIM_Period�������һ�����»����ж�*/		
	//����ʱ����0������4999����Ϊ5000�Σ�Ϊһ����ʱ����
	TIM_TimeBaseStructure.Init.Period = 5000-1;       
	// �߼����ƶ�ʱ��ʱ��ԴAPBxCLK = HCLK=200MHz 
	// �趨��ʱ��Ƶ��Ϊ=APBxCLK/(TIM_Prescaler+1)=10000Hz
	TIM_TimeBaseStructure.Init.Prescaler = 20000-1;	
	// ����ʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	// ������ʽ
	TIM_TimeBaseStructure.Init.CounterMode=TIM_COUNTERMODE_UP;
	// �ظ�������:�ظ�1�Σ����������β�����һ���ж�
	TIM_TimeBaseStructure.Init.RepetitionCounter=1;
	// ��ʼ����ʱ��TIMx, x[1,8]
	HAL_TIM_Base_Init(&TIM_TimeBaseStructure);
	// �߼���ʱ��ʱ��ԴΪ�ڲ�ʱ��Դ
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&TIM_TimeBaseStructure, &sClockSourceConfig);
	// ������ʱ�������ж�
	HAL_TIM_Base_Start_IT(&TIM_TimeBaseStructure);	
}

/**
  * @brief  ��ʼ���߼����ƶ�ʱ����ʱ��1s����һ���ж�
  * @param  ��
  * @retval ��
  */
void TIMx_Configuration(void)
{
	TIMx_NVIC_Configuration();	

	TIM_Mode_Config();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim==(&TIM_TimeBaseStructure))
    {
        LED1_TOGGLE;  //���������˸
    }
}

/*********************************************END OF FILE**********************/
