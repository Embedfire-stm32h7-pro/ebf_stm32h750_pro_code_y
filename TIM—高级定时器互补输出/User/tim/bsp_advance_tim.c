/**
  ******************************************************************************
  * @file    bsp_basic_tim.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   �߼���ʱ���������
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:����  STM32 H750 ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./tim/bsp_advance_tim.h"
#include "./led/bsp_led.h" 

TIM_HandleTypeDef TIM_Handle;
TIM_OC_InitTypeDef sConfig;
TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

__IO uint16_t ChannelPulse = 4999;
 /**
  * @brief  �߼���ʱ�� TIMx,x=[1,8]ͨ�����ų�ʼ��
  * @param  ��
  * @retval ��
  */
static void TIMx_GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    ADVANCE_TIM_CHx_CLK();
    ADVANCE_TIM_CHxN_CLK();
    ADVANCE_TIM_BKIN_CLK();
  
    GPIO_InitStruct.Pin = ADVANCE_TIM_CHx_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; 
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH ;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(ADVANCE_TIM_CHx_PORT, &GPIO_InitStruct);
  
    GPIO_InitStruct.Pin = ADVANCE_TIM_CHxN_PIN;
    HAL_GPIO_Init(ADVANCE_TIM_CHxN_PORT, &GPIO_InitStruct);
  
    GPIO_InitStruct.Pin = ADVANCE_TIM_BKIN_PIN;
    HAL_GPIO_Init(ADVANCE_TIM_BKIN_PORT, &GPIO_InitStruct);   
    
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
 
  /**
  * @brief  �߼���ʱ�� TIMx,x=[1,8]��ʼ������
  * @param  ��
  * @retval ��
  */
static void TIMx_Configuration(void)
{
    //ʹ��TIMx��ʱ��
    ADVANCE_TIM_CLK_ENABLE();    
    //TIMx������Ĵ�������ַ
    TIM_Handle.Instance = ADVANCE_TIM;
    //����ģʽΪ���ϼ���������
    TIM_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    /* �ۼ� TIM_Period�������һ�����»����ж�*/		
    //����ʱ����0������9999����Ϊ10000�Σ�Ϊһ����ʱ����
    TIM_Handle.Init.Period = 10000 - 1;
    //ʱ�Ӳ���Ƶ����һ��TIM_CLKʱ�Ӽ�1��
    TIM_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    //��ʱ��ʱ��ԴTIMxCLK = 2 * PCLK1  
    //				PCLK1 = HCLK / 4 
    //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=200MHz
    // �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=1000000Hz
    TIM_Handle.Init.Prescaler =  200 - 1;   
    // ��ʼ����ʱ��TIM
    HAL_TIM_PWM_Init(&TIM_Handle);
  
    /* ����TIMΪ�������ģʽ */
    //���������PWM1ģʽ
    //����ֵCNTС�ڱȽ�ֵCCR��sConfig.Pulse����ͨ��xΪ��Ч��ƽ
    //����ֵCNT���ڱȽ�ֵCCR��sConfig.Pulse����ͨ��xΪ��Ч��ƽ
    sConfig.OCMode = TIM_OCMODE_PWM1;
    //CHx����Ч��ƽΪ�ߵ�ƽ
    sConfig.OCPolarity = TIM_OCNPOLARITY_LOW;
    //CHx�ڿ���״̬��Ϊ�͵�ƽ
    sConfig.OCIdleState = TIM_OCNIDLESTATE_SET;
    //CHxN�ڿ���״̬��Ϊ�ߵ�ƽ(������CHx�෴)
    sConfig.OCNIdleState = TIM_OCIDLESTATE_RESET;
    //CHxN����Ч��ƽΪ�ߵ�ƽ
    sConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    //�Ƚ���CCR��ֵ
    sConfig.Pulse = ChannelPulse;
    //��ʼ������Ƚ�ͨ��
    HAL_TIM_PWM_ConfigChannel(&TIM_Handle, &sConfig, ADVANCE_TIM_CH);
 
    //ʹ���Զ��������
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
    //��·���뼫�ԣ��͵�ƽ��Ч
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
    //ʹ�ܶ�·����
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
    //����ʱ�䣺11*Fdts
    sBreakDeadTimeConfig.DeadTime = 11;
    //�ԼĴ����ṩд����������Խ�ߣ��ɲ����ļĴ���λԽ��
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
    //����ģʽ�µĶ�·״̬������ͨ�����
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    //����ģʽ�µĶ�·״̬������ͨ�����
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
    //����TIMΪ��������·����
    HAL_TIMEx_ConfigBreakDeadTime(&TIM_Handle, &sBreakDeadTimeConfig);
    //�������CHxʹ��
    HAL_TIM_PWM_Start(&TIM_Handle, ADVANCE_TIM_CH);
    //�������CHxNʹ��
    HAL_TIMEx_PWMN_Start(&TIM_Handle, ADVANCE_TIM_CH);
}

/**
  * @brief  ��ʼ��������ʱ����ʱ��1ms����һ���ж�
  * @param  ��
  * @retval ��
  */
void TIM_Advance_Init(void)
{
    //TIMͨ�����ų�ʼ��
    TIMx_GPIO_Configuration();
    //TIM�����ʼ��
    TIMx_Configuration();
  
}

/*********************************************END OF FILE**********************/

