/**
  ******************************************************************
  * @file    bsp_key.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   LPTIM-�͹��Ķ�ʱ��PWM�����ʼ������
  ******************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� STM32H743 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************
  */ 
#include "./lptim/bsp_lptim.h" 
LPTIM_HandleTypeDef LPTIM_Handle;
/**
  * @brief  LPTIM�����ų�ʼ��
  * @param  ��
  * @retval ��
  */  
void LPTIM_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /*����LPTIM1���ŵ�ʱ��*/
    LPTIMx_OUT_GPIO_CLK_ENABLE();
    /*ѡ�񰴼�������*/	
    GPIO_InitStructure.Pin = LPTIMx_OUT_PIN; 
    /*��������Ϊ�����ƒ�ģʽ*/
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP; 
    /*�������Ų�����Ҳ������*/
    GPIO_InitStructure.Pull = GPIO_PULLUP;
  	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH; 	
    GPIO_InitStructure.Alternate = LPTIMx_OUT_AF;
    /*ʹ������Ľṹ���ʼ������*/
    HAL_GPIO_Init(LPTIMx_OUT_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  ����TIM���PWM
  * @param  ��
  * @retval ��
  */
void LPTIM_MODE_Config(void)
{

  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct;  
  uint32_t PeriodValue;
	uint32_t PulseValue;
  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
  /* ѡ��LSIʱ����ΪLPTIMʱ��Դ */
#if (LPTIMx_CLK_Source == LSI)    
  RCC_PeriphCLKInitStruct.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSI;
  /* ѡ��LSEʱ����ΪLPTIMʱ��Դ */
#elif (LPTIMx_CLK_Source == LSE)    
  RCC_PeriphCLKInitStruct.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSE;
#endif
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
  /*����LPTIM1��ʱ��*/
  
  LPTIMx_CLK_ENABLE();   
	/* ���嶨ʱ���ľ����ȷ����ʱ���Ĵ����Ļ���ַ*/  
  LPTIM_Handle.Instance = LPTIMx_Instance;
  //ѡ���ڲ�ʱ��ԴLPTIM_CLK = LSE=32.768KHz   
  LPTIM_Handle.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC; 
	// ��ʱ��ʱ�ӷ�Ƶϵ��  
  LPTIM_Handle.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
	// ��ʱ������Դ���ڲ�  
  LPTIM_Handle.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
	// ����Դ���������
	LPTIM_Handle.Init.Trigger.Source  = LPTIM_TRIGSOURCE_SOFTWARE;  
	// ��ʱ���������
	LPTIM_Handle.Init.OutputPolarity  = LPTIM_OUTPUTPOLARITY_HIGH;
  // ��ʱ�����·�ʽ
	LPTIM_Handle.Init.UpdateMode      = LPTIM_UPDATE_IMMEDIATE;
  
  HAL_LPTIM_Init(&LPTIM_Handle);
	/*PWMģʽ����*/
	/*����ʱ����0������99����Ϊ100�Σ�
  Ϊһ����ʱ����PWM���ڣ�32.768KHz/100=327.68Hz*/
	PeriodValue = 100-1;
	/*PWM����Ϊ����һ�뼴50% */
	PulseValue = 50-1;
	HAL_LPTIM_PWM_Start(&LPTIM_Handle, PeriodValue, PulseValue);
}
/**
  * @brief  �͹��Ķ�ʱ���ڵ͹���ģʽ���PWM
  * @param  ��
  * @retval ��
  */
void LPTIM_PWM_OUT(void)
{
	LPTIM_GPIO_Config();	
	
	LPTIM_MODE_Config();
	/* ����͹���ģʽ */
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	/* �ȴ�ϵͳ������KEY2���ѣ��˳��͹���ģʽ��ֹͣ���PWM */
  HAL_LPTIM_PWM_Stop(&LPTIM_Handle);
}

/*********************************************END OF FILE**********************/
