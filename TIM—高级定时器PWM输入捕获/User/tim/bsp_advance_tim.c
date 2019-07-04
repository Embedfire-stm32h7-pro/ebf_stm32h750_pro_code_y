/**
  ******************************************************************************
  * @file    bsp_basic_tim.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   ������ʱ����ʱ����
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� STM32 H750 ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./tim/bsp_advance_tim.h"
#include "./led/bsp_led.h" 

TIM_HandleTypeDef TIM_PWMOUTNPUT_Handle;
TIM_HandleTypeDef TIM_PWMINPUT_Handle;
TIM_OC_InitTypeDef sConfig;


__IO uint16_t ChannelPulse = 5000;

__IO uint16_t IC2Value = 0;
__IO uint16_t IC1Value = 0;
__IO float DutyCycle = 0;
__IO float Frequency = 0;
 /**
  * @brief  �߼���ʱ�� TIMx,x=[1,8]ͨ�����ų�ʼ��
  * @param  ��
  * @retval ��
  */
static void TIMx_GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    //ʹ�ܸ߼���ʱ��������ʱ��
    ADVANCE_TIM_CHx_CLK();
    //ʹ��ͨ�ö�ʱ��������ʱ��
    GENERAL_TIM_CHx_CLK();
  
   
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; 
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH ;
    
    GPIO_InitStruct.Pin = GENERAL_TIM_CHx_PIN;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GENERAL_TIM_CHx_PORT, &GPIO_InitStruct); 

    GPIO_InitStruct.Pin = ADVANCE_TIM_CHx_PIN;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(ADVANCE_TIM_CHx_PORT, &GPIO_InitStruct);
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
  * @brief  ͨ�ö�ʱ��TIMx��ʼ������
  * @param  ��
  * @retval ��
  */
static void TIMx_Output_Configuration(void)
{
    //ʹ��TIMx��ʱ��
    GENERAL_TIM_CLK_ENABLE();    
    //TIMx������Ĵ�������ַ
    TIM_PWMOUTNPUT_Handle.Instance = GENERAL_TIM;
    //����ģʽΪ���ϼ���������
    TIM_PWMOUTNPUT_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    /* �ۼ� TIM_Period�������һ�����»����ж�*/		
    //����ʱ����0������9999����Ϊ10000�Σ�Ϊһ����ʱ����
    TIM_PWMOUTNPUT_Handle.Init.Period = 10000 - 1;
    //ʱ�Ӳ���Ƶ����һ��TIM_CLKʱ�Ӽ�1��
    TIM_PWMOUTNPUT_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    //��ʱ��ʱ��ԴTIMxCLK = 2 * PCLK1  
    //				PCLK1 = HCLK / 4 
    //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=200MHz
    // �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=1000000Hz
    TIM_PWMOUTNPUT_Handle.Init.Prescaler =  200 - 1;   
    // ��ʼ����ʱ��TIM
    HAL_TIM_PWM_Init(&TIM_PWMOUTNPUT_Handle);
  
    /* ����TIMΪ�������ģʽ */
    //���������PWM1ģʽ
    //����ֵCNTС�ڱȽ�ֵCCR��sConfig.Pulse����ͨ��xΪ��Ч��ƽ
    //����ֵCNT���ڱȽ�ֵCCR��sConfig.Pulse����ͨ��xΪ��Ч��ƽ
    sConfig.OCMode = TIM_OCMODE_PWM1;
    //CHx����Ч��ƽΪ�ߵ�ƽ
    sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
    //CHx�ڿ���״̬��Ϊ�͵�ƽ
    sConfig.OCIdleState = TIM_OCNIDLESTATE_RESET;
    //CHxN�ڿ���״̬��Ϊ�ߵ�ƽ(������CHx�෴)
    sConfig.OCNIdleState = TIM_OCIDLESTATE_SET;
    //CHxN����Ч��ƽΪ�ߵ�ƽ
    sConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    //�Ƚ���CCR��ֵ
    sConfig.Pulse = ChannelPulse;
    //��ʼ������Ƚ�ͨ��
    HAL_TIM_OC_ConfigChannel(&TIM_PWMOUTNPUT_Handle, &sConfig, GENERAL_TIM_CH);
 
    //�������CHxʹ��
    HAL_TIM_OC_Start(&TIM_PWMOUTNPUT_Handle, GENERAL_TIM_CH);

}


/**
  * @brief  ͨ�ö�ʱ��TIMx��ʼ������
  * @param  ��
  * @retval ��
  */
static void TIMx_Input_Configuration(void)
{
  
    TIM_IC_InitTypeDef TIM_IC_Config; 
    TIM_SlaveConfigTypeDef TIM_SlaveConfig;
  
    //ʹ��TIMx��ʱ��
    ADVANCE_TIM_CLK_ENABLE();    
    //TIMx������Ĵ�������ַ
    TIM_PWMINPUT_Handle.Instance = ADVANCE_TIM;
    //����ģʽΪ���ϼ���������
    TIM_PWMINPUT_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    /* �ۼ� TIM_Period�������һ�����»����ж�*/		
    //����ʱ����0������9999����Ϊ10000�Σ�Ϊһ����ʱ����
    TIM_PWMINPUT_Handle.Init.Period = 10000;
    //ʱ�Ӳ���Ƶ����һ��TIM_CLKʱ�Ӽ�1��
    TIM_PWMINPUT_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    //��ʱ��ʱ��ԴTIMxCLK = 2 * PCLK1  
    //				PCLK1 = HCLK / 4 
    //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=200MHz
    // �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=100Hz
    TIM_PWMINPUT_Handle.Init.Prescaler =  200 - 1;   
    // ��ʼ����ʱ��TIM
    HAL_TIM_PWM_Init(&TIM_PWMINPUT_Handle);    
    
    /* IC1���������ش���TI1FP1 */
    TIM_IC_Config.ICPolarity = TIM_ICPOLARITY_RISING;
    TIM_IC_Config.ICFilter = 0;
    TIM_IC_Config.ICPrescaler = TIM_ICPSC_DIV1;
    TIM_IC_Config.ICSelection = TIM_ICSELECTION_DIRECTTI; 
    HAL_TIM_IC_ConfigChannel(&TIM_PWMINPUT_Handle, &TIM_IC_Config, ADVANCE_TIM_IC_CH1);
    /* IC2���������ش���TI1FP2 */
    TIM_IC_Config.ICPolarity = TIM_ICPOLARITY_FALLING;
    TIM_IC_Config.ICFilter = 0;
    TIM_IC_Config.ICPrescaler = TIM_ICPSC_DIV1;
    TIM_IC_Config.ICSelection = TIM_ICSELECTION_INDIRECTTI; 
    HAL_TIM_IC_ConfigChannel(&TIM_PWMINPUT_Handle, &TIM_IC_Config, ADVANCE_TIM_IC_CH2);    
    /* ѡ���ģʽ: ��λģʽ */
    TIM_SlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
    /* ѡ��ʱ�����봥��: TI1FP1 */
    TIM_SlaveConfig.InputTrigger = TIM_TS_TI1FP1;
    HAL_TIM_SlaveConfigSynchronization(&TIM_PWMINPUT_Handle,&TIM_SlaveConfig);
    
    HAL_TIM_IC_Start_IT(&TIM_PWMINPUT_Handle,ADVANCE_TIM_IC_CH1);
    HAL_TIM_IC_Start_IT(&TIM_PWMINPUT_Handle,ADVANCE_TIM_IC_CH2);
}


/**
  * @brief  TIM�ж����ȼ����ú���
  * @param  ��
  * @retval ��
  */  
static void ADVANCE_TIM_NVIC_Config(void)
{
    HAL_NVIC_SetPriority(ADVANCE_TIM_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(ADVANCE_TIM_IRQ);
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
    //�߼���ʱ���ж�����
    ADVANCE_TIM_NVIC_Config();
    //ͨ��TIM�����ʼ��
    TIMx_Output_Configuration();
    //�߼�TIM�����ʼ��
    TIMx_Input_Configuration();
}
/**
  * @brief  ���벶������жϻص�����
  * @param  TIM_Handle : ADC���
  * @retval ��
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      /* ��ȡ���벶��ֵ */
      IC1Value = HAL_TIM_ReadCapturedValue(&TIM_PWMINPUT_Handle,TIM_CHANNEL_1);
      IC2Value = HAL_TIM_ReadCapturedValue(&TIM_PWMINPUT_Handle,TIM_CHANNEL_2);	

      if (IC1Value != 0)
      {
      /* ռ�ձȼ��� */
      DutyCycle = (float)((IC2Value+1) * 100) / (IC1Value+1);
 
      /* Ƶ�ʼ��� */
      Frequency = 200000000/200/(float)(IC1Value+1);
      

      }
      else
      {
      DutyCycle = 0;
      Frequency = 0;
      }

    }    
}
/*********************************************END OF FILE**********************/

