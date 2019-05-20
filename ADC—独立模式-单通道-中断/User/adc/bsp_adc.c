/**
  ******************************************************************
  * @file    bsp_adcd.c
  * @author  fire
  * @version V1.1
  * @date    2018-xx-xx
  * @brief   adcӦ�ú����ӿ�
  ******************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� STM32H743������ 
  * ��˾    :http://www.embedfire.com
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************
  */
#include "./adc/bsp_adc.h" 

extern float ADC_vol;

ADC_HandleTypeDef ADC_Handle;
__IO uint16_t ADC_ConvertedValue = 0;


/**
  * @brief  ADC�������ú���
  * @param  ��
  * @retval ��
  */  
static void ADC_GPIO_Mode_Config(void)
{
  /* ����һ��GPIO_InitTypeDef���͵Ľṹ�� */
  GPIO_InitTypeDef  GPIO_InitStruct;
  /* ʹ��ADC3���ŵ�ʱ�� */
  RHEOSTAT_ADC_GPIO_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();
    
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG; 
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = RHEOSTAT_ADC_PIN;
  /* ����Ϊģ�����룬����Ҫ�������� */ 
  HAL_GPIO_Init(RHEOSTAT_ADC_GPIO_PORT, &GPIO_InitStruct);
  
  /**ADC3 GPIO Configuration    
  PC3_C     ------> ADC3_INP1 
  */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_OPEN);  
}

/**
  * @brief  ADC����ģʽ���ú���
  * @param  ��
  * @retval ��
  */  
static void ADC_Mode_Config(void)
{
    ADC_ChannelConfTypeDef ADC_Config;
  
    RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;  
    /*            ����ADC3ʱ��Դ             */
    /*    HSE Frequency(Hz)    = 25000000   */                                             
    /*         PLL_M                = 5     */
    /*         PLL_N                = 160   */
    /*         PLL_P                = 25    */
    /*         PLL_Q                = 2     */
    /*         PLL_R                = 2     */
    /*     ADC_ker_clk         = 32000000   */
		RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    RCC_PeriphClkInit.PLL2.PLL2FRACN = 0;
    RCC_PeriphClkInit.PLL2.PLL2M = 5;
    RCC_PeriphClkInit.PLL2.PLL2N = 160;
    RCC_PeriphClkInit.PLL2.PLL2P = 25;
    RCC_PeriphClkInit.PLL2.PLL2Q = 2;
    RCC_PeriphClkInit.PLL2.PLL2R = 2;
    RCC_PeriphClkInit.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
    RCC_PeriphClkInit.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    RCC_PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2; 
		HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);  
  
    /* ʹ��ADC3ʱ�� */
    RHEOSTAT_ADC_CLK_ENABLE();  
      
    ADC_Handle.Instance = RHEOSTAT_ADC;
    //ʹ��Boostģʽ
//    ADC_Handle.Init.BoostMode = ENABLE;
    //ADCʱ��1��Ƶ
    ADC_Handle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    //ʹ������ת��ģʽ
    ADC_Handle.Init.ContinuousConvMode = ENABLE;
    //ת��ͨ�� 1��
    ADC_Handle.Init.NbrOfConversion = 1;
    //���ݴ�������ݼĴ�����
    ADC_Handle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    //�رղ�����ת��ģʽ
    ADC_Handle.Init.DiscontinuousConvMode = DISABLE;
    // ������ת������
    ADC_Handle.Init.NbrOfDiscConversion = 0;
    //�����Ҷ���	
    ADC_Handle.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    
    //ʹ��EOC��־λ
    ADC_Handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    //�������
    ADC_Handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    //�رյ͹����Զ��ȴ�
    ADC_Handle.Init.LowPowerAutoWait = DISABLE;
    //�������ʱ������д��
    ADC_Handle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    //��ʹ�ܹ�����ģʽ
    ADC_Handle.Init.OversamplingMode = DISABLE;
    //�ֱ���Ϊ��16bit
    ADC_Handle.Init.Resolution = ADC_RESOLUTION_16B;
    //��ʹ�ܶ�ͨ��ɨ��
    ADC_Handle.Init.ScanConvMode = DISABLE;
    //��ʼ�� ADC
    HAL_ADC_Init(&ADC_Handle);

    //ʹ��ͨ��1
    ADC_Config.Channel = ADC_CHANNEL_1;
    //ת��˳��Ϊ1
    ADC_Config.Rank = ADC_REGULAR_RANK_1 ;
    //��������Ϊ64.5������
    ADC_Config.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
    //��ʹ�ò������Ĺ���
    ADC_Config.SingleDiff = ADC_SINGLE_ENDED ;
    //����ADCͨ��
    HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config);
//    //ʹ��ADC
//    ADC_Enable(&ADC_Handle);
}
/**
  * @brief  ADC�ж����ȼ����ú���
  * @param  ��
  * @retval ��
  */  
static void Rheostat_ADC_NVIC_Config(void)
{
    HAL_NVIC_SetPriority(Rheostat_ADC_IRQ, 1, 1);
    HAL_NVIC_EnableIRQ(Rheostat_ADC_IRQ);
}

/**
  * @brief  ADC��ʼ������
  * @param  ��
  * @retval ��
  */
void ADC_Init(void)
{
    
    ADC_GPIO_Mode_Config();
  
    ADC_Mode_Config();
    //ADC�ж����ȼ�����
    Rheostat_ADC_NVIC_Config();
    //ʹ��ADC�ж� 
    HAL_ADC_Start_IT(&ADC_Handle);
    //ʹ��ADC
    ADC_Enable(&ADC_Handle);
    //�������ADC����
    HAL_ADC_Start(&ADC_Handle);
}

/**
  * @brief  ת������жϻص�������������ģʽ��
  * @param  AdcHandle : ADC���
  * @retval ��
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  /* ��ȡ��� */
  ADC_ConvertedValue = HAL_ADC_GetValue(AdcHandle);  
}
/*********************************************END OF FILE**********************/


