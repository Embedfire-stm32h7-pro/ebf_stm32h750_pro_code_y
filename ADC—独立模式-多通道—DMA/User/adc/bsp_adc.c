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
#include "stm32h7xx.h"

extern float ADC_vol;

ADC_HandleTypeDef ADC1_Handle;
DMA_HandleTypeDef hdma_adc1;
__IO uint16_t ADC_ConvertedValue[4];


/**
  * @brief  ADC�������ú���
  * @param  ��
  * @retval ��
  */  
static void ADC_GPIO_Mode_Config(void)
{
    /* ����һ��GPIO_InitTypeDef���͵Ľṹ�� */
    GPIO_InitTypeDef  GPIO_InitStruct;
    /* ʹ��ADC���ŵ�ʱ�� */
    RHEOSTAT_ADC_GPIO_CLK_ENABLE();
    //ͨ��18����IO��ʼ��
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG; 
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = RHEOSTAT_ADC_PIN1; 
    /* ����Ϊģ�����룬����Ҫ�������� */ 
    HAL_GPIO_Init(RHEOSTAT_ADC_GPIO_PORT, &GPIO_InitStruct);
    //ͨ��19����IO��ʼ��
    GPIO_InitStruct.Pin = RHEOSTAT_ADC_PIN2;
    HAL_GPIO_Init(RHEOSTAT_ADC_GPIO_PORT, &GPIO_InitStruct);
    //ͨ��3����IO��ʼ��
    GPIO_InitStruct.Pin = RHEOSTAT_ADC_PIN3;
    HAL_GPIO_Init(RHEOSTAT_ADC_GPIO_PORT, &GPIO_InitStruct);
    //ͨ��7����IO��ʼ��
    GPIO_InitStruct.Pin = RHEOSTAT_ADC_PIN4;
    HAL_GPIO_Init(RHEOSTAT_ADC_GPIO_PORT, &GPIO_InitStruct);  
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
  
    /* ʹ��ADC1��2ʱ�� */
    RHEOSTAT_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    
    hdma_adc1.Instance = DMA1_Stream1;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if(HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {}
    __HAL_LINKDMA(&ADC1_Handle,DMA_Handle,hdma_adc1);    
      
    
    ADC1_Handle.Instance = RHEOSTAT_ADC1;
    //ʹ��Boostģʽ
    ADC1_Handle.Init.BoostMode = ENABLE;
    //ADCʱ��1��Ƶ
    ADC1_Handle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
    //ʹ������ת��ģʽ
    ADC1_Handle.Init.ContinuousConvMode = ENABLE;
    //���ݴ�������ݼĴ�����
    ADC1_Handle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    //�رղ�����ת��ģʽ
    ADC1_Handle.Init.DiscontinuousConvMode = DISABLE;
    //����ת��
    ADC1_Handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    //�������
    ADC1_Handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    //�رյ͹����Զ��ȴ�
    ADC1_Handle.Init.LowPowerAutoWait = DISABLE;
    //�������ʱ������д��
    ADC1_Handle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    //��ʹ�ܹ�����ģʽ
    ADC1_Handle.Init.OversamplingMode = DISABLE;
    //�ֱ���Ϊ��16bit
    ADC1_Handle.Init.Resolution = ADC_RESOLUTION_16B;
    //��ʹ�ܶ�ͨ��ɨ��
    ADC1_Handle.Init.ScanConvMode = ENABLE;
    //ɨ���ĸ�ͨ��
    ADC1_Handle.Init.NbrOfConversion = 4;
    //��ʼ�� ADC1
    HAL_ADC_Init(&ADC1_Handle);
          
    //ʹ��ͨ��18
    ADC_Config.Channel = RHEOSTAT_ADC_CHANNEL1;
    //ת��˳��Ϊ1
    ADC_Config.Rank = ADC_REGULAR_RANK_1;
    //��������Ϊ64.5������
    ADC_Config.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
    //��ʹ�ò������Ĺ���
    ADC_Config.SingleDiff = ADC_SINGLE_ENDED ;
    //����ADCͨ��
    HAL_ADC_ConfigChannel(&ADC1_Handle, &ADC_Config);    

    //ʹ��ͨ��19
    ADC_Config.Channel = RHEOSTAT_ADC_CHANNEL2;
    //ת��˳��Ϊ2
    ADC_Config.Rank = ADC_REGULAR_RANK_2;
    //����ADCͨ��
    HAL_ADC_ConfigChannel(&ADC1_Handle, &ADC_Config);
    
    //ʹ��ͨ��3
    ADC_Config.Channel = RHEOSTAT_ADC_CHANNEL3;
    //ת��˳��Ϊ1
    ADC_Config.Rank = ADC_REGULAR_RANK_3;
    //����ADCͨ��
    HAL_ADC_ConfigChannel(&ADC1_Handle, &ADC_Config); 

    //ʹ��ͨ��7
    ADC_Config.Channel = RHEOSTAT_ADC_CHANNEL4;
    //ת��˳��Ϊ1
    ADC_Config.Rank = ADC_REGULAR_RANK_4;
    //����ADCͨ��
    HAL_ADC_ConfigChannel(&ADC1_Handle, &ADC_Config);
    
    //ʹ��ADC1
    ADC_Enable(&ADC1_Handle);
    
    HAL_ADC_Start_DMA(&ADC1_Handle, (uint32_t *)ADC_ConvertedValue, 4);
    
}
/**
  * @brief  ADC�ж����ȼ����ú���
  * @param  ��
  * @retval ��
  */  
void Rheostat_ADC_NVIC_Config(void)
{
    HAL_NVIC_SetPriority(Rheostat_ADC12_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(Rheostat_ADC12_IRQ);
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
  
    HAL_ADC_Start(&ADC1_Handle);
}

/*********************************************END OF FILE**********************/


