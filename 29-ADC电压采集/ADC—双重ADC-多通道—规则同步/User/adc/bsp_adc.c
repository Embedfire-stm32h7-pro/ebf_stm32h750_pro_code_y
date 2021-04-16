/**
  ******************************************************************
  * @file    bsp_adcd.c
  * @author  fire
  * @version V1.1
  * @date    2019-xx-xx
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
ADC_HandleTypeDef ADC_SLAVE_Handle;
DMA_HandleTypeDef hdma_adc;
ADC_MultiModeTypeDef ADC_multimode;

ALIGN_32BYTES (__attribute__ ((at(0x30000000))) __IO uint16_t ADC_ConvertedValue = 0);


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
	RHEOSTAT_ADC_MASTER_GPIO_CLK_ENABLE();

	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG; 
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = RHEOSTAT_ADC_MASTER_PIN;
	/* ����Ϊģ�����룬����Ҫ�������� */ 
	HAL_GPIO_Init(RHEOSTAT_ADC_MASTER_GPIO_PORT, &GPIO_InitStruct);

	/* ʹ��ADC���ŵ�ʱ�� */
	RHEOSTAT_ADC_SLAVE_GPIO_CLK_ENABLE();

	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = RHEOSTAT_ADC_SLAVE_PIN;
	/* ����Ϊģ�����룬����Ҫ�������� */
	HAL_GPIO_Init(RHEOSTAT_ADC_SLAVE_GPIO_PORT, &GPIO_InitStruct);
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

	/* ʹ��ADCʱ�� */
	RHEOSTAT_ADC_MASTER_CLK_ENABLE();
	/* ʹ��DMAʱ�� */
	RHEOSTAT_ADC_DMA_CLK_ENABLE();
	/* ʹ��ADC_SLAVEʱ�� */
	RHEOSTAT_ADC_SLAVE_CLK_ENABLE();

	//ѡ��DMA1��Stream1
	hdma_adc.Instance = RHEOSTAT_ADC_DMA_Base;
	//ADC1��DMA����
	hdma_adc.Init.Request = RHEOSTAT_ADC_DMA_Request;
	//���䷽������-���ڴ�
	hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
	//�����ַ������
	hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
	//�ڴ��ַ������
	hdma_adc.Init.MemInc = DMA_PINC_DISABLE;
	//�������ݿ�ȣ�����
	hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	//�ڴ����ݿ�ȣ�����
	hdma_adc.Init.MemDataAlignment = DMA_PDATAALIGN_WORD;
	//DMAѭ������
	hdma_adc.Init.Mode = DMA_CIRCULAR;
	//DMA��������ȼ�����
	hdma_adc.Init.Priority = DMA_PRIORITY_VERY_HIGH;
	//FIFOģʽ�ر�
	hdma_adc.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	//DMA��ʼ��
	HAL_DMA_Init(&hdma_adc);
	//hdma_adc��ADC_Handle.DMA_Handle����
	__HAL_LINKDMA(&ADC_Handle,DMA_Handle,hdma_adc);    

	ADC_Handle.Instance = RHEOSTAT_ADC_MASTER;
	//ʹ��Boostģʽ
	ADC_Handle.Init.BoostMode = ENABLE;
	//ADCʱ��1��Ƶ
	ADC_Handle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	//ʹ������ת��ģʽ
	ADC_Handle.Init.ContinuousConvMode = ENABLE;
	//���ݴ�������ݼĴ�����
	ADC_Handle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
	//�رղ�����ת��ģʽ
	ADC_Handle.Init.DiscontinuousConvMode = DISABLE;
	//����ת��
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
	//��ʼ�� ADC_MASTER
	HAL_ADC_Init(&ADC_Handle);
	
	//ADCУ׼
  HAL_ADCEx_Calibration_Start(&ADC_Handle,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED); 
	
	//��ʼ�� ADC_SLAVE
	ADC_SLAVE_Handle.Instance = RHEOSTAT_ADC_SLAVE;
	ADC_SLAVE_Handle.Init = ADC_Handle.Init;
	HAL_ADC_Init(&ADC_SLAVE_Handle);
					
	//ADCУ׼
  HAL_ADCEx_Calibration_Start(&ADC_SLAVE_Handle,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED); 
	 
	//ʹ��ͨ��18
	ADC_Config.Channel = RHEOSTAT_ADC_MASTER_CHANNEL;
	//ת��˳��Ϊ1
	ADC_Config.Rank = ADC_REGULAR_RANK_1;
	//��������Ϊ64.5������
	ADC_Config.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
	//��ʹ�ò������Ĺ���
	ADC_Config.SingleDiff = ADC_SINGLE_ENDED ;
	//����ADC_MASTERͨ��
	HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config); 
  //ʹ��ͨ��12
	ADC_Config.Channel = RHEOSTAT_ADC_SLAVE_CHANNEL;	
	//����ADC_SLAVEͨ��
	HAL_ADC_ConfigChannel(&ADC_SLAVE_Handle, &ADC_Config);

	//ʹ��ADC1��2
	ADC_Enable(&ADC_Handle);
	ADC_Enable(&ADC_SLAVE_Handle);

	//���ݸ�ʽ
	ADC_multimode.DualModeData = ADC_DUALMODEDATAFORMAT_32_10_BITS;
	//˫��ADC����ͬ��ģʽ
	ADC_multimode.Mode = ADC_DUALMODE_REGSIMULT;
	//ADC_MASTER��ADC_SLAVE�������3��ADCʱ��
	ADC_multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
	//ADC˫��ģʽ���ó�ʼ��
	HAL_ADCEx_MultiModeConfigChannel(&ADC_Handle, &ADC_multimode);
	//ʹ��DMA,�˺����Ὺ��DMA�жϣ���Ҫ�û���������DMA�ж����ȼ�
	HAL_ADCEx_MultiModeStart_DMA(&ADC_Handle, (uint32_t*)&ADC_ConvertedValue, 1);
}
/**
  * @brief  ADC�ж����ȼ����ú���
  * @param  ��
  * @retval ��
  */  
void Rheostat_DMA_NVIC_Config(void)
{
    HAL_NVIC_SetPriority(Rheostat_ADC12_IRQ, 1, 1);
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
	
	HAL_Delay(5);
	//��ʼADCת�����������
	HAL_ADC_Start(&ADC_Handle);
	HAL_ADC_Start(&ADC_SLAVE_Handle);
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


