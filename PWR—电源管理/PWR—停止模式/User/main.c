/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   Ұ��H743 PWR��ֹͣģʽ
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 H743 ������
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
#include "main.h"  
#include "stm32h7xx.h"
#include "./led/bsp_led.h" 
#include "./usart/bsp_usart.h"
#include "./key/bsp_exti.h" 
#include "./delay/core_delay.h"  

/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void) 
{
	uint32_t SYSCLK_Frequency=0; 
	uint32_t HCLK_Frequency=0;
	uint32_t PCLK1_Frequency=0;
	uint32_t PCLK2_Frequency=0;
	uint32_t SYSCLK_Source=0;
	
	/* ��ʼ��ϵͳʱ��Ϊ400MHz */
	SystemClock_Config();
	/* ��ʼ��LED */
	LED_GPIO_Config();	
	/* ��ʼ�����Դ��ڣ�һ��Ϊ����1 */
	UARTx_Config();	
	/* ��ʼ������Ϊ�ж�ģʽ�������жϺ������жϷ�����	*/
	EXTI_Key_Config(); 
	
	printf("\r\n ��ӭʹ��Ұ�� STM32H743 �����塣\r\n");
    printf("\r\n Ұ��H743 ֹͣģʽ����\r\n");
	
	printf("\r\n ʵ��˵����\r\n");

	printf("\r\n 1.�������У��̵Ʊ�ʾSTM32�������У���Ʊ�ʾ˯��״̬�����Ʊ�ʾ�մ�ֹͣ״̬������\r\n");
	printf("\r\n 2.��������һ��ʱ����Զ�����ֹͣ״̬����ֹͣ״̬�£���ʹ��KEY1��KEY2����\r\n");
	printf("\r\n 3.��ʵ��ִ������һ��ѭ����\r\n ------�����̵�(��������)->�����(ֹͣģʽ)->��KEY1��KEY2����->������(�ձ�����)-----��\r\n");
	printf("\r\n 4.��ֹͣ״̬�£�DAP�������޷���STM32���س���\r\n �ɰ�KEY1��KEY2���Ѻ����أ�\r\n �򰴸�λ��ʹоƬ���ڸ�λ״̬��Ȼ���ڵ����ϵ�����ذ�ť�����ͷŸ�λ��������������\r\n");

	while(1)
	{	
		/*********ִ������***************************/
		printf("\r\n STM32�������У����̵�\r\n");

		LED_GREEN;	
		HAL_Delay(2000);	
		
		/*****����ִ����ϣ�����˯�߽��͹���***********/
		printf("\r\n ����ֹͣģʽ�������,��KEY1��KEY2�����ɻ���\r\n");
		//ʹ�ú��ָʾ������˯��״̬
		LED_RED;
		//��ͣ�δ�ʱ�ӣ���ֹͨ���δ�ʱ���жϻ���
		HAL_SuspendTick();
		/*����ֹͣģʽʱ��FLASH�������״̬*/
		HAL_PWREx_EnableFlashPowerDown();
		/* ����ֹͣģʽ�����õ�ѹ������Ϊ�͹���ģʽ���ȴ��жϻ��� */
		HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON,PWR_STOPENTRY_WFI);
		//�ȴ��жϻ���  K1��K2�����ж�	
		/***�����ѣ�������ָʾ***/
		LED_BLUE;
		//����ʱ�ӼĴ�����ֵ����SystemCoreClock����
		SystemCoreClockUpdate();
		//��ȡ���Ѻ��ʱ��״̬	
		SYSCLK_Frequency = HAL_RCC_GetSysClockFreq(); 
		HCLK_Frequency   = HAL_RCC_GetHCLKFreq();
		PCLK1_Frequency  = HAL_RCC_GetPCLK1Freq();
		PCLK2_Frequency  = HAL_RCC_GetPCLK2Freq();
		SYSCLK_Source    = __HAL_RCC_GET_SYSCLK_SOURCE();
		//����ϵͳֱ��ʹ��HSIʱ�ӣ�Ӱ�촮�ڲ����ʣ���Ҫ���³�ʼ������
		UARTx_HSI_Config();
		printf("\r\n�ջ��ѵ�ʱ��״̬��\r\n");
		printf(" SYSCLKƵ��:%d,\r\n HCLKƵ��:%d,\r\n PCLK1Ƶ��:%d,\r\n PCLK2Ƶ��:%d,\r\n ʱ��Դ:%d (0��ʾHSI��8��ʾPLLCLK)\n", 
		SYSCLK_Frequency,HCLK_Frequency,PCLK1_Frequency,PCLK2_Frequency,SYSCLK_Source);		  
		
		/* ��ֹͣģʽ���Ѻ�����ϵͳʱ��:����HSE��PLL*/
		/* ѡ��PLL��Ϊϵͳʱ��Դ(HSE��PLL��ֹͣģʽ�±�����)*/
		SYSCLKConfig_STOP();
		//�����Ѻ󣬻ָ��δ�ʱ��			
		HAL_ResumeTick();
		//��ȡ�������ú��ʱ��״̬
		SYSCLK_Frequency = HAL_RCC_GetSysClockFreq(); 
		HCLK_Frequency   = HAL_RCC_GetHCLKFreq();
		PCLK1_Frequency  = HAL_RCC_GetPCLK1Freq();
		PCLK2_Frequency  = HAL_RCC_GetPCLK2Freq();
		SYSCLK_Source    = __HAL_RCC_GET_SYSCLK_SOURCE();
		
		//��������ʱ��Դ��ʼ��״̬
		printf("\r\n�������ú��ʱ��״̬��\r\n");
		printf(" SYSCLKƵ��:%d,\r\n HCLKƵ��:%d,\r\n PCLK1Ƶ��:%d,\r\n PCLK2Ƶ��:%d,\r\n ʱ��Դ:%d (0��ʾHSI��8��ʾPLLCLK)\n", 
		SYSCLK_Frequency,HCLK_Frequency,PCLK1_Frequency,PCLK2_Frequency,SYSCLK_Source);

		HAL_Delay(2000);	
			
		printf("\r\n ���˳�ֹͣģʽ\r\n");
		//����ִ��whileѭ��
	}
}

/**
  * @brief  System Clock ����
  *         system Clock ��������:
	*            System Clock source  = PLL (HSE)
	*            SYSCLK(Hz)           = 400000000 (CPU Clock)
	*            HCLK(Hz)             = 200000000 (AXI and AHBs Clock)
	*            AHB Prescaler        = 2
	*            D1 APB3 Prescaler    = 2 (APB3 Clock  100MHz)
	*            D2 APB1 Prescaler    = 2 (APB1 Clock  100MHz)
	*            D2 APB2 Prescaler    = 2 (APB2 Clock  100MHz)
	*            D3 APB4 Prescaler    = 2 (APB4 Clock  100MHz)
	*            HSE Frequency(Hz)    = 25000000
	*            PLL_M                = 5
	*            PLL_N                = 160
	*            PLL_P                = 2
	*            PLL_Q                = 4
	*            PLL_R                = 2
	*            VDD(V)               = 3.3
	*            Flash Latency(WS)    = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	HAL_StatusTypeDef ret = HAL_OK;
	/*ʹ�ܹ������ø��� */
	MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);

	/* ��������ʱ��Ƶ�ʵ������ϵͳƵ��ʱ����ѹ���ڿ����Ż����ģ�
		   ����ϵͳƵ�ʵĵ�ѹ����ֵ�ĸ��¿��Բο���Ʒ�����ֲᡣ  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	/* ����HSE������ʹ��HSE��ΪԴ����PLL */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
	RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

	RCC_OscInitStruct.PLL.PLLM = 5;
	RCC_OscInitStruct.PLL.PLLN = 160;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLQ = 4;

	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
	ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
	if (ret != HAL_OK)
	{
		while (1) { ; }
	}

	/* ѡ��PLL��Ϊϵͳʱ��Դ����������ʱ�ӷ�Ƶ�� */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | \
		RCC_CLOCKTYPE_HCLK | \
		RCC_CLOCKTYPE_D1PCLK1 | \
		RCC_CLOCKTYPE_PCLK1 | \
		RCC_CLOCKTYPE_PCLK2 | \
		RCC_CLOCKTYPE_D3PCLK1);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
	ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
	if (ret != HAL_OK)
	{
		while (1) { ; }
	}
}

/**
  * @brief  ��ֹͣģʽ���Ѻ�����ϵͳʱ��:����HSE��PLL��ѡ��PLL��Ϊϵͳʱ��Դ��
  * @param  ��
  * @retval ��
  */
static void SYSCLKConfig_STOP(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	uint32_t pFLatency = 0;
	
	/* �����ڲ�RCC�Ĵ�����ȡ�������� */
	HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

	/* ��ֹͣģʽ���Ѻ���������ϵͳʱ��: ����HSE��PLL */
	RCC_OscInitStruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState        = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState    = RCC_PLL_ON;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
	while(1) { ; }
	}

	/* �����ڲ�RCC�Ĵ�����ȡʱ������ */
	HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

	/* ѡ�� PLL ��Ϊϵͳʱ��Դ, ������ HCLK��PCLK1 �� PCLK2ʱ�ӷ�Ƶϵ�� */
	RCC_ClkInitStruct.ClockType     = RCC_CLOCKTYPE_SYSCLK;
	RCC_ClkInitStruct.SYSCLKSource  = RCC_SYSCLKSOURCE_PLLCLK;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, pFLatency) != HAL_OK)
	{
	while(1) { ; }
	}
}
/*********************************************END OF FILE**********************/

