/**
  ******************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   GPIO���--ʹ�ù̼������LED��
  ******************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� STM32H743������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************
  */  
#include "stm32h7xx.h"
#include "main.h"
#include "./led/bsp_led.h"
#include "./key/bsp_key.h" 
#include "./iwdg/bsp_iwdg.h"  
/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{
	/* ϵͳʱ�ӳ�ʼ����400MHz */
	SystemClock_Config();
	/* LED �˿ڳ�ʼ�� */
	LED_GPIO_Config();	
	HAL_Delay(1000);
	/* ����Ƿ�Ϊ�������Ź���λ */
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST) != RESET)
	{
		/* �������Ź���λ */
		/*  ����� */
		LED_RED;

		/* �����־ */
		__HAL_RCC_CLEAR_RESET_FLAGS();
		
		/*���һֱ��ι������һֱ��λ������ǰ�����ʱ���ῴ�������˸
		��1s ʱ����ι���Ļ������������̵�*/
	}
	else
	{
		/*���Ƕ������Ź���λ(����Ϊ�ϵ縴λ�����ֶ�������λ֮���) */
		/* ������ */
		LED_BLUE;
	}		

	/*��ʼ������*/
	Key_GPIO_Config();	
	
	// IWDG 1s ��ʱ���
	IWDG_Config(IWDG_PRESCALER_64 ,625); 
	
 
	//while��������������Ŀ�о�����Ҫд�Ĵ��룬�ⲿ�ֵĳ�������ö������Ź������
	//�������֪���ⲿ�ִ����ִ��ʱ�䣬������500ms����ô���ǿ������ö������Ź���
	//���ʱ����600ms����500ms��һ�㣬���Ҫ����صĳ���û���ܷ�����ִ�еĻ�����ô
	//ִ�����֮��ͻ�ִ��ι���ĳ�����������ܷ����ǳ���ͻᳬʱ�����ﲻ��ι����
	//���򣬴�ʱ�ͻ����ϵͳ��λ������Ҳ���ų������ܷ������ܻ����ˣ��պ�ι���ˣ�
	//������š�����Ҫ�����ȷ�ļ�س��򣬿���ʹ�ô��ڿ��Ź������ڿ��Ź��涨����
	//�ڹ涨�Ĵ���ʱ����ι����
	while(1)                            
	{	   
		if( Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON  )
		{
			// ι���������ι����ϵͳ��Ḵλ����λ������ƣ������1s
			// ʱ����׼ʱι���Ļ���������̵�
			IWDG_Feed();		
			//ι�������̵�
			LED_GREEN;
		}
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

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
 
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
  if(ret != HAL_OK)
  {

    while(1) { ; }
  }
  
	/* ѡ��PLL��Ϊϵͳʱ��Դ����������ʱ�ӷ�Ƶ�� */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK  | \
																 RCC_CLOCKTYPE_HCLK    | \
																 RCC_CLOCKTYPE_D1PCLK1 | \
																 RCC_CLOCKTYPE_PCLK1   | \
                                 RCC_CLOCKTYPE_PCLK2   | \
																 RCC_CLOCKTYPE_D3PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;  
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2; 
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2; 
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2; 
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}
/****************************END OF FILE***************************/
