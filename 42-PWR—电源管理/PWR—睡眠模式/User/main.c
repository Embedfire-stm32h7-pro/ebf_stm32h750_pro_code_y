/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   野火H743 PWR—睡眠模式
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 H743 开发板
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
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
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void) 
{
	HAL_Init();
	/* 初始化系统时钟为400MHz */
	SystemClock_Config();
	/* 初始化LED */
	LED_GPIO_Config();	
	/* 初始化调试串口，一般为串口1 */
	UARTx_Config();	
	/* 初始化按键为中断模式，按下中断后会进入中断服务函数	*/
	EXTI_Key_Config(); 
	
	printf("\r\n 欢迎使用野火 STM32H743 开发板。\r\n");
    printf("\r\n 野火H743 睡眠模式例程\r\n");
	
	printf("\r\n 实验说明：\r\n");

	printf("\r\n 1.本程序中，绿灯表示STM32正常运行，红灯表示睡眠状态，蓝灯表示刚从睡眠状态被唤醒\r\n");
	printf("\r\n 2.程序运行一段时间后自动进入睡眠状态，在睡眠状态下，可使用KEY1或KEY2唤醒\r\n");
	printf("\r\n 3.本实验执行这样一个循环：\r\n ------》亮绿灯(正常运行)->亮红灯(睡眠模式)->按KEY1或KEY2唤醒->亮蓝灯(刚被唤醒)-----》\r\n");
	printf("\r\n 4.在睡眠状态下，DAP下载器无法给STM32下载程序，\r\n 可按KEY1、KEY2唤醒后下载，\r\n 或按复位键使芯片处于复位状态，然后在电脑上点击下载按钮，再释放复位按键，即可下载\r\n");

  while(1)
  {	
		/*********执行任务***************************/
		printf("\r\n STM32正常运行，亮绿灯\r\n");
	
		LED_GREEN;	
		HAL_Delay(2000);		
		/*****任务执行完毕，进入睡眠降低功耗***********/
		
		printf("\r\n 进入睡眠模式，亮红灯,按KEY1或KEY2按键可唤醒\r\n");

		//使用红灯指示，进入睡眠状态
		LED_RED;
		//暂停滴答时钟，防止通过滴答时钟中断唤醒
		HAL_SuspendTick();
		//进入睡眠模式
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON,PWR_SLEEPENTRY_WFI);
		//等待中断唤醒  K1或K2按键中断	
		/***被唤醒，亮蓝灯指示***/
		LED_BLUE;
		//被唤醒后，恢复滴答时钟	  
		HAL_ResumeTick();
		HAL_Delay(2000);	
			
		printf("\r\n 已退出睡眠模式\r\n");
		//继续执行while循环
  }
}

/**
  * @brief  System Clock 配置
  *         system Clock 配置如下:
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
	/*使能供电配置更新 */
	MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);

	/* 当器件的时钟频率低于最大系统频率时，电压调节可以优化功耗，
		   关于系统频率的电压调节值的更新可以参考产品数据手册。  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	/* 启用HSE振荡器并使用HSE作为源激活PLL */
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

	/* 选择PLL作为系统时钟源并配置总线时钟分频器 */
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
/*********************************************END OF FILE**********************/

