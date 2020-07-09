/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   野火H743 PVD监控例程
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
#include "./usart/bsp_debug_usart.h"
#include "./key/bsp_key.h" 
#include "./pvd/bsp_pvd.h"

/*
*【 ！】实验操作：
* 1.使用外部可调电源，调节成5V输出，连接到实验板引出的 5V<--->GND排针给板子供电；
*2.复位实验板，电压正常时板子上的LED彩灯应为绿色
*3.向下调节可调电源的电压，大约当降至4V的时候，LED彩灯会转为红色。
*（程序中控制PVD监控电压约为2.8V,当5V电源降至4V的时候，连接STM32的VDD电源(3.3V电源)会低于2.8V，产生PVD事件，在中断中控制亮红灯）
*/

/*【 ！！】注意事项：
*使用可调电源给实验板供电，其它电源线都拔掉(包括下载器、USB线)。
*由于直接接排针供电没有电路保护，调节电源时小心不要使供电电压远高于5V,电压太高会烧坏实验板！！
*/
/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{ 
	/* 配置系统时钟为400 MHz */
  SystemClock_Config();
	/* 初始化LED */
	LED_GPIO_Config();	
	//亮绿灯，表示正常运行
	LED_GREEN; 
	
	//配置PVD，当电压过低时，会进入中断服务函数，亮红灯
	PVD_Config();
	
	while(1)
	{				
		/*正常运行的程序*/
	}

}
/**
  * @brief  PWR PVD interrupt callback
  * @param  None 
  * @retval None
  */
void HAL_PWR_PVDCallback(void)
{
  /* 亮红灯，实际应用中应进入紧急状态处理 */
  LED_RED;
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

