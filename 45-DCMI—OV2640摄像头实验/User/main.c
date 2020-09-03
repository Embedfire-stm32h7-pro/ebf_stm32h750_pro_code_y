/**
  ******************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   OV2640����ͷʵ��
  ******************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� STM32H750������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************
  */  
#include "stm32h7xx.h"
#include "main.h"
#include "./led/bsp_led.h" 
#include "./usart/bsp_usart.h"
#include "./sdram/bsp_sdram.h" 
#include "./lcd/bsp_lcd.h"
#include "string.h"
#include "./i2c/bsp_i2c.h"
#include "./camera/bsp_ov2640.h"

#include "./delay/core_delay.h"  

/*���������*/
#define TASK_ENABLE 0
#define NumOfTask 3

__IO uint8_t* qspi_addr = (__IO uint8_t*)(0x90000000);

extern uint32_t Task_Delay[NumOfTask];
uint32_t Task_Delay[NumOfTask]={0};
uint8_t dispBuf[100];
uint8_t fps=0;
/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{  
  OV2640_IDTypeDef OV2640_Camera_ID; 		
	/* ϵͳʱ�ӳ�ʼ����400MHz */
	SystemClock_Config();
	/* LED �˿ڳ�ʼ�� */
	LED_GPIO_Config();
	/* ���ô���1Ϊ��115200 8-N-1 */
	UARTx_Config();	
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0x0E);
	printf("\r\n ��ӭʹ��Ұ��  STM32 H750 �����塣\r\n");		 
	printf("\r\nҰ��STM32H750 OV2640����ͷ��������\r\n");
	/*����������ʾ���ڶ�дSDRAM����*/
	LED_BLUE;
	/* LCD �˿ڳ�ʼ�� */ 
	LCD_Init();
	/* LCD ��һ���ʼ�� */ 
	LCD_LayerInit(0, LCD_FB_START_ADDRESS,RGB565);
	/* LCD �ڶ����ʼ�� */ 
	LCD_LayerInit(1, LCD_FB_START_ADDRESS+(LCD_GetXSize()*LCD_GetYSize()*4),ARGB8888);
	/* ʹ��LCD������������ */ 
	LCD_DisplayOn(); 

	/* ѡ��LCD��һ�� */
	LCD_SelectLayer(0);

	/* ��һ����������ʾȫ��ɫ */ 
	LCD_Clear(LCD_COLOR_BLUE);  

	/* ѡ��LCD�ڶ��� */
	LCD_SelectLayer(1);

	/* �ڶ�����������ʾ͸�� */ 
	LCD_Clear(TRANSPARENCY);

	/* ���õ�һ�͵ڶ����͸����,0λ��ȫ͸����255Ϊ��ȫ��͸��*/
	LCD_SetTransparency(0, 255);
	LCD_SetTransparency(1, 255);
	
	LCD_SetColors(LCD_COLOR_WHITE,TRANSPARENCY);
	LCD_DisplayStringLine_EN_CH(1,(uint8_t* )" mode:UXGA 800x480");
	CAMERA_DEBUG("STM32H750 DCMI ����OV2640����");
		
	//��ʼ�� I2C
	I2CMaster_Init(); 
	HAL_Delay(100);
	OV2640_HW_Init();
	/* ��ȡ����ͷоƬID��ȷ������ͷ�������� */
	OV2640_ReadID(&OV2640_Camera_ID);

	if(OV2640_Camera_ID.PIDH  == 0x26)
	{
		CAMERA_DEBUG("%x%x",OV2640_Camera_ID.PIDH ,OV2640_Camera_ID.PIDL);
	}
	else
	{
		LCD_SetColors(LCD_COLOR_WHITE,TRANSPARENCY);
		LCD_DisplayStringLine_EN_CH(8,(uint8_t*) "         no check OV2640��please check the connect��");
		CAMERA_DEBUG("û�м�⵽OV2640����ͷ�������¼�����ӡ�");
		while(1);  
	}
  /* ��������ͷ������ظ�ʽ */
	OV2640_UXGAConfig();
  /* ��ʼ������ͷ��������ʾͼ�� */
	OV2640_Init();
	//����
  fps =0;
	Task_Delay[0]=1000;
	
	while(1)
	{
		if(Task_Delay[0]==0)
		{
			LCD_SelectLayer(1);       
			LCD_SetColors(LCD_COLOR_WHITE,TRANSPARENCY);
			sprintf((char*)dispBuf, " ֡��:%d FPS", fps/1);
			LCD_ClearLine(2);
			/*���֡��*/
			LCD_DisplayStringLine_EN_CH(2,dispBuf);		
      fps =0;
			Task_Delay[0]=1000; //��ֵÿ1ms���1������0�ſ������½�������
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
	*            PLL_Q                = 2
	*            PLL_R                = 2
	*            VDD(V)               = 3.3
	*            Flash Latency(WS)    = 4
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** ���õ�Դ���ø���
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** ����������ѹ�������ѹ
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
 
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    while(1) { ; }
  }
}
/****************************END OF FILE***************************/
