/**
  ******************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   LTDC��Һ����ʾ����
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
#include "./camera/bsp_ov5640.h"
#include "./camera/ov5640_AF.h"
#include "./delay/core_delay.h"  
#include "./mpu/bsp_mpu.h"

/*���������*/
#define TASK_ENABLE 0
#define NumOfTask 3

#if USE_ExtFlash_Single
__IO uint8_t* qspi_addr = (__IO uint8_t*)(0x90000000);
#endif

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
	OV5640_IDTypeDef OV5640_Camera_ID;
	
	/* ϵͳʱ�ӳ�ʼ����400MHz */
	SystemClock_Config();
	
  /* Ĭ�ϲ����� MPU������Ҫ�������ܣ������� MPU ��ʹ�� 
   DMA ʱ��ע�� Cache �� �ڴ�����һ���Ե����⣬
   ����ע��������ο����׽̵̳� MPU ��������½� */
  Board_MPU_Config(0, MPU_Normal_WT, 0xD0000000, MPU_32MB);
  Board_MPU_Config(1, MPU_Normal_WT, 0x24000000, MPU_128KB);
	/* ����I-Cache */
	SCB_EnableICache();
	/* ����D-Cache */
	SCB_EnableDCache();
	/* ����SysTick */
	SysTick_Init();
	
	/* LED �˿ڳ�ʼ�� */
	LED_GPIO_Config();
	/* ���ô���1Ϊ��115200 8-N-1 */
	UARTx_Config();	
	
	printf("\r\n ��ӭʹ��Ұ��  STM32 H750 �����塣\r\n");		 
	printf("\r\nҰ��STM32H750 OV5640����ͷ��������\r\n");
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
	LCD_DisplayStringLine_EN_CH(1,(uint8_t* )" ģʽ:UXGA 800x480");
	CAMERA_DEBUG("STM32H750 DCMI ����OV5640����");
	I2CMaster_Init();
	OV5640_HW_Init();			
	//��ʼ�� I2C
	
	/* ��ȡ����ͷоƬID��ȷ������ͷ�������� */
	OV5640_ReadID(&OV5640_Camera_ID);

	if(OV5640_Camera_ID.PIDH  == 0x56)
	{
		CAMERA_DEBUG("%x%x",OV5640_Camera_ID.PIDH ,OV5640_Camera_ID.PIDL);
	}
	else
	{
		LCD_SetColors(LCD_COLOR_WHITE,TRANSPARENCY);
		LCD_DisplayStringLine_EN_CH(8,(uint8_t*) "         û�м�⵽OV5640�������¼�����ӡ�");
		CAMERA_DEBUG("û�м�⵽OV5640����ͷ�������¼�����ӡ�");
		while(1);  
	}
		/* ��������ͷ������ظ�ʽ */
	OV5640_RGB565Config();	
	/* ��ʼ������ͷ��������ʾͼ�� */
	OV5640_Init();
	//ˢOV5640���Զ��Խ��̼�
	OV5640_AUTO_FOCUS();
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
			//����
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
  RCC_OscInitStruct.PLL.PLLQ = 2;
 
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

/**
  * @brief  ����ϵͳ�δ�ʱ�� SysTick
  * @param  ��
  * @retval ��
  */
void SysTick_Init(void)
{
	/* SystemFrequency / 1000    1ms�ж�һ��
	 * SystemFrequency / 100000	 10us�ж�һ��
	 * SystemFrequency / 1000000 1us�ж�һ��
	 */
	if (HAL_SYSTICK_Config(SystemCoreClock / 1000))
	{ 
		/* Capture error */ 
		while (1);
	}
}

/****************************END OF FILE***************************/
