/**
  ******************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   FMC-SDRAM
  ******************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� STM32H750������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************
  */  
#include <stdlib.h>
#include "stm32h7xx.h"
#include "main.h"
#include "./led/bsp_led.h" 
#include "./usart/bsp_usart.h"
#include "./sdram/bsp_sdram.h" 
#include "./delay/core_delay.h" 

void SystemClock_Config(void);

void Delay(__IO uint32_t nCount); 

//���������SDRAM
uint32_t testValue  =7 ;
//���������SDRAM
uint32_t testValue2  =0;

//�������鵽SDRAM
uint8_t testGrup[100]  ={0};
//�������鵽SDRAM
uint8_t testGrup2[100] ={1,2,3};

/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{
  uint32_t inerTestValue =10;
  uint32_t * pointer;
  
  /* ϵͳʱ�ӳ�ʼ����400 MHz */
  SystemClock_Config();
  
  /* LED �˿ڳ�ʼ�� */
  LED_GPIO_Config();   
    
  /* ��ʼ������ */
  UARTx_Config();
  
  printf("\r\nSCT�ļ�Ӧ�á����Զ����������SDRAMʵ��\r\n");

  printf("\r\nʹ�á� uint32_t inerTestValue =10; ����䶨��ľֲ�������\r\n");
  printf("��������ĵ�ַΪ��0x%x,����ֵΪ��%d\r\n",(uint32_t)&inerTestValue,inerTestValue);

  printf("\r\nʹ�á�uint32_t testValue  =7 ;����䶨���ȫ�ֱ�����\r\n");
  printf("��������ĵ�ַΪ��0x%x,����ֵΪ��%d\r\n",(uint32_t)&testValue,testValue);

  printf("\r\nʹ�á�uint32_t testValue2  =0 ; ����䶨���ȫ�ֱ�����\r\n");
  printf("��������ĵ�ַΪ��0x%x,����ֵΪ��%d\r\n",(uint32_t)&testValue2,testValue2); 

  printf("\r\nʹ�á�uint8_t testGrup[100]  ={0};����䶨���ȫ�����飺\r\n");
  printf("��������ĵ�ַΪ��0x%x,����ֵΪ��%d,%d,%d\r\n",(uint32_t)&testGrup,testGrup[0],testGrup[1],testGrup[2]);

  printf("\r\nʹ�á�uint8_t testGrup2[100] ={1,2,3};����䶨���ȫ�����飺\r\n");
  printf("��������ĵ�ַΪ��0x%x,����ֵΪ��%d��%d,%d\r\n",(uint32_t)&testGrup2,testGrup2[0],testGrup2[1],testGrup2[2]);

  pointer = (uint32_t*)malloc(sizeof(uint32_t)*3);
  if(pointer != NULL)
  {
    *(pointer)=1;
    *(++pointer)=2;
    *(++pointer)=3; 

    printf("\r\nʹ�á� uint32_t *pointer = (uint32_t*)malloc(sizeof(uint32_t)*3); ����̬����ı���\r\n");
    printf("\r\n�����Ĳ���Ϊ��\r\n*(pointer++)=1;\r\n*(pointer++)=2;\r\n*pointer=3;");
    printf("\r\n��������������ĵ�ַΪ��0x%x,�鿴����ֵ������\r\n",(uint32_t)pointer); 
    printf("*(pointer--)=%d, \r\n",*(pointer--));
    printf("*(pointer--)=%d, \r\n",*(pointer--));
    printf("*(pointer)=%d, \r\n",*(pointer));
  }
  else
  {
    printf("\r\nʹ��malloc��̬���������������\r\n");  
  }
  /*�̵���*/
  LED_BLUE; 
  while(1);
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
void SystemClock_Config(void)
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
/****************************END OF FILE***************************/
