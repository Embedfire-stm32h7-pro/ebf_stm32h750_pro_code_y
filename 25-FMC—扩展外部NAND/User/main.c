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
  * ʵ��ƽ̨:Ұ�� STM32H50 
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
#include "./delay/core_delay.h" 
#include "./malloc/malloc.h"
#include "./nand/ftl.h"
#include "./nand/bsp_nand.h"


void SDRAM_Check(void);
uint32_t RadomBuffer[10000];

uint32_t ReadBuffer[10000];

uint32_t *pSDRAM;

long long count=0,sdram_count=0;
RNG_HandleTypeDef hrng;

/**
  * @brief  �ӳ�һ��ʱ��
  * @param  �ӳٵ�ʱ�䳤��
  * @retval None
  */
static void Delay(__IO uint32_t nCount)
{
  __IO uint32_t index = 0; 
  for(index = (100000 * nCount); index != 0; index--)
  {
  }
}
/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{   
	uint8_t *buf;
	uint8_t *backbuf;	
	
//	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
	
	/* ϵͳʱ�ӳ�ʼ����400MHz */
	SystemClock_Config();

	LED_GPIO_Config();
	/* ���ô���1Ϊ��115200 8-N-1 */
	UARTx_Config();	
	
	printf("\r\n ��ӭʹ��Ұ��  STM32 H750 �����塣\r\n");		 

	printf("\r\nҰ��STM32H50 ˫SDRAM 64MB 32bit��д��������\r\n");
		
	/*��ʼ��SDRAMģ��*/
	SDRAM_Init();
	/*��ʼ���ⲿ�ڴ��*/
	my_mem_init(SRAMEX);		
	/*����������ʾ���ڶ�дSDRAM����*/
	LED_BLUE;

	 	while(FTL_Init())			    //���NAND FLASH,����ʼ��FTL
	{
		printf("NAND Error!");
		Delay(500);				 
		printf("Please Check");
		Delay(500);				 
		LED1_TOGGLE;//�����˸
	}
	backbuf=mymalloc(SRAMEX,NAND_ECC_SECTOR_SIZE);	//����һ�������Ļ���
	buf=mymalloc(SRAMIN,NAND_ECC_SECTOR_SIZE);		//����һ�������Ļ���
	sprintf((char*)buf,"NAND Size:%dMB",(nand_dev.block_totalnum/1024)*(nand_dev.page_mainsize/1024)*nand_dev.block_pagenum);
	printf((char*)buf);	//��ʾNAND����  
	FTL_ReadSectors(backbuf,2,NAND_ECC_SECTOR_SIZE,1);//Ԥ�ȶ�ȡ����0����������,��ֹ��д�����ļ�ϵͳ��.
	
//test_writepage(9,0,256);
	//NAND_EraseBlock(0);
	test_readpage(9,0,256);
	
//  /*ѡ��PLL�����ΪRNGʱ��Դ */
//  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RNG;
//  PeriphClkInitStruct.RngClockSelection = RCC_RNGCLKSOURCE_PLL;
//  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

//  /*ʹ��RNGʱ��*/
//  __HAL_RCC_RNG_CLK_ENABLE();
//	/*��ʼ��RNGģ����������*/
//	hrng.Instance = RNG;
//	HAL_RNG_Init(&hrng);

//	printf("��ʼ����10000��SDRAM���������\r\n");   
//	for(count=0;count<10000;count++)

//	{
//			HAL_RNG_GenerateRandomNumber(&hrng,&RadomBuffer[count]);

//	}    
//	printf("10000��SDRAM����������������\r\n");

//	SDRAM_Check();
//	SDRAM_Test();
	while(1)
	{
		
	}		
}

void SDRAM_Check(void)
{
    pSDRAM=(uint32_t*)SDRAM_BANK_ADDR;
	count=0;
	printf("��ʼд��SDRAM\r\n");
	for(sdram_count=0;sdram_count<SDRAM_SIZE/4;sdram_count++)
	{
		*pSDRAM=RadomBuffer[count];
		count++;
		pSDRAM++;
		if(count>=10000)

		{
			count=0;
		}
	}
	printf("д�����ֽ���:%d\r\n",(uint32_t)pSDRAM-SDRAM_BANK_ADDR);

	count=0;
	pSDRAM=(uint32_t*)SDRAM_BANK_ADDR;
	printf("��ʼ��ȡSDRAM����ԭ������Ƚ�\r\n");
	sdram_count=0;
	for(;sdram_count<SDRAM_SIZE/4;sdram_count++)
	{
		if(*pSDRAM != RadomBuffer[count])
		{
			printf("���ݱȽϴ��󡪡��˳�~\r\n");
			break;
		}
		count++;
		pSDRAM++;
		if(count>=10000)
		{
			count=0;
		}
	}

	printf("�Ƚ�ͨ�����ֽ���:%d\r\n",(uint32_t)pSDRAM-SDRAM_BANK_ADDR);

	if(sdram_count == SDRAM_SIZE/4)
	{
		LED_GREEN;
		printf("SDRAM���Գɹ�\r\n");
	}
	else
	{
		LED_RED;
		printf("SDRAM����ʧ��\r\n");
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
/****************************END OF FILE***************************/
