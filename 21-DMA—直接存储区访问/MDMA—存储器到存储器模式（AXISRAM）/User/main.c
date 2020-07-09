/**
  ******************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   USART��USART1�ӷ�����
  ******************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� STM32H750 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************
  */  
#include "stm32h7xx.h"
#include "main.h"
#include "./led/bsp_led.h" 
#include "./delay/core_delay.h"   
/* ��غ궨�壬ʹ�ô洢�����洢���������ʹ��DMA2 */
MDMA_HandleTypeDef     MDMA_Handle;

#define MDMA_INSTANCE            MDMA_Channel0
#define DMA_STREAM_CLOCK()       __HAL_RCC_MDMA_CLK_ENABLE();

#define BUFFER_SIZE              32


__IO uint32_t TransferCompleted = 0;

/* ����aSRC_Const_Buffer������ΪDMA��������Դ
  const�ؼ��ֽ�aSRC_Const_Buffer�����������Ϊ�������� */
static const uint32_t SRC_Const_Buffer[BUFFER_SIZE] =
{
  0x01020304, 0x05060708, 0x090A0B0C, 0x0D0E0F10,
  0x11121314, 0x15161718, 0x191A1B1C, 0x1D1E1F20,
  0x21222324, 0x25262728, 0x292A2B2C, 0x2D2E2F30,
  0x31323334, 0x35363738, 0x393A3B3C, 0x3D3E3F40,
  0x41424344, 0x45464748, 0x494A4B4C, 0x4D4E4F50,
  0x51525354, 0x55565758, 0x595A5B5C, 0x5D5E5F60,
  0x61626364, 0x65666768, 0x696A6B6C, 0x6D6E6F70,
  0x71727374, 0x75767778, 0x797A7B7C, 0x7D7E7F80
};

/* ����DMA����Ŀ��洢��AXISRAM */
__attribute__((at(0x24004000))) uint32_t DESTBuffer[BUFFER_SIZE];
                                                                       
static void MDMA_Config(void);                                    
static void Delay(__IO uint32_t nCount);
static void SystemClock_Config(void); 
uint8_t Buffercmp(const uint32_t* pBuffer, uint32_t* pBuffer1, uint16_t BufferLength); 
static void MDMA_TransferCompleteCallback(MDMA_HandleTypeDef *hmdma);         
        

/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{
	/* �����űȽϽ������ */
	uint8_t TransferStatus;
	/* ϵͳʱ�ӳ�ʼ����400 MHz */
	SystemClock_Config();
  
    /* Ĭ�ϲ����� MPU������Ҫ�������ܣ������� MPU ��ʹ�� 
   DMA ʱ��ע�� Cache �� �ڴ�����һ���Ե����⣬
   ����ע��������ο����׽̵̳� MPU ��������½� */
//  Board_MPU_Config(0, MPU_Normal_WT, 0xD0000000, MPU_32MB);
//  Board_MPU_Config(1, MPU_Normal_WT, 0x24000000, MPU_512KB);
  
  SCB_EnableICache();    // ʹ��ָ�� Cache
//  SCB_EnableDCache();    // ʹ������ Cache
  
	/* LED �˿ڳ�ʼ�� */
	LED_GPIO_Config();
	/* ����RGB��ɫ��Ϊ��ɫ */
	LED_PURPLE;  

	/* ����ʱ���� */
	Delay(0xFFFFFF);  

	/* DMA�������� */
	MDMA_Config(); 
  
	/* �ȴ�DMA������� */
	while(TransferCompleted == 0)
	{

	}   

	/* �Ƚ�Դ�����봫������� */
	TransferStatus=Buffercmp(SRC_Const_Buffer, DESTBuffer, BUFFER_SIZE);

	/* �ж�Դ�����봫������ݱȽϽ��*/
	if(TransferStatus==0)  
	{
		/* Դ�����봫������ݲ����ʱRGB��ɫ����ʾ��ɫ */
		LED_RED;
	}
	else
	{ 
		/* Դ�����봫����������ʱRGB��ɫ����ʾ��ɫ */
		LED_BLUE;
	}

	while (1)
	{		
	}
}

/* �򵥵���ʱ���� */
static void Delay(__IO uint32_t nCount)
{
	for(; nCount != 0; nCount--);
}

/**
  * DMA��������
  */
static void MDMA_Config(void)
{
	HAL_StatusTypeDef DMA_status = HAL_ERROR; 
	/* ʹ��MDMAʱ�� */
	DMA_STREAM_CLOCK();
  //MDMA����Ĵ�������ַ����ͨ��0
	MDMA_Handle.Instance = MDMA_INSTANCE;
  //�������MDMA�ɼ�
  MDMA_Handle.Init.Request              = MDMA_REQUEST_SW;
  
  MDMA_Handle.Init.TransferTriggerMode  = MDMA_FULL_TRANSFER;
  //MDMA��������ȼ�����   
  MDMA_Handle.Init.Priority             = MDMA_PRIORITY_HIGH;
  
  MDMA_Handle.Init.Endianness           = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  //Դ��ַ��ַ����������Ϊ��
  MDMA_Handle.Init.SourceInc            = MDMA_SRC_INC_WORD;
  //Ŀ���ַ��ַ����������Ϊ�ֽ�
  MDMA_Handle.Init.DestinationInc       = MDMA_DEST_INC_BYTE;
  //Դ��ַ���ݿ�ȣ���  
  MDMA_Handle.Init.SourceDataSize       = MDMA_SRC_DATASIZE_WORD;
  //Ŀ���ַ���ݿ�ȣ��ֽ�
  MDMA_Handle.Init.DestDataSize         = MDMA_DEST_DATASIZE_BYTE;
  //���ݶ����ʽ���Ҷ��루С��ģʽ��
  MDMA_Handle.Init.DataAlignment        = MDMA_DATAALIGN_PACKENABLE;  
  //Դ��ַ��ַ������  
  MDMA_Handle.Init.SourceBurst          = MDMA_SOURCE_BURST_SINGLE;
  //Ŀ���ַ��ַ������  
  MDMA_Handle.Init.DestBurst            = MDMA_DEST_BURST_SINGLE;
  
  MDMA_Handle.Init.BufferTransferLength = 4;
  MDMA_Handle.Init.SourceBlockAddressOffset  = 0;
  MDMA_Handle.Init.DestBlockAddressOffset    = 0;

  
  HAL_MDMA_Init(&MDMA_Handle);
  
  HAL_MDMA_RegisterCallback(&MDMA_Handle, HAL_MDMA_XFER_CPLT_CB_ID, MDMA_TransferCompleteCallback);

  /* Set Interrupt Group Priority */
  HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);
  
  /* Enable the MDMA channel global Interrupt */
  HAL_NVIC_EnableIRQ(MDMA_IRQn);
  

  DMA_status = HAL_MDMA_Start_IT(&MDMA_Handle, (uint32_t)&SRC_Const_Buffer, 
                                 (uint32_t)&DESTBuffer, 
                                 (BUFFER_SIZE * 4), 
                                 1);	/* �ж�DMA״̬ */
	if (DMA_status != HAL_OK)
	{
		/* DMA������ó�����������ѭ����RGB��ɫ����˸ */
		while (1)
		{      
			LED_RED;
			Delay(0xFFFFFF);
			LED_RGBOFF;
			Delay(0xFFFFFF);
		}
	} 
}

/**
  * �ж�ָ�����ȵ���������Դ�Ƿ���ȫ��ȣ�
  * �����ȫ��ȷ���1��ֻҪ����һ�����ݲ���ȷ���0
  */
uint8_t Buffercmp(const uint32_t* pBuffer, 
                  uint32_t* pBuffer1, uint16_t BufferLength)
{
	/* ���ݳ��ȵݼ� */
	while(BufferLength--)
	{
		/* �ж���������Դ�Ƿ��Ӧ��� */
		if(*pBuffer != *pBuffer1)
		{
		  /* ��Ӧ����Դ����������˳�������������0 */
		  return 0;
		}
		/* ������������Դ�ĵ�ַָ�� */
		pBuffer++;
		pBuffer1++;
	}
	/* ����жϲ��Ҷ�Ӧ������� */
	return 1;  
}
/**
  * @brief  MDMA conversion complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
static void MDMA_TransferCompleteCallback(MDMA_HandleTypeDef *hmdma)
{
  TransferCompleted = 1;
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
