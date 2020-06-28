/**
  ******************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   QSPI FLASH������д����
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
#include "bsp_led.h"
#include "bsp_debug_usart.h"

typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus;
/* ��ȡ�������ĳ��� */
#define TxBufferSize1   (countof(TxBuffer1) - 1)
#define RxBufferSize1   (countof(TxBuffer1) - 1)
#define countof(a)      (sizeof(a) / sizeof(*(a)))
#define  BufferSize (countof(Tx_Buffer)-1)

#define  FLASH_WriteAddress     0
#define  FLASH_ReadAddress      FLASH_WriteAddress
#define  FLASH_SectorToErase    FLASH_WriteAddress

   
/* ���ͻ�������ʼ�� */
volatile uint8_t Tx_Buffer[256] = "��л��ѡ��Ұ��stm32������\r\nhttp://firestm32.taobao.com";
volatile uint8_t Rx_Buffer[BufferSize];

//��ȡ��ID�洢λ��
__IO uint32_t DeviceID = 0;
__IO uint32_t FlashID = 0;
__IO TestStatus TransferStatus1 = FAILED;

// ����ԭ������
void Delay(__IO uint32_t nCount);

extern uint32_t QSPI_WritePage(unsigned long adr, unsigned long sz, unsigned char *buf);
extern uint32_t QSPI_ReadPage(unsigned long adr, unsigned long sz, unsigned char *buf);
extern uint32_t QSPI_EraseSector(uint32_t adr);

TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{
	int i=0;
	/* ʹ��ָ��� */
	SCB_EnableICache();
  /* ʹ�����ݻ��� */
  SCB_EnableDCache();
	/* ϵͳʱ�ӳ�ʼ����400MHz */
//	SystemClock_Config();
//	/* LED �˿ڳ�ʼ�� */
//	LED_GPIO_Config();	
//	LED_BLUE;
	
	/* ���ô���1Ϊ��115200 8-N-1 */
	DEBUG_USART_Config();
  
	printf("\r\n����һ��64M����flash(W25Q256)ʵ��(QSPI����) \r\n");
	
	/* 64M����flash W25Q256��ʼ�� */
	QSPI_Init();

//	/* ��ȡ Flash Device ID */
//	DeviceID = QSPI_FLASH_ReadDeviceID();
//	
//	Delay( 200 );
//	
//	/* ��ȡ SPI Flash ID */
//	FlashID = QSPI_FLASH_ReadID();
//	
//	printf("\r\nFlashID is 0x%X,  Manufacturer Device ID is 0x%X\r\n", FlashID, DeviceID);
//	
//	/* ���� SPI Flash ID */
//	if (FlashID == sFLASH_ID) 
	{	
			/* Flash�������ֽڵ�ַģʽ */
//		QSPI_EnterFourBytesAddress();
//		printf("\r\n��⵽QSPI FLASH W25Q256 !\r\n");
		/* ������Ҫд��� QSPI FLASH ������FLASHд��ǰҪ�Ȳ��� */
//		for(i=0;i<32;i++)
//		BSP_QSPI_Erase_Block(i*0x1000);
//		FlashErase(FLASH_WriteAddress,NULL);	
  	QSPI_EraseSector((uint32_t)FLASH_WriteAddress);		
//		Delay(1000);
		for(i=0;i<256;i++)
		Rx_Buffer[i]=0;
//		QSPI_FLASH_Init();
		/* ������������ݶ������ŵ����ջ������� */
		QSPI_ReadPage(FLASH_WriteAddress, BufferSize,Rx_Buffer);
		printf("\r\n����������Ϊ��\r\n");
		for(i=0;i<256;i++)
		printf("0x%x ", Rx_Buffer[i]);
//		QSPI_FLASH_Init();
		printf("\r\nд�������Ϊ��\r\n");
		for(i=0;i<256;i++)
		{	
			Tx_Buffer[i]=0xab;
			printf("0x%x ", Tx_Buffer[i]);
		}
		
		/* �����ͻ�����������д��flash�� */
		QSPI_WritePage(FLASH_WriteAddress, BufferSize,Tx_Buffer);
//		FlashWrite(FLASH_WriteAddress,0,256,Tx_Buffer);
//		printf("\r\nд�������Ϊ��\r\n%s", Tx_Buffer);
//		QSPI_FLASH_Init();
		/* ���ո�д������ݶ������ŵ����ջ������� */
		QSPI_ReadPage(FLASH_ReadAddress, BufferSize,Rx_Buffer);
//		printf("\r\n����������Ϊ��\r\n%s", Rx_Buffer);
		printf("\r\n����������Ϊ��\r\n");
		for(i=0;i<256;i++)
		printf("0x%x ", Rx_Buffer[i]);		
		/* ���д�������������������Ƿ���� */
		TransferStatus1 = Buffercmp(Tx_Buffer, Rx_Buffer, BufferSize);
		
		if( PASSED == TransferStatus1 )
		{    
//			LED_GREEN;
			printf("\r\n64M����flash(W25Q256)���Գɹ�!\n\r");
		}
		else
		{        
//			LED_RED;
			printf("\r\n64M����flash(W25Q256)����ʧ��!\n\r");
		}
	}// if (FlashID == sFLASH_ID)
//	else
//	{    
//		LED_RED;
//		printf("\r\n��ȡ���� W25Q256 ID!\n\r");
//	}
	
	while(1);  
}
/*
 * ��������Buffercmp
 * ����  ���Ƚ������������е������Ƿ����
 * ����  ��-pBuffer1     src������ָ��
 *         -pBuffer2     dst������ָ��
 *         -BufferLength ����������
 * ���  ����
 * ����  ��-PASSED pBuffer1 ����   pBuffer2
 *         -FAILED pBuffer1 ��ͬ�� pBuffer2
 */
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while(BufferLength--)
  {
    if(*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }

    pBuffer1++;
    pBuffer2++;
  }
  return PASSED;
}
void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
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
