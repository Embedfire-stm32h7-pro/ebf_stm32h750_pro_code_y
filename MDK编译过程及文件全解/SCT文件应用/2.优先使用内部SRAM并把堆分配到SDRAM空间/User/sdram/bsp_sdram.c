/**
  ******************************************************************************
  * @file    bsp_sdram.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   sdramӦ�ú����ӿ�
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 H750 ������  
  * ��̳    :http://www.chuxue123.com
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./sdram/bsp_sdram.h"  
//static FMC_SDRAM_CommandTypeDef Command;
SDRAM_HandleTypeDef hsdram1;
#define sdramHandle hsdram1
/**
  * @brief  �ӳ�һ��ʱ��
  * @param  �ӳٵ�ʱ�䳤��
  * @retval None
  */
static void SDRAM_delay(__IO uint32_t nCount)
{
  __IO uint32_t index = 0; 
  for(index = (100000 * nCount); index != 0; index--)
  {
  }
}

/**
  * @brief  ��ʼ������SDRAM��IO
  * @param  ��
  * @retval ��
  */
static void SDRAM_GPIO_Config(void)
{		
  //ʹ��GPIO��ʱ��
  RCC->AHB4ENR |= 0x1FC; 

  //���ö˿�C��D��E��F��G��H��I��Ӧ������Ϊ�����������
  GPIOC->MODER = 0xFFFFFFFE;
  GPIOC->OTYPER = 0;
  GPIOC->OSPEEDR = 0x00000003;
  GPIOC->PUPDR = 0x00000001;
  GPIOC->AFR[0] = 0x0000000C;
  
  GPIOD->MODER = 0xAFEAFFFA;
  GPIOD->OTYPER = 0;
  GPIOD->OSPEEDR = 0xF03F000F;
  GPIOD->PUPDR = 0x50150005;
  GPIOD->AFR[0] = 0x000000CC;  
  GPIOD->AFR[1] = 0xCC000CCC;

  GPIOE->MODER = 0xAAAABFFA;
  GPIOE->OTYPER = 0;
  GPIOE->OSPEEDR = 0xFFFFC00F;
  GPIOE->PUPDR = 0x55554005;
  GPIOE->AFR[0] = 0xC00000CC;  
  GPIOE->AFR[1] = 0xCCCCCCCC;
  
  GPIOF->MODER = 0xAABFFAAA;
  GPIOF->OTYPER = 0;
  GPIOF->OSPEEDR = 0xFFC00FFF;
  GPIOF->PUPDR = 0x55400555;
  GPIOF->AFR[0] = 0x00CCCCCC;
  GPIOF->AFR[1] = 0xCCCCC000;
  
  GPIOG->MODER = 0xBFFEFAEA;
  GPIOG->OTYPER = 0;
  GPIOG->OSPEEDR = 0xC0030F3F;
  GPIOG->PUPDR = 0x40010515;
  GPIOG->AFR[0] = 0x00CC0CCC;  
  GPIOG->AFR[1] = 0xC000000C;

  GPIOH->MODER = 0xAAAAAFFF;
  GPIOH->OTYPER = 0;
  GPIOH->OSPEEDR = 0xFFFFF000;
  GPIOH->PUPDR = 0x55555000;
  GPIOH->AFR[0] = 0xCC000000;  
  GPIOH->AFR[1] = 0xCCCCCCCC;  

  GPIOI->MODER = 0xFFEBAAAA;
  GPIOI->OTYPER = 0;
  GPIOI->OSPEEDR = 0x003CFFFF;
  GPIOI->PUPDR = 0x00145555;
  GPIOI->AFR[0] = 0xCCCCCCCC;  
  GPIOI->AFR[1] = 0x00000CC0;

}

/**
  * @brief  ��SDRAMоƬ���г�ʼ������
  * @param  None. 
  * @retval None.
  */
static void SDRAM_InitSequence(void)
{


	/* Step 1 ----------------------------------------------------------------*/
	/* ������������ṩ��SDRAM��ʱ�� */
  FMC_Bank5_6->SDCMR = 0x00000009;
	/* Step 2: ��ʱ100us */ 
	SDRAM_delay(1);
	/* Step 3 ----------------------------------------------------------------*/
	/* ������������е�bankԤ��� */  
  FMC_Bank5_6->SDCMR = 0x0000000A;
	/* Step 4 ----------------------------------------------------------------*/
	/* ��������Զ�ˢ�� */   
  FMC_Bank5_6->SDCMR = 0x000000EB;
	/* Step 5 ----------------------------------------------------------------*/
	/* ����sdram�Ĵ������� */
  FMC_Bank5_6->SDCMR = 0x0004600C;
	/* Step 6 ----------------------------------------------------------------*/
  /* ������ˢ������ */
  FMC_Bank5_6->SDRTR |= (1855<<1);
}


/**
  * @brief  ��ʼ������ʹ��SDRAM��FMC��GPIO�ӿڣ�
  *         ��������SDRAM��д����ǰ��Ҫ������
  * @param  None
  * @retval None
  */
void SDRAM_Init(void)
{

  /* ����FMC�ӿ���ص� GPIO*/
  SDRAM_GPIO_Config();
  
  //ʹ��HSE
  RCC->CR |= RCC_CR_HSEON;
  while(!(RCC->CR&(1<<17)));
  //ѡ��HSE��ΪPLLʱ��Դ
  RCC->PLLCKSELR |= (1 << 1);
  //ʹ��PLL2R
  RCC->PLLCKSELR |= (25 << 12);//PLL2M
  RCC->PLLCFGR |= (1 << 21); 
  RCC->PLLCFGR &= ~(3 << 6);
  RCC->PLLCFGR &= ~(1 << 5);
  RCC->PLLCFGR &= ~(1 << 4);
  RCC->PLL2DIVR |= (265 << 0);//PLL2N
  RCC->PLL2DIVR |= (2 << 9);//PLL2P
  RCC->PLL2DIVR |= (2 << 16);//PLL2Q
  RCC->PLL2DIVR |= (2 << 24);//PLL2R

  //ʹ��PLL2
  RCC->CR |= (1 << 26);
  while(!(RCC->CR&(1<<27)));
  RCC->D1CCIPR |= (2<<0); 
  
  (RCC->AHB3ENR |= (RCC_AHB3ENR_FMCEN));
  
  FMC_Bank5_6->SDCR[FMC_SDRAM_BANK1] = 0x00003AD0;
  FMC_Bank5_6->SDCR[FMC_SDRAM_BANK2] = 0x000001E9;
  
  FMC_Bank5_6->SDTR[FMC_SDRAM_BANK1] = 0x0F1F7FFF;
  FMC_Bank5_6->SDTR[FMC_SDRAM_BANK2] = 0x01010471;
  
  __FMC_ENABLE();                      
  /* FMC SDRAM �豸ʱ���ʼ�� */
  SDRAM_InitSequence(); 
  
}



/**
  * @brief  �ԡ��֡�Ϊ��λ��sdramд������ 
  * @param  pBuffer: ָ�����ݵ�ָ�� 
  * @param  uwWriteAddress: Ҫд���SDRAM�ڲ���ַ
  * @param  uwBufferSize: Ҫд�����ݴ�С
  * @retval None.
  */
void SDRAM_WriteBuffer(uint32_t* pBuffer, uint32_t uwWriteAddress, uint32_t uwBufferSize)
{
  __IO uint32_t write_pointer = (uint32_t)uwWriteAddress;

  /* ��ֹд���� */
  //HAL_SDRAM_WriteProtection_Disable(&hsdram1);
  FMC_Bank5_6->SDCR[1] &= ~FMC_SDRAM_WRITE_PROTECTION_ENABLE;
  /* ���SDRAM��־���ȴ���SDRAM���� */ 
//  while(HAL_SDRAM_GetState(&hsdram1) != RESET)
//  {
//  }

  /* ѭ��д������ */
  for (; uwBufferSize != 0; uwBufferSize--) 
  {
    /* �������ݵ�SDRAM */
    *(uint32_t *) (SDRAM_BANK_ADDR + write_pointer) = *pBuffer++;

    /* ��ַ����*/
    write_pointer += 4;
  }
    
}

/**
  * @brief  ��SDRAM�ж�ȡ���� 
  * @param  pBuffer: ָ��洢���ݵ�buffer
  * @param  ReadAddress: Ҫ��ȡ���ݵĵ�ʮ
  * @param  uwBufferSize: Ҫ��ȡ�����ݴ�С
  * @retval None.
  */
void SDRAM_ReadBuffer(uint32_t* pBuffer, uint32_t uwReadAddress, uint32_t uwBufferSize)
{
  __IO uint32_t write_pointer = (uint32_t)uwReadAddress;
  
   
  /* ���SDRAM��־���ȴ���SDRAM���� */  
//  while ( HAL_SDRAM_GetState(&hsdram1) != RESET)
//  {
//  }
  
  /*��ȡ���� */
  for(; uwBufferSize != 0x00; uwBufferSize--)
  {
   *pBuffer++ = *(__IO uint32_t *)(SDRAM_BANK_ADDR + write_pointer );
    
   /* ��ַ����*/
    write_pointer += 4;
  } 
}


/**
  * @brief  ����SDRAM�Ƿ����� 
  * @param  None
  * @retval ��������1���쳣����0
  */
uint8_t SDRAM_Test(void)
{
  /*д�����ݼ�����*/
  uint32_t counter=0;
  
  /* 8λ������ */
  uint8_t ubWritedata_8b = 0, ubReaddata_8b = 0;  
  
  /* 16λ������ */
  uint16_t uhWritedata_16b = 0, uhReaddata_16b = 0; 
  
  SDRAM_INFO("���ڼ��SDRAM����8λ��16λ�ķ�ʽ��дsdram...");


  /*��8λ��ʽ��д���ݣ���У��*/
  
  /* ��SDRAM����ȫ������Ϊ0 ��SDRAM_SIZE����8λΪ��λ�� */
  for (counter = 0x00; counter < SDRAM_SIZE; counter++)
  {
    *(__IO uint8_t*) (SDRAM_BANK_ADDR + counter) = (uint8_t)0x0;
  }
  
  /* ������SDRAMд������  8λ */
  for (counter = 0; counter < SDRAM_SIZE; counter++)
  {
    *(__IO uint8_t*) (SDRAM_BANK_ADDR + counter) = (uint8_t)(ubWritedata_8b + counter);
  }
  
  /* ��ȡ SDRAM ���ݲ����*/
  for(counter = 0; counter<SDRAM_SIZE;counter++ )
  {
    ubReaddata_8b = *(__IO uint8_t*)(SDRAM_BANK_ADDR + counter);  //�Ӹõ�ַ��������
    
    if(ubReaddata_8b != (uint8_t)(ubWritedata_8b + counter))      //������ݣ�������ȣ���������,���ؼ��ʧ�ܽ����
    {
      SDRAM_ERROR("8λ���ݶ�д���󣡳���λ�ã�%d",counter);
      return 0;
    }
  }
	
  
  /*��16λ��ʽ��д���ݣ������*/
  
  /* ��SDRAM����ȫ������Ϊ0 */
  for (counter = 0x00; counter < SDRAM_SIZE/2; counter++)
  {
    *(__IO uint16_t*) (SDRAM_BANK_ADDR + 2*counter) = (uint16_t)0x00;
  }
  
  /* ������SDRAMд������  16λ */
  for (counter = 0; counter < SDRAM_SIZE/2; counter++)
  {
    *(__IO uint16_t*) (SDRAM_BANK_ADDR + 2*counter) = (uint16_t)(uhWritedata_16b + counter);
  }
  
    /* ��ȡ SDRAM ���ݲ����*/
  for(counter = 0; counter<SDRAM_SIZE/2;counter++ )
  {
    uhReaddata_16b = *(__IO uint16_t*)(SDRAM_BANK_ADDR + 2*counter);  //�Ӹõ�ַ��������
    
    if(uhReaddata_16b != (uint16_t)(uhWritedata_16b + counter))      //������ݣ�������ȣ���������,���ؼ��ʧ�ܽ����
    {
      SDRAM_ERROR("16λ���ݶ�д���󣡳���λ�ã�%d",counter);

      return 0;
    }
  }

  
  SDRAM_INFO("SDRAM��д����������"); 
  /*���������return 1 */
  return 1;
  

}


/*********************************************END OF FILE**********************/

