/**
  ******************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   ��V1.2.0�汾�⽨�Ĺ���ģ��
  ******************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32H750������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************
  */  
#include "stm32h7xx.h"
#include "main.h"
#include "./led/bsp_led.h" 
#include "./usart/bsp_debug_usart.h"
#include "./sd_card/bsp_sdio_sd.h"
#include "./key/bsp_key.h" 
#include "./delay/core_delay.h"  
/* FatFs includes component */
#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include "./flash/bsp_qspi_flash.h"
#include "RES_MGR.h"
/**
  ******************************************************************************
  *                              �������
  ******************************************************************************
  */
char SDPath[4]; /* SD�߼�������·�� */
extern FATFS sd_fs;	
FRESULT res_sd;                /* �ļ�������� */
extern char src_dir[];

extern FATFS flash_fs;
extern Diskio_drvTypeDef  SD_Driver;
/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();

  //��Cache����write-through��ʽ
  //SCB->CACR|=1<<2;
}
/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
uint8_t state = QSPI_ERROR;
int main(void)
{
    /* ϵͳʱ�ӳ�ʼ����400MHz */
    SystemClock_Config();
  
    CPU_CACHE_Enable();
  
    LED_GPIO_Config();
    LED_BLUE;	
    /* ��ʼ��USART1 ����ģʽΪ 115200 8-N-1 */
    DEBUG_USART_Config();	
  
    Key_GPIO_Config();
    QSPI_FLASH_Init();
    QSPI_Set_WP_High();
    /*д״̬�Ĵ���*/
    /*��flash������һ���ò���*/
    QSPI_FLASH_WriteStatusReg(1,0X00);
    QSPI_FLASH_WriteStatusReg(2,0X00);
    //QSPI_FLASH_WriteStatusReg(3,0X60);
    printf("\r\nFlash Status Reg1 is 0x%02X,\r\n", QSPI_FLASH_ReadStatusReg(1));	
    printf("\r\nFlash Status Reg2 is 0x%02X,\r\n", QSPI_FLASH_ReadStatusReg(2));
    printf("\r\nFlash Status Reg3 is 0x%02X,\r\n", QSPI_FLASH_ReadStatusReg(3));
    QSPI_Set_WP_TO_QSPI_IO();  
  
    //�����������������̷�
    FATFS_LinkDriver(&SD_Driver, SDPath);
    //���ⲿSD�������ļ�ϵͳ���ļ�ϵͳ����ʱ���SD����ʼ��
    res_sd = f_mount(&sd_fs,"0:",1);  
  
    if(res_sd != FR_OK)
    {
      printf("f_mount ERROR!������������SD��Ȼ�����¸�λ������!");
      LED_RED;
      while(1);
    }    
      
    printf("\r\n ��һ��KEY1��ʼ��д�ֿⲢ�����ļ���FLASH�� \r\n"); 
    printf("\r\n ע��ò������FLASH��ԭ���ݻᱻɾ������ \r\n"); 

    while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0);
    while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1); 
    printf("\r\n ���ڽ�����Ƭ������ʱ��ܳ��������ĵȺ�...\r\n"); 
    
    BSP_QSPI_Erase_Chip();
    // for(int i = 0; i < 50; i++)
    // {
      
    //   state = BSP_QSPI_Erase_Block(i*4096);
    //   if(state != QSPI_OK)
    //     printf("����Blockʧ��\n");
    //   else
    //     printf("OK\n");
    // }
    
    /* ������¼Ŀ¼��Ϣ�ļ� */
    Make_Catalog(src_dir,0);

    /* ��¼ Ŀ¼��Ϣ��FLASH*/
    Burn_Catalog();
     /* ���� Ŀ¼ ��¼������FLASH*/
    Burn_Content();
    /* У����¼������ */
    Check_Resource();    
    /* ������ɣ�ͣ�� */
    while(1)
    {
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
