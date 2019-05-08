/**
  ******************************************************************************
  * @file    system_stm32h7xx.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-M Device Peripheral Access Layer System Source File.
  *
  *   This file provides two functions and one global variable to be called from
  *   user application:
  *      - SystemInit(): This function is called at startup just after reset and
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32h7xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick
  *                                  timer or configure other parameters.
  *
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32h7xx_system
  * @{
  */

/** @addtogroup STM32H7xx_System_Private_Includes
  * @{
  */

#include "stm32h7xx_hal.h"

/**
  * @}
  */

/** @addtogroup STM32H7xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32H7xx_System_Private_Defines
  * @{
  */

/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x00       /*!< Vector Table base offset field.
                                      This value must be a multiple of 0x200. */
/******************************************************************************/

/**
  * @}
  */

/** @addtogroup STM32H7xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32H7xx_System_Private_Variables
  * @{
  */
  /* This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */
  uint32_t SystemCoreClock = 64000000;
  uint32_t SystemD2Clock = 64000000;
  const  uint8_t D1CorePrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};
  void SystemInit_ExtMemCtl(void);
/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup STM32H7xx_System_Private_Functions
  * @{
  */

/**
  * @brief  Setup the microcontroller system
  *         Initialize the FPU setting, vector table location and External memory
  *         configuration.
  * @param  None
  * @retval None
  */
void SystemInit (void)
{
  /* FPU settings ------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  #endif
  /* Reset the RCC clock configuration to the default reset state ------------*/
  /* Set HSION bit */
  RCC->CR |= RCC_CR_HSION;

  /* Reset CFGR register */
  RCC->CFGR = 0x00000000;

  /* Reset HSEON, CSSON , CSION,RC48ON, CSIKERON PLL1ON, PLL2ON and PLL3ON bits */
  RCC->CR &= (uint32_t)0xEAF6ED7F;

  /* Reset D1CFGR register */
  RCC->D1CFGR = 0x00000000;

  /* Reset D2CFGR register */
  RCC->D2CFGR = 0x00000000;

  /* Reset D3CFGR register */
  RCC->D3CFGR = 0x00000000;

  /* Reset PLLCKSELR register */
  RCC->PLLCKSELR = 0x00000000;

  /* Reset PLLCFGR register */
  RCC->PLLCFGR = 0x00000000;
  /* Reset PLL1DIVR register */
  RCC->PLL1DIVR = 0x00000000;
  /* Reset PLL1FRACR register */
  RCC->PLL1FRACR = 0x00000000;

  /* Reset PLL2DIVR register */
  RCC->PLL2DIVR = 0x00000000;

  /* Reset PLL2FRACR register */

  RCC->PLL2FRACR = 0x00000000;
  /* Reset PLL3DIVR register */
  RCC->PLL3DIVR = 0x00000000;

  /* Reset PLL3FRACR register */
  RCC->PLL3FRACR = 0x00000000;

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Disable all interrupts */
  RCC->CIER = 0x00000000;

  /* Change  the switch matrix read issuing capability to 1 for the AXI SRAM target (Target 7) */
  *((__IO uint32_t*)0x51008108) = 0x00000001;
  
  
//	PWR->CR3&=~(1<<2);				
//	PWR->D3CR|=3<<14;				//VOS=3,Scale1,1.15~1.26V内核电压,FLASH访问可以得到最高性能
//	while((PWR->D3CR&(1<<13))==0);	//等待电压稳定 
//	RCC->CR|=1<<16;					//HSEON=1,开启HSE
//	while(((RCC->CR&(1<<17))==0));//等待HSE RDY 
//	
//  RCC->PLLCKSELR|=2<<0;		//PLLSRC[1:0]=2,选择HSE作为PLL的输入时钟源
//  RCC->PLLCKSELR|=5<<4;	//DIVM1[5:0]=pllm,设置PLL1的预分频系数
//  RCC->PLL1DIVR|=(160-1)<<0;	//DIVN1[8:0]=plln-1,设置PLL1的倍频系数,设置值需减1
//  RCC->PLL1DIVR|=(2-1)<<9;	//DIVP1[6:0]=pllp-1,设置PLL1的p分频系数,设置值需减1
//  RCC->PLL1DIVR|=(4-1)<<16;//DIVQ1[6:0]=pllq-1,设置PLL1的q分频系数,设置值需减1
//  RCC->PLL1DIVR|=(2-1)<<24;		//DIVR1[6:0]=pllr-1,设置PLL1的r分频系数,设置值需减1,r分频出来的时钟没用到
//  RCC->PLLCFGR|=2<<2;			//PLL1RGE[1:0]=2,PLL1输入时钟频率在4~8Mhz之间(25/5=5Mhz),如修改pllm,请确认此参数
//  RCC->PLLCFGR|=0<<1;			//PLL1VCOSEL=0,PLL1宽的VCO范围,192~836Mhz
//  RCC->PLLCFGR|=3<<16;		//DIVP1EN=1,DIVQ1EN=1,使能pll1_p_ck和pll1_q_ck
//  RCC->CR|=1<<24;				//PLL1ON=1,使能PLL1
//  while((RCC->CR&(1<<25))==0);//PLL1RDY=1?,等待PLL1准备好  

//  RCC->D1CFGR|=8<<0;			//HREF[3:0]=8,rcc_hclk1/2/3/4=sys_d1cpre_ck/2=400/2=200Mhz,即AHB1/2/3/4=200Mhz
//  RCC->D1CFGR|=0<<8;			//D1CPRE[2:0]=0,sys_d1cpre_ck=sys_clk/1=400/1=400Mhz,即CPU时钟=400Mhz
//  RCC->CFGR|=3<<0;			//SW[2:0]=3,系统时钟(sys_clk)选择来自pll1_p_ck,即400Mhz
//  while(((RCC->CFGR&(7<<3))>>3) != 3);
//    
//  FLASH->ACR|=2<<0;			//LATENCY[2:0]=2,2个CPU等待周期(@VOS1 Level,maxclock=210Mhz)
//  FLASH->ACR|=2<<4;			//WRHIGHFREQ[1:0]=2,flash访问频率<285Mhz
//  
//  RCC->D1CFGR|=4<<4;			//D1PPRE[2:0]=4,rcc_pclk3=rcc_hclk3/2=100Mhz,即APB3=100Mhz
//  RCC->D2CFGR|=4<<4;			//D2PPRE1[2:0]=4,rcc_pclk1=rcc_hclk1/2=100Mhz,即APB1=100Mhz
//  RCC->D2CFGR|=4<<8;			//D2PPRE2[2:0]=4,rcc_pclk2=rcc_hclk1/2=100Mhz,即APB2=100Mhz
//  RCC->D3CFGR|=4<<4;			//D3PPRE[2:0]=4,rcc_pclk4=rcc_hclk4/2=100Mhz,即APB4=100Mhz

//  RCC->CR|=1<<7;				//CSION=1,使能CSI,为IO补偿单元提供时钟
//  RCC->APB4ENR|=1<<1;			//SYSCFGEN=1,使能SYSCFG时钟
//  SYSCFG->CCCSR|=1<<0;		//EN=1,使能IO补偿单元
  
  SystemInit_ExtMemCtl();
  /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
  SCB->VTOR = D1_AXISRAM_BASE  | VECT_TAB_OFFSET;       /* Vector Table Relocation in Internal SRAM */
#else
  SCB->VTOR = FLASH_BANK1_BASE | VECT_TAB_OFFSET;       /* Vector Table Relocation in Internal FLASH */
#endif
}
__IO uint32_t index = 0; 
/**
  * @brief  延迟一段时间
  * @param  延迟的时间长度
  * @retval None
  */
static void SDRAM_delay(__IO uint32_t nCount)
{
  
  for(index = (100000 * nCount); index != 0; index--)
  {
  }
}

/**
  * @brief  Setup the external memory controller.
  *         Called in startup_stm32h7xx.s before jump to main.
  *         This function configures the external memories (SRAM/SDRAM)
  *         This SRAM/SDRAM will be used as program data memory (including heap and stack).
  * @param  None
  * @retval None
  */
void SystemInit_ExtMemCtl(void)
{
  //使能GPIO口时钟
  RCC->AHB4ENR |= 0x1FC; 

  //配置端口C、D、E、F、G、H、I相应的引脚为复用推挽输出
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
  
//  //使能PLL2R
//  RCC->PLLCFGR |= (1 << 21); 
//  //禁用PLL2
//  RCC->CR &= ~(1 << 26);
//  RCC->PLLCKSELR |= (5 << 12);
//  RCC->PLL2DIVR = 0x0201028F;
//  RCC->PLLCFGR |= (2<<6);
//  RCC->PLLCFGR &= ~(1<<5);
//  RCC->CR |= (1 << 26);
//  while(!(RCC->CR&(1<<27)));
  //RCC->D1CCIPR |= (1<<0); 

  //使能FMC时钟
  RCC->AHB3ENR |= (1 << 12); 
  FMC_Bank5_6->SDCR[0] = 0x00003AD0;  
  FMC_Bank5_6->SDCR[1] = 0x000001E9;  
  FMC_Bank5_6->SDTR[0] = 0x0F1F7FFF;  
  FMC_Bank5_6->SDTR[1] = 0x01010471;                                         
  FMC_Bank1->BTCR[0] |= 0x80000000;  
  
  
	/* Step 1 ----------------------------------------------------------------*/
	/* 配置命令：开启提供给SDRAM的时钟 */
   
  FMC_Bank5_6->SDCMR = 0x00000009;
  while((FMC_Bank5_6->SDSR&(1<<5)));
	/* Step 2: 延时100us */ 
	SDRAM_delay(1);

	/* Step 3 ----------------------------------------------------------------*/
	/* 配置命令：对所有的bank预充电 */ 

  FMC_Bank5_6->SDCMR = 0x0000000A;
  while((FMC_Bank5_6->SDSR&(1<<5)));

	/* Step 4 ----------------------------------------------------------------*/
	/* 配置命令：自动刷新 */   
  FMC_Bank5_6->SDCMR = 0x000000EB;
  while((FMC_Bank5_6->SDSR&(1<<5)));

	/* Step 5 ----------------------------------------------------------------*/
	/* 设置sdram寄存器配置 */
  FMC_Bank5_6->SDCMR =0x0004600C;
  while((FMC_Bank5_6->SDSR&(1<<5)));
	/* Step 6 ----------------------------------------------------------------*/
  /* 设置自刷新速率 */
  FMC_Bank5_6->SDRTR |= (1855<<1);  
  
}
/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
