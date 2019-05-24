/**
  ******************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   ʹ�ð��������л�ϵͳʱ��Դ
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
#include "./led/bsp_led.h"
#include "./rcc/bsp_clkconfig.h"
#include "./key/bsp_key.h" 
#include "./delay/core_delay.h" 
/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{
	/* ϵͳʱ�ӳ�ʼ����400MHz */
	SystemClock_Config();
	// LED �˿ڳ�ʼ�� 
	LED_GPIO_Config();	
	/*��ʼ������*/
	Key_GPIO_Config();		
  /* ��MCO2���ţ�PC.09�������SYSCLK / 4 */
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_4);

	while (1)
	{
		/* ����Ƿ�����KEY2��ť���л�ʱ������ */
    if( Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_ON  )
		{
      
      //�л���ͬ��ʱ��Դ,ϵͳʱ��Ϊ400MHz��
			SwitchSystemClock();

		} 		

    /* LED��˸ */
    LED3_TOGGLE;
    HAL_Delay(100);
	}
}

/****************************END OF FILE***************************/
