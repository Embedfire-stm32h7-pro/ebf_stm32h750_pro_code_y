/**
  ******************************************************************************
  * @file    bsp_wwdg.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   ���ڿ��Ź�����
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� STM32H743 ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./wwdg/bsp_wwdg.h"   
#include "./led/bsp_led.h"

WWDG_HandleTypeDef WWDG_Handle;

// WWDG �ж����ȼ���ʼ��
static void WWDG_NVIC_Config(void)
{
	HAL_NVIC_SetPriority(WWDG_IRQn,0,0);
	HAL_NVIC_EnableIRQ(WWDG_IRQn);
}

/* WWDG ���ú���
 * tr ���ݼ���ʱ����ֵ�� ȡֵ��ΧΪ��0x7f~0x40��������Χ��ֱ�Ӹ�λ
 * wr ������ֵ��ȡֵ��ΧΪ��0x7f~0x40
 * prv��Ԥ��Ƶ��ֵ��ȡֵ������
 * @arg WWDG_PRESCALER_1: WWDG counter clock = (PCLK1(54MHz)/4096)/1   
	 Լ13184 76us
 * @arg WWDG_PRESCALER_2: WWDG counter clock = (PCLK1(54MHz)/4096)/2	 
	 Լ6592Hz 152us
 * @arg WWDG_PRESCALER_4: WWDG counter clock = (PCLK1(54MHz)/4096)/4	 
	 Լ3296Hz 304us
 * @arg WWDG_PRESCALER_8: WWDG counter clock = (PCLK1(54MHz)/4096)/8   
   Լ1648Hz	608us
 *      
 * ����tr = 127(0x7f��tr�����ֵ)  
 *     wr = 80��0x50, 0x40Ϊ��Сwr��Сֵ��  
 *	  prv = WWDG_PRESCALER_8
 * ����ʱ��Ϊ608 * (127-80) = 28.6ms < ˢ�´��� < ~608 * 64 = 38.9ms
 * Ҳ����˵����WWDG_Config�������������ã�����֮���28.6msǰι����
 * ϵͳ�Ḵλ����38.9ms��û��ι����ϵͳҲ�Ḵλ��
 * ��Ҫ��ˢ�´��ڵ�ʱ����ι����ϵͳ�Ų��Ḵλ��	
*/
void WWDG_Config(uint8_t tr, uint8_t wr, uint32_t prv)
{		
	// ���� WWDG ʱ��
	__HAL_RCC_WWDG1_CLK_ENABLE();
	// ����WWDG�ж����ȼ�
	WWDG_NVIC_Config();
	// ����WWDG������Ĵ�������ַ
	WWDG_Handle.Instance = WWDG1;
	// ����Ԥ��Ƶ��ֵ
	WWDG_Handle.Init.Prescaler = prv;
	// �����ϴ���ֵ
	WWDG_Handle.Init.Window = wr;	
	// ���ü�������ֵ
	WWDG_Handle.Init.Counter = tr;
	// ʹ����ǰ�����ж�
	WWDG_Handle.Init.EWIMode = WWDG_EWI_ENABLE;
	// ��ʼ��WWDG
	HAL_WWDG_Init(&WWDG_Handle);	
}

// ι��
void WWDG_Feed(void)
{
	// ι����ˢ�µݼ���������ֵ�����ó����WDG_CNT=0X7F
	HAL_WWDG_Refresh(&WWDG_Handle);
}

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef* hwwdg)
{
	//�Ƶ���������LEDֻ��ʾ���ԵĲ�����
	//����ʹ�õ�ʱ������Ӧ����������Ҫ������
	LED_YELLOW; 
}
/*********************************************END OF FILE**********************/
