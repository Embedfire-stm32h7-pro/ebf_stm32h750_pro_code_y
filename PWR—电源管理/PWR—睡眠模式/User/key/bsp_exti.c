/**
  ******************************************************************************
  * @file    bsp_exti.c
  * @author  fire
  * @version V1.0
  * @date    2017-xx-xx
  * @brief   I/O���ж�Ӧ��bsp
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� STM32F767 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./key/bsp_exti.h"
#include "./led/bsp_led.h" 

 /**
  * @brief  ����Ƕ�������жϿ�����NVIC
  * @param  ��
  * @retval ��
  */
static void NVIC_Configuration(void)
{
	/* ����NVICΪ���ȼ���1 */
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_1);	
	/*����1�ж����ã���ռ���ȼ�0�������ȼ�Ϊ0*/
	HAL_NVIC_SetPriority(KEY1_INT_EXTI_IRQ, 1 ,1);
	HAL_NVIC_EnableIRQ(KEY1_INT_EXTI_IRQ);	
	/*����2�ж����ã���ռ���ȼ�0�������ȼ�Ϊ0*/
	HAL_NVIC_SetPriority(KEY2_INT_EXTI_IRQ, 1 ,1);
	HAL_NVIC_EnableIRQ(KEY2_INT_EXTI_IRQ);
}

 /**
  * @brief  ���� KEY1��KEY2Ϊ�жϣ��������ж����ȼ�
  * @param  ��
  * @retval ��
  */
void EXTI_Key_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
  
	/*��������GPIO�ڵ�ʱ��*/
	KEY1_INT_GPIO_CLK_ENABLE();
	KEY2_INT_GPIO_CLK_ENABLE();
   
	/* ���� NVIC */
	NVIC_Configuration();  
	/*ѡ�񰴼�������*/	
	GPIO_InitStructure.Pin = KEY1_INT_GPIO_PIN; 
	/*��������Ϊ�������ж�ģʽ*/
	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING; 
	/*�������Ų�����Ҳ������*/
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	/*ʹ������Ľṹ���ʼ������*/
	HAL_GPIO_Init(KEY1_INT_GPIO_PORT, &GPIO_InitStructure);
	HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA0, SYSCFG_SWITCH_PA0_CLOSE);

	/*ѡ�񰴼�2������*/	
	GPIO_InitStructure.Pin = KEY2_INT_GPIO_PIN; 
	/*��������Ϊ�������ж�ģʽ*/
	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING; 
	/*�������Ų�����Ҳ������*/
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	/*ʹ������Ľṹ���ʼ������*/
	HAL_GPIO_Init(KEY2_INT_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	LED_BLUE;
	if(GPIO_Pin==KEY1_INT_GPIO_PIN)
		printf("\r\n KEY1 �����жϻ��� \r\n");
	else if(GPIO_Pin==KEY2_INT_GPIO_PIN)
		printf("\r\n KEY2 �����жϻ��� \r\n"); 
	else
		{}
}

/*********************************************END OF FILE**********************/
