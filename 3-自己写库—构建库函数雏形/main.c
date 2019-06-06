
/*
	ʹ�üĴ����ķ�������LED��
  */
#include "stm32h7xx_hal_gpio.h"

void Delay( uint32_t nCount);
/**
  *   ������
  */
int main(void)
{	
	GPIO_InitTypeDef InitStruct;
	
	/*���� GPIOB ʱ�ӣ�ʹ������ʱ��Ҫ�ȿ�������ʱ��*/
	RCC->AHB4ENR |= (1<<1);

	/* LED �˿ڳ�ʼ�� */
	
	/*��ʼ��PB0����*/
	/*ѡ��Ҫ���Ƶ�GPIO����*/															   
	InitStruct.Pin = GPIO_PIN_0;
	/*�������ŵ��������Ϊ�������*/
	InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	/*��������Ϊ����ģʽ*/
	InitStruct.Pull = GPIO_PULLUP;
	/*������������Ϊ����ģʽ */   
	InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	/*���ÿ⺯����ʹ���������õ�GPIO_InitStructure��ʼ��GPIO*/
	HAL_GPIO_Init(GPIOB, &InitStruct);	

	/*ʹ��������͵�ƽ,����LED1*/
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);

	/*��ʱһ��ʱ��*/
	Delay(0xFFFFFF);	
	
	/*ʹ��������ߵ�ƽ���ر�LED1*/
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
	
	/*��ʼ��PB1����*/
	InitStruct.Pin = GPIO_PIN_1;
	HAL_GPIO_Init(GPIOB,&InitStruct);
	
	/*ʹ��������͵�ƽ������LED2*/
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);

	while(1);

}

//�򵥵���ʱ��������cpuִ��������ָ�����ʱ��
//������ʱʱ�����Լ��㣬�Ժ����ǿ�ʹ�ö�ʱ����ȷ��ʱ
void Delay( uint32_t nCount)	 
{
	for(; nCount != 0; nCount--);
}
// ����Ϊ�գ�Ŀ����Ϊ��ƭ��������������
void SystemInit(void)
{	
}






/*********************************************END OF FILE**********************/

