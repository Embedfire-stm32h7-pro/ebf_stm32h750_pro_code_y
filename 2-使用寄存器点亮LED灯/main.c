
/*
	ʹ�üĴ����ķ�������LED��
  */
#include  "stm32h743xx.h" 


/**
  *   ������
  */
int main(void)
{	
	/*���� GPIOB ʱ�ӣ�ʹ������ʱ��Ҫ�ȿ�������ʱ��*/
	RCC_AHB4ENR |= (1<<1);	
	
	/* LED �˿ڳ�ʼ�� */
	
	/*GPIOB MODER0���*/
	GPIOB_MODER  &= ~0x03;	
	/*PH10 MODER10 = 01b ���ģʽ*/
	GPIOB_MODER |= 1;
	
	/*GPIOB OTYPER0���*/
	GPIOB_OTYPER &= ~1;
	/*PB0 OTYPER0 = 0b ����ģʽ*/
	GPIOB_OTYPER |= 0;
	
	/*���GPIOB OSPEEDR0*/
	GPIOB_OSPEEDR &= ~0x03;
	/*PB0 OSPEEDR0 = 0b ����ģʽ*/
	GPIOB_OSPEEDR |= 0;
	
	/*���GPIOB PUPDR0*/
	GPIOB_PUPDR &= ~0x03;
	/*PB0 PUPDR0 = 01b ����ģʽ*/
	GPIOB_PUPDR |= 1;
	
	/*PB0 BSRR�Ĵ����� BR0��1��ʹ��������͵�ƽ*/
	GPIOB_BSRRH |= 1;
	
	/*PH10 BSRR�Ĵ����� BS10��1��ʹ��������ߵ�ƽ*/
	//GPIOH_BSRRL |= (1<<10);

	while(1);

}

// ����Ϊ�գ�Ŀ����Ϊ��ƭ��������������
void SystemInit(void)
{	
}






/*********************************************END OF FILE**********************/

