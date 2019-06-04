
#include "stm32h7xx_hal_gpio.h"


/**
  * �������ܣ��������ŵ�ƽ
  * ����˵����GPIOx: �ò���ΪGPIO_TypeDef���͵�ָ�룬ָ��GPIO�˿ڵĵ�ַ
  *			  GPIO_Pin: ѡ��Ҫ���õ�GPIO�˿����ţ��������GPIO_Pin_0-15��
  *					    ��ʾGPIOx�˿ڵ�0-15������
  * 			PinState: ������ѡ���ŵĵ�ƽ
  *            @arg GPIO_PIN_RESET:���õ͵�ƽ
  *            @arg GPIO_PIN_SET: ���øߵ�ƽ
  * ����ֵ�� ��
  */
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
	/*����GPIOx�˿�BSRR�Ĵ����ĵ�16λ��Ӧ��GPIO_Pinλ��ʹ������ߵ�ƽ*/
	/*����GPIOx�˿�BSRR�Ĵ����ĸ�16λ��Ӧ��GPIO_Pinλ��ʹ������͵�ƽ*/
	/*��ΪBSRR�Ĵ���д0��Ӱ�죬GPIO_Pinֻ�Ƕ�ӦλΪ1������λ��Ϊ0�����Կ���ֱ�Ӹ�ֵ*/

  if(PinState != GPIO_PIN_RESET)
  {
    GPIOx->BSRRL = GPIO_Pin;
  }
  else
  {
    GPIOx->BSRRH = GPIO_Pin ;
  }
}

/**
  *�������ܣ���ʼ������ģʽ
  *����˵����GPIOx���ò���ΪGPIO_TypeDef���͵�ָ�룬ָ��GPIO�˿ڵĵ�ַ
  * 			  GPIO_InitTypeDef:GPIO_InitTypeDef�ṹ��ָ�룬ָ���ʼ������
  */
void HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
  uint32_t position = 0x00;
  uint32_t ioposition = 0x00;
  uint32_t iocurrent = 0x00;
  uint32_t temp = 0x00;

  /* Configure the port pins */
  for(position = 0; position < 16; position++)
  {
	/*����������Ϊ��ͨ�� GPIO_InitStruct->GPIO_Pin ������ź�0-15*/
	/*���������pos��pinposλΪ1������Ϊ0����GPIO_Pin_x���Ӧ��pinpos����ÿ��ѭ����1��*/
    ioposition = ((uint32_t)0x01) << position;
    /* pos��GPIO_InitStruct->Pin�� & ���㣬��������currentpin == pos��
	���ʾGPIO_InitStruct->Pin��pinposλҲΪ1��
	�Ӷ���֪pinpos����GPIO_InitStruct->Pin��Ӧ�����źţ�0-15*/
    iocurrent = (uint32_t)(GPIO_Init->Pin) & ioposition;

    if(iocurrent == ioposition)
    {
      /*--------------------- GPIO Mode Configuration ------------------------*/
      /* �ڸ��ù���ģʽѡ�������� */
      if((GPIO_Init->Mode == GPIO_MODE_AF_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_OD))
      {
        /* �����뵱ǰIOӳ��ı��ù��� */
        temp = GPIOx->AFR[position >> 3];
        temp &= ~((uint32_t)0xF << ((uint32_t)(position & (uint32_t)0x07) * 4)) ;
        temp |= ((uint32_t)(GPIO_Init->Alternate) << (((uint32_t)position & (uint32_t)0x07) * 4));
        GPIOx->AFR[position >> 3] = temp;
      }

      /* ����IO����ģʽ�����룬��������û�ģ�⣩ */
      temp = GPIOx->MODER;
      temp &= ~(GPIO_MODER_MODER0 << (position * 2));
      temp |= ((GPIO_Init->Mode & GPIO_MODE) << (position * 2));
      GPIOx->MODER = temp;

      /* ��������ù���ģʽѡ�������� */
      if((GPIO_Init->Mode == GPIO_MODE_OUTPUT_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_PP) ||
         (GPIO_Init->Mode == GPIO_MODE_OUTPUT_OD) || (GPIO_Init->Mode == GPIO_MODE_AF_OD))
      {

        /* �����ٶȲ��� */
        temp = GPIOx->OSPEEDR; 
        temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (position * 2));
        temp |= (GPIO_Init->Speed << (position * 2));
        GPIOx->OSPEEDR = temp;

        /* ����IO������� */
        temp = GPIOx->OTYPER;
        temp &= ~(GPIO_OTYPER_OT_0 << position) ;
        temp |= (((GPIO_Init->Mode & GPIO_OUTPUT_TYPE) >> 4) << position);
        GPIOx->OTYPER = temp;
      }

      /* ���ǰIO���������������� */
      temp = GPIOx->PUPDR;
      temp &= ~(GPIO_PUPDR_PUPDR0 << (position * 2));
      temp |= ((GPIO_Init->Pull) << (position * 2));
      GPIOx->PUPDR = temp;
    }
  }
}

/*********************************************END OF FILE**********************/

