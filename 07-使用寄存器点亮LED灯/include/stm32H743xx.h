

/*Ƭ���������ַ  */
#define PERIPH_BASE           ((unsigned int)0x40000000)                          

/*���߻���ַ */
#define D3_AHB1PERIPH_BASE    (PERIPH_BASE + 0x18020000)	

/*GPIO�������ַ*/
#define GPIOB_BASE            (D3_AHB1PERIPH_BASE + 0x0400)


/* GPIOB�Ĵ�����ַ,ǿ��ת����ָ�� */
#define GPIOB_MODER				*(unsigned int*)(GPIOB_BASE+0x00)
#define GPIOB_OTYPER			*(unsigned int*)(GPIOB_BASE+0x04)
#define GPIOB_OSPEEDR			*(unsigned int*)(GPIOB_BASE+0x08)
#define GPIOB_PUPDR				*(unsigned int*)(GPIOB_BASE+0x0C)
#define GPIOB_IDR					*(unsigned int*)(GPIOB_BASE+0x10)
#define GPIOB_ODR					*(unsigned int*)(GPIOB_BASE+0x14)
#define GPIOB_BSRRL				*(unsigned int*)(GPIOB_BASE+0x18)
#define GPIOB_BSRRH				*(unsigned int*)(GPIOB_BASE+0x1A)
#define GPIOB_LCKR				*(unsigned int*)(GPIOB_BASE+0x1C)
#define GPIOB_AFRL				*(unsigned int*)(GPIOB_BASE+0x20)
#define GPIOB_AFRB				*(unsigned int*)(GPIOB_BASE+0x24)

/*RCC�������ַ*/
#define RCC_BASE          (D3_AHB1PERIPH_BASE + 0x4400)

/*RCC��AHB1ʱ��ʹ�ܼĴ�����ַ,ǿ��ת����ָ��*/
#define RCC_AHB4ENR				*(unsigned int*)(RCC_BASE+0xE0)

	



