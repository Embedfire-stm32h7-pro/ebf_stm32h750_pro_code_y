
/*Ƭ���������ַ  */

#define PERIPH_BASE           ((unsigned int)0x40000000)                          

/*���߻���ַ */

#define D3_AHB1PERIPH_BASE    (PERIPH_BASE + 0x18020000)	

/*RCC�������ַ*/

#define RCC_BASE          (D3_AHB1PERIPH_BASE + 0x4400)


	


//�Ĵ�����ֵ������оƬ�����Զ����ĵģ���ʹCPUû��ִ�г���Ҳ�п��ܷ����仯

//�������п��ܻ��û��ִ�г���ı��������Ż�



//volatile��ʾ�ױ�ı�������ֹ�������Ż���

#define     __IO    volatile

typedef unsigned int uint32_t;

typedef unsigned short uint16_t;

typedef unsigned char  uint8_t;

/* GPIO�Ĵ����б� */
typedef struct

{
	__IO	uint32_t MODER;    /*GPIOģʽ�Ĵ���					��ַƫ��: 0x00      */

	__IO	uint32_t OTYPER;   /*GPIO������ͼĴ���				��ַƫ��: 0x04      */

	__IO	uint32_t OSPEEDR;  /*GPIO����ٶȼĴ���				��ַƫ��: 0x08      */

	__IO	uint32_t PUPDR;    /*GPIO����/�����Ĵ���				��ַƫ��: 0x0C      		*/

	__IO	uint32_t IDR;      /*GPIO�������ݼĴ���				��ַƫ��: 0x10      		*/

	__IO	uint32_t ODR;      /*GPIO������ݼĴ���				��ַƫ��: 0x14      		*/

	__IO	uint16_t BSRRL;    /*GPIO��λ/��λ�Ĵ�������16λ�� 				��ַƫ��: 0x18 	*/
	
	__IO	uint16_t BSRRH;    /*GPIO��λ/��λ�Ĵ�������16λ�� 				��ַƫ��: 0x1A 	*/

	__IO	uint32_t LCKR;     /*GPIO���������Ĵ���				��ַƫ��: 0x1C      */

	__IO	uint32_t AFR[2];   /*GPIO���ù������üĴ���			��ַƫ��: 0x20-0x24 		*/

} GPIO_TypeDef;
/*RCC�Ĵ����б�*/

typedef struct

{
	__IO uint32_t CR;             /*!< RCCʱ�ӿ��ƼĴ�������ַƫ������0x00  */
	__IO uint32_t ICSCR;          /*!< RCC�ڲ�ʱ��ԴУ׼�Ĵ�������ַƫ������0x04  */
	__IO uint32_t CRRCR;          /*!< ʱ�ӻָ�RC�Ĵ�������ַƫ������0x08  */
	uint32_t     RESERVED0;       /*!< ��������ַƫ�ƣ�0x0C  */
	__IO uint32_t CFGR;           /*!< RCCʱ�����üĴ�������ַƫ������0x10  */
	uint32_t     RESERVED1;       /*!< ��������ַƫ������0x14  */
	__IO uint32_t D1CFGR;         /*!< RCC��1���üĴ�������ַƫ������0x18 */
	__IO uint32_t D2CFGR;         /*!< RCC��2���üĴ�������ַƫ������0x1C */
	__IO uint32_t D3CFGR;         /*!< RCC��3���üĴ�������ַƫ������0x20  */
	uint32_t     RESERVED2;       /*!< ��������ַƫ������0x24  */
	__IO uint32_t PLLCKSELR;      /*!< RCC PLLʱ��Դѡ��Ĵ�������ַƫ������0x28  */
	__IO uint32_t PLLCFGR;        /*!< RCC PLL���üĴ�������ַƫ������0x2C  */
	__IO uint32_t PLL1DIVR;       /*!< RCC PLL1��Ƶ�����üĴ�������ַƫ�ƣ�0x30  */
	__IO uint32_t PLL1FRACR;      /*!< RCC PLL1С����Ƶ�����üĴ�������ַƫ������0x34  */
	__IO uint32_t PLL2DIVR;       /*!< RCC PLL2��Ƶ�����üĴ�������ַƫ�ƣ�0x38  */
	__IO uint32_t PLL2FRACR;      /*!< RCC PLL2С����Ƶ�����üĴ�������ַƫ������0x3C  */
	__IO uint32_t PLL3DIVR;       /*!< RCC PLL3��Ƶ�����üĴ�������ַƫ������0x40  */
	__IO uint32_t PLL3FRACR;      /*!< RCC PLL3С����Ƶ�����üĴ�������ַƫ������0x44  */
	uint32_t      RESERVED3;      /*!< ��������ַƫ������0x48  */
	__IO uint32_t  D1CCIPR;       /*!< RCC��1�ں�ʱ�����üĴ�����ַƫ������0x4C  */
	__IO uint32_t  D2CCIP1R;      /*!< RCC��2�ں�ʱ�����üĴ�����ַƫ������0x50  */
	__IO uint32_t  D2CCIP2R;      /*!< RCC��2�ں�ʱ�����üĴ�����ַƫ������0x54  */
	__IO uint32_t  D3CCIPR;       /*!< RCC��3�ں�ʱ�����üĴ�����ַƫ������0x58  */
	uint32_t      RESERVED4;      /*!< ��������ַƫ�ƣ�0x5C  */
	__IO uint32_t  CIER;          /*!< RCCʱ��Դ�ж�ʹ�ܼĴ�����ַƫ������0x60  */
	__IO uint32_t  CIFR;          /*!< RCCʱ��Դ�жϱ�־�Ĵ�����ַƫ������0x64  */
	__IO uint32_t  CICR;          /*!< RCCʱ��Դ�ж�����Ĵ�����ַƫ������0x68  */
	uint32_t     RESERVED5;       /*!< ��������ַƫ�ƣ�0x6C  */
	__IO uint32_t  BDCR;          /*!< RCC Vswitch��������ƼĴ�������ַƫ������0x70  */
	__IO uint32_t  CSR;           /*!< RCCʱ�ӿ��ƺ�״̬�Ĵ�������ַƫ�ƣ�0x74  */
	uint32_t     RESERVED6;       /*!< ��������ַƫ������0x78  */
	__IO uint32_t AHB3RSTR;       /*!< RCC AHB3���踴λ�Ĵ�������ַƫ�ƣ�0x7C  */
	__IO uint32_t AHB1RSTR;       /*!< RCC AHB1���踴λ�Ĵ�������ַƫ������0x80  */
	__IO uint32_t AHB2RSTR;       /*!< RCC AHB2���踴λ�Ĵ�������ַƫ������0x84  */
	__IO uint32_t AHB4RSTR;       /*!< RCC AHB4���踴λ�Ĵ�������ַƫ������0x88  */
	__IO uint32_t APB3RSTR;       /*!< RCC APB3���踴λ�Ĵ�������ַƫ������0x8C  */
	__IO uint32_t APB1LRSTR;      /*!< RCC APB1���踴λ�Ĵ�����λ����ַƫ�ƣ�0x90  */
	__IO uint32_t APB1HRSTR;      /*!< RCC APB1���踴λ�Ĵ�����λ����ַƫ�ƣ�0x94  */
	__IO uint32_t APB2RSTR;       /*!< RCC APB2���踴λ�Ĵ�������ַƫ������0x98  */
	__IO uint32_t APB4RSTR;       /*!< RCC APB4���踴λ�Ĵ�������ַƫ�ƣ�0x9C  */
	__IO uint32_t GCR;            /*!< RCC RCCȫ�ֿ��ƼĴ�������ַƫ������0xA0  */
	uint32_t     RESERVED7;       /*!< ��������ַƫ������0xA4  */ 
	__IO uint32_t D3AMR;          /*!< RCC��3����ģʽ�Ĵ�������ַƫ������0xA8  */
	uint32_t     RESERVED8[9];    /*!< ������0xAC-0xCC ��ַƫ������0xAC  */
	__IO uint32_t RSR;            /*!< RCC��λ״̬�Ĵ�������ַƫ�ƣ�0xD0  */
	__IO uint32_t AHB3ENR;        /*!< RCC AHB3����ʱ�ӼĴ�������ַƫ������0xD4  */
	__IO uint32_t AHB1ENR;        /*!< RCC AHB1����ʱ�ӼĴ�������ַƫ������0xD8  */
	__IO uint32_t AHB2ENR;        /*!< RCC AHB2����ʱ�ӼĴ�������ַƫ������0xDC  */
	__IO uint32_t AHB4ENR;        /*!< RCC AHB4����ʱ�ӼĴ�������ַƫ������0xE0  */
	__IO uint32_t APB3ENR;        /*!< RCC APB3����ʱ�ӼĴ�������ַƫ������0xE4  */
	__IO uint32_t APB1LENR;       /*!< RCC APB1����ʱ�ӵ��ּĴ�������ַƫ�ƣ�0xE8  */
	__IO uint32_t APB1HENR;       /*!< RCC APB1����ʱ�Ӹ��ּĴ�������ַƫ�ƣ�0xEC  */
	__IO uint32_t APB2ENR;        /*!< RCC APB2����ʱ�ӼĴ�������ַƫ������0xF0 */
	__IO uint32_t APB4ENR;        /*!< RCC APB4����ʱ�ӼĴ�������ַƫ������0xF4  */
	uint32_t      RESERVED9;      /*!< ��������ַƫ������0xF8  */
	__IO uint32_t AHB3LPENR;      /*!< RCC AHB3����˯��ʱ�ӼĴ�������ַƫ������0xFC  */
	__IO uint32_t AHB1LPENR;      /*!< RCC AHB1����˯��ʱ�ӼĴ�������ַƫ������0x100 */
	__IO uint32_t AHB2LPENR;      /*!< RCC AHB2����˯��ʱ�ӼĴ�������ַƫ������0x104 */
	__IO uint32_t AHB4LPENR;      /*!< RCC AHB4����˯��ʱ�ӼĴ�������ַƫ������0x108 */
	__IO uint32_t APB3LPENR;      /*!< RCC APB3����˯��ʱ�ӼĴ�������ַƫ������0x10C */
	__IO uint32_t APB1LLPENR;     /*!< RCC APB1����˯��ʱ�ӵ�λ�ּĴ�������ַƫ������0x110 */
	__IO uint32_t APB1HLPENR;     /*!< RCC APB1��������ʱ�Ӹ��ּĴ�������ַƫ������0x114 */
	__IO uint32_t APB2LPENR;      /*!< RCC APB2����˯��ʱ�ӼĴ�������ַƫ������0x118 */
	__IO uint32_t APB4LPENR;      /*!< RCC APB4����˯��ʱ�ӼĴ�������ַƫ������0x11C */
	uint32_t     RESERVED10[4];   /*!< ������0x120-0x12C��ַƫ������0x120 */
} RCC_TypeDef;
/******************************************************************************/
/*                                                                            */
/*                            General Purpose I/O                             */
/*                                                                            */
/******************************************************************************/
/******************  Bits definition for GPIO_MODER register  *****************/
#define GPIO_MODER_MODER0                    0x00000003U
#define GPIO_MODER_MODER0_0                  0x00000001U
#define GPIO_MODER_MODER0_1                  0x00000002U
#define GPIO_MODER_MODER1                    0x0000000CU
#define GPIO_MODER_MODER1_0                  0x00000004U
#define GPIO_MODER_MODER1_1                  0x00000008U
#define GPIO_MODER_MODER2                    0x00000030U
#define GPIO_MODER_MODER2_0                  0x00000010U
#define GPIO_MODER_MODER2_1                  0x00000020U
#define GPIO_MODER_MODER3                    0x000000C0U
#define GPIO_MODER_MODER3_0                  0x00000040U
#define GPIO_MODER_MODER3_1                  0x00000080U
#define GPIO_MODER_MODER4                    0x00000300U
#define GPIO_MODER_MODER4_0                  0x00000100U
#define GPIO_MODER_MODER4_1                  0x00000200U
#define GPIO_MODER_MODER5                    0x00000C00U
#define GPIO_MODER_MODER5_0                  0x00000400U
#define GPIO_MODER_MODER5_1                  0x00000800U
#define GPIO_MODER_MODER6                    0x00003000U
#define GPIO_MODER_MODER6_0                  0x00001000U
#define GPIO_MODER_MODER6_1                  0x00002000U
#define GPIO_MODER_MODER7                    0x0000C000U
#define GPIO_MODER_MODER7_0                  0x00004000U
#define GPIO_MODER_MODER7_1                  0x00008000U
#define GPIO_MODER_MODER8                    0x00030000U
#define GPIO_MODER_MODER8_0                  0x00010000U
#define GPIO_MODER_MODER8_1                  0x00020000U
#define GPIO_MODER_MODER9                    0x000C0000U
#define GPIO_MODER_MODER9_0                  0x00040000U
#define GPIO_MODER_MODER9_1                  0x00080000U
#define GPIO_MODER_MODER10                   0x00300000U
#define GPIO_MODER_MODER10_0                 0x00100000U
#define GPIO_MODER_MODER10_1                 0x00200000U
#define GPIO_MODER_MODER11                   0x00C00000U
#define GPIO_MODER_MODER11_0                 0x00400000U
#define GPIO_MODER_MODER11_1                 0x00800000U
#define GPIO_MODER_MODER12                   0x03000000U
#define GPIO_MODER_MODER12_0                 0x01000000U
#define GPIO_MODER_MODER12_1                 0x02000000U
#define GPIO_MODER_MODER13                   0x0C000000U
#define GPIO_MODER_MODER13_0                 0x04000000U
#define GPIO_MODER_MODER13_1                 0x08000000U
#define GPIO_MODER_MODER14                   0x30000000U
#define GPIO_MODER_MODER14_0                 0x10000000U
#define GPIO_MODER_MODER14_1                 0x20000000U
#define GPIO_MODER_MODER15                   0xC0000000U
#define GPIO_MODER_MODER15_0                 0x40000000U
#define GPIO_MODER_MODER15_1                 0x80000000U

/******************  Bits definition for GPIO_OTYPER register  ****************/
#define GPIO_OTYPER_OT_0                     0x00000001U
#define GPIO_OTYPER_OT_1                     0x00000002U
#define GPIO_OTYPER_OT_2                     0x00000004U
#define GPIO_OTYPER_OT_3                     0x00000008U
#define GPIO_OTYPER_OT_4                     0x00000010U
#define GPIO_OTYPER_OT_5                     0x00000020U
#define GPIO_OTYPER_OT_6                     0x00000040U
#define GPIO_OTYPER_OT_7                     0x00000080U
#define GPIO_OTYPER_OT_8                     0x00000100U
#define GPIO_OTYPER_OT_9                     0x00000200U
#define GPIO_OTYPER_OT_10                    0x00000400U
#define GPIO_OTYPER_OT_11                    0x00000800U
#define GPIO_OTYPER_OT_12                    0x00001000U
#define GPIO_OTYPER_OT_13                    0x00002000U
#define GPIO_OTYPER_OT_14                    0x00004000U
#define GPIO_OTYPER_OT_15                    0x00008000U

/******************  Bits definition for GPIO_OSPEEDR register  ***************/
#define GPIO_OSPEEDER_OSPEEDR0               0x00000003U
#define GPIO_OSPEEDER_OSPEEDR0_0             0x00000001U
#define GPIO_OSPEEDER_OSPEEDR0_1             0x00000002U
#define GPIO_OSPEEDER_OSPEEDR1               0x0000000CU
#define GPIO_OSPEEDER_OSPEEDR1_0             0x00000004U
#define GPIO_OSPEEDER_OSPEEDR1_1             0x00000008U
#define GPIO_OSPEEDER_OSPEEDR2               0x00000030U
#define GPIO_OSPEEDER_OSPEEDR2_0             0x00000010U
#define GPIO_OSPEEDER_OSPEEDR2_1             0x00000020U
#define GPIO_OSPEEDER_OSPEEDR3               0x000000C0U
#define GPIO_OSPEEDER_OSPEEDR3_0             0x00000040U
#define GPIO_OSPEEDER_OSPEEDR3_1             0x00000080U
#define GPIO_OSPEEDER_OSPEEDR4               0x00000300U
#define GPIO_OSPEEDER_OSPEEDR4_0             0x00000100U
#define GPIO_OSPEEDER_OSPEEDR4_1             0x00000200U
#define GPIO_OSPEEDER_OSPEEDR5               0x00000C00U
#define GPIO_OSPEEDER_OSPEEDR5_0             0x00000400U
#define GPIO_OSPEEDER_OSPEEDR5_1             0x00000800U
#define GPIO_OSPEEDER_OSPEEDR6               0x00003000U
#define GPIO_OSPEEDER_OSPEEDR6_0             0x00001000U
#define GPIO_OSPEEDER_OSPEEDR6_1             0x00002000U
#define GPIO_OSPEEDER_OSPEEDR7               0x0000C000U
#define GPIO_OSPEEDER_OSPEEDR7_0             0x00004000U
#define GPIO_OSPEEDER_OSPEEDR7_1             0x00008000U
#define GPIO_OSPEEDER_OSPEEDR8               0x00030000U
#define GPIO_OSPEEDER_OSPEEDR8_0             0x00010000U
#define GPIO_OSPEEDER_OSPEEDR8_1             0x00020000U
#define GPIO_OSPEEDER_OSPEEDR9               0x000C0000U
#define GPIO_OSPEEDER_OSPEEDR9_0             0x00040000U
#define GPIO_OSPEEDER_OSPEEDR9_1             0x00080000U
#define GPIO_OSPEEDER_OSPEEDR10              0x00300000U
#define GPIO_OSPEEDER_OSPEEDR10_0            0x00100000U
#define GPIO_OSPEEDER_OSPEEDR10_1            0x00200000U
#define GPIO_OSPEEDER_OSPEEDR11              0x00C00000U
#define GPIO_OSPEEDER_OSPEEDR11_0            0x00400000U
#define GPIO_OSPEEDER_OSPEEDR11_1            0x00800000U
#define GPIO_OSPEEDER_OSPEEDR12              0x03000000U
#define GPIO_OSPEEDER_OSPEEDR12_0            0x01000000U
#define GPIO_OSPEEDER_OSPEEDR12_1            0x02000000U
#define GPIO_OSPEEDER_OSPEEDR13              0x0C000000U
#define GPIO_OSPEEDER_OSPEEDR13_0            0x04000000U
#define GPIO_OSPEEDER_OSPEEDR13_1            0x08000000U
#define GPIO_OSPEEDER_OSPEEDR14              0x30000000U
#define GPIO_OSPEEDER_OSPEEDR14_0            0x10000000U
#define GPIO_OSPEEDER_OSPEEDR14_1            0x20000000U
#define GPIO_OSPEEDER_OSPEEDR15              0xC0000000U
#define GPIO_OSPEEDER_OSPEEDR15_0            0x40000000U
#define GPIO_OSPEEDER_OSPEEDR15_1            0x80000000U

/******************  Bits definition for GPIO_PUPDR register  *****************/
#define GPIO_PUPDR_PUPDR0                    0x00000003U
#define GPIO_PUPDR_PUPDR0_0                  0x00000001U
#define GPIO_PUPDR_PUPDR0_1                  0x00000002U
#define GPIO_PUPDR_PUPDR1                    0x0000000CU
#define GPIO_PUPDR_PUPDR1_0                  0x00000004U
#define GPIO_PUPDR_PUPDR1_1                  0x00000008U
#define GPIO_PUPDR_PUPDR2                    0x00000030U
#define GPIO_PUPDR_PUPDR2_0                  0x00000010U
#define GPIO_PUPDR_PUPDR2_1                  0x00000020U
#define GPIO_PUPDR_PUPDR3                    0x000000C0U
#define GPIO_PUPDR_PUPDR3_0                  0x00000040U
#define GPIO_PUPDR_PUPDR3_1                  0x00000080U
#define GPIO_PUPDR_PUPDR4                    0x00000300U
#define GPIO_PUPDR_PUPDR4_0                  0x00000100U
#define GPIO_PUPDR_PUPDR4_1                  0x00000200U
#define GPIO_PUPDR_PUPDR5                    0x00000C00U
#define GPIO_PUPDR_PUPDR5_0                  0x00000400U
#define GPIO_PUPDR_PUPDR5_1                  0x00000800U
#define GPIO_PUPDR_PUPDR6                    0x00003000U
#define GPIO_PUPDR_PUPDR6_0                  0x00001000U
#define GPIO_PUPDR_PUPDR6_1                  0x00002000U
#define GPIO_PUPDR_PUPDR7                    0x0000C000U
#define GPIO_PUPDR_PUPDR7_0                  0x00004000U
#define GPIO_PUPDR_PUPDR7_1                  0x00008000U
#define GPIO_PUPDR_PUPDR8                    0x00030000U
#define GPIO_PUPDR_PUPDR8_0                  0x00010000U
#define GPIO_PUPDR_PUPDR8_1                  0x00020000U
#define GPIO_PUPDR_PUPDR9                    0x000C0000U
#define GPIO_PUPDR_PUPDR9_0                  0x00040000U
#define GPIO_PUPDR_PUPDR9_1                  0x00080000U
#define GPIO_PUPDR_PUPDR10                   0x00300000U
#define GPIO_PUPDR_PUPDR10_0                 0x00100000U
#define GPIO_PUPDR_PUPDR10_1                 0x00200000U
#define GPIO_PUPDR_PUPDR11                   0x00C00000U
#define GPIO_PUPDR_PUPDR11_0                 0x00400000U
#define GPIO_PUPDR_PUPDR11_1                 0x00800000U
#define GPIO_PUPDR_PUPDR12                   0x03000000U
#define GPIO_PUPDR_PUPDR12_0                 0x01000000U
#define GPIO_PUPDR_PUPDR12_1                 0x02000000U
#define GPIO_PUPDR_PUPDR13                   0x0C000000U
#define GPIO_PUPDR_PUPDR13_0                 0x04000000U
#define GPIO_PUPDR_PUPDR13_1                 0x08000000U
#define GPIO_PUPDR_PUPDR14                   0x30000000U
#define GPIO_PUPDR_PUPDR14_0                 0x10000000U
#define GPIO_PUPDR_PUPDR14_1                 0x20000000U
#define GPIO_PUPDR_PUPDR15                   0xC0000000U
#define GPIO_PUPDR_PUPDR15_0                 0x40000000U
#define GPIO_PUPDR_PUPDR15_1                 0x80000000U

/******************  Bits definition for GPIO_IDR register  *******************/
#define GPIO_IDR_IDR_0                       0x00000001U
#define GPIO_IDR_IDR_1                       0x00000002U
#define GPIO_IDR_IDR_2                       0x00000004U
#define GPIO_IDR_IDR_3                       0x00000008U
#define GPIO_IDR_IDR_4                       0x00000010U
#define GPIO_IDR_IDR_5                       0x00000020U
#define GPIO_IDR_IDR_6                       0x00000040U
#define GPIO_IDR_IDR_7                       0x00000080U
#define GPIO_IDR_IDR_8                       0x00000100U
#define GPIO_IDR_IDR_9                       0x00000200U
#define GPIO_IDR_IDR_10                      0x00000400U
#define GPIO_IDR_IDR_11                      0x00000800U
#define GPIO_IDR_IDR_12                      0x00001000U
#define GPIO_IDR_IDR_13                      0x00002000U
#define GPIO_IDR_IDR_14                      0x00004000U
#define GPIO_IDR_IDR_15                      0x00008000U

/******************  Bits definition for GPIO_ODR register  *******************/
#define GPIO_ODR_ODR_0                       0x00000001U
#define GPIO_ODR_ODR_1                       0x00000002U
#define GPIO_ODR_ODR_2                       0x00000004U
#define GPIO_ODR_ODR_3                       0x00000008U
#define GPIO_ODR_ODR_4                       0x00000010U
#define GPIO_ODR_ODR_5                       0x00000020U
#define GPIO_ODR_ODR_6                       0x00000040U
#define GPIO_ODR_ODR_7                       0x00000080U
#define GPIO_ODR_ODR_8                       0x00000100U
#define GPIO_ODR_ODR_9                       0x00000200U
#define GPIO_ODR_ODR_10                      0x00000400U
#define GPIO_ODR_ODR_11                      0x00000800U
#define GPIO_ODR_ODR_12                      0x00001000U
#define GPIO_ODR_ODR_13                      0x00002000U
#define GPIO_ODR_ODR_14                      0x00004000U
#define GPIO_ODR_ODR_15                      0x00008000U

/******************  Bits definition for GPIO_BSRR register  ******************/
#define GPIO_BSRR_BS_0                       0x00000001U
#define GPIO_BSRR_BS_1                       0x00000002U
#define GPIO_BSRR_BS_2                       0x00000004U
#define GPIO_BSRR_BS_3                       0x00000008U
#define GPIO_BSRR_BS_4                       0x00000010U
#define GPIO_BSRR_BS_5                       0x00000020U
#define GPIO_BSRR_BS_6                       0x00000040U
#define GPIO_BSRR_BS_7                       0x00000080U
#define GPIO_BSRR_BS_8                       0x00000100U
#define GPIO_BSRR_BS_9                       0x00000200U
#define GPIO_BSRR_BS_10                      0x00000400U
#define GPIO_BSRR_BS_11                      0x00000800U
#define GPIO_BSRR_BS_12                      0x00001000U
#define GPIO_BSRR_BS_13                      0x00002000U
#define GPIO_BSRR_BS_14                      0x00004000U
#define GPIO_BSRR_BS_15                      0x00008000U
#define GPIO_BSRR_BR_0                       0x00010000U
#define GPIO_BSRR_BR_1                       0x00020000U
#define GPIO_BSRR_BR_2                       0x00040000U
#define GPIO_BSRR_BR_3                       0x00080000U
#define GPIO_BSRR_BR_4                       0x00100000U
#define GPIO_BSRR_BR_5                       0x00200000U
#define GPIO_BSRR_BR_6                       0x00400000U
#define GPIO_BSRR_BR_7                       0x00800000U
#define GPIO_BSRR_BR_8                       0x01000000U
#define GPIO_BSRR_BR_9                       0x02000000U
#define GPIO_BSRR_BR_10                      0x04000000U
#define GPIO_BSRR_BR_11                      0x08000000U
#define GPIO_BSRR_BR_12                      0x10000000U
#define GPIO_BSRR_BR_13                      0x20000000U
#define GPIO_BSRR_BR_14                      0x40000000U
#define GPIO_BSRR_BR_15                      0x80000000U

/****************** Bit definition for GPIO_LCKR register *********************/
#define GPIO_LCKR_LCK0                       0x00000001U
#define GPIO_LCKR_LCK1                       0x00000002U
#define GPIO_LCKR_LCK2                       0x00000004U
#define GPIO_LCKR_LCK3                       0x00000008U
#define GPIO_LCKR_LCK4                       0x00000010U
#define GPIO_LCKR_LCK5                       0x00000020U
#define GPIO_LCKR_LCK6                       0x00000040U
#define GPIO_LCKR_LCK7                       0x00000080U
#define GPIO_LCKR_LCK8                       0x00000100U
#define GPIO_LCKR_LCK9                       0x00000200U
#define GPIO_LCKR_LCK10                      0x00000400U
#define GPIO_LCKR_LCK11                      0x00000800U
#define GPIO_LCKR_LCK12                      0x00001000U
#define GPIO_LCKR_LCK13                      0x00002000U
#define GPIO_LCKR_LCK14                      0x00004000U
#define GPIO_LCKR_LCK15                      0x00008000U
#define GPIO_LCKR_LCKK                       0x00010000U



/*GPIO�������ַ*/

#define GPIOA_BASE            (D3_AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASE            (D3_AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASE            (D3_AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASE            (D3_AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASE            (D3_AHB1PERIPH_BASE + 0x1000U)
#define GPIOF_BASE            (D3_AHB1PERIPH_BASE + 0x1400U)
#define GPIOG_BASE            (D3_AHB1PERIPH_BASE + 0x1800U)
#define GPIOH_BASE            (D3_AHB1PERIPH_BASE + 0x1C00U)
#define GPIOI_BASE            (D3_AHB1PERIPH_BASE + 0x2000U)
#define GPIOJ_BASE            (D3_AHB1PERIPH_BASE + 0x2400U)
#define GPIOK_BASE            (D3_AHB1PERIPH_BASE + 0x2800U)

/*����GPIOA-K �Ĵ����ṹ��ָ��*/
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
#define GPIOI               ((GPIO_TypeDef *) GPIOI_BASE)
#define GPIOJ               ((GPIO_TypeDef *) GPIOJ_BASE)
#define GPIOK               ((GPIO_TypeDef *) GPIOK_BASE)

/*����RCC���� �Ĵ����ṹ��ָ��*/
#define RCC                 ((RCC_TypeDef *) RCC_BASE)


