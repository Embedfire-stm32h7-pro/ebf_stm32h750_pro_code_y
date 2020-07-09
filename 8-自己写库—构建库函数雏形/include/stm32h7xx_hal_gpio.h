

#include "stm32h7xx.h"


/*GPIO���źŶ���*/
#define GPIO_PIN_0                 ((uint16_t)0x0001)  /*!< ѡ��Pin0 (1<<0) */
#define GPIO_PIN_1                 ((uint16_t)0x0002)  /*!< ѡ��Pin1 (1<<1)*/
#define GPIO_PIN_2                 ((uint16_t)0x0004)  /*!< ѡ��Pin2 (1<<2)*/
#define GPIO_PIN_3                 ((uint16_t)0x0008)  /*!< ѡ��Pin3 (1<<3)*/
#define GPIO_PIN_4                 ((uint16_t)0x0010)  /*!< ѡ��Pin4 */
#define GPIO_PIN_5                 ((uint16_t)0x0020)  /*!< ѡ��Pin5 */
#define GPIO_PIN_6                 ((uint16_t)0x0040)  /*!< ѡ��Pin6 */
#define GPIO_PIN_7                 ((uint16_t)0x0080)  /*!< ѡ��Pin7 */
#define GPIO_PIN_8                 ((uint16_t)0x0100)  /*!< ѡ��Pin8 */
#define GPIO_PIN_9                 ((uint16_t)0x0200)  /*!< ѡ��Pin9 */
#define GPIO_PIN_10                ((uint16_t)0x0400)  /*!< ѡ��Pin10 */
#define GPIO_PIN_11                ((uint16_t)0x0800)  /*!< ѡ��Pin11 */
#define GPIO_PIN_12                ((uint16_t)0x1000)  /*!< ѡ��Pin12 */
#define GPIO_PIN_13                ((uint16_t)0x2000)  /*!< ѡ��Pin13 */
#define GPIO_PIN_14                ((uint16_t)0x4000)  /*!< ѡ��Pin14 */
#define GPIO_PIN_15                ((uint16_t)0x8000)  /*!< ѡ��Pin15 */
#define GPIO_PIN_All               ((uint16_t)0xFFFF)  /*!< ѡ��ȫ������ */




#define  GPIO_MODE_INPUT           ((uint32_t)0x00000000U)   /*!< ��������*/
#define  GPIO_MODE_OUTPUT_PP       ((uint32_t)0x00000001U)   /*!< ������� */
#define  GPIO_MODE_OUTPUT_OD       ((uint32_t)0x00000011U)   /*!< ��©��� */
#define  GPIO_MODE_AF_PP           ((uint32_t)0x00000002U)   /*!< ���츴�����*/
#define  GPIO_MODE_AF_OD           ((uint32_t)0x00000012U)   /*!< ��©�������*/

#define  GPIO_MODE_ANALOG          ((uint32_t)0x00000003U)   /*!< ģ��ģʽ*/
    
#define  GPIO_SPEED_FREQ_LOW         ((uint32_t)0x00000000U)  /*!< ����*/
#define  GPIO_SPEED_FREQ_MEDIUM      ((uint32_t)0x00000001U)  /*!< ����*/
#define  GPIO_SPEED_FREQ_HIGH        ((uint32_t)0x00000002U)  /*!< ����*/
#define  GPIO_SPEED_FREQ_VERY_HIGH   ((uint32_t)0x00000003U)  /*!< ����*/

#define  GPIO_NOPULL        ((uint32_t)0x00000000U)   /*!< ��������  */
#define  GPIO_PULLUP        ((uint32_t)0x00000001U)   /*!< ����   */
#define  GPIO_PULLDOWN      ((uint32_t)0x00000002U)   /*!< ����   */

/** 
  * @brief GPIO��ʼ���ṹ�����Ͷ���  
  */ 
typedef struct
{
  uint32_t Pin;       /*ָ��Ҫ���õ�GPIO���� */

  uint32_t Mode;      /*ָ����ѡ���ŵĹ���ģʽ*/

  uint32_t Pull;      /*ָ����ѡ���ŵ��������������� */

  uint32_t Speed;     /*ָ����ѡ���ŵ��ٶ� */

  uint32_t Alternate;  /*Ҫ���ӵ���ѡ���ŵ�����*/ 
}GPIO_InitTypeDef;

/** 
  * @brief  GPIO Bit SET and Bit RESET enumeration 
  */
typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState;

#define GPIO_MODE             ((uint32_t)0x00000003U)
#define GPIO_OUTPUT_TYPE      ((uint32_t)0x00000010U)

#define GPIO_GET_INDEX(__GPIOx__)   (uint8_t)(((__GPIOx__) == (GPIOA))? 0U :\
                                              ((__GPIOx__) == (GPIOB))? 1U :\
                                              ((__GPIOx__) == (GPIOC))? 2U :\
                                              ((__GPIOx__) == (GPIOD))? 3U :\
                                              ((__GPIOx__) == (GPIOE))? 4U :\
                                              ((__GPIOx__) == (GPIOF))? 5U :\
                                              ((__GPIOx__) == (GPIOG))? 6U :\
                                              ((__GPIOx__) == (GPIOH))? 7U :\
                                              ((__GPIOx__) == (GPIOI))? 8U :\
                                              ((__GPIOx__) == (GPIOJ))? 9U : 10U)

void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);


