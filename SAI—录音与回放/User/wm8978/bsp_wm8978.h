#ifndef __WM8978_H__
#define	__WM8978_H__

#include "stm32h7xx.h"
/*---------------------------------------------------------------------------------------------*/
/*------------------------   I2C����WM8978���ò���  -------------------------------------------*/
#define WM8978_SLAVE_ADDRESS    0x34	/* WM8978 I2C�ӻ���ַ */
/* ����������� */
#define VOLUME_MAX		63		/* ������� */
#define VOLUME_STEP		1		/* �������ڲ��� */

/* �������MIC���� */
#define GAIN_MAX		63		/* ������� */
#define GAIN_STEP		1		/* ���沽�� */

/* WM8978 ��Ƶ����ͨ������ѡ��, ����ѡ���·������ MIC_LEFT_ON | LINE_ON */
typedef enum
{
	IN_PATH_OFF		= 0x00,	/* ������ */
	MIC_LEFT_ON 	= 0x01,	/* LIN,LIP�ţ�MIC��������������û�õ���*/
	MIC_RIGHT_ON	= 0x02,	/* RIN,RIP�ţ�MIC���������Ӱ�����ͷ��  */
	LINE_ON			= 0x04, /* L2,R2 ���������� */
	AUX_ON			= 0x08,	/* AUXL,AUXR ���������루������û�õ��� */
	DAC_ON			= 0x10,	/* I2S����DAC (CPU������Ƶ�ź�) */
	ADC_ON			= 0x20	/* �������Ƶ����WM8978�ڲ�ADC ��I2S¼��) */
}IN_PATH_E;

/* WM8978 ��Ƶ���ͨ������ѡ��, ����ѡ���· */
typedef enum
{
	OUT_PATH_OFF	= 0x00,	/* ����� */
	EAR_LEFT_ON 	= 0x01,	/* LOUT1 ���������� */
	EAR_RIGHT_ON	= 0x02,	/* ROUT1 ���������� */
	SPK_ON			= 0x04,	/* LOUT2��ROUT2�������������,�������� */
	OUT3_4_ON		= 0x08,	/* OUT3 �� OUT4 �����������Ƶ*/
}OUT_PATH_E;


/*�ȴ���ʱʱ��*/
#define WM8978_I2C_FLAG_TIMEOUT         ((uint32_t)0x4000)
#define WM8978_I2C_LONG_TIMEOUT         ((uint32_t)(10 * WM8978_I2C_FLAG_TIMEOUT))

/*��Ϣ���*/
#define WM8978_DEBUG_ON         1

#define WM8978_INFO(fmt,arg...)           printf("<<-EEPROM-INFO->> "fmt"\n",##arg)
#define WM8978_ERROR(fmt,arg...)          printf("<<-EEPROM-ERROR->> "fmt"\n",##arg)
#define WM8978_DEBUG(fmt,arg...)          do{\
                                          if(EEPROM_DEBUG_ON)\
                                          printf("<<-EEPROM-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                          }while(0)

/* ���ⲿ���õĺ������� */
uint8_t wm8978_Init(void);
uint8_t wm8978_Reset(void);
void wm8978_CfgAudioIF(uint16_t _usStandard, uint8_t _ucWordLen);
void wm8978_OutMute(uint8_t _ucMute);
void wm8978_PowerDown(void);
void wm8978_CfgAudioPath(uint16_t _InPath, uint16_t _OutPath);
void wm8978_SetMicGain(uint8_t _ucGain);
void wm8978_SetLineGain(uint8_t _ucGain);
void wm8978_SetOUT2Volume(uint8_t _ucVolume);
void wm8978_SetOUT1Volume(uint8_t _ucVolume);
uint8_t wm8978_ReadOUT1Volume(void);
uint8_t wm8978_ReadOUT2Volume(void);
void wm8978_NotchFilter(uint16_t _NFA0, uint16_t _NFA1);
void I2S_READ(uint32_t buffer0,uint32_t buffer1,const uint32_t num);
																					
/*---------------------------------------------------------------------------------------------*/
/*------------------------   I2S�������ݴ��䲿��  ---------------------------------------------*/																					
/**
	* I2S���ߴ�����Ƶ���ݿ���
	* WM8978_LRC    -> PB12/I2S2_WS
	* WM8978_BCLK   -> PD3/I2S2_CK
	* WM8978_ADCDAT -> PC2/I2S2_SD
	* WM8978_DACDAT -> PI3/I2S2_SD
	* WM8978_MCLK   -> PC6/I2S2_MCK
	*/
#define WM8978_CLK_ENABLE()            __SPI2_CLK_ENABLE()
#define WM8978_CLK_DISABLE()           __SPI2_CLK_DISABLE()
#define WM8978_I2Sx_SPI                SPI2

#define WM8978_LRC_GPIO_CLK()          __GPIOB_CLK_ENABLE()
#define WM8978_LRC_PORT            	   GPIOB
#define WM8978_LRC_PIN             	   GPIO_PIN_12
#define WM8978_LRC_AF                  GPIO_AF5_SPI2

#define WM8978_BCLK_GPIO_CLK()         __GPIOD_CLK_ENABLE()
#define WM8978_BCLK_PORT            	 GPIOD
#define WM8978_BCLK_PIN             	 GPIO_PIN_3
#define WM8978_BCLK_AF                 GPIO_AF5_SPI2

#define WM8978_ADCDAT_GPIO_CLK()       __GPIOC_CLK_ENABLE()
#define WM8978_ADCDAT_PORT             GPIOC
#define WM8978_ADCDAT_PIN              GPIO_PIN_2
#define WM8978_ADCDAT_AF               GPIO_AF5_SPI2

#define WM8978_DACDAT_GPIO_CLK()       __GPIOI_CLK_ENABLE()
#define WM8978_DACDAT_PORT             GPIOI
#define WM8978_DACDAT_PIN              GPIO_PIN_3
#define WM8978_DACDAT_AF               GPIO_AF5_SPI2

#define WM8978_MCLK_GPIO_CLK()         __GPIOC_CLK_ENABLE()
#define WM8978_MCLK_PORT            	 GPIOC
#define WM8978_MCLK_PIN             	 GPIO_PIN_6
#define WM8978_MCLK_AF                 GPIO_AF5_SPI2

#define I2Sx_DMA                       DMA1
#define I2Sx_DMA_CLK_ENABLE()          __HAL_RCC_DMA1_CLK_ENABLE()
#define I2Sx_TX_DMA_STREAM             DMA1_Stream4
#define I2Sx_TX_DMA_STREAM_IRQn        DMA1_Stream4_IRQn 

#define I2Sx_RX_DMA_STREAM             DMA1_Stream0
#define I2Sx_RX_DMA_STREAM_IRQn        DMA1_Stream0_IRQn 

static HAL_StatusTypeDef I2Cx_ReadMultiple(I2C_HandleTypeDef *i2c_handler, uint8_t Addr, uint16_t Reg, uint16_t MemAddSize, uint8_t *Buffer, uint16_t Length);
static HAL_StatusTypeDef I2Cx_WriteMultiple(I2C_HandleTypeDef *i2c_handler, uint8_t Addr, uint16_t Reg, uint16_t MemAddSize, uint8_t *Buffer, uint16_t Length);
static HAL_StatusTypeDef I2Cx_IsDeviceReady(I2C_HandleTypeDef *i2c_handler, uint16_t DevAddress, uint32_t Trials);
static void              I2Cx_Error(I2C_HandleTypeDef *i2c_handler, uint8_t Addr);

void wm8978_CtrlGPIO1(uint8_t _ucValue);
extern void (*I2S_DMA_TX_Callback)(void);		//I2S DMA TX�ص�����ָ��  
extern void (*I2S_DMA_RX_Callback)(void);	  //I2S DMA RX�ص�����
extern void I2S_DMAConvCplt(DMA_HandleTypeDef *hdma);
extern void I2Sxext_DMAConvCplt(DMA_HandleTypeDef *hdma);

void I2S_GPIO_Config(void);
void I2S_Stop(void);
void I2Sx_Mode_Config(const uint16_t _usStandard, const uint16_t _usWordLen,const uint32_t _usAudioFreq);
void I2Sx_TX_DMA_Init(uint32_t buffer0,uint32_t buffer1,const uint32_t num);
void I2S_Play_Start(void);
void I2S_Play_Stop(void);
extern void I2Sx_TX_DMA_STREAM_IRQFUN(void);
extern void I2Sx_RX_DMA_STREAM_IRQFUN(void);
void I2Sxext_Mode_Config(const uint16_t _usStandard, const uint16_t _usWordLen,const uint32_t _usAudioFreq);
void I2Sxext_RX_DMA_Init(uint32_t buffer0,uint32_t buffer1,const uint32_t num);
void I2Sxext_Recorde_Start(void);
void I2Sxext_Recorde_Stop(void);

#define DMA_MAX_SZE          0xFFFF
#define DMA_MAX(x)           (((x) <= 0xFFFF)? (x):0xFFFF)
#define AUDIODATA_SIZE        2   /* 16-bits audio data size */


uint32_t AUDIO_Play(uint16_t* pBuffer, uint32_t Size);
uint32_t AudioFlashPlay(uint16_t* pBuffer, uint32_t FullSize, uint32_t StartAdd);

#endif /* __WM8978_H__ */
