/*
******************************************************************************
* @file    bsp_led.c
* @author  fire
* @version V1.0
* @date    2015-xx-xx
* @brief   wm8978����
******************************************************************************
* @attention
*
* ʵ��ƽ̨:Ұ��  STM32 H750 ������  
* ��̳    :http://www.chuxue123.com
* �Ա�    :http://firestm32.taobao.com
*
******************************************************************************
*/
#include "./wm8978/bsp_wm8978.h"  
#include "./usart/bsp_debug_usart.h"
#include "main.h"
#include "./i2c/i2c.h"
//#include "./Recorder/Recorder.h"
#include "./delay/core_delay.h" 
extern I2C_HandleTypeDef I2C_Handle;
uint32_t AudioTotalSize ;         /* ��Ƶ�ļ����ܴ�С */
uint32_t AudioRemSize;            /* ��ʣ������ݱ�������Ƶ�ļ��� */
uint16_t *CurrentPos;             /* ��Ƶ����ָ��ĵ�ǰλ�� */


DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;
/**
	*******************************************************************************************************
	*	                     I2C����WM8978���ò��� 
	*******************************************************************************************************
	*/
static uint8_t WM8978_I2C_WriteRegister(uint8_t RegisterAddr, uint16_t RegisterValue);
static uint16_t wm8978_ReadReg(uint8_t _ucRegAddr);
static uint8_t wm8978_WriteReg(uint8_t _ucRegAddr, uint16_t _usValue);
/*
	wm8978�Ĵ�������
	����WM8978��I2C���߽ӿڲ�֧�ֶ�ȡ��������˼Ĵ���ֵ�������ڴ��У���д�Ĵ���ʱͬ�����»��棬���Ĵ���ʱ
	ֱ�ӷ��ػ����е�ֵ��
	�Ĵ���MAP ��WM8978(V4.5_2011).pdf �ĵ�89ҳ���Ĵ�����ַ��7bit�� �Ĵ���������9bit
*/
static uint16_t wm8978_RegCash[] = {
	0x000, 0x000, 0x000, 0x000, 0x050, 0x000, 0x140, 0x000,
	0x000, 0x000, 0x000, 0x0FF, 0x0FF, 0x000, 0x100, 0x0FF,
	0x0FF, 0x000, 0x12C, 0x02C, 0x02C, 0x02C, 0x02C, 0x000,
	0x032, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000,
	0x038, 0x00B, 0x032, 0x000, 0x008, 0x00C, 0x093, 0x0E9,
	0x000, 0x000, 0x000, 0x000, 0x003, 0x010, 0x010, 0x100,
	0x100, 0x002, 0x001, 0x001, 0x039, 0x039, 0x039, 0x039,
	0x001, 0x001
};
/**
  * @brief  ͨ��I2C�������Ĵ������ֽ�д����Ƶ�������
  * @param  RegisterAddr: ��д��Ĵ����ĵ�ַ
  * @param  RegisterValue: Ҫд��Ŀ��Ĵ������ֽ�ֵ
  * @retval ͨ�ųɹ�����1��ʧ�ܷ���0
  */
static uint8_t WM8978_I2C_WriteRegister(uint8_t RegisterAddr, uint16_t RegisterValue)
{	
	uint16_t tmp;

	tmp  = (RegisterValue&0xff) << 8;
	tmp |= ((RegisterAddr << 1) & 0xFE) | ((RegisterValue >> 8) & 0x1);
	if(HAL_I2C_Master_Transmit(&I2C_Handle,WM8978_SLAVE_ADDRESS,(uint8_t *)&tmp,2,WM8978_I2C_FLAG_TIMEOUT)==HAL_OK)
	{
		return 1; 
	}
	else 		
	return 0;  
}

/**
	* @brief  ��cash�ж��ض���wm8978�Ĵ���
	* @param  _ucRegAddr �� �Ĵ�����ַ
	* @retval �Ĵ���ֵ
	*/
static uint16_t wm8978_ReadReg(uint8_t _ucRegAddr)
{
	return wm8978_RegCash[_ucRegAddr];
}

/**
	* @brief  дwm8978�Ĵ���
	* @param  _ucRegAddr�� �Ĵ�����ַ
	* @param  _usValue�� �Ĵ���ֵ
	* @retval 0��д��ʧ��
    *         1��д��ɹ�
	*/
static uint8_t wm8978_WriteReg(uint8_t _ucRegAddr, uint16_t _usValue)
{
	uint8_t res;
	res=WM8978_I2C_WriteRegister(_ucRegAddr,_usValue);
	wm8978_RegCash[_ucRegAddr] = _usValue;
	return res;
}

/**
	* @brief  ����I2C GPIO�������I2C�����ϵ�WM8978�Ƿ�����
	* @param  ��
	* @retval 1,��ʼ���ɹ�;
	*         0,��ʼ��ʧ�ܡ�
	*/
uint8_t wm8978_Init(void)
{
	uint8_t res;
	
	I2cMaster_Init();		/* ��ʼ��I2C�ӿ� */
	res=wm8978_Reset();		/* Ӳ����λWM8978���мĴ�����ȱʡ״̬ */
	wm8978_CtrlGPIO1(1);	/* ����WM8978��һ��GPIO�ӿڿ�����Ϊ����״̬ */
	return res;
}

/**
	* @brief  �޸����ͨ��1����
	* @param  _ucVolume ������ֵ, 0-63
	* @retval ��
	*/
void wm8978_SetOUT1Volume(uint8_t _ucVolume)
{
	uint16_t regL;
	uint16_t regR;

	if (_ucVolume > 0x3F)
	{
		_ucVolume = 0x3F;
	}

	regL = _ucVolume;
	regR = _ucVolume;

	/*
		R52	LOUT1 Volume control
		R53	ROUT1 Volume control
	*/
	/* �ȸ�������������ֵ */
	wm8978_WriteReg(52, regL | 0x00);

	/* ��ͬ�������������������� */
	wm8978_WriteReg(53, regR | 0x100);	/* 0x180��ʾ ������Ϊ0ʱ�ٸ��£���������������ֵġ����ա��� */
}

/**
	* @brief  �޸����ͨ��2����
	* @param  _ucVolume ������ֵ, 0-63
	* @retval ��
	*/
void wm8978_SetOUT2Volume(uint8_t _ucVolume)
{
	uint16_t regL;
	uint16_t regR;

	if (_ucVolume > 0x3F)
	{
		_ucVolume = 0x3F;
	}

	regL = _ucVolume;
	regR = _ucVolume;

	/*
		R54	LOUT2 (SPK) Volume control
		R55	ROUT2 (SPK) Volume control
	*/
	/* �ȸ�������������ֵ */
	wm8978_WriteReg(54, regL | 0x00);

	/* ��ͬ�������������������� */
	wm8978_WriteReg(55, regR | 0x100);	/* ������Ϊ0ʱ�ٸ��£���������������ֵġ����ա��� */
}

/**
	* @brief  ��ȡ���ͨ��1����
	* @param  ��
	* @retval ��ǰ����ֵ
	*/
uint8_t wm8978_ReadOUT1Volume(void)
{
	return (uint8_t)(wm8978_ReadReg(52) & 0x3F );
}

/**
	* @brief  ��ȡ���ͨ��2����
	* @param  ��
	* @retval ��ǰ����ֵ
	*/
uint8_t wm8978_ReadOUT2Volume(void)
{
	return (uint8_t)(wm8978_ReadReg(54) & 0x3F );
}

/**
	* @brief  �������.
	* @param  _ucMute��ģʽѡ��
	*         @arg 1������
	*         @arg 0��ȡ������
	* @retval ��
	*/
void wm8978_OutMute(uint8_t _ucMute)
{
	uint16_t usRegValue;

	if (_ucMute == 1) /* ���� */
	{
		usRegValue = wm8978_ReadReg(52); /* Left Mixer Control */
		usRegValue |= (1u << 6);
		wm8978_WriteReg(52, usRegValue);

		usRegValue = wm8978_ReadReg(53); /* Left Mixer Control */
		usRegValue |= (1u << 6);
		wm8978_WriteReg(53, usRegValue);

		usRegValue = wm8978_ReadReg(54); /* Right Mixer Control */
		usRegValue |= (1u << 6);
		wm8978_WriteReg(54, usRegValue);

		usRegValue = wm8978_ReadReg(55); /* Right Mixer Control */
		usRegValue |= (1u << 6);
		wm8978_WriteReg(55, usRegValue);
	}
	else	/* ȡ������ */
	{
		usRegValue = wm8978_ReadReg(52);
		usRegValue &= ~(1u << 6);
		wm8978_WriteReg(52, usRegValue);

		usRegValue = wm8978_ReadReg(53); /* Left Mixer Control */
		usRegValue &= ~(1u << 6);
		wm8978_WriteReg(53, usRegValue);

		usRegValue = wm8978_ReadReg(54);
		usRegValue &= ~(1u << 6);
		wm8978_WriteReg(54, usRegValue);

		usRegValue = wm8978_ReadReg(55); /* Left Mixer Control */
		usRegValue &= ~(1u << 6);
		wm8978_WriteReg(55, usRegValue);
	}
}

/**
	* @brief  ��������
	* @param  _ucGain ������ֵ, 0-63
	* @retval ��
	*/
void wm8978_SetMicGain(uint8_t _ucGain)
{
	if (_ucGain > GAIN_MAX)
	{
		_ucGain = GAIN_MAX;
	}

	/* PGA ��������  R45�� R46   WM8978(V4.5_2011).pdf 25ҳ
		Bit8	INPPGAUPDATE
		Bit7	INPPGAZCL		�����ٸ���
		Bit6	INPPGAMUTEL		PGA����
		Bit5:0	����ֵ��010000��0dB
	*/
	wm8978_WriteReg(45, _ucGain);
	wm8978_WriteReg(46, _ucGain | (1 << 8));
}

/**
	* @brief  ����Line����ͨ��������
	* @param  _ucGain ������ֵ, 0-7. 7���0��С�� ��˥���ɷŴ�
	* @retval ��
	*/
void wm8978_SetLineGain(uint8_t _ucGain)
{
	uint16_t usRegValue;

	if (_ucGain > 7)
	{
		_ucGain = 7;
	}

	/*
		Mic �����ŵ��������� PGABOOSTL �� PGABOOSTR ����
		Aux �����ŵ������������� AUXL2BOOSTVO[2:0] �� AUXR2BOOSTVO[2:0] ����
		Line �����ŵ��������� LIP2BOOSTVOL[2:0] �� RIP2BOOSTVOL[2:0] ����
	*/
	/*	WM8978(V4.5_2011).pdf 29ҳ��R47������������R48����������, MIC ������ƼĴ���
		R47 (R48���������ͬ)
		B8		PGABOOSTL	= 1,   0��ʾMIC�ź�ֱͨ�����棬1��ʾMIC�ź�+20dB���棨ͨ���Ծٵ�·��
		B7		= 0�� ����
		B6:4	L2_2BOOSTVOL = x�� 0��ʾ��ֹ��1-7��ʾ����-12dB ~ +6dB  ������˥��Ҳ���ԷŴ�
		B3		= 0�� ����
		B2:0`	AUXL2BOOSTVOL = x��0��ʾ��ֹ��1-7��ʾ����-12dB ~ +6dB  ������˥��Ҳ���ԷŴ�
	*/

	usRegValue = wm8978_ReadReg(47);
	usRegValue &= 0x8F;/* ��Bit6:4��0   1000 1111*/
	usRegValue |= (_ucGain << 4);
	wm8978_WriteReg(47, usRegValue);	/* д����������������ƼĴ��� */

	usRegValue = wm8978_ReadReg(48);
	usRegValue &= 0x8F;/* ��Bit6:4��0   1000 1111*/
	usRegValue |= (_ucGain << 4);
	wm8978_WriteReg(48, usRegValue);	/* д����������������ƼĴ��� */
}

/**
	* @brief  �ر�wm8978������͹���ģʽ
	* @param  ��
	* @retval ��
	*/
void wm8978_PowerDown(void)
{
	wm8978_Reset();			/* Ӳ����λWM8978���мĴ�����ȱʡ״̬ */
}

/**
	* @brief  ����WM8978����Ƶ�ӿ�(I2S)
	* @param  _usStandard : �ӿڱ�׼��I2S_Standard_Phillips, I2S_Standard_MSB �� I2S_Standard_LSB
	* @param  _ucWordLen : �ֳ���16��24��32  �����������õ�20bit��ʽ��
	* @retval ��
	*/
void wm8978_CfgAudioIF(uint16_t _usStandard, uint8_t _ucWordLen)
{
	uint16_t usReg;

	/* WM8978(V4.5_2011).pdf 73ҳ���Ĵ����б� */

	/*	REG R4, ��Ƶ�ӿڿ��ƼĴ���
		B8		BCP	 = X, BCLK���ԣ�0��ʾ������1��ʾ����
		B7		LRCP = x, LRCʱ�Ӽ��ԣ�0��ʾ������1��ʾ����
		B6:5	WL = x�� �ֳ���00=16bit��01=20bit��10=24bit��11=32bit ���Ҷ���ģʽֻ�ܲ��������24bit)
		B4:3	FMT = x����Ƶ���ݸ�ʽ��00=�Ҷ��룬01=����룬10=I2S��ʽ��11=PCM
		B2		DACLRSWAP = x, ����DAC���ݳ�����LRCʱ�ӵ���߻����ұ�
		B1 		ADCLRSWAP = x������ADC���ݳ�����LRCʱ�ӵ���߻����ұ�
		B0		MONO	= 0��0��ʾ��������1��ʾ������������������Ч
	*/
	usReg = 0;
	if (_usStandard == SAI_I2S_STANDARD)	/* I2S�����ֱ�׼ */
	{
		usReg |= (2 << 3);
	}
	else if (_usStandard == SAI_I2S_MSBJUSTIFIED)	/* MSB�����׼(�����) */
	{
		usReg |= (1 << 3);
	}
	else if (_usStandard == SAI_I2S_LSBJUSTIFIED)	/* LSB�����׼(�Ҷ���) */
	{
		usReg |= (0 << 3);
	}
	else	/* PCM��׼(16λͨ��֡�ϴ������֡ͬ������16λ����֡��չΪ32λͨ��֡) */
	{
		usReg |= (3 << 3);;
	}

	if (_ucWordLen == 24)
	{
		usReg |= (2 << 5);
	}
	else if (_ucWordLen == 32)
	{
		usReg |= (3 << 5);
	}
	else
	{
		usReg |= (0 << 5);		/* 16bit */
	}
	wm8978_WriteReg(4, usReg);

	/*
		R6��ʱ�Ӳ������ƼĴ���
		MS = 0,  WM8978����ʱ�ӣ���MCU�ṩMCLKʱ��
	*/
	wm8978_WriteReg(6, 0x000);
}


/**
	* @brief  ����wm8978��Ƶͨ��
	* @param  _InPath : ��Ƶ����ͨ������
	* @param  _OutPath : ��Ƶ���ͨ������
	* @retval ��
	*/
void wm8978_CfgAudioPath(uint16_t _InPath, uint16_t _OutPath)
{
	uint16_t usReg;

	/* �鿴WM8978�����ֲ�� REGISTER MAP �½ڣ� ��89ҳ */

	if ((_InPath == IN_PATH_OFF) && (_OutPath == OUT_PATH_OFF))
	{
		wm8978_PowerDown();
		return;
	}

	/* --------------------------- ��1������������ͨ���������üĴ��� -----------------------*/
	/*
		R1 �Ĵ��� Power manage 1
		Bit8    BUFDCOPEN,  Output stage 1.5xAVDD/2 driver enable
 		Bit7    OUT4MIXEN, OUT4 mixer enable
		Bit6    OUT3MIXEN, OUT3 mixer enable
		Bit5    PLLEN	.����
		Bit4    MICBEN	,Microphone Bias Enable (MICƫ�õ�·ʹ��)
		Bit3    BIASEN	,Analogue amplifier bias control ��������Ϊ1ģ��Ŵ����Ź���
		Bit2    BUFIOEN , Unused input/output tie off buffer enable
		Bit1:0  VMIDSEL, ��������Ϊ��00ֵģ��Ŵ����Ź���
	*/
	usReg = (1 << 3) | (3 << 0);
	if (_OutPath & OUT3_4_ON) 	/* OUT3��OUT4ʹ�������GSMģ�� */
	{
		usReg |= ((1 << 7) | (1 << 6));
	}
	if ((_InPath & MIC_LEFT_ON) || (_InPath & MIC_RIGHT_ON))
	{
		usReg |= (1 << 4);
	}
	wm8978_WriteReg(1, usReg);	/* д�Ĵ��� */

	/*
		R2 �Ĵ��� Power manage 2
		Bit8	ROUT1EN,	ROUT1 output enable �������������ʹ��
		Bit7	LOUT1EN,	LOUT1 output enable �������������ʹ��
		Bit6	SLEEP, 		0 = Normal device operation   1 = Residual current reduced in device standby mode
		Bit5	BOOSTENR,	Right channel Input BOOST enable ����ͨ���Ծٵ�·ʹ��. �õ�PGA�Ŵ���ʱ����ʹ��
		Bit4	BOOSTENL,	Left channel Input BOOST enable
		Bit3	INPGAENR,	Right channel input PGA enable ����������PGAʹ��
		Bit2	INPGAENL,	Left channel input PGA enable
		Bit1	ADCENR,		Enable ADC right channel
		Bit0	ADCENL,		Enable ADC left channel
	*/
	usReg = 0;
	if (_OutPath & EAR_LEFT_ON)
	{
		usReg |= (1 << 7);
	}
	if (_OutPath & EAR_RIGHT_ON)
	{
		usReg |= (1 << 8);
	}
	if (_InPath & MIC_LEFT_ON)
	{
		usReg |= ((1 << 4) | (1 << 2));
	}
	if (_InPath & MIC_RIGHT_ON)
	{
		usReg |= ((1 << 5) | (1 << 3));
	}
	if (_InPath & LINE_ON)
	{
		usReg |= ((1 << 4) | (1 << 5));
	}
	if (_InPath & MIC_RIGHT_ON)
	{
		usReg |= ((1 << 5) | (1 << 3));
	}
	if (_InPath & ADC_ON)
	{
		usReg |= ((1 << 1) | (1 << 0));
	}
	wm8978_WriteReg(2, usReg);	/* д�Ĵ��� */

	/*
		R3 �Ĵ��� Power manage 3
		Bit8	OUT4EN,		OUT4 enable
		Bit7	OUT3EN,		OUT3 enable
		Bit6	LOUT2EN,	LOUT2 output enable
		Bit5	ROUT2EN,	ROUT2 output enable
		Bit4	0,
		Bit3	RMIXEN,		Right mixer enable
		Bit2	LMIXEN,		Left mixer enable
		Bit1	DACENR,		Right channel DAC enable
		Bit0	DACENL,		Left channel DAC enable
	*/
	usReg = 0;
	if (_OutPath & OUT3_4_ON)
	{
		usReg |= ((1 << 8) | (1 << 7));
	}
	if (_OutPath & SPK_ON)
	{
		usReg |= ((1 << 6) | (1 << 5));
	}
	if (_OutPath != OUT_PATH_OFF)
	{
		usReg |= ((1 << 3) | (1 << 2));
	}
	if (_InPath & DAC_ON)
	{
		usReg |= ((1 << 1) | (1 << 0));
	}
	wm8978_WriteReg(3, usReg);	/* д�Ĵ��� */

	/*
		R44 �Ĵ��� Input ctrl

		Bit8	MBVSEL, 		Microphone Bias Voltage Control   0 = 0.9 * AVDD   1 = 0.6 * AVDD
		Bit7	0
		Bit6	R2_2INPPGA,		Connect R2 pin to right channel input PGA positive terminal
		Bit5	RIN2INPPGA,		Connect RIN pin to right channel input PGA negative terminal
		Bit4	RIP2INPPGA,		Connect RIP pin to right channel input PGA amplifier positive terminal
		Bit3	0
		Bit2	L2_2INPPGA,		Connect L2 pin to left channel input PGA positive terminal
		Bit1	LIN2INPPGA,		Connect LIN pin to left channel input PGA negative terminal
		Bit0	LIP2INPPGA,		Connect LIP pin to left channel input PGA amplifier positive terminal
	*/
	usReg = 0 << 8;
	if (_InPath & LINE_ON)
	{
		usReg |= ((1 << 6) | (1 << 2));
	}
	if (_InPath & MIC_RIGHT_ON)
	{
		usReg |= ((1 << 5) | (1 << 4));
	}
	if (_InPath & MIC_LEFT_ON)
	{
		usReg |= ((1 << 1) | (1 << 0));
	}
	wm8978_WriteReg(44, usReg);	/* д�Ĵ��� */


	/*
		R14 �Ĵ��� ADC Control
		���ø�ͨ�˲�������ѡ�ģ� WM8978(V4.5_2011).pdf 31 32ҳ,
		Bit8 	HPFEN,	High Pass Filter Enable��ͨ�˲���ʹ�ܣ�0��ʾ��ֹ��1��ʾʹ��
		BIt7 	HPFAPP,	Select audio mode or application mode ѡ����Ƶģʽ��Ӧ��ģʽ��0��ʾ��Ƶģʽ��
		Bit6:4	HPFCUT��Application mode cut-off frequency  000-111ѡ��Ӧ��ģʽ�Ľ�ֹƵ��
		Bit3 	ADCOSR,	ADC oversample rate select: 0=64x (lower power) 1=128x (best performance)
		Bit2   	0
		Bit1 	ADC right channel polarity adjust:  0=normal  1=inverted
		Bit0 	ADC left channel polarity adjust:  0=normal 1=inverted
	*/
	if (_InPath & ADC_ON)
	{
		usReg = (1 << 3) | (0 << 8) | (4 << 0);		/* ��ֹADC��ͨ�˲���, ���ý���Ƶ�� */
	}
	else
	{
		usReg = 0;
	}
	wm8978_WriteReg(14, usReg);	/* д�Ĵ��� */

	/* �����ݲ��˲�����notch filter������Ҫ�������ƻ�Ͳ����������������Х��.  ��ʱ�ر�
		R27��R28��R29��R30 ���ڿ����޲��˲�����WM8978(V4.5_2011).pdf 33ҳ
		R7�� Bit7 NFEN = 0 ��ʾ��ֹ��1��ʾʹ��
	*/
	if (_InPath & ADC_ON)
	{
		usReg = (0 << 7);
		wm8978_WriteReg(27, usReg);	/* д�Ĵ��� */
		usReg = 0;
		wm8978_WriteReg(28, usReg);	/* д�Ĵ���,��0����Ϊ�Ѿ���ֹ������Ҳ�ɲ��� */
		wm8978_WriteReg(29, usReg);	/* д�Ĵ���,��0����Ϊ�Ѿ���ֹ������Ҳ�ɲ��� */
		wm8978_WriteReg(30, usReg);	/* д�Ĵ���,��0����Ϊ�Ѿ���ֹ������Ҳ�ɲ��� */
	}

	/* �Զ�������� ALC, R32  - 34  WM8978(V4.5_2011).pdf 36ҳ */
	{
		usReg = 0;		/* ��ֹ�Զ�������� */
		wm8978_WriteReg(32, usReg);
		wm8978_WriteReg(33, usReg);
		wm8978_WriteReg(34, usReg);
	}

	/*  R35  ALC Noise Gate Control
		Bit3	NGATEN, Noise gate function enable
		Bit2:0	Noise gate threshold:
	*/
	usReg = (3 << 1) | (7 << 0);		/* ��ֹ�Զ�������� */
	wm8978_WriteReg(35, usReg);

	/*
		Mic �����ŵ��������� PGABOOSTL �� PGABOOSTR ����
		Aux �����ŵ������������� AUXL2BOOSTVO[2:0] �� AUXR2BOOSTVO[2:0] ����
		Line �����ŵ��������� LIP2BOOSTVOL[2:0] �� RIP2BOOSTVOL[2:0] ����
	*/
	/*	WM8978(V4.5_2011).pdf 29ҳ��R47������������R48����������, MIC ������ƼĴ���
		R47 (R48���������ͬ)
		B8		PGABOOSTL	= 1,   0��ʾMIC�ź�ֱͨ�����棬1��ʾMIC�ź�+20dB���棨ͨ���Ծٵ�·��
		B7		= 0�� ����
		B6:4	L2_2BOOSTVOL = x�� 0��ʾ��ֹ��1-7��ʾ����-12dB ~ +6dB  ������˥��Ҳ���ԷŴ�
		B3		= 0�� ����
		B2:0`	AUXL2BOOSTVOL = x��0��ʾ��ֹ��1-7��ʾ����-12dB ~ +6dB  ������˥��Ҳ���ԷŴ�
	*/
	usReg = 0;
	if ((_InPath & MIC_LEFT_ON) || (_InPath & MIC_RIGHT_ON))
	{
		usReg |= (1 << 8);	/* MIC����ȡ+20dB */
	}
	if (_InPath & AUX_ON)
	{
		usReg |= (3 << 0);	/* Aux����̶�ȡ3���û��������е��� */
	}
	if (_InPath & LINE_ON)
	{
		usReg |= (3 << 4);	/* Line����̶�ȡ3���û��������е��� */
	}
	wm8978_WriteReg(47, usReg);	/* д����������������ƼĴ��� */
	wm8978_WriteReg(48, usReg);	/* д����������������ƼĴ��� */

	/* ����ADC�������ƣ�pdf 35ҳ
		R15 ����������ADC������R16����������ADC����
		Bit8 	ADCVU  = 1 ʱ�Ÿ��£�����ͬ����������������ADC����
		Bit7:0 	����ѡ�� 0000 0000 = ����
						   0000 0001 = -127dB
						   0000 0010 = -12.5dB  ��0.5dB ������
						   1111 1111 = 0dB  ����˥����
	*/
	usReg = 0xFF;
	wm8978_WriteReg(15, usReg);	/* ѡ��0dB���Ȼ��������� */
	usReg = 0x1FF;
	wm8978_WriteReg(16, usReg);	/* ͬ�������������� */

	/* ͨ�� wm8978_SetMicGain ��������mic PGA���� */

	/*	R43 �Ĵ���  AUXR �C ROUT2 BEEP Mixer Function
		B8:6 = 0

		B5	 MUTERPGA2INV,	Mute input to INVROUT2 mixer
		B4	 INVROUT2,  Invert ROUT2 output �����������������
		B3:1 BEEPVOL = 7;	AUXR input to ROUT2 inverter gain
		B0	 BEEPEN = 1;	Enable AUXR beep input

	*/
	usReg = 0;
	if (_OutPath & SPK_ON)
	{
		usReg |= (1 << 4);	/* ROUT2 ����, �������������� */
	}
	if (_InPath & AUX_ON)
	{
		usReg |= ((7 << 1) | (1 << 0));
	}
	wm8978_WriteReg(43, usReg);

	/* R49  Output ctrl
		B8:7	0
		B6		DACL2RMIX,	Left DAC output to right output mixer
		B5		DACR2LMIX,	Right DAC output to left output
		B4		OUT4BOOST,	0 = OUT4 output gain = -1; DC = AVDD / 2��1 = OUT4 output gain = +1.5��DC = 1.5 x AVDD / 2
		B3		OUT3BOOST,	0 = OUT3 output gain = -1; DC = AVDD / 2��1 = OUT3 output gain = +1.5��DC = 1.5 x AVDD / 2
		B2		SPKBOOST,	0 = Speaker gain = -1;  DC = AVDD / 2 ; 1 = Speaker gain = +1.5; DC = 1.5 x AVDD / 2
		B1		TSDEN,   Thermal Shutdown Enable  �������ȱ���ʹ�ܣ�ȱʡ1��
		B0		VROI,	Disabled Outputs to VREF Resistance
	*/
	usReg = 0;
	if (_InPath & DAC_ON)
	{
		usReg |= ((1 << 6) | (1 << 5));
	}
	if (_OutPath & SPK_ON)
	{
		usReg |=  ((1 << 2) | (1 << 1));	/* SPK 1.5x����,  �ȱ���ʹ�� */
	}
	if (_OutPath & OUT3_4_ON)
	{
		usReg |=  ((1 << 4) | (1 << 3));	/* BOOT3  BOOT4  1.5x���� */
	}
	wm8978_WriteReg(49, usReg);

	/*	REG 50    (50����������51�������������üĴ�������һ��) WM8978(V4.5_2011).pdf 56ҳ
		B8:6	AUXLMIXVOL = 111	AUX����FM�������ź�����
		B5		AUXL2LMIX = 1		Left Auxilliary input to left channel
		B4:2	BYPLMIXVOL			����
		B1		BYPL2LMIX = 0;		Left bypass path (from the left channel input boost output) to left output mixer
		B0		DACL2LMIX = 1;		Left DAC output to left output mixer
	*/
	usReg = 0;
	if (_InPath & AUX_ON)
	{
		usReg |= ((7 << 6) | (1 << 5));
	}
	if ((_InPath & LINE_ON) || (_InPath & MIC_LEFT_ON) || (_InPath & MIC_RIGHT_ON))
	{
		usReg |= ((7 << 2) | (1 << 1));
	}
	if (_InPath & DAC_ON)
	{
		usReg |= (1 << 0);
	}
	wm8978_WriteReg(50, usReg);
	wm8978_WriteReg(51, usReg);

	/*	R56 �Ĵ���   OUT3 mixer ctrl
		B8:7	0
		B6		OUT3MUTE,  	0 = Output stage outputs OUT3 mixer;  1 = Output stage muted �C drives out VMID.
		B5:4	0
		B3		BYPL2OUT3,	OUT4 mixer output to OUT3  (����)
		B4		0
		B2		LMIX2OUT3,	Left ADC input to OUT3
		B1		LDAC2OUT3,	Left DAC mixer to OUT3
		B0		LDAC2OUT3,	Left DAC output to OUT3
	*/
	usReg = 0;
	if (_OutPath & OUT3_4_ON)
	{
		usReg |= (1 << 3);
	}
	wm8978_WriteReg(56, usReg);

	/* R57 �Ĵ���		OUT4 (MONO) mixer ctrl
		B8:7	0
		B6		OUT4MUTE,	0 = Output stage outputs OUT4 mixer  1 = Output stage muted �C drives outVMID.
		B5		HALFSIG,	0 = OUT4 normal output	1 = OUT4 attenuated by 6dB
		B4		LMIX2OUT4,	Left DAC mixer to OUT4
		B3		LDAC2UT4,	Left DAC to OUT4
		B2		BYPR2OUT4,	Right ADC input to OUT4
		B1		RMIX2OUT4,	Right DAC mixer to OUT4
		B0		RDAC2OUT4,	Right DAC output to OUT4
	*/
	usReg = 0;
	if (_OutPath & OUT3_4_ON)
	{
		usReg |= ((1 << 4) |  (1 << 1));
	}
	wm8978_WriteReg(57, usReg);


	/* R11, 12 �Ĵ��� DAC��������
		R11		Left DAC Digital Volume
		R12		Right DAC Digital Volume
	*/
	if (_InPath & DAC_ON)
	{
		wm8978_WriteReg(11, 255);
		wm8978_WriteReg(12, 255 | 0x100);
	}
	else
	{
		wm8978_WriteReg(11, 0);
		wm8978_WriteReg(12, 0 | 0x100);
	}

	/*	R10 �Ĵ��� DAC Control
		B8	0
		B7	0
		B6	SOFTMUTE,	Softmute enable:
		B5	0
		B4	0
		B3	DACOSR128,	DAC oversampling rate: 0=64x (lowest power) 1=128x (best performance)
		B2	AMUTE,		Automute enable
		B1	DACPOLR,	Right DAC output polarity
		B0	DACPOLL,	Left DAC output polarity:
	*/
	if (_InPath & DAC_ON)
	{
		wm8978_WriteReg(10, 0);
	}
	;
}

/**
	* @brief  �����ݲ��˲�����notch filter������Ҫ�������ƻ�Ͳ����������������Х��
	* @param  NFA0[13:0] and NFA1[13:0]
	* @retval ��
	*/
void wm8978_NotchFilter(uint16_t _NFA0, uint16_t _NFA1)
{
	uint16_t usReg;

	/*  page 26
		A programmable notch filter is provided. This filter has a variable centre frequency and bandwidth,
		programmable via two coefficients, a0 and a1. a0 and a1 are represented by the register bits
		NFA0[13:0] and NFA1[13:0]. Because these coefficient values require four register writes to setup
		there is an NFU (Notch Filter Update) flag which should be set only when all four registers are setup.
	*/
	usReg = (1 << 7) | (_NFA0 & 0x3F);
	wm8978_WriteReg(27, usReg);	/* д�Ĵ��� */

	usReg = ((_NFA0 >> 7) & 0x3F);
	wm8978_WriteReg(28, usReg);	/* д�Ĵ��� */

	usReg = (_NFA1 & 0x3F);
	wm8978_WriteReg(29, usReg);	/* д�Ĵ��� */

	usReg = (1 << 8) | ((_NFA1 >> 7) & 0x3F);
	wm8978_WriteReg(30, usReg);	/* д�Ĵ��� */
}


/**
	* @brief  ����WM8978��GPIO1�������0��1��
	*		  ����ģ�⿪�����л�¼������
	*		  1������
	*		  0��¼��
	* @param  _ucValue ��GPIO1���ֵ��0��1
	* @retval ��
	*/
void wm8978_CtrlGPIO1(uint8_t _ucValue)
{
	uint16_t usRegValue;

	/* R8�� pdf 62ҳ */
	if (_ucValue == 0) /* ���0 */
	{
		usRegValue = 6; /* B2:0 = 110 */
	}
	else
	{
		usRegValue = 7; /* B2:0 = 111 */
	}
	wm8978_WriteReg(8, usRegValue);
}

/**
	* @brief  ��λwm8978�����еļĴ���ֵ�ָ���ȱʡֵ
	* @param  ��
	* @retval 1: ��λ�ɹ�
	* 				0����λʧ��
	*/
uint8_t wm8978_Reset(void)
{
	/* wm8978�Ĵ���ȱʡֵ */
	const uint16_t reg_default[] = {
	0x000, 0x000, 0x000, 0x000, 0x050, 0x000, 0x140, 0x000,
	0x000, 0x000, 0x000, 0x0FF, 0x0FF, 0x000, 0x100, 0x0FF,
	0x0FF, 0x000, 0x12C, 0x02C, 0x02C, 0x02C, 0x02C, 0x000,
	0x032, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000,
	0x038, 0x00B, 0x032, 0x000, 0x008, 0x00C, 0x093, 0x0E9,
	0x000, 0x000, 0x000, 0x000, 0x003, 0x010, 0x010, 0x100,
	0x100, 0x002, 0x001, 0x001, 0x039, 0x039, 0x039, 0x039,
	0x001, 0x001
	};
	uint8_t res;
	uint8_t i;

	res=wm8978_WriteReg(0x00, 0);

	for (i = 0; i < sizeof(reg_default) / 2; i++)
	{
		wm8978_RegCash[i] = reg_default[i];
	}
	return res;
}


/***************************** (END OF FILE) *********************************/
