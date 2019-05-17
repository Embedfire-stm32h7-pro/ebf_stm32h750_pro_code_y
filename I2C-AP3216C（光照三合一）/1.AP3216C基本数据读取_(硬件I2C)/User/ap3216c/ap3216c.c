/**
  ******************************************************************************
  * @file    ap3216.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   AP3216C �����ļ�
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� STM32 H743 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */

#include "./ap3216c/ap3216c.h"
#include "./usart/bsp_debug_usart.h"
#include "./delay/core_delay.h" 
#include "./i2c/bsp_i2c.h"


#define AP3216C_ERROR 		I2C_ERROR
#define AP3216C_INFO 		I2C_INFO
/**
  * @brief   д���ݵ�AP3216C�Ĵ���
  * @param   reg_add:�Ĵ�����ַ
	* @param	 reg_data:Ҫд�������
  * @retval  
  */
void AP3216C_WriteReg(uint8_t reg_add,uint8_t reg_dat)
{
	Sensors_I2C_WriteRegister(AP3216C_ADDRESS,reg_add,1,&reg_dat);
}

/**
  * @brief   ��AP3216C�Ĵ�����ȡ����
  * @param   reg_add:�Ĵ�����ַ
	* @param	 Read���洢���ݵĻ�����
	* @param	 num��Ҫ��ȡ��������
  * @retval  
  */
void AP3216C_ReadData(uint8_t reg_add,unsigned char* Read,uint8_t num)
{
	Sensors_I2C_ReadRegister(AP3216C_ADDRESS,reg_add,num,Read);
}


/**
  * @brief   ��ʼ��AP3216CоƬ
  * @param   
  * @retval  
  */
void AP3216C_Init(void)
{
  AP3216C_WriteReg(AP3216C_SYS_CONFIG, 0x00);//�ر����й���
  AP3216C_WriteReg(AP3216C_SYS_CONFIG, AP3216C_SW_RST_BIT);//��λ
  HAL_Delay(10);//��λ��һ��Ҫ��ʱ10ms����������
  AP3216C_WriteReg(AP3216C_SYS_CONFIG, AP3216C_ALS_PS_IR_ACTIVE_BIT);//�������й���
}

/**
  * @brief   ��ȡAP3216C�Ļ����⴫��������
  * @param   
  * @retval  
  */
void AP3216CReadALS(uint16_t *alsData)
{
  uint8_t buf[2];
  AP3216C_ReadData(AP3216C_ALS_DATA_LOW, buf, 2);
  *alsData = (buf[1] << 8) | buf[0];
}

/**
  * @brief   ��ȡAP3216C�Ľӽ�����������
  * @param   
  * @retval  
  */
void AP3216CReadPS(uint16_t *psData)
{
  uint8_t buf[2];
  AP3216C_ReadData(AP3216C_PS_DATA_LOW, buf, 2);
  buf[0] = buf[0] & 0x0F; // PS Data LOW 4 bits
  buf[1] = buf[1] & 0x3F; // PS Data HIGH 6 bits
  *psData = (buf[1] << 4) | buf[0];
}

/**
  * @brief   ��ȡAP3216C�ĺ���⴫��������
  * @param   
  * @retval  
  */
void AP3216CReadIR(uint16_t *irData)
{
  uint8_t buf[2];
  AP3216C_ReadData(AP3216C_IR_DATA_LOW, buf, 2);
  buf[0] = buf[0] & 0x03; // IR Data LOW 2 bits
  *irData = (buf[1] << 2) | buf[0];
}
