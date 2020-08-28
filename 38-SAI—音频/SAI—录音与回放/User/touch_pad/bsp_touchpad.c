/**
  ******************************************************************************
  * @file    bsp_touchpad.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   ���ݰ���Ӧ�ú����ӿ�
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 H750 ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
#include "./touch_pad/bsp_touchpad.h"
#include "./delay/core_delay.h" 

TIM_HandleTypeDef TIM_Handle;
//����û����ʱ��ʱ������ֵ
__IO uint16_t tpad_default_val=0;
//��ʱ��������ֵ
#define TPAD_ARR_MAX_VAL 	0XFFFF	
//��ֵ������ʱ��������(tpad_default_val + TPAD_GATE_VAL),����Ϊ����Ч����.
#define TPAD_GATE_VAL 	100	
/**
  * @brief  ���ݰ����ŵ纯��
  * @param  ��
  * @retval ��
  */
static void TPAD_Reset(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure; 
    //��������Ϊ��ͨ�������
    GPIO_InitStructure.Pin = TPAD_TIM_CH_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;	
    GPIO_InitStructure.Pull = GPIO_PULLDOWN; 
    HAL_GPIO_Init(TPAD_TIM_CH_PORT, &GPIO_InitStructure);

    //����͵�ƽ,�ŵ�
    HAL_GPIO_WritePin(TPAD_TIM_CH_PORT, TPAD_TIM_CH_PIN ,GPIO_PIN_RESET);						 
    //����һС��ʱ��͵�ƽ����֤�ŵ���ȫ
    HAL_Delay(5);

    //������±�־
    __HAL_TIM_CLEAR_FLAG(&TIM_Handle,TIM_FLAG_CC1);
    __HAL_TIM_CLEAR_FLAG(&TIM_Handle,TIM_FLAG_UPDATE);
    //��������0
    __HAL_TIM_SET_COUNTER(&TIM_Handle,0);
    //��������Ϊ���ù��ܣ����ϡ�����
    GPIO_InitStructure.Pin = TPAD_TIM_CH_PIN; 
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Alternate = TPAD_TIMx_AF;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH; 
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(TPAD_TIM_CH_PORT,&GPIO_InitStructure);
}

/**
  * @brief  ��ʱ��TIMx��ʼ������
  * @param  ��
  * @retval ��
  */
void TIM_Mode_Config(void)
{
    TIM_IC_InitTypeDef TIM_IC_Config; 
    GPIO_InitTypeDef  GPIO_InitStructure;
  
    TPAD_TIM_CLK_ENABLE();
    TPAD_TIM_GPIO_CLK_ENABLE();

    //�˿�����
    GPIO_InitStructure.Pin = TPAD_TIM_CH_PIN;	
    //���ù���
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Alternate = TPAD_TIMx_AF;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    //���������� 
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(TPAD_TIM_CH_PORT, &GPIO_InitStructure); 
  
    TIM_Handle.Instance = TPAD_TIMx;
    TIM_Handle.Init.RepetitionCounter = 0;
    
    TIM_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM_Handle.Init.Period = TPAD_ARR_MAX_VAL;
    //�趨��ʱ��Ԥ��Ƶ��Ŀ��ʱ��Ϊ��9MHz(200Mhz/22)
    TIM_Handle.Init.Prescaler = 23 - 1;
      
    HAL_TIM_IC_Init(&TIM_Handle);
  
    TIM_IC_Config.ICFilter = 0;
    TIM_IC_Config.ICPolarity = TIM_ICPOLARITY_RISING;
    TIM_IC_Config.ICPrescaler = TIM_ICPSC_DIV1;
    TIM_IC_Config.ICSelection = TIM_ICSELECTION_DIRECTTI;
    HAL_TIM_IC_ConfigChannel(&TIM_Handle, &TIM_IC_Config, TPAD_TIM_Channel_X);
  
    //����TIM 
    HAL_TIM_IC_Start(&TIM_Handle, TPAD_TIM_Channel_X);    
}
/**
  * @brief  ��ȡ��ʱ������ֵ
  * @param  ��
  * @retval TIMx_CCRx�Ĵ���ֵ
  */
static uint16_t TPAD_Get_Val(void)
{		
  uint16_t temp;
  /* �ȷŵ���ȫ������λ������ */	
	TPAD_Reset();
	//�ȴ�����������
	while(__HAL_TIM_GET_FLAG(&TIM_Handle,TIM_FLAG_CC1) == RESET )
	{
		//��ʱ��,ֱ�ӷ���CNT��ֵ
		if(__HAL_TIM_GET_COUNTER( &TIM_Handle)>TPAD_ARR_MAX_VAL-500)
			return __HAL_TIM_GET_COUNTER(&TIM_Handle);
	};
	/* ���������غ����TIMx_CCRx�Ĵ���ֵ */
	 temp = HAL_TIM_ReadCapturedValue(&TIM_Handle, TPAD_TIM_Channel_X);  
//  printf("scan_rval=%d\n",temp);
  return temp;
} 	
/**
  * @brief  ����n�β�������ȡ����ֵ�е����ֵ
  * @param  n��������ȡ�Ĵ���
  * @retval n�ζ������������������ֵ
  */
static uint16_t TPAD_Get_MaxVal(uint8_t n)
{
	uint16_t temp=0;
	uint16_t res=0;
	while(n--)
	{
		temp=TPAD_Get_Val();//�õ�һ��ֵ
		if(temp>res)res=temp;
	};
	return res;
}  

/**
  * @brief  ��ʼ��������������
  * @note   ��ÿ��ص�ʱ����������ȡֵ
  * @param  ��
  * @retval 0,��ʼ���ɹ�;1,��ʼ��ʧ��
  */

uint8_t TPAD_Init(void)
{
	uint16_t buf[10];
	uint32_t temp=0;
	uint8_t j,i;
	
	TIM_Mode_Config();
  
	for(i=0;i<10;i++)//������ȡ10��
	{				 
		buf[i]=TPAD_Get_Val();
		HAL_Delay(10);	    
	}				    
	for(i=0;i<9;i++)//����
	{
		for(j=i+1;j<10;j++)
		{
			if(buf[i]>buf[j])//��������
			{
				temp=buf[i];
				buf[i]=buf[j];
				buf[j]=temp;
			}
		}
	}
	temp=0;
	//ȡ�м��6�����ݽ���ƽ��
	for(i=2;i<8;i++)
	{
	  temp+=buf[i];
	}
	
	tpad_default_val=temp/6;	
	/* printf��ӡ��������ʹ�ã�����ȷ����ֵTPAD_GATE_VAL����Ӧ�ù�����Ӧע�͵� */
	//printf("tpad_default_val:%d\r\n",tpad_default_val);	
	
	//��ʼ����������TPAD_ARR_MAX_VAL/2����ֵ,������!
	if(tpad_default_val>TPAD_ARR_MAX_VAL/2)
	{
		return 1;
	}
	
	return 0;		     	    					   
}

/**
  * @brief  ��������ɨ�躯��
  * @note   ��ÿ��ص�ʱ����������ȡֵ
  * @param  mode:0,��֧����������(����һ�α����ɿ����ܰ���һ��);1,֧����������(����һֱ����)
  * @retval 0,û�а���;1,�а���;
  */
uint8_t TPAD_Scan(uint8_t mode)
{
	//0,���Կ�ʼ���;>0,�����ܿ�ʼ���	
	static uint8_t keyen=0;
	//ɨ����
	uint8_t res=0;
	//Ĭ�ϲ�������Ϊ3��
	uint8_t sample=3;	
  //����ֵ	
	uint16_t rval;
	
	if(mode)
	{
		//֧��������ʱ�����ò�������Ϊ6��
		sample=6;	
		//֧������	
		keyen=0;	  
	}	
	/* ��ȡ��ǰ����ֵ(���� sample ��ɨ������ֵ) */
	rval=TPAD_Get_MaxVal(sample); 	
	/* printf��ӡ��������ʹ�ã�����ȷ����ֵTPAD_GATE_VAL����Ӧ�ù�����Ӧע�͵� */
//	printf("scan_rval=%d\n",rval);
	
	//����tpad_default_val+TPAD_GATE_VAL,��С��10��tpad_default_val,����Ч
	if(rval>(tpad_default_val+TPAD_GATE_VAL)&&rval<(10*tpad_default_val))
	{			
    //keyen==0,��Ч 		
		if(keyen==0)
		{
			res=1;		 
		}			
		keyen=3;				//����Ҫ�ٹ�3��֮����ܰ�����Ч   
	}
	
	if(keyen)
	{
		keyen--;		
	}		
	return res;
}	 
/*********************************************END OF FILE**********************/
