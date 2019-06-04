/**
  ******************************************************************
  * @file    bsp_sai.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   ����Ӧ��bsp��ɨ��ģʽ��
  ******************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� STM32H750������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************
  */ 
#include "./sai/bsp_sai.h" 

#include "./mp3Player/mp3Player.h"
SAI_HandleTypeDef h_sai;
DMA_HandleTypeDef h_txdma;   //DMA���;��
void (*SAI_DMA_TX_Callback)(void);
/**
  * @brief  SAI_MSP_Init
  * @param  ��
  * @retval ��
  */
void SAI_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  //Msp Clock Init
  SAI_LRC_GPIO_CLK_ENABLE();
  SAI_BCLK_GPIO_CLK_ENABLE();
  SAI_SDA_GPIO_CLK_ENABLE();
  SAI_MCLK_GPIO_CLK_ENABLE();  
  
  //SAI1_blockA
  GPIO_InitStruct.Pin = SAI_LRC_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = SAI_LRC_GPIO_AF;
  HAL_GPIO_Init(SAI_LRC_PORT, &GPIO_InitStruct);  
  
  GPIO_InitStruct.Pin = SAI_BCLK_PIN;
  GPIO_InitStruct.Alternate = SAI_BCLK_GPIO_AF;
  HAL_GPIO_Init(SAI_BCLK_PORT, &GPIO_InitStruct);    
  

  GPIO_InitStruct.Pin = SAI_SDA_PIN;
  GPIO_InitStruct.Alternate = SAI_SDA_GPIO_AF;
  HAL_GPIO_Init(SAI_SDA_PORT, &GPIO_InitStruct);    
  
  GPIO_InitStruct.Pin = SAI_MCLK_PIN;
  GPIO_InitStruct.Alternate = SAI_MCLK_GPIO_AF;
  HAL_GPIO_Init(SAI_MCLK_PORT, &GPIO_InitStruct);     
}

void BSP_AUDIO_OUT_ClockConfig(uint32_t AudioFreq)
{
  RCC_PeriphCLKInitTypeDef RCC_ExCLKInitStruct; 
	RCC_ExCLKInitStruct.PeriphClockSelection=RCC_PERIPHCLK_SAI1;
	RCC_ExCLKInitStruct.Sai1ClockSelection=RCC_SAI1CLKSOURCE_PLL2;  
  RCC_ExCLKInitStruct.PLL2.PLL2M=25;
  switch(AudioFreq)
  {
    case SAI_AUDIO_FREQUENCY_44K :
    {
      RCC_ExCLKInitStruct.PLL2.PLL2N=344;
      RCC_ExCLKInitStruct.PLL2.PLL2P=2;      
      break;
    }
    default:
      while(1);
  }
  HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);
  
}


/**
  * @brief  SAI_BlockA_Init
  * @param  ��
  * @retval ��
  */
void SAIxA_Tx_Config(const uint16_t _usStandard, const uint16_t _usWordLen, const uint32_t _usAudioFreq)
{
  
  
  
  SAI_CLK_ENABLE();
  h_sai.Instance = SAI1_Block_A;
  
  h_sai.Init.AudioMode = SAI_MODEMASTER_TX;//����Ϊ����ģʽ
  h_sai.Init.Synchro = SAI_ASYNCHRONOUS; //ģ���ڲ�Ϊ�첽
  h_sai.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;//�������
  h_sai.Init.NoDivider=SAI_MASTERDIVIDER_ENABLE;//ʹ��MCK���NOACK
  h_sai.Init.FIFOThreshold=SAI_FIFOTHRESHOLD_1QF;//1/4FIFO
  h_sai.Init.MonoStereoMode=SAI_STEREOMODE;
  h_sai.Init.AudioFrequency = _usAudioFreq;
  
  HAL_SAI_InitProtocol(&h_sai, SAI_I2S_STANDARD, _usWordLen, 2);//2--left_channel and right_channel
}

void SAIA_TX_DMA_Init(uint32_t buffer0,uint32_t buffer1,const uint32_t num)
{
  DMA_CLK_ENABLE();
  
  h_txdma.Instance = DMA_Instance;
  h_txdma.Init.Request = DMA_REQUEST_SAI1_A;
  h_txdma.Init.Direction = DMA_MEMORY_TO_PERIPH;
  h_txdma.Init.PeriphInc = DMA_PINC_DISABLE;
  h_txdma.Init.MemInc = DMA_MINC_ENABLE;
  h_txdma.Init.Mode = DMA_CIRCULAR;
  h_txdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  h_txdma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD ;
  h_txdma.Init.FIFOMode = DMA_FIFOMODE_DISABLE ;
  h_txdma.Init.Priority = DMA_PRIORITY_HIGH;
  h_txdma.Init.MemBurst=DMA_MBURST_SINGLE;             //�洢������ͻ������
  h_txdma.Init.PeriphBurst=DMA_PBURST_SINGLE;          //����ͻ�����δ��� 
  
  __HAL_LINKDMA(&h_sai,hdmatx,h_txdma);        //��DMA��SAI��ϵ����
  HAL_DMA_Init(&h_txdma);	                            //��ʼ��DMA
  
  __HAL_DMA_ENABLE_IT(&h_txdma,DMA_IT_TC);
  HAL_DMAEx_MultiBufferStart(&h_txdma,(uint32_t)buffer0,(uint32_t)&SAI1_Block_A->DR,(uint32_t)buffer1,num);//����˫����
  
  HAL_NVIC_SetPriority(DMA_IRQn,0,0);                    //DMA�ж����ȼ�
  HAL_NVIC_EnableIRQ(DMA_IRQn);

}


void SAI_Play_Stop(void)
{
  __HAL_SAI_DISABLE(&h_sai);
  __HAL_DMA_DISABLE(&h_txdma);
}


void SAI_Play_Start(void)
{
	SAI1_Block_A->CR1|=1<<17;			    //д��CR1�Ĵ�����
  __HAL_SAI_ENABLE(&h_sai);
  __HAL_DMA_ENABLE(&h_txdma);//����DMA TX���� 
}
/**
* @brief  DMA conversion complete callback. 
* @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
*                the configuration information for the specified DMA module.
* @retval None
*/
void SAI_DMAConvCplt(DMA_HandleTypeDef *hdma)
{
    MusicPlayer_SAI_DMA_TX_Callback();
}
/**
	* @brief  SPIx_TX_DMA_STREAM�жϷ�����
	* @param  ��
	* @retval ��
	*/
void I2Sx_TX_DMA_STREAM_IRQFUN(void)
{  
	//ִ�лص�����,��ȡ���ݵȲ����������洦��	
	h_txdma.XferCpltCallback = SAI_DMAConvCplt;
	h_txdma.XferM1CpltCallback = SAI_DMAConvCplt;
	HAL_DMA_IRQHandler(&h_txdma);   	
	
} 


