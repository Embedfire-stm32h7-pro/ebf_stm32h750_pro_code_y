/**
  ******************************************************************************
  * @file    bsp_can.c
  * @author  fire
  * @version V1.0
  * @date    2016-xx-xx
  * @brief   can�������ػ�ģʽ��
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��STM32 H743������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "./can/bsp_can.h"

FDCAN_HandleTypeDef hfdcan;
uint8_t TxData0[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
/*
 * ��������CAN_GPIO_Config
 * ����  ��CAN��GPIO ����
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;

	/* ʹ������ʱ�� */
	CAN_TX_GPIO_CLK_ENABLE();
	CAN_RX_GPIO_CLK_ENABLE();	

	/* Select PLL1Q as source of FDCANx clock */
	RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
	RCC_PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
	HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

	/* ����CAN�������� */
	GPIO_InitStructure.Pin = CAN_TX_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull  = GPIO_PULLUP;
	GPIO_InitStructure.Alternate =  GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(CAN_TX_GPIO_PORT, &GPIO_InitStructure);

	/* ����CAN�������� */
	GPIO_InitStructure.Pin = CAN_RX_PIN ;
	HAL_GPIO_Init(CAN_RX_GPIO_PORT, &GPIO_InitStructure);
}

/*
 * ��������CAN_NVIC_Config
 * ����  ��CAN��NVIC ����,��1���ȼ��飬0��0���ȼ�
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_NVIC_Config(void)
{
  /* NVIC for FDCANx */
  HAL_NVIC_SetPriority(FDCANx_IT0_IRQn, 0, 1);
  HAL_NVIC_SetPriority(FDCANx_IT1_IRQn, 0, 1);
  HAL_NVIC_SetPriority(FDCAN_CAL_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FDCANx_IT0_IRQn);
  HAL_NVIC_EnableIRQ(FDCANx_IT1_IRQn);
  HAL_NVIC_EnableIRQ(FDCAN_CAL_IRQn);
}

/*
 * ��������CAN_Mode_Config
 * ����  ��CAN��ģʽ ����
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_Mode_Config(void)
{

	/************************CANͨ�Ų�������**********************************/
	/* ʹ��CANʱ�� */
	CAN_CLK_ENABLE();

  /* ��ʼ��FDCAN���蹤���ڻ���ģʽ */
  hfdcan.Instance = CANx;
  hfdcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan.Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
  hfdcan.Init.AutoRetransmission = ENABLE;
  hfdcan.Init.TransmitPause = DISABLE;
  hfdcan.Init.ProtocolException = ENABLE;
  /* λʱ������:
    ************************
    λʱ�����         		| Nominal      |  Data
    ----------------------|--------------|----------------
    CAN��ϵͳ�ں�ʱ������	| 40 MHz       | 40 MHz
    ʱ�䳣�� (tq)      		| 25 ns        | 25 ns
    ͬ����    						| 1 tq         | 1 tq
    ������        				| 23 tq        | 23 tq
    ��λ��1            		| 8 tq         | 8 tq
    ��λ��2  	          	| 8 tq         | 8 tq
    ͬ����ת���					| 8 tq         | 8 tq
    λ����         				| 40 tq = 1 us | 40 tq = 1 us
    λ����             		| 1 MBit/s  	 | 1 MBit/s
  */
  hfdcan.Init.NominalPrescaler = 0x1; /* tq = NominalPrescaler x (1/40MHz) */
  hfdcan.Init.NominalSyncJumpWidth = 0x8;
  hfdcan.Init.NominalTimeSeg1 = 0x1F; /* NominalTimeSeg1 = ������ + ��λ��1 */
  hfdcan.Init.NominalTimeSeg2 = 0x8;
  hfdcan.Init.MessageRAMOffset = 0;
  hfdcan.Init.StdFiltersNbr = 1;
  hfdcan.Init.ExtFiltersNbr = 2;
  hfdcan.Init.RxFifo0ElmtsNbr = 1;
  hfdcan.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan.Init.RxFifo1ElmtsNbr = 2;
  hfdcan.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan.Init.RxBuffersNbr = 4;
  hfdcan.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan.Init.TxEventsNbr = 2;
  hfdcan.Init.TxBuffersNbr = 1;
  hfdcan.Init.TxFifoQueueElmtsNbr = 2;
  hfdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  HAL_FDCAN_Init(&hfdcan);
}

/*
 * ��������CAN_Filter_Config
 * ����  ��CAN�Ĺ����� ����
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_Filter_Config(void)
{
	FDCAN_FilterTypeDef  sFilterConfig;

  /* ���ñ�׼ID���չ�������Rx������0 */
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXBUFFER;
  sFilterConfig.FilterID1 = 0x2568;
  sFilterConfig.FilterID2 = 0x2568;
  sFilterConfig.RxBufferIndex = 0;
  HAL_FDCAN_ConfigFilter(&hfdcan, &sFilterConfig);
  

}


/*
 * ��������CAN_Config
 * ����  ����������CAN�Ĺ���
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */
void CAN_Config(void)
{
  CAN_GPIO_Config();
  CAN_NVIC_Config();
  CAN_Mode_Config();
  CAN_Filter_Config();

}

/*
 * ��������CAN_SetMsg
 * ����  ��CANͨ�ű�����������,����һ����������Ϊ0-7�����ݰ�
 * ����  �����ͱ��Ľṹ��
 * ���  : ��
 * ����  ���ⲿ����
 */	 
void CAN_SetMsg(void)
{
	FDCAN_TxHeaderTypeDef TxHeader;	
  /* ����Tx��������Ϣ */
  TxHeader.Identifier = 0x2568;
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_FD_CAN;
  TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
  TxHeader.MessageMarker = 0x01;
  HAL_FDCAN_AddMessageToTxBuffer(&hfdcan, &TxHeader, TxData0, FDCAN_TX_BUFFER0);

  /* ����FDCANģ�� */
  HAL_FDCAN_Start(&hfdcan);	
	/* ���ͻ�������Ϣ */
  HAL_FDCAN_EnableTxBufferRequest(&hfdcan, FDCAN_TX_BUFFER0);	 
}



/**************************END OF FILE************************************/
