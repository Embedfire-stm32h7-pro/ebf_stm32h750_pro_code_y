/*
******************************************************************************
* @file    recorder.c
* @author  fire
* @version V1.0
* @date    2019-xx-xx
* @brief   WM8978¼�����ܲ���
******************************************************************************
* @attention
*
* ʵ��ƽ̨:Ұ��  STM32 H750 ������  
* ��̳    :http://www.firebbs.cn
* �Ա�    :https://fire-stm32.taobao.com
*
******************************************************************************
*/
#include "./usart/bsp_debug_usart.h"
#include "./key/bsp_key.h" 
#include "./wm8978/bsp_wm8978.h"
#include "./touch_pad/bsp_touchpad.h"
//#include "./systick/bsp_SysTick.h"
#include "ff.h" 
#include "./Recorder/Recorder.h"
#include "./delay/core_delay.h" 
#include "./sai/bsp_sai.h" 
/* ��Ƶ��ʽ�л��б�(�����Զ���) */
#define RECORDERDIR	"0:/recorder"
#define FMT_COUNT	6		/* ��Ƶ��ʽ����Ԫ�ظ��� */
static REC_TYPE Recorder;          /* ¼���豸 */
static uint8_t Isread=0;           /* DMA������ɱ�־ */
static uint8_t bufflag=0;          /* ���ݻ�����ѡ���־ */
static WavHead rec_wav;            /* WAV�豸  */
static uint32_t wavsize=0;         /* wav��Ƶ���ݴ�С */
FIL file;											/* file objects */
FRESULT result; 
UINT bw;            					/* File R/W count */
__align(4) uint16_t buffer0[RECBUFFER_SIZE];  /* ���ݻ�����1 ��ʵ��ռ���ֽ�����RECBUFFER_SIZE*2 */
__align(4) uint16_t buffer1[RECBUFFER_SIZE];  /* ���ݻ�����2 ��ʵ��ռ���ֽ�����RECBUFFER_SIZE*2 */
///* ¼���ļ�·��ȫ�ƣ���ʼ��Ϊrec001.wav */
static char recfilename[25]={"0:/recorder/rec001.wav"};   
uint32_t g_FmtList[FMT_COUNT][3] =
{
	{SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, SAI_AUDIO_FREQUENCY_44K},
	{SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, SAI_AUDIO_FREQUENCY_44K},
	{SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, SAI_AUDIO_FREQUENCY_44K},
	{SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, SAI_AUDIO_FREQUENCY_44K},
	{SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, SAI_AUDIO_FREQUENCY_44K},
	{SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, SAI_AUDIO_FREQUENCY_44K},
};
 uint16_t recplaybuf[4]={0X0000,0X0000};//2��16λ����,����¼��ʱI2S Master����.ѭ������0.
/**
  * @brief   WAV��ʽ��Ƶ����������
	* @note   
  * @param  ��
  * @retval ��
  */
int i = 0;
uint16_t data[100];
int Buffer[100];
void fill(int *p)
{
  int i = 0;
  for(;i < 100; i++)
    Buffer[i] = i;
}
/**
  * @brief  ��ʾ��ǰ״̬
  * @param  ��
  * @retval ��
  */
static void DispStatus(void)
{
	char buf[100];
	printf("\n    ����KEY2����ʼ¼��    \n");
	printf(  "   ���µ��ݰ�����ʼ�ط�    \n");	
	printf(  "      ����KEY1������    \n\r");
	/* ��ʾ��ǰ��Ƶ��ʽ */
	sprintf(buf, "��Ƶ��ʽ: Philips,16Bit,%d.%dkHz     ",
		g_FmtList[Recorder.ucFmtIdx][2]/1000,
		(g_FmtList[Recorder.ucFmtIdx][2]%1000)/100
		);
	printf("%s\n",buf);

	if (Recorder.ucStatus == STA_IDLE)
	{
		printf("״̬ = ����    ");
	}
	else if (Recorder.ucStatus == STA_RECORDING)
	{
		printf("״̬ = ����¼��");
	}
	else if (Recorder.ucStatus == STA_PLAYING)
	{
		printf("״̬ = ���ڻط�");
	}

	sprintf(buf, "  ���� = %d ", Recorder.ucGain);
	printf("%s\n",buf);

	sprintf(buf, "�������� = %d \r", Recorder.ucVolume);
	printf("%s\n",buf);
}


void StartPlay(const char *filename)
{
	printf("��ǰ�����ļ� -> %s\n",filename);
	
	result=f_open(&file, filename, FA_OPEN_EXISTING | FA_READ);
	if(result!=FR_OK)
	{
		printf("����Ƶ�ļ�ʧ��!!!->%d\r\n",result);
		result = f_close (&file);
		Recorder.ucStatus = STA_ERR;
		return;
	}
	//��ȡWAV�ļ�ͷ
	result = f_read(&file,&rec_wav,sizeof(rec_wav),&bw);
	//�ȶ�ȡ��Ƶ���ݵ�������
	result = f_read(&file,(uint16_t *)buffer0,RECBUFFER_SIZE,&bw);
	result = f_read(&file,(uint16_t *)buffer1,RECBUFFER_SIZE,&bw);
	
	HAL_Delay(10);	/* �ӳ�һ��ʱ�䣬�ȴ�I2S�жϽ��� */
	SAI_Play_Stop();			/* ֹͣI2S¼���ͷ��� */
	wm8978_Reset();		/* ��λWM8978����λ״̬ */	
	wm8978_CtrlGPIO1(1);
	Recorder.ucStatus = STA_PLAYING;		/* ����״̬ */

	/* ����WM8978оƬ������ΪDAC�����Ϊ���� */
	wm8978_CfgAudioPath(DAC_ON, EAR_LEFT_ON | EAR_RIGHT_ON);//SPK_ON
	/* ����������������ͬ���� */
	wm8978_SetOUT1Volume(Recorder.ucVolume);
	/* ����WM8978��Ƶ�ӿ�Ϊ�����ֱ�׼I2S�ӿڣ�16bit */
	wm8978_CfgAudioIF(SAI_I2S_STANDARD, 16);
	
	SAIxA_Tx_Config(g_FmtList[Recorder.ucFmtIdx][0],g_FmtList[Recorder.ucFmtIdx][1],g_FmtList[Recorder.ucFmtIdx][2]);
//	I2Sxext_Mode_Config(g_FmtList[Recorder.ucFmtIdx][0],g_FmtList[Recorder.ucFmtIdx][1],g_FmtList[Recorder.ucFmtIdx][2]);
//	
//	I2Sxext_RX_DMA_Init((uint32_t)&recplaybuf[0],(uint32_t)&recplaybuf[1],1);
//	
//  I2Sxext_Recorde_Stop();
	
	SAIA_TX_DMA_Init((uint32_t)buffer0,(uint32_t)buffer1,RECBUFFER_SIZE);		
	SAI_Play_Start();
}

/**
  * @brief  ����WM8978��STM32��I2S��ʼ¼����
  * @param  ��
  * @retval ��
  */
static void StartRecord(const char *filename)
{
	printf("��ǰ¼���ļ� -> %s\n",filename);
	result = f_close (&file);  
	result=f_open(&file,filename,FA_CREATE_ALWAYS|FA_WRITE);
	if(result!=FR_OK)
	{
		printf("Open wavfile fail!!!->%d\r\n",result);
		result = f_close (&file);
		Recorder.ucStatus = STA_ERR;
		return;
	}
	
	// д��WAV�ļ�ͷ���������д��д����ļ�ָ���Զ�ƫ�Ƶ�sizeof(rec_wav)λ�ã�
	// ������д����Ƶ���ݲŷ��ϸ�ʽҪ��
	result=f_write(&file,(const void *)&rec_wav,sizeof(rec_wav),&bw);
	
	HAL_Delay(10);		/* �ӳ�һ��ʱ�䣬�ȴ�I2S�жϽ��� */
	SAI_Rec_Stop();			/* ֹͣI2S¼���ͷ��� */
  SAI_Play_Stop();
	wm8978_Reset();		/* ��λWM8978����λ״̬ */
	wm8978_CtrlGPIO1(0);
	Recorder.ucStatus = STA_RECORDING;		/* ¼��״̬ */
		
	/* ���ڷ���������������ͬ���� */
	wm8978_SetOUT1Volume(Recorder.ucVolume);

	if(Recorder.ucInput == 1)   /* ������ */
	{
		/* ����WM8978оƬ������Ϊ�����룬���Ϊ���� */
		wm8978_CfgAudioPath(LINE_ON | ADC_ON, EAR_LEFT_ON | EAR_RIGHT_ON);
		wm8978_SetLineGain(Recorder.ucGain);
	}
	else   /* MIC���� */
	{
		/* ����WM8978оƬ������ΪMic�����Ϊ���� */
//		wm8978_CfgAudioPath(MIC_LEFT_ON | ADC_ON, EAR_LEFT_ON | EAR_RIGHT_ON);
//		wm8978_CfgAudioPath(MIC_RIGHT_ON | ADC_ON, EAR_LEFT_ON | EAR_RIGHT_ON);
		wm8978_CfgAudioPath(MIC_LEFT_ON | MIC_RIGHT_ON | ADC_ON, EAR_LEFT_ON | EAR_RIGHT_ON);	
		wm8978_SetMicGain(Recorder.ucGain);	
	}
		
	/* ����WM8978��Ƶ�ӿ�Ϊ�����ֱ�׼I2S�ӿڣ�16bit */
	wm8978_CfgAudioIF(SAI_I2S_STANDARD, 16);
   
  
   SAIA_TX_DMA_Init((uint32_t)&recplaybuf[0],(uint32_t)&recplaybuf[1],1);
   __HAL_DMA_DISABLE_IT(&h_txdma,DMA_IT_TC);
   SAIB_RX_DMA_Init((uint32_t)buffer0,(uint32_t)buffer1,RECBUFFER_SIZE);
//  

// 
  SAI_Rec_Start();
  SAI_Play_Start();
  
  
  
}


void RecorderDemo(void)
{
	uint8_t i;
	uint8_t ucRefresh;	/* ͨ�����ڴ�ӡ�����Ϣ��־ */
	DIR dir;

	Recorder.ucStatus=STA_IDLE;    /* ��ʼ����Ϊ����״̬  */
	Recorder.ucInput=0;            /* ȱʡMIC����  */
	Recorder.ucFmtIdx=3;           /* ȱʡ������I2S��׼��16bit���ݳ��ȣ�44K������  */
	Recorder.ucVolume=35;          /* ȱʡ��������  */
	if(Recorder.ucInput==0) //MIC 
	{
		Recorder.ucGain=50;          /* ȱʡMIC����  */
		rec_wav.wChannels=2;         /* ȱʡMIC��ͨ�� */
	}
	else                    //LINE
	{
		Recorder.ucGain=6;           /* ȱʡ��·�������� */
		rec_wav.wChannels=2;         /* ȱʡ��·����˫���� */
	}
	
	rec_wav.riff=0x46464952;       /* ��RIFF��; RIFF ��־ */
	rec_wav.size_8=0;              /* �ļ����ȣ�δȷ�� */
	rec_wav.wave=0x45564157;       /* ��WAVE��; WAVE ��־ */ 
	
	rec_wav.fmt=0x20746d66;        /* ��fmt ��; fmt ��־�����һλΪ�� */
	rec_wav.fmtSize=16;            /* sizeof(PCMWAVEFORMAT) */ 
	rec_wav.wFormatTag=1;          /* 1 ��ʾΪPCM ��ʽ���������� */ 
	/* ÿ����������λ������ʾÿ�������и�������������λ���� */
	rec_wav.wBitsPerSample=16;
	/* ����Ƶ�ʣ�ÿ���������� */
	rec_wav.dwSamplesPerSec=g_FmtList[Recorder.ucFmtIdx][2];
	/* ÿ������������ֵΪͨ������ÿ������λ����ÿ����������λ���� 8�� */
	rec_wav.dwAvgBytesPerSec=rec_wav.wChannels*rec_wav.dwSamplesPerSec*rec_wav.wBitsPerSample/8;  
	/* ���ݿ�ĵ����������ֽ���ģ�����ֵΪͨ������ÿ����������λֵ��8�� */
	rec_wav.wBlockAlign=rec_wav.wChannels*rec_wav.wBitsPerSample/8; 
	
	rec_wav.data=0x61746164;       /* ��data��; ���ݱ�Ƿ� */
	rec_wav.datasize=0;            /* �������ݴ�С Ŀǰδȷ��*/

	/*  ���·�������ڣ������ļ���  */
	result = f_opendir(&dir,RECORDERDIR);
	while(result != FR_OK)
	{
		f_mkdir(RECORDERDIR);
		result = f_opendir(&dir,RECORDERDIR);
	}	

  SAI_GPIO_Config();
  //SAI_Play_Stop();
  SAIxA_Tx_Config(g_FmtList[Recorder.ucFmtIdx][0],g_FmtList[Recorder.ucFmtIdx][1],g_FmtList[Recorder.ucFmtIdx][2]);
	SAIxB_Rx_Config(g_FmtList[Recorder.ucFmtIdx][0],g_FmtList[Recorder.ucFmtIdx][1],g_FmtList[Recorder.ucFmtIdx][2]);
  //SAI_DMA_TX_Callback=MusicPlayer_SAI_DMA_TX_Callback;
  ucRefresh = 1;
	bufflag=0;
	Isread=0;
  
	/* ����������ѭ���� */
	while (1)
	{
		/* ���ʹ�ܴ��ڴ�ӡ��־���ӡ�����Ϣ */
		if (ucRefresh == 1)
		{						
			DispStatus();		/* ��ʾ��ǰ״̬��Ƶ�ʣ������� */
			ucRefresh = 0;
		}
		if(Recorder.ucStatus == STA_IDLE)
		{				
			/*  KEY2��ʼ¼��  */
			if(Key_Scan(KEY2_GPIO_PORT,KEY2_PIN)==KEY_ON)
			{
				/* Ѱ�Һ����ļ��� */
				for(i=1;i<0xff;++i)
				{
					sprintf(recfilename,"0:/recorder/rec%03d.wav",i);
					result=f_open(&file,(const TCHAR *)recfilename,FA_READ);
					if(result==FR_NO_FILE)break;					
				}
				f_close(&file);
				
				if(i==0xff)
				{
					Recorder.ucStatus =STA_ERR;
					continue;
				}
				/* ��ʼ¼�� */
				StartRecord(recfilename);
				ucRefresh = 1;
			}
			/*  TouchPAD��ʼ�ط�¼��  */
			if(TPAD_Scan(0)==1)
			{			
				/* ��ʼ�ط� */
        StartPlay(recfilename);
				ucRefresh = 1;				
			}
		}
		else
		{			
			/*  KEY1ֹͣ¼����ط�  */
			if(Key_Scan(KEY1_GPIO_PORT,KEY1_PIN)==KEY_ON)
			{
				/* ����¼������Ҫ��WAV�ļ������������ */
				if(Recorder.ucStatus == STA_RECORDING)
				{

					rec_wav.size_8=wavsize+36;
					rec_wav.datasize=wavsize;
					result=f_lseek(&file,0);
					result=f_write(&file,(const void *)&rec_wav,sizeof(rec_wav),&bw);
					result=f_close(&file);
					printf("¼������\r\n");
				}
				ucRefresh = 1;
				Recorder.ucStatus = STA_IDLE;		/* ����״̬ */
				SAI_Rec_Stop();
				SAI_Play_Stop();		/* ֹͣI2S¼���ͷ��� */
				wm8978_Reset();	/* ��λWM8978����λ״̬ */
			}
		}
		/* DMA������� */
		if(Isread==1)
		{
			Isread=0;
			switch (Recorder.ucStatus)
			{
				case STA_RECORDING:  // ¼�����ܣ�д�����ݵ��ļ�  
              
						if(bufflag==0){
              SCB_InvalidateDCache_by_Addr((uint32_t*)buffer0, RECBUFFER_SIZE*2);
              result=f_write(&file,buffer0,RECBUFFER_SIZE*2,(UINT*)&bw);//д���ļ�
            }              
						else{  
              SCB_InvalidateDCache_by_Addr((uint32_t*)buffer1, RECBUFFER_SIZE*2);
              result=f_write(&file,buffer1,RECBUFFER_SIZE*2,(UINT*)&bw);//д���ļ�.         
            }
						wavsize+=RECBUFFER_SIZE*2;	
					break;
				case STA_PLAYING:   // �طŹ��ܣ���ȡ���ݵ����Ż�����
						if(bufflag==0){
							result = f_read(&file,buffer0,RECBUFFER_SIZE*2,&bw);
            }
						else
            {
							result = f_read(&file,buffer1,RECBUFFER_SIZE*2,&bw);

            }
						/* ������ɻ��ȡ����ֹͣ���� */
						if((result!=FR_OK)||(file.fptr==file.fsize))
						{
							printf("��������߶�ȡ�����˳�...->%d\r\n", result);
							SAI_Play_Stop();
							file.fptr=0;
							f_close(&file);
							Recorder.ucStatus = STA_IDLE;		/* ����״̬ */
							SAI_Play_Start();		/* ֹͣI2S¼���ͷ��� */
							wm8978_Reset();	/* ��λWM8978����λ״̬ */							
						}		
					break;
			}			
		}
		
	}

  
}


/* DMA��������жϻص����� */
/* �����������Ѿ�������ɣ���Ҫ�л��������������»��������ݲ��� 
   ͬʱ��ȡWAV�ļ�������䵽�Ѳ�������  */
void MusicPlayer_SAI_DMA_TX_Callback(void)
{
	if(Recorder.ucStatus == STA_PLAYING)
	{
		if(DMA_Instance->CR&(1<<19)) //��ǰʹ��Memory1����
		{
			bufflag=0;                       //���Խ����ݶ�ȡ��������0
		}
		else                               //��ǰʹ��Memory0����
		{
			bufflag=1;                       //���Խ����ݶ�ȡ��������1
		}
		Isread=1;                          // DMA������ɱ�־
	}
}
void MusicPlayer_SAI_DMA_RX_Callback(void)
{
	if(Recorder.ucStatus == STA_RECORDING)
	{
		if(DMA1_Stream3->CR&(1<<19)) //��ǰʹ��Memory1����
		{
			bufflag=0;                       //���Խ����ݶ�ȡ��������0
		}
		else                               //��ǰʹ��Memory0����
		{
			bufflag=1;                       //���Խ����ݶ�ȡ��������1
		}
		Isread=1;                          // DMA������ɱ�־
	}
}
///***************************** (END OF FILE) *********************************/
