/*
******************************************************************************
* @file    recorder.c
* @author  fire
* @version V1.0
* @date    2019-xx-xx
* @brief   WM8978�������ܲ���+mp3����
******************************************************************************
* @attention
*
* ʵ��ƽ̨:Ұ��  STM32 H750 ������  
* ��̳    :http://www.chuxue123.com
* �Ա�    :http://firestm32.taobao.com
*
******************************************************************************
*/
#include <stdio.h>
#include <string.h>
#include "./usart/bsp_debug_usart.h"
#include "./wm8978/bsp_wm8978.h"
#include "ff.h" 
#include "./mp3Player/mp3Player.h"
#include "mp3dec.h"
#include "./sai/bsp_sai.h" 
/* �Ƽ�ʹ�����¸�ʽmp3�ļ���
 * �����ʣ�44100Hz
 * ��  ����2
 * �����ʣ�320kbps
 */

/* ������������Ƶ����ʱ�������������Ҫ������СΪ2304*16/8�ֽ�(16ΪPCM����Ϊ16λ)��
 * �������Ƕ���MP3BUFFER_SIZEΪ2304��ʵ�����������ΪMP3BUFFER_SIZE*2���ֽ�
 */
#define MP3BUFFER_SIZE  2304		
#define INPUTBUF_SIZE   3000	

static HMP3Decoder		Mp3Decoder;			/* mp3������ָ��	*/
static MP3FrameInfo		Mp3FrameInfo;		/* mP3֡��Ϣ  */
static MP3_TYPE mp3player;         /* mp3�����豸 */
volatile uint8_t Isread=0;           /* DMA������ɱ�־ */
static uint8_t bufflag=0;          /* ���ݻ�����ѡ���־ */

uint32_t led_delay=0;

uint8_t inputbuf[INPUTBUF_SIZE]={0};        /* �������뻺������1940�ֽ�Ϊ���MP3֡��С  */
__attribute__((at(0x30000000))) short outbuffer[2][MP3BUFFER_SIZE];  /* ���������������Ҳ��I2S�������ݣ�ʵ��ռ���ֽ�����RECBUFFER_SIZE*2 */

FIL file;											/* file objects */
FRESULT result; 
UINT bw;            					/* File R/W count */


/**
  * @brief   MP3��ʽ��Ƶ����������
  * @param  ��
  * @retval ��
  */
int times = 0;
void mp3PlayerDemo(const char *mp3file)
{
	uint8_t *read_ptr=inputbuf;
	uint32_t frames=0;
	int err=0, i=0, outputSamps=0;	
	int	read_offset = 0;				/* ��ƫ��ָ�� */
	int	bytes_left = 0;					/* ʣ���ֽ��� */	
	
	mp3player.ucFreq = SAI_AUDIOFREQ_DEFAULT;
	mp3player.ucStatus = STA_IDLE;
	mp3player.ucVolume = 40;
	
	result=f_open(&file,mp3file,FA_READ);
	if(result!=FR_OK)
	{
		printf("Open mp3file :%s fail!!!->%d\r\n",mp3file,result);
		result = f_close (&file);
		return;	/* ֹͣ���� */
	}
	printf("��ǰ�����ļ� -> %s\n",mp3file);
	
	//��ʼ��MP3������
	Mp3Decoder = MP3InitDecoder();	
	if(Mp3Decoder==0)
	{
		printf("��ʼ��helix������豸\n");
		return;	/* ֹͣ���� */
	}
	printf("��ʼ����...\n");
	
	Delay_ms(10);	/* �ӳ�һ��ʱ�䣬�ȴ�I2S�жϽ��� */
	wm8978_Reset();		/* ��λWM8978����λ״̬ */
	/* ����WM8978оƬ������ΪDAC�����Ϊ���� */
	wm8978_CfgAudioPath(DAC_ON, EAR_LEFT_ON | EAR_RIGHT_ON);

	/* ����������������ͬ���� */
	wm8978_SetOUT1Volume(mp3player.ucVolume);

	/* ����WM8978��Ƶ�ӿ�Ϊ�����ֱ�׼I2S�ӿڣ�16bit */
	wm8978_CfgAudioIF(SAI_I2S_STANDARD, 16);
	
	/*  ��ʼ��������I2S  */
  SAI_Play_Stop();
	SAI_GPIO_Config();
//	SAIxA_Tx_Config(SAI_I2S_STANDARD,SAI_PROTOCOL_DATASIZE_16BIT,mp3player.ucFreq);	
//	SAIA_TX_DMA_Init();	
	
	bufflag=0;
	Isread=0;
	
	mp3player.ucStatus = STA_PLAYING;		/* ����״̬ */
  result=f_read(&file,inputbuf,INPUTBUF_SIZE,&bw);
	if(result!=FR_OK)
	{
		printf("��ȡ%sʧ�� -> %d\r\n",mp3file,result);
		MP3FreeDecoder(Mp3Decoder);
		return;
	}
	read_ptr=inputbuf;
	bytes_left=bw;
	/* ����������ѭ���� */
	while(mp3player.ucStatus == STA_PLAYING)
	{
		read_offset = MP3FindSyncWord(read_ptr, bytes_left);	//Ѱ��֡ͬ�������ص�һ��ͬ���ֵ�λ��
		if(read_offset < 0)										//û���ҵ�ͬ����
		{
			result=f_read(&file,inputbuf,INPUTBUF_SIZE,&bw);
			if(result!=FR_OK)
			{
				printf("��ȡ%sʧ�� -> %d\r\n",mp3file,result);
				break;
			}
			read_ptr=inputbuf;
			bytes_left=bw;
			continue;		//����ѭ��2���ص�ѭ��1	
		}
		read_ptr += read_offset;				//ƫ����ͬ���ֵ�λ��
		bytes_left -= read_offset;				//ͬ����֮������ݴ�С	
		if(bytes_left < 1024)							//��������
		{
			/* ע������ط���Ϊ���õ���DMA��ȡ������һ��Ҫ4�ֽڶ���  */
			i=(uint32_t)(bytes_left)&3;			//�ж϶�����ֽ�
			if(i) i=4-i;						//��Ҫ������ֽ�
			memcpy(inputbuf+i, read_ptr, bytes_left);	//�Ӷ���λ�ÿ�ʼ����
			read_ptr = inputbuf+i;										//ָ�����ݶ���λ��
			result = f_read(&file, inputbuf+bytes_left+i, INPUTBUF_SIZE-bytes_left-i, &bw);//��������
			if(result!=FR_OK)
			{
				printf("��ȡ%sʧ�� -> %d\r\n",mp3file,result);
				break;
			}
			bytes_left += bw;		//��Ч��������С
		}
		err = MP3Decode(Mp3Decoder, &read_ptr, &bytes_left, outbuffer[bufflag], 0);	//bufflag��ʼ���� ������mp3����ṹ�塢������ָ�롢��������С�������ָ�롢���ݸ�ʽ
		frames++;	
		if (err != ERR_MP3_NONE)	//������
		{
			switch (err)
			{
				case ERR_MP3_INDATA_UNDERFLOW:
					printf("ERR_MP3_INDATA_UNDERFLOW\r\n");
					result = f_read(&file, inputbuf, INPUTBUF_SIZE, &bw);
					read_ptr = inputbuf;
					bytes_left = bw;
					break;		
				case ERR_MP3_MAINDATA_UNDERFLOW:
					/* do nothing - next call to decode will provide more mainData */
					printf("ERR_MP3_MAINDATA_UNDERFLOW\r\n");
					break;		
				default:
					printf("UNKNOWN ERROR:%d\r\n", err);		
					// ������֡
					if (bytes_left > 0)
					{
						bytes_left --;
						read_ptr ++;
					}	
					break;
			}
			Isread=1;
		}
		else		//�����޴���׼�������������PCM
		{
			MP3GetLastFrameInfo(Mp3Decoder, &Mp3FrameInfo);		//��ȡ������Ϣ				
			/* �����DAC */
			outputSamps = Mp3FrameInfo.outputSamps;							//PCM���ݸ���
			if (outputSamps > 0)
			{
				if (Mp3FrameInfo.nChans == 1)	//������
				{
					//������������Ҫ����һ�ݵ���һ������
					for (i = outputSamps - 1; i >= 0; i--)
					{
						outbuffer[bufflag][i * 2] = outbuffer[bufflag][i];
						outbuffer[bufflag][i * 2 + 1] = outbuffer[bufflag][i];
					}
					outputSamps *= 2;
				}//if (Mp3FrameInfo.nChans == 1)	//������
			}//if (outputSamps > 0)
			
			/* ���ݽ�����Ϣ���ò����� */
			if (Mp3FrameInfo.samprate != mp3player.ucFreq)	//������ 
			{
				mp3player.ucFreq = Mp3FrameInfo.samprate;
				
				printf(" \r\n Bitrate       %dKbps", Mp3FrameInfo.bitrate/1000);
				printf(" \r\n Samprate      %dHz", mp3player.ucFreq);
				printf(" \r\n BitsPerSample %db", Mp3FrameInfo.bitsPerSample);
				printf(" \r\n nChans        %d", Mp3FrameInfo.nChans);
				printf(" \r\n Layer         %d", Mp3FrameInfo.layer);
				printf(" \r\n Version       %d", Mp3FrameInfo.version);
				printf(" \r\n OutputSamps   %d", Mp3FrameInfo.outputSamps);
				printf("\r\n");
				if(mp3player.ucFreq >= SAI_AUDIOFREQ_DEFAULT)	//I2S_AudioFreq_Default = 2��������֡��ÿ�ζ�Ҫ������
				{
          
					SAIxA_Tx_Config(SAI_I2S_STANDARD,SAI_PROTOCOL_DATASIZE_16BIT,mp3player.ucFreq);						//���ݲ������޸�iis����
          SAIA_TX_DMA_Init((uint32_t)(&outbuffer[0]),(uint32_t)&outbuffer[1],outputSamps);
				}

				SAI_Play_Start();
			}
		}//else ��������
		
		if(file.fptr==file.fsize) 		//mp3�ļ���ȡ��ɣ��˳�
		{
			printf("END\r\n");
			break;
		}	

		while(Isread==0)
		{
			led_delay++;
			if(led_delay==0xffffff)
			{
				led_delay=0;
//				LED4_TOGGLE;
			}
			//Input_scan();		//�ȴ�DMA������ɣ��˼�������а���ɨ�輰�����¼�
		}
		Isread=0;
	}
	SAI_Play_Stop();
	mp3player.ucStatus=STA_IDLE;
	MP3FreeDecoder(Mp3Decoder);
	f_close(&file);	
}

/* DMA��������жϻص����� */
/* �����������Ѿ�������ɣ���Ҫ�л��������������»��������ݲ��� 
   ͬʱ��ȡWAV�ļ�������䵽�Ѳ�������  */
void MusicPlayer_SAI_DMA_TX_Callback(void)
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

/***************************** (END OF FILE) *********************************/
