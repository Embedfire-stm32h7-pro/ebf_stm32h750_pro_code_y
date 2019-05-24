#ifndef __RECORD_H__
#define __RECORD_H__

#include <inttypes.h>

/* RECBUFFER_SIZE����������������С��ʵ��ÿ��������ռ��RAM�ռ��ֽ���Ϊ��RECBUFFER_SIZE*2
 * ����ѡ��I2S_AudioFreq_16k������ʹ��RECBUFFER_SIZEΪ1024*4���Ϳɱ�֤¼��������Ч��������
 * ���ѡ���������߲�������Ҫ�ʵ�������RECBUFFER_SIZE��С�����統ѡ����I2S_AudioFreq_44k
 * ������RECBUFFER_SIZEΪ1024*8
 * ���⣬SD������Ķ�д�ٶ�Ҳ��һ��������Ӱ��¼��Ч��
 */
#define RECBUFFER_SIZE  1024*8

/* ¼����״̬ */
enum
{
	STA_IDLE = 0,	/* ����״̬ */
	STA_RECORDING,	/* ¼��״̬ */
	STA_PLAYING,	/* ����״̬ */
	STA_ERR,			/*  error  */
};

typedef struct
{
	uint8_t ucInput;			/* ����Դ��0:MIC, 1:������ */
	uint8_t ucFmtIdx;			/* ��Ƶ��ʽ����׼��λ��������Ƶ�� */
	uint8_t ucVolume;			/* ��ǰ�������� */
	uint8_t ucGain;			  /* ��ǰ���� */	
	uint8_t ucStatus;			/* ¼����״̬��0��ʾ������1��ʾ¼���У�2��ʾ������ */	
}REC_TYPE;	

/* WAV�ļ�ͷ��ʽ */
typedef __packed struct
{ 
	uint32_t	riff;							/* = "RIFF"	0x46464952*/
	uint32_t	size_8;						/* ���¸���ַ��ʼ���ļ�β�����ֽ���	*/
	uint32_t	wave;							/* = "WAVE" 0x45564157*/
	
	uint32_t	fmt;							/* = "fmt " 0x20746d66*/
	uint32_t	fmtSize;					/* ��һ���ṹ��Ĵ�С(һ��Ϊ16) */
	uint16_t	wFormatTag;				/* ���뷽ʽ,һ��Ϊ1	*/
	uint16_t	wChannels;				/* ͨ������������Ϊ1��������Ϊ2 */
	uint32_t	dwSamplesPerSec;	/* ������ */
	uint32_t	dwAvgBytesPerSec;	/* ÿ���ֽ���(= ������ �� ÿ���������ֽ���) */
	uint16_t	wBlockAlign;			/* ÿ���������ֽ���(=����������/8*ͨ����) */
	uint16_t	wBitsPerSample;		/* ����������(ÿ��������Ҫ��bit��) */
																			  
	uint32_t	data;							/* = "data" 0x61746164*/
	uint32_t	datasize;					/* �����ݳ��� */
} WavHead;	
void StartPlay(const char *filename);
void RecorderDemo(void);
void MusicPlayer_I2S_DMA_TX_Callback(void);
void Recorder_I2S_DMA_RX_Callback(void);
void MusicPlayer_SAI_DMA_TX_Callback(void);
#endif  /* __RECORD_H__   */

