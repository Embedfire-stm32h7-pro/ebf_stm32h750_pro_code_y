#ifndef __FTL_H
#define __FTL_H

#include "stm32h7xx.h"
//����˵��
//V1.1 20160124
//�޸�FTL_CopyAndWriteToBlock��FTL_WriteSectors����,��߷�0XFFʱ��д���ٶ�.  
//V1.2 20160520
//1,�޸�FTL_ReadSectors,����ECC�����ж�,��⻵�鴦��,�����Ӷ������,����ٶ�
//2,����FTL_BlockCompare��FTL_SearchBadBlock����,������Ѱ����
//3,�޸�FTL_Format�����ⷽʽ,����FTL_USE_BAD_BLOCK_SEARCH��
//V1.3 20160530
//�޸ĵ�1bit ECC�������ʱ����ȡ2�Σ���ȷ��1bit �����Է�������޸�����
////////////////////////////////////////////////////////////////////////////////// 	

//������������
//�������Ϊ1,����FTL_Format��ʱ��,��Ѱ����,��ʱ��(512M,3��������),�һᵼ��RGB������
#define FTL_USE_BAD_BLOCK_SEARCH		0		//�����Ƿ�ʹ�û�������



uint8_t FTL_Init(void); 
void FTL_BadBlockMark(uint32_t blocknum);
uint8_t FTL_CheckBadBlock(uint32_t blocknum); 
uint8_t FTL_UsedBlockMark(uint32_t blocknum);
uint32_t FTL_FindUnusedBlock(uint32_t sblock,uint8_t flag);
uint32_t FTL_FindSamePlaneUnusedBlock(uint32_t sblock);
uint8_t FTL_CopyAndWriteToBlock(uint32_t Source_PageNum,uint16_t ColNum,uint8_t *pBuffer,uint32_t NumByteToWrite);
uint16_t FTL_LBNToPBN(uint32_t LBNNum); 
uint8_t FTL_WriteSectors(uint8_t *pBuffer,uint32_t SectorNo,uint16_t SectorSize,uint32_t SectorCount);
uint8_t FTL_ReadSectors(uint8_t *pBuffer,uint32_t SectorNo,uint16_t SectorSize,uint32_t SectorCount);
uint8_t FTL_CreateLUT(uint8_t mode);
uint8_t FTL_BlockCompare(uint32_t blockx,uint32_t cmpval);
uint32_t FTL_SearchBadBlock(void);
uint8_t FTL_Format(void); 
#endif

