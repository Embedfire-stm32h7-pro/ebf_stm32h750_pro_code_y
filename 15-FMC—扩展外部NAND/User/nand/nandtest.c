#include "./nand/nandtest.h"
#include "./nand/bsp_nand.h"
#include "./nand/ftl.h"
#include "string.h"
#include "./malloc/malloc.h"


//��NANDĳһҳд��ָ����С������
//pagenum:Ҫд���ҳ��ַ
//colnum:Ҫд��Ŀ�ʼ�е�ַ(ҳ�ڵ�ַ)
//writebytes:Ҫд������ݴ�С��MT29F16G���Ϊ4320��MT29F4G���Ϊ2112
uint8_t test_writepage(uint32_t pagenum,uint16_t colnum,uint16_t writebytes)
{
	uint8_t *pbuf;
	uint8_t sta=0;
    uint16_t i=0;
	pbuf=mymalloc(SRAMEX,5000);  
    for(i=0;i<writebytes;i++)//׼��Ҫд�������,�������,��0��ʼ����
    { 
        pbuf[i]=i;	
    }
	sta=NAND_WritePage(pagenum,colnum,pbuf,writebytes);	//��nandд������	
	myfree(SRAMEX,pbuf);	//�ͷ��ڴ�
	return sta;
}

//��ȡNANDĳһҳָ����С������
//pagenum:Ҫ��ȡ��ҳ��ַ
//colnum:Ҫ��ȡ�Ŀ�ʼ�е�ַ(ҳ�ڵ�ַ)
//readbytes:Ҫ��ȡ�����ݴ�С��MT29F16G���Ϊ4320��MT29F4G���Ϊ2112
uint8_t test_readpage(uint32_t pagenum,uint16_t colnum,uint16_t readbytes)
{
	uint8_t *pbuf;
	uint8_t sta=0;
    uint16_t i=0;
	pbuf=mymalloc(SRAMEX,5000);  
	sta=NAND_ReadPage(pagenum,colnum,pbuf,readbytes);	//��ȡ����
	
	if(sta==0||sta==NSTA_ECC1BITERR||sta==NSTA_ECC2BITERR)//��ȡ�ɹ�
	{ 
		printf("read page data is:\r\n");
		for(i=0;i<readbytes;i++)	 
		{ 
			printf("%d ",pbuf[i]);  //���ڴ�ӡ��ȡ��������
		}
		printf("\r\nend\r\n");
	}
	myfree(SRAMEX,pbuf);	//�ͷ��ڴ�
	return sta;
}

//��һҳ���ݿ���������һҳ,��д��һ��������.
//ע��:Դҳ��Ŀ��ҳҪ��ͬһ��Plane�ڣ�(ͬΪ����/ͬΪż��)
//spnum:Դҳ��ַ
//epnum:Ŀ��ҳ��ַ
//colnum:Ҫд��Ŀ�ʼ�е�ַ(ҳ�ڵ�ַ)
//writebytes:Ҫд������ݴ�С�����ܳ���ҳ��С
uint8_t test_copypageandwrite(uint32_t spnum,uint32_t dpnum,uint16_t colnum,uint16_t writebytes)
{
	uint8_t *pbuf;
	uint8_t sta=0;
    uint16_t i=0;
	pbuf=mymalloc(SRAMEX,5000);  
    for(i=0;i<writebytes;i++)//׼��Ҫд�������,�������,��0X80��ʼ����
    { 
        pbuf[i]=i+0X80;	
    }
	sta=NAND_CopyPageWithWrite(spnum,dpnum,colnum,pbuf,writebytes);	//��nandд������	
	myfree(SRAMEX,pbuf);	//�ͷ��ڴ�
	return sta;
}
 
//��ȡNANDĳһҳSpare��ָ����С������
//pagenum:Ҫ��ȡ��ҳ��ַ
//colnum:Ҫ��ȡ��spare����ʼ��ַ
//readbytes:Ҫ��ȡ�����ݴ�С��MT29F16G���Ϊ64��MT29F4G���Ϊ224
uint8_t test_readspare(uint32_t pagenum,uint16_t colnum,uint16_t readbytes)
{
	uint8_t *pbuf;
	uint8_t sta=0;
    uint16_t i=0;
	pbuf=mymalloc(SRAMEX,512); 
	memset(pbuf,0,512);	
	sta=NAND_ReadSpare(pagenum,colnum,pbuf,readbytes);	//��ȡ����
	if(sta==0)//��ȡ�ɹ�
	{ 
		printf("read spare data is:\r\n");
		for(i=0;i<readbytes;i++)	 
		{ 
			printf("%x ",pbuf[i]);  //���ڴ�ӡ��ȡ��������
		}
		printf("\r\nend\r\n");
	}
	myfree(SRAMEX,pbuf);	//�ͷ��ڴ�
	return sta;
}

//��ָ��λ�ÿ�ʼ,��ȡ����NAND,ÿ��BLOCK�ĵ�һ��page��ǰ5���ֽ�
//sblock:ָ����ʼ��block���
void test_readallblockinfo(uint32_t sblock)
{
    uint8_t j=0;
    uint32_t i=0;
	uint8_t sta;
    uint8_t buffer[5];
    for(i=sblock;i<nand_dev.block_totalnum;i++)
    {
        printf("block %d info:",i);
        sta=NAND_ReadSpare(i*nand_dev.block_pagenum,0,buffer,5);//��ȡÿ��block,��һ��page��ǰ5���ֽ�
        if(sta)printf("failed\r\n");
		for(j=0;j<5;j++)
        {
            printf("%x ",buffer[j]);
        }
        printf("\r\n");
    }	
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//FTL����Դ���

//��ĳ��������ʼ,д��seccnt������������
//secx:��ʼ���������
//secsize:������С
//seccnt:Ҫд�����������
uint8_t test_ftlwritesectors(uint32_t secx,uint16_t secsize,uint16_t seccnt)
{
	uint8_t *pbuf;
	uint8_t sta=0;
    uint32_t i=0;
	pbuf=mymalloc(SRAMEX,secsize*seccnt);  
    for(i=0;i<secsize*seccnt;i++)	//׼��Ҫд�������,�������,��0��ʼ����
    { 
        pbuf[i]=i;	
    }
	sta=FTL_WriteSectors(pbuf,secx,secsize,seccnt);	//��nandд������	
	myfree(SRAMEX,pbuf);	//�ͷ��ڴ�
	return sta;
}


//��ĳ��������ʼ,����seccnt������������
//secx:��ʼ���������
//secsize:������С
//seccnt:Ҫ��ȡ����������
uint8_t test_ftlreadsectors(uint32_t secx,uint16_t secsize,uint16_t seccnt)
{
	uint8_t *pbuf;
	uint8_t sta=0;
    uint32_t i=0;
	pbuf=mymalloc(SRAMEX,secsize*seccnt);
	memset(pbuf,0,secsize*seccnt);	
	sta=FTL_ReadSectors(pbuf,secx,secsize,seccnt);	//��ȡ����
	if(sta==0)
	{
		printf("read sec %d data is:\r\n",secx); 
		for(i=0;i<secsize*seccnt;i++)	//׼��Ҫд�������,�������,��0��ʼ����
		{ 
			printf("%x ",pbuf[i]);  //���ڴ�ӡ��ȡ��������
		}
		printf("\r\nend\r\n");
	}
	myfree(SRAMEX,pbuf);	//�ͷ��ڴ�
	return sta;
}





























