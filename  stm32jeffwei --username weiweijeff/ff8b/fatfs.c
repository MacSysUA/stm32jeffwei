

#include "fatfs.h"
#include "ff.h"
#include "stdlib.h"
char buffer[512]={0,0,0,0};

void list_file(void)
{
        FATFS fs;
	FILINFO finfo;
	FRESULT res;
	DIR dirs;
	int i;
	int files_num=0;
	res = f_mount(0,&fs);
	if (res != FR_OK)
	{
		printf("\r\n�����ļ�ϵͳʧ��,�������: %u",res);
		return;
	}	
	res = f_opendir(&dirs, (const TCHAR*) "/");
	if (res == FR_OK)
	{
          printf("\r\n------------�ļ��б�------------");
		while ((f_readdir(&dirs, &finfo) == FR_OK) && finfo.fname[0])
		{
			i = strlen(finfo.fname);
			if (finfo.fattrib & AM_DIR)//������ļ���
			{
				files_num++;
				printf("\r\n/%s", &finfo.fname[0]);
				switch(i)//���ã�����ļ��������
				{
				case 1:printf(" ");
				case 2:printf(" ");
				case 3:printf(" ");
				case 4:printf(" ");
				case 5:printf(" ");
				case 6:printf(" ");
				case 7:printf(" ");
				case 8:printf("%15s"," ");
				}

				
			}
			else
			{
				continue;
			}	
		}

	}
	else
	{
		printf("\r\n�򿪸�Ŀ¼ʧ��!");
		printf("\r\n�������: %u",res);
	}
	res = f_opendir(&dirs, (const TCHAR*)"/");



	if (res == FR_OK)
	{
		//i = strlen(path);
		
		while ((f_readdir(&dirs, &finfo) == FR_OK) && finfo.fname[0])
		{
			if (finfo.fattrib & AM_DIR)
			{
				continue;
			}
			else
			{
				files_num++;				
				printf("\r\n/.%12s%7ld KB ",  &finfo.fname[0],(finfo.fsize+512)/1024);				
			}
		}
		if( files_num==0 )//���ļ�
		{
			printf("\r\n���ļ�!");
		}
	}
	else
	{
		printf("\r\n�򿪸�Ŀ¼ʧ��!");
		printf("\r\n�������: %u",res);
	}
        printf("\n\r");
	f_mount(0,NULL);

}

void get_disk_info(void)
{
	FATFS fs;
	FATFS *fls = &fs;
	FRESULT res;
	DWORD clust,tot_sect,fre_sect;	
	
	res = f_mount(0,&fs);
        printf("f_mount---%u---",res);
	if (res != FR_OK)
	{
		printf("\r\n�����ļ�ϵͳʧ��,�������: %u",res);
		return;
	}	

	res = f_getfree((const TCHAR*)"0:",&clust,&fls);
	if (res == FR_OK)
	{
		tot_sect = (fls->n_fatent - 2) * fls->csize;
		fre_sect = clust * fls->csize;

		printf("\r\nfree space in unit of KB (assuming 512B/sector)");
		printf("\r\n%lu KB total drive space.\r\n"
			"%lu KB available.",
			fre_sect / 2, tot_sect / 2);
	}
	else
	{
		printf("\r\n��ô�����Ϣʧ��!");
		printf("\r\n�������: %u",res);
	}
	
	f_mount(0,NULL);

}

void format_disk(
                 BYTE drv,			/* Logical drive number */
                 BYTE partition,		/* Partitioning rule 0:FDISK, 1:SFD */
                 WORD allocsize		/* Allocation unit size [bytes] */
                 )
{
	FATFS fs;
//	FATFS *fls = &fs;
	uint8_t res;
	res = f_mount(0,&fs);
	if (res != FR_OK)
	{
		printf("\r\n�����ļ�ϵͳʧ��,�������: %u",res);
		return;
	}	
	printf("\r\n���ڸ�ʽ������,���Ժ�...");
	res = f_mkfs(drv,partition,allocsize);
	if (res == FR_OK)
	{
		printf("\r\n��ʽ���ɹ�...");
	}
	else
	{
		printf("\r\n��ʽ��ʧ��...");
		printf("\r\n�������: %u",res);
	}
	f_mount(0,NULL);
}


