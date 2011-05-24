

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
		printf("\r\n挂载文件系统失败,错误代码: %u",res);
		return;
	}	
	res = f_opendir(&dirs, (const TCHAR*) "/");
	if (res == FR_OK)
	{
          printf("\r\n------------文件列表------------");
		while ((f_readdir(&dirs, &finfo) == FR_OK) && finfo.fname[0])
		{
			i = strlen(finfo.fname);
			if (finfo.fattrib & AM_DIR)//如果是文件夹
			{
				files_num++;
				printf("\r\n/%s", &finfo.fname[0]);
				switch(i)//作用：输出文件名左对齐
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
		printf("\r\n打开根目录失败!");
		printf("\r\n错误代码: %u",res);
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
		if( files_num==0 )//无文件
		{
			printf("\r\n无文件!");
		}
	}
	else
	{
		printf("\r\n打开根目录失败!");
		printf("\r\n错误代码: %u",res);
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
//        printf("f_mount---%u---",res);
	if (res != FR_OK)
	{
		printf("\r\n挂载文件系统失败,错误代码: %u",res);
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
		printf("\r\n获得磁盘信息失败!");
		printf("\r\n错误代码: %u",res);
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
		printf("\r\n挂载文件系统失败,错误代码: %u",res);
		return;
	}	
	printf("\r\n正在格式化磁盘,请稍候...");
	res = f_mkfs(drv,partition,allocsize);
	if (res == FR_OK)
	{
		printf("\r\n格式化成功...");
	}
	else
	{
		printf("\r\n格式化失败...");
		printf("\r\n错误代码: %u",res);
	}
	f_mount(0,NULL);
}

/************************************************************************/
//函数名称:char *read_file(char *dir,char *p,int n,int k)
//用途:读取文件夹*dir下文件n字节开始的k字节数据,k<512;
//返回:读出数据缓存的指针
//用法:xxx=read_file("/book","小说.txt",100,256)

/************************************************************************/

char *read_file(const TCHAR *dir,const TCHAR *file_name,int offset,int length)
{

  FATFS fs;
  FIL	file;
  FRESULT res;
//  DIR dirs;
//  FILINFO finfo;
  uint32_t re;
  res = f_mount(0,&fs);
  res = f_chdir(dir);
  res = f_open(&file,file_name,FA_READ);
  res = f_lseek (&file, offset);
  res = f_read(&file,buffer,length,&re);
  f_close(&file);
  f_mount(0,NULL);
  return buffer;
}


void creat_file(const TCHAR *file_name)
{
	FIL file;
	FIL *pf = &file;
	FATFS fs;
//	FATFS *fls = &fs;
	uint8_t res;

	
	res = f_mount(0,&fs);
	if (res != FR_OK)
	{
		printf("\r\n挂载文件系统失败,错误代码: %u",res);
		return;
	}	
	res = f_open(pf,file_name,FA_READ | FA_WRITE | FA_CREATE_NEW);
	if (res == FR_OK)
	{
		printf("\r\n创建文件成功!");
		res = f_close(pf);
		if (res != FR_OK)
		{
			printf("\r\n创建文件成功,但关闭文件时,失败!");
			printf("\r\n错误代码: %u",res);				
		}				
	}
	else
	{
		printf("\r\n创建文件失败!");
		printf("\r\n错误代码: %u",res);	
	}
	f_mount(0,NULL);
}

void creat_dir(const TCHAR *dir_name)
{
	FATFS fs;
	FRESULT res;
	res = f_mount(0,&fs);
	if (res != FR_OK)
	{
		printf("\r\n挂载文件系统失败,错误代码: %u",res);
		return;
	}		
	res = f_mkdir(dir_name);
	if (res == FR_OK)
	{
		printf("\r\n创建文件目录成功!");
	}
	else
	{
		printf("\r\n创建目录失败...");
		printf("\r\n错误代码: %u",res);
	}
	f_mount(0,NULL);
}


void delete_file(const TCHAR *dir, const TCHAR *file_name)
{
	FATFS fs;
        DIR dirs;
	FRESULT res;
	res = f_mount(0,&fs);        
	if (res != FR_OK)
	{
		printf("\r\n挂载文件系统失败,错误: %u",res);
		return;
	}
        res = f_opendir(&dirs,(const TCHAR*)"/");
        res = f_chdir(dir);	
	res = f_unlink((TCHAR *)file_name);	
	f_mount(0,NULL);
}

void edit_file(const TCHAR *dir,const TCHAR *write_file,char *write_data,uint32_t index)
{
	FATFS fs;
	FIL	file;
	FRESULT res;
	DIR dirs;
        uint32_t n_write=0x00;
        uint32_t n_written = 0x00;
        for(n_write=0;*write_data++!='\0';)
        {
          n_write++;
        }
        write_data=write_data-n_write-1;
	res = f_mount(0,&fs);
	if (res != FR_OK)
	{
		printf("\r\n挂载文件系统失败,错误代码: %u",res);
		return;
	}
	res = f_opendir(&dirs,(const TCHAR*)"/");
        res = f_chdir(dir);
	res = f_open(&file,write_file,FA_READ | FA_WRITE);
        res = f_lseek (&file, index);
	if (res == FR_OK)
	{
          if(n_write<=512)
          {
            res = f_write(&file,write_data,n_write,&n_written);
            if ((res == FR_OK) && (n_written==n_write))
            {
              f_close(&file);
            }
            else
            {
              printf("\r\n保存数据失败!");
              printf("\r\n错误代码: %u",res);
            }
          }
          else
          {
            while(*write_data!='\0')
            {
              res = f_write(&file,write_data,512,&n_written);
              if ((res == FR_OK) && (n_written==512))
              {
                f_close(&file);
              }
              else
              {
                printf("\r\n保存数据失败!");
                printf("\r\n错误代码: %u",res);
              }
              write_data+=512;
              index+=512;
              res = f_open(&file,write_file,FA_READ | FA_WRITE);
              res = f_lseek (&file, index);
            }
          }
	}
	else
	{
		printf("\r\n打开文件失败,错误代码: %u",res);
	}
        f_mount(0,NULL);
}
