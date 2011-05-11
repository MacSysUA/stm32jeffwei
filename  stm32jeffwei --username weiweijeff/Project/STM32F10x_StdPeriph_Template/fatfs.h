
#ifndef __FATFS_H
#define __FATFS_H

#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include "ff.h"
#include "diskio.h"

void list_file(void);
//char *read_file(const XCHAR *dir,const XCHAR *p,int n,int k);
//void creat_file(const XCHAR *file_name);
//void delete_file(const XCHAR *file_name);
void get_disk_info(void);
void format_disk(
                 BYTE drv,			/* Logical drive number */
                 BYTE partition,		/* Partitioning rule 0:FDISK, 1:SFD */
                 WORD allocsize		/* Allocation unit size [bytes] */
                 );

//void edit_file(const XCHAR *dir,const XCHAR *write_file,char *write_data,uint32_t index);

#endif