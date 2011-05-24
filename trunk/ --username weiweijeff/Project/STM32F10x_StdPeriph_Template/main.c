/**
  ******************************************************************************
  * @file    SDIO/uSDCard/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "usart_init.h"
#include "stdio.h"
#include "ff.h"
#include "stdio.h"
#include "stm32_eval_sdio_sd.h"
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}


SD_Error Status = SD_OK;

uint8_t buffer[512];
uint16_t i=0;
int main()
{
  USART1_Init();
//  printf("\n\r-hello-\n\r");
  Status=SD_Init();
  if(Status == SD_OK)
  {
    printf("\n\r-SD_OK-\n\r");
  }
  else
  {
    printf("\n\r-SD_FAILED-\n\r");
  }
  for(i=0;i<512;i++);
  {
    buffer[i]=0;
  }
//  Status=SD_ReadBlock(buffer,512,512);
//  Status=SD_ReadMultiBlocks(buffer,  512,  512,  2);
  Status=SD_WriteBlock( buffer,  0,  512);
  if(Status == SD_OK)
  {
    printf("\n\r-SD_ReadBlock_OK-\n\r");
  }
  else
  {
    printf("\n\r-SD_ReadBlock_FAILED-\n\r");
  }
  
#if 0
  FATFS fs;
	FATFS *fls = &fs;
	FRESULT res;
	DWORD clust,tot_sect,fre_sect;	
	
	res = f_mount(0,&fs);
        printf("\n\rf_mount---%u---\n\r",res);
	if (res != FR_OK)
	{
		printf("\r\n挂载文件系统失败,错误代码: %u",res);
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
#endif
        
  while(1);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
