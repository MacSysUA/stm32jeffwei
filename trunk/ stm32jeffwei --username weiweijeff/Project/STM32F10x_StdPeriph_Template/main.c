/**
  ******************************************************************************
  * @file    GPIO/IOToggle/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body.
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
#include "stm32f10x_conf.h"
#include <stdio.h>
#include "stm3210e_eval_lcd.h"
#include "fatfs.h"
#include "ff.h"
#include <assert.h>
#include "queue.h"

/**
  * @brief  Sets System clock frequency to 72MHz and configure HCLK, PCLK2 
  *         and PCLK1 prescalers. 
  * @param  None
  * @retval None
  */



uint8_t rx_buffer1=0;
uint8_t rx_buffer2=0;
#define USART1_Tx_DMA_Channel DMA1_Channel4
#define USART2_Tx_DMA_Channel DMA1_Channel7
#define USART1_DR_Base ((uint32_t)0x40013804)
#define USART2_DR_Base ((uint32_t)0x40004404)

void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void NVIC_Configuration(void);
void SetSysClockTo72(void);

void USART1_Init();

void USART2_Init();

void delay(void)//延时函数，流水灯显示用
{
 unsigned int i;
 for(i=0;i<0xFFFFF;i++);
}
/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
char *str;
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     
       
  SetSysClockTo72();
  NVIC_Configuration();
  USART1_Init();
  USART2_Init();
  get_disk_info();



#if 1
  list_file();
//  str=read_file("/","test.txt",0,32);
  printf("\n\r");
//  printf(str);
//  delete_file("hello.txt");
//  creat_file("hello.txt");  
//  delete_file("/","hello.txt");
  creat_file("hello.txt");
//  delay();
  edit_file("/","hello.txt","creat_file is ok!",0x00);
  str=read_file("/","hello.txt",0,32);
  printf("\n\r");
  printf(str);
  
#endif
  printf("\n\r");
  list_file();
  
  
  LCD_DeInit();
  STM3210E_LCD_Init();
  LCD_Clear(LCD_COLOR_BLUE);
  LCD_DrawLine( 100,  150,  10,  LCD_DIR_HORIZONTAL);
  LCD_DisplayChar(0,0,'F');
  LCD_DisplayStringLine(200,"hello");
  
  
  /* Enable USART1 DMA TX request */
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

  /* Enable USART2 DMA TX request */
  USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);

  /* Enable the USART1 Receive Interrupt */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  
  /* Enable the USART2 Receive Interrupt */
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  
  /* Enable USART1 DMA TX Channel */
  DMA_Cmd(USART1_Tx_DMA_Channel, ENABLE);

  /* Enable USART2 DMA TX Channel */
  DMA_Cmd(USART2_Tx_DMA_Channel, ENABLE);

 
  
  /* GPIOF Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);

  /* Configure PF7 in output pushpull mode */
  
  GPIO_InitTypeDef GPIO_InitStructure;

  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  
    /* To achieve GPIO toggling maximum frequency, the following  sequence is mandatory. 
     You can monitor PD0 or PD2 on the scope to measure the output signal. 
     If you need to fine tune this frequency, you can add more GPIO set/reset 
     cycles to minimize more the infinite loop timing.
     This code needs to be compiled with high speed optimization option.  */
  
  while (1)
  {
    /* Set PF7 */
    GPIOF->BSRR = 0x00000080;
    USART_SendData(USART2, 0x50);
//    printf("LED ON\n\r");
    delay();
    /* Reset PF7 */
    GPIOF->BRR  = 0x00000080;
    USART_SendData(USART2, 0x51);
//    printf("LED OFF\n\r");
    delay();
  }
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
  {
  }
}

#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/


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


void SetSysClockTo72(void)
{
  
  ErrorStatus HSEStartUpStatus;
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/   
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if (HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
 
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);
    
    RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
    
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); 

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock configuration.
       User can add here some code to deal with this error */    

    /* Go to infinite loop */
    while (1)
    {
    }
  }
}


void USART1_Init()
{
  /* USARTx configured as follow:
        - BaudRate = 9600 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;


  GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

  /* Enable UART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init( GPIOA, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(USART1, &USART_InitStructure);

  /*********************************************/
  //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  /*********************************************/

  /* Enable USART */
  USART_Cmd(USART1, ENABLE);
  USART_GetITStatus( USART1,  USART_IT_TC);
}

void USART2_Init()
{
  /* USARTx configured as follow:
        - BaudRate = 9600 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;


  GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

  /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init( GPIOA, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(USART2, &USART_InitStructure);

  /*********************************************/
  //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  /*********************************************/

  /* Enable USART */
  USART_Cmd(USART2, ENABLE);
  USART_GetITStatus( USART2,  USART_IT_TC);
}

/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
   NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the USART2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configures the DMA.
  * @param  None
  * @retval None
  */
void DMA_Configuration(void)
{
  DMA_InitTypeDef DMA_InitStructure;

  /* USART1_Tx_DMA_Channel (triggered by USART1 Tx event) Config */
  DMA_DeInit(USART1_Tx_DMA_Channel);
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &rx_buffer2;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(USART1_Tx_DMA_Channel, &DMA_InitStructure);
  
  /* USART2_Tx_DMA_Channel (triggered by USART2 Tx event) Config */
  DMA_DeInit(USART2_Tx_DMA_Channel);
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART2_DR_Base;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &rx_buffer1;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_Init(USART2_Tx_DMA_Channel, &DMA_InitStructure);
}

void USART2_IRQHandler(void)
{
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */    
    USART_SendData(USART1, USART_ReceiveData(USART2));
  }
}

/**
  * @brief  This function handles USART3 global interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
  USART_SendData(USART2, USART_ReceiveData(USART1));
  while(USART_GetITStatus(USART1, USART_IT_TC)!=1);
  USART_ClearITPendingBit(USART1,USART_IT_RXNE);    
}