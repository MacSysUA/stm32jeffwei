/**
  ******************************************************************************
  * @file    SDIO/uSDCard/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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
#include "stm32f10x_it.h"
#include "stm32_eval_sdio_sd.h"
#include "stdio.h"
#include "fatfs.h"
#include "stm32f10x_adc.h"
#include "stm3210e_eval_lcd.h"
#include "touch.h"
#include <stdlib.h>

uint8_t tp_flag=0;
uint16_t tp_x[5],tp_y[5],x,y;

uint32_t timer=0;
uint8_t last_data1_flag=0;    
extern uint8_t rx_buffer1a[512];
extern uint8_t rx_buffer1b[512];
extern uint8_t rx_buffer2a[512];
extern uint8_t rx_buffer2b[512];
extern uint8_t *busy_buffer1,*busy_buffer2,*free_buffer1,*free_buffer2;
extern void USART1_Rx_DMA_Config(void);
extern void TIM2_Config(void);
extern void TIM3_Config(void);
extern uint32_t file1_index;
extern vu16 ADCConvertedValue;
/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup SDIO_uSDCard
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void TP_EXTI_DISABLE(void);
void TP_EXTI_ENABLE(void);


void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles SDIO global interrupt request.
  * @param  None
  * @retval None
  */
void SDIO_IRQHandler(void)
{
  /* Process All SDIO Interrupt Sources */
  SD_ProcessIRQSrc();
}

/**
  * @brief  This function handles USART2 global interrupt request.
  * @param  None
  * @retval None
  */

/**
  * @brief  This function handles USART1 global interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{ 
  last_data1_flag=1;
  USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);  
  USART_ClearITPendingBit(USART1,USART_IT_RXNE);
//  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  //TIM_Cmd(TIM2, ENABLE);
//  GPIO_WriteBit(GPIOF, GPIO_Pin_6, (BitAction)0x01);
//  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//  TIM2->CNT = 0 ;
}

void USART2_IRQHandler(void)
{
  USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);  
  USART_ClearITPendingBit(USART2,USART_IT_RXNE);
  TIM_Cmd(TIM3, ENABLE);
}
void TIM2_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM2, TIM_IT_Update)!=RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
//    TIM_Cmd(TIM2, DISABLE);
//    GPIO_WriteBit(GPIOF, GPIO_Pin_6, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOF, GPIO_Pin_6)));
//    GPIO_WriteBit(GPIOF, GPIO_Pin_6, (BitAction)0x00);
#if 0
    if(last_data1_flag)
    {
      uint8_t n=512-DMA_GetCurrDataCounter(DMA1_Channel5);
      write_buffer("test.txt", busy_buffer1,n, file1_index);
      file1_index+=n;
      last_data1_flag=0;
      printf("\n\r-%u-\n\r",file1_index);
    }
#endif
    if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10)==0)
    {
      TIM_Cmd(TIM3, ENABLE);
    }    
    
  }
   
}
void TIM3_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  
  if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10)==0)
  {
    if(tp_flag<4)
    {
      tp_x[tp_flag]=TP_MeasureX();
      tp_y[tp_flag]=TP_MeasureY();
      if((-10<(tp_x[tp_flag]-tp_x[tp_flag-1])<10)&&(-10<(tp_y[tp_flag]-tp_y[tp_flag-1])<10))
      {tp_flag+=1;}
      else
      {tp_flag=0;}
    }
    else
    {
      tp_flag=0;
      TIM_Cmd(TIM3, DISABLE);
     
      u16 j=0,i=0;
      uint16_t sum_x=0,sum_y=0,temp=0;
#if 1 
      for(j=0;j<=4;j++)
      {
        for (i=0;i<4-j;i++)
        {
          if (tp_x[i]>tp_x[i+1])
          {
            temp=tp_x[i];
            tp_x[i]=tp_x[i+1];
            tp_x[i+1]=temp;
          }
        }
      }
      for(j=0;j<=4;j++)
      {
        for (i=0;i<4-j;i++)
        {
          if (tp_y[i]>tp_y[i+1])
          {
            temp=tp_y[i];
            tp_y[i]=tp_y[i+1];
            tp_y[i+1]=temp;
          }
        }
      }
#endif      
      x=(tp_x[1]+tp_x[2]+tp_x[3])/3;
      y=(tp_y[1]+tp_y[2]+tp_y[3])/3;
      printf("x=%u\ty=%u\r\n",x,y);
     if((x<800)&&(y<479))
     {
       LCD_DrawFullCircle(x,y,3,1,0xaaaa);
     }
    }
  }
}


void DMA1_Channel5_IRQHandler(void)
{
  if(DMA_GetITStatus(DMA1_IT_TC5)!=RESET)
  {
    TIM2->CNT=0x00;
    if(busy_buffer1==rx_buffer1a)
    {
      busy_buffer1=rx_buffer1b;
      free_buffer1=rx_buffer1a;
    }
    else
    {
      busy_buffer1=rx_buffer1a;
      free_buffer1=rx_buffer1b;
    }
//    GPIO_WriteBit(GPIOF, GPIO_Pin_9, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOF, GPIO_Pin_9)));
    DMA_ClearITPendingBit(DMA1_IT_GL5);
    USART1_Rx_DMA_Config();
    write_buffer("test.txt", free_buffer1,512, file1_index);
    file1_index+=512;
    timer=TIM2->CNT;
  }
}

void DMA1_Channel6_IRQHandler(void)
{
  GPIO_WriteBit(GPIOF, GPIO_Pin_9, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOF, GPIO_Pin_9)));
}


void DMA1_Channel1_IRQHandler(void)
{
  if(DMA_GetITStatus(DMA1_IT_TC1)!=RESET)
  {
    printf("\n\r%u",ADCConvertedValue);
  }
}

void ADC1_2_IRQnHandler(void)
{
//  if(ADC_GetITStatus(ADC1,ADC_IT_EOC)!=RESET)
  {
//    ADC_ClearITPendingBit(ADC1,ADC_IT_EOC);
//    printf("\n\r%u",ADC_GetConversionValue(ADC1));
  }
}

//�ⲿ�жϷ������
void EXTI15_10_IRQHandler(void)
{
  TP_EXTI_DISABLE();
  EXTI_ClearITPendingBit(EXTI_Line10);//����жϱ�־λ   
  printf("%u,%u\r\n",TPReadX(),TPReadY());
//  DrawPixel(GUI_TOUCH_X_MeasureX(),GUI_TOUCH_X_MeasureY(),0xaaaa);
//printf("%d\t%d\r",GUI_TOUCH_X_MeasureX(),GUI_TOUCH_X_MeasureY());
//printf("hello\n\t");
  TP_EXTI_ENABLE();    
}




void TP_EXTI_DISABLE(void)
{
  (*((uint32_t*)0x40010400))=(*((uint32_t*)0x40010400))&~(1<<10);
}
void TP_EXTI_ENABLE(void)
{
  (*((uint32_t*)0x40010400))=(*((uint32_t*)0x40010400))|1<<10;
}


/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/