/******************************************************************************
* 这是STM32的SD_FAT模块演示程序
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "includes.h"
#include <stdio.h>
#include "ili9320.h"
#include "touch.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
time_t current_time;
struct tm time_now;

vu32 TimeDisplay;

u8 state;

FATFS fs;
FRESULT res;
UINT br;


/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void USART1_Configuration(void);
void NVIC_Config(void);
void EXTI_Config(void);

//外部中断服务程序
void EXTI9_5_IRQHandler(void);


/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
  
  RCC_Configuration();
  Touch_SPI_Config();
  NVIC_Config();
  EXTI_Config();
  Lcd_Configuration();
  ili9320_Initializtion();
  ili9320_Clear(0xffff);
  ili9320_writestr(32, 50, "HELLO,WORLD!",Red,Blue);
  ili9320_PutCN(0,0,"肏你妈",Red,Blue);
  USART1_Configuration();
  u16 i;
  FIL file;
  char data[512];
  RTC_Config();
  SPI3_SD_init();
  res = f_mount(0, &fs);
  res = f_open(&file, "hello.txt", FA_OPEN_EXISTING | FA_READ);
    
    if(res!=FR_OK)
    {
      while(1);
    }
    
    for(i=0;i<512;i++)
	{
		data[i] = 0;
	}

    while(1)
    {
        if(f_gets(data, sizeof(data), &file)==NULL)
        {
            break;
        }
        prints(data);
    }
	/* Close open files */
    f_close(&file);
	/* Unregister work area prior to discard it */
    f_mount(0, NULL);

    /* 
    write_time = Time_GetUnixTime();                            
    res = f_open(&file, "331.txt", FA_CREATE_ALWAYS | FA_WRITE);
    for(i=0;i<3;i++)                                            
    {                                                           
        res = f_write(&file, data, 512, &br);                   
        if(br<512)  //判断是否磁盘写满                          
        {                                                       
            break;                                              
        }                                                       
    }                                                           
    write_time = Time_GetUnixTime() - write_time;               
        f_close(&file);                                         
    */


    while(1)
    {
        
        ///* 
        if(TimeDisplay)                                                           
        {                                                                         
            current_time = Time_GetUnixTime();                                    
            time_now = Time_GetCalendarTime();                                    
                                                                                  
            USART_SendData(USART1, 0x0c);                                         
            while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET);             
                                                                                  
            printf("\r\nUNIX时间：%d", current_time);                             
            printf("\t当前时间：%d-%d-%d %d %02d:%02d:%02d\t", time_now.tm_year,
                   time_now.tm_mon+1, time_now.tm_mday, time_now.tm_wday+1,
                   time_now.tm_hour, time_now.tm_min, time_now.tm_sec);           
            TimeDisplay = 0;                                                      
        }                                                                         
        //*/
        
    }
}

void RCC_Configuration(void)
{
  ErrorStatus HSEStartUpStatus;
  //使能外部晶振
  RCC_HSEConfig(RCC_HSE_ON);
  //等待外部晶振稳定
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  //如果外部晶振启动成功，则进行下一步操作
  if (HSEStartUpStatus==SUCCESS)
  {
    //FLASH时序控制
    //推荐值：SYSCLK = 0~24MHz   Latency=0
    //        SYSCLK = 24~48MHz  Latency=1
    //        SYSCLK = 48~72MHz  Latency=2
    FLASH_SetLatency(FLASH_Latency_2);
    //开启FLASH预取指功能
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable); 
    //预分频2 HSE25M/5=5M
    RCC_PREDIV2Config(RCC_PREDIV2_Div5);
    //锁相环2 5M*8=40M
    RCC_PLL2Cmd(DISABLE);
    RCC_PLL2Config(RCC_PLL2Mul_8);
    //开启锁相环2
    RCC_PLL2Cmd(ENABLE);
    //预分频1 40M/5=8M
    RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2,RCC_PREDIV1_Div5);            
    //PLL设置 8M*9 = 72MHz
    RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);
    //USB时钟
    RCC_OTGFSCLKConfig(RCC_OTGFSCLKSource_PLLVCO_Div3);
    RCC_PLL3Cmd(DISABLE);
    RCC_PLL3Config(RCC_PLL3Mul_10);
    RCC_PLL3Cmd(ENABLE);
    //以太网时钟MII模式
    RCC_MCOConfig(RCC_MCO_XT1);
    //设置HCLK（AHB时钟）=SYSCLK
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    //PCLK1(APB1) = HCLK/2
    RCC_PCLK1Config(RCC_HCLK_Div2);
    //PCLK2(APB2) = HCLK
    RCC_PCLK2Config(RCC_HCLK_Div1);
    //启动PLL
    RCC_PLLCmd(ENABLE);
    //等待PLL稳定
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    //系统时钟SYSCLK来自PLL输出
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    //切换时钟后等待系统时钟稳定
    while (RCC_GetSYSCLKSource()!=0x08);
  }
  return;
}



////////////////////////////////////////////////////////////////////////////////
//下面是USART模块的初始化
////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
* Function Name  : USART_Configuration
* Description    : USART设置
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	//启动USART1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	//USART1_TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//USART1_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//USART1模块配置
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);

#if USART_RXINT_EN
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
#endif

	USART_Cmd(USART1, ENABLE);
	return;
}



void NVIC_Config(void)
{
  //NVIC_SetVectorTable(NVIC_VectTab_RAM, 0);
  NVIC_InitTypeDef NVIC_InitStructure;	  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);//优先级分到第0组 总共5组		 
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; //使用外部中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//阶级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 //阶层0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);   
}


void EXTI_Config(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line = EXTI_Line5;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  // Configure EXTI Line5 to generate an interrupt on falling edge
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

//外部中断服务程序
void EXTI9_5_IRQHandler(void)
{ 	
  EXTI_ClearITPendingBit(EXTI_Line5);//清除中断标志位 
  ili9320_SetPoint(GUI_TOUCH_X_MeasureX(),GUI_TOUCH_X_MeasureY(),Blue);
  //printf("%d\t%d\r",GUI_TOUCH_X_MeasureX(),GUI_TOUCH_X_MeasureY());
  //printf("hello\n\t");
    
}