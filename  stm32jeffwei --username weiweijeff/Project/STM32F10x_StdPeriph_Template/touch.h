#ifndef __TOUCH_H
#define __TOUCH_H

#include "stm32f10x.h"


#define TP_CS()  GPIO_ResetBits(GPIOF,GPIO_Pin_10)//ʹ��Ƭѡ
#define TP_DCS() GPIO_SetBits(GPIOF,GPIO_Pin_10)
#define WaitTPReady() while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)==0)
#define CHX  0x90  //ͨ��X+��ѡ�������
#define CHY  0xd0 //ͨ��Y+��ѡ�������



void Touch_GPIO_Config(void);
void Touch_Config(void);

unsigned char SPI_WriteByte(unsigned char data);

void SpiDelay(unsigned int DelayCnt);

u16 TPReadX(void);

u16 TPReadY(void);

int  GUI_TOUCH_X_MeasureX(void);
int  GUI_TOUCH_X_MeasureY(void);
#endif
