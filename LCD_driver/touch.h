#ifndef __TOUCH_H
#define __TOUCH_H

#include "stm32f10x.h"


#define TP_CS()  GPIO_ResetBits(GPIOC,GPIO_Pin_8)//ʹ��Ƭѡ
#define TP_DCS() GPIO_SetBits(GPIOC,GPIO_Pin_8)
#define WaitTPReady() while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==0)
#define CHX  0x90  //ͨ��X+��ѡ�������
#define CHY  0xd0 //ͨ��Y+��ѡ�������



void Touch_SPI_Config(void);

unsigned char SPI_WriteByte(unsigned char data);

void SpiDelay(unsigned int DelayCnt);

u16 TPReadX(void);

u16 TPReadY(void);

int  GUI_TOUCH_X_MeasureX(void);
int  GUI_TOUCH_X_MeasureY(void);
#endif
