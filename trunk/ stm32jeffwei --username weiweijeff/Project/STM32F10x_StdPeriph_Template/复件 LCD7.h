#ifndef __LCD7_H
#define __LCD7_H


#include "stm32f10x.h"

#define Set_CE  GPIOG->BSRR  = GPIO_Pin_12;
#define Clr_CE  GPIOG->BRR   = GPIO_Pin_12;

#define Set_A0  GPIOF->BSRR  = GPIO_Pin_0;
#define Clr_A0  GPIOF->BRR   = GPIO_Pin_0;

#define Set_RE  GPIOD->BSRR  = GPIO_Pin_4;
#define Clr_RE  GPIOD->BRR   = GPIO_Pin_4;

#define Set_WE  GPIOD->BSRR  = GPIO_Pin_5;
#define Clr_WE  GPIOD->BRR   = GPIO_Pin_5;

#define Set_RESET  GPIOA->BSRR  = GPIO_Pin_8;
#define Clr_RESET  GPIOA->BRR   = GPIO_Pin_8;

void LCD7_GPIO_Config(void);
void LCD7_Init(void);
void LCD7_Wregaddr(uint8_t addr);
void LCD7_Wregdata(uint16_t data);
void LCD7_Init(void);
void  LCD_RectFill(int start_x,int start_y,int end_x,int end_y,int color);


#endif