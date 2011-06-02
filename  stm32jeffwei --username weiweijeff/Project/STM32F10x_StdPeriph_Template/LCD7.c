

#include"LCD7.h"

static void delay(vu32 nCount)
{
  vu32 index = 0; 
  for(index = (100000 * nCount); index != 0; index--)
  {
  }
}


void LCD7_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /*开启相应时钟 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG, ENABLE);
  /*所有Lcd引脚配置为推挽输出*/
  /*16位数据,PE[15:0]*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  /*LCD控制脚,PD[15:13]->LCD_RD,LCD_WR,LCD_RS*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOG, &GPIO_InitStructure);

}



void LCD7_Wregaddr(uint8_t addr)
{
  Clr_CE;
  Clr_A0;
  GPIOE->ODR=(uint16_t)(addr<<8);
  Clr_WE;
  delay(1);
  Set_WE;
  Set_CE;
}
void LCD7_Wregdata(uint16_t data)
{
  Clr_CE;
  Set_A0;
  GPIOE->ODR=data;
  Clr_WE;
  delay(1);
  Set_WE;
  delay(3);
  GPIOE->ODR=data<<8;
  Clr_WE;
  delay(1);
  Set_WE;
  Set_CE;
}

void LCD7_Init(void)
{
  Clr_RESET;
  delay(5);
  Set_RESET;
  delay(5);
  LCD7_Wregaddr(0x05);
  LCD7_Wregdata(0x0f);
}