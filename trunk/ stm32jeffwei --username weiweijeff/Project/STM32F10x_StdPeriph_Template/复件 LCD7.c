

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
  /*������Ӧʱ�� */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG, ENABLE);
  /*����Lcd��������Ϊ�������*/
  /*16λ����,PE[15:0]*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  /*LCD���ƽ�,PD[15:13]->LCD_RD,LCD_WR,LCD_RS*/
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
  LCD7_Wregdata(0x00);
}

void  LCD_RectFill(int start_x,int start_y,int end_x,int end_y,int color)
{
  int i,j,w,h;
  LCD7_Wregaddr(0); // .............. CUR_Y
  LCD7_Wregdata(start_y);
  LCD7_Wregaddr(1); // .............. CUR_X
  LCD7_Wregdata(start_x);
  LCD7_Wregaddr(3); // ............ END_X
  LCD7_Wregdata(end_x);
  LCD7_Wregaddr(2); // .............. PIXELS
  h=100; // ..............
  w=100; // ..............
  for(i=0;i<h;i++)
  {
    for(j=0;j<w;j++)
    { // ............
      LCD7_Wregdata(color);
    }
  }
}
