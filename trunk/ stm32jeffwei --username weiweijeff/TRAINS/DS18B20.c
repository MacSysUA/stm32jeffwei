#define _DS18B20_LOCAL
#include "ds18b20.h"
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
//��������θ��Ƶ�������
/**************************************************************************/
//  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
//  TIM_TimeBaseStructure.TIM_Period = 1;                 //�Զ�װ��
//  TIM_TimeBaseStructure.TIM_Prescaler = 72;       //72M��Ƶ�ʵ�1MHz
//  TIM_TimeBaseStructure.TIM_ClockDivision = 0;   
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;  //���¼���
//  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
/**************************************************************************/
void delay_nus(vu32 nCount)
{
  u16 TIMCounter = nCount;
  TIM_Cmd(TIM2, ENABLE);
  TIM_SetCounter(TIM2, TIMCounter);
  while (TIMCounter)
  {
    TIMCounter = TIM_GetCounter(TIM2);
  }
  TIM_Cmd(TIM2, DISABLE);
}

/*******************************************************************************
*******************************************************************************/

void DS18B20_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

        // PC9-DQ
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(DS18B20_PORT, &GPIO_InitStructure);
}

//�������ϵ����д������ӳ�ʼ����ʼ
void init_1820()  
{  
    DIR_1WIRE_OUT();
    SET_OP_1WIRE();   
    CLR_OP_1WIRE();  
    delay_nus(500);    //480us����  
    SET_OP_1WIRE();  
    DIR_1WIRE_IN();  
    delay_nus(30);     //15~60us  
    while(CHECK_IP_1WIRE());  
    DIR_1WIRE_OUT();  
    SET_OP_1WIRE();  
    delay_nus(100);   //60~240us  
}
/*
ָ��                    ����
Read ROM(��ROM)         [33H]
Match ROM(ƥ��ROM)      [55H]
Skip ROM(����ROM]       [CCH]
Search ROM(����ROM)     [F0H]
Alarm search(�澯����)  [ECH]

ָ��                                                         ����
Write Scratchpad(д�ݴ�洢��)         [4EH]
Read Scratchpad(���ݴ�洢��)         [BEH]
Copy Scratchpad(�����ݴ�洢��) [48H]
Convert Temperature(�¶ȱ任)         [44H]
Recall EPROM(���µ���)                         [B8H]
Read Power supply(����Դ)                 [B4H]
*/  
void write_1820(u8 x)  
{     
    u16 i;  
   
        for(i=0; i<8; i++)  
    {  
               CLR_OP_1WIRE();
               if(x & 0x01)    //д�����ˣ���д��λ�ģ�
                       SET_OP_1WIRE();  
               else  
                       CLR_OP_1WIRE();  
                x>>=1;
               delay_nus(30);   //15~60us  
               SET_OP_1WIRE();  
    }  
    SET_OP_1WIRE();  
}  
u8 read_1820(void)  
{      
        u16 i;
        u8 x = 0;
       
        DIR_1WIRE_OUT();
    for(i=0; i<8; i++)  
    {  
            CLR_OP_1WIRE();  
            x >>= 1;
                SET_OP_1WIRE();  
            DIR_1WIRE_IN();
            //������,�ӵ�λ��ʼ  
            if(CHECK_IP_1WIRE())  
                    x |= 0x80;
            delay_nus(100); //60~120us      
            DIR_1WIRE_OUT();  
    }
    return (x);  
}   

/*
        ����ֵ���¶�
        ��ʽs,s,s,s, s,b6,b5,b4, b3,b2,b1,b0, b-1,b-2,b-3,b-4
        ת��Ϊʮ����
        �¶� T = (int)val*10/16;
        �Ŵ���10������λ��С�����֡�
       

*/
void DS18B20_Convert(void)                //�¶�ת��
{
     init_1820();        //��λ18b20  
     write_1820(0xcc);   // ����ת������  
     write_1820(0x44);
}

u16 DS18B20_Read(void)                   //��ȡ�¶�ֵ
{  
    u16 res;
    init_1820();  
    write_1820(0xcc);  //����������  
    write_1820(0xbe);  
    res=read_1820();  //������  
    res+=(u16)read_1820()<<8;  
    return res;
}