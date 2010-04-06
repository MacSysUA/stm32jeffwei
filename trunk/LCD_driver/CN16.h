#ifndef __CN16_H__
#define __CN16_H__
typedef struct CN16    // 汉字字模数据结构
{
  unsigned char Font_Index[2];  //汉字
  unsigned char Font_Code[32]; //字模
}CN16;

extern const CN16 CN16Lib[];

#endif