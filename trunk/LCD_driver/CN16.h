#ifndef __CN16_H__
#define __CN16_H__
typedef struct CN16    // ������ģ���ݽṹ
{
  unsigned char Font_Index[2];  //����
  unsigned char Font_Code[32]; //��ģ
}CN16;

extern const CN16 CN16Lib[];

#endif