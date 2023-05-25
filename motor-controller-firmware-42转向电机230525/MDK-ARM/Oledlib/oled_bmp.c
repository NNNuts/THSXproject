/*
此c文件用于存储图片取模
取模方式:
*/

#include "oled_bmp.h"

//16*16
const unsigned char BmpTest1[]=
{
0x00,0x00,0x00,0x37,0xCD,0x82,0x12,0x41,0x41,0x12,0x82,0xCD,0x37,0x00,0x00,0x00,
0x00,0x0C,0x32,0x49,0x84,0x8D,0x79,0x25,0x25,0x79,0x8D,0x84,0x49,0x32,0x0C,0x00,
};
//32*32;
const unsigned char BmpTest2[]=
{
0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x39,0xE1,0x63,0x12,0x0E,0x84,0x84,0x04,0x06,
0x06,0x04,0x84,0x84,0x0E,0x12,0x63,0xE1,0x39,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x8F,0x70,0xC0,0x81,0x9B,0x33,0x60,0x58,
0x58,0x60,0x33,0x1B,0x81,0xC0,0x70,0xDB,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xF0,0x98,0x0C,0x06,0xC3,0xE1,0x20,0x7E,0x3F,0xA3,0xE7,0x85,0x25,0x7E,
0x7E,0x25,0x85,0xC7,0xA3,0x3F,0x7E,0x30,0xE1,0xC3,0x06,0x0C,0x98,0xF0,0x00,0x00,
0x00,0x00,0x00,0x01,0x01,0x07,0x3F,0x40,0xBC,0xE2,0xA2,0xDC,0x60,0x3F,0x20,0x20,
0x20,0x20,0x3F,0x60,0xDC,0xA6,0xE2,0xBE,0x48,0x30,0x0F,0x01,0x01,0x00,0x00,0x00,
};
//64*64
const unsigned char BmpTest3[]=
{
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3E,0xFF,0xC3,0x03,
0x03,0x03,0x06,0x06,0x0E,0x8C,0xDC,0xF8,0x78,0x70,0x30,0x30,0x18,0x18,0x18,0x18,
0x18,0x18,0x18,0x18,0x30,0x30,0x70,0x78,0xF8,0xDC,0x8C,0x0E,0x06,0x06,0x03,0x03,
0x03,0xC3,0xFF,0x3E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x0F,
0x9C,0xF8,0x78,0x1E,0x07,0x03,0x01,0x80,0xC0,0xC0,0xC0,0x80,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x80,0xC0,0xC0,0xC0,0x80,0x01,0x03,0x07,0x0E,0x7C,0xF8,0x9C,
0x0F,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x7F,0xFF,0x80,0x00,0x00,0x00,0x00,0x07,0xCF,0xCC,0x0D,0x07,0x03,0x00,0x20,0xE0,
0xE0,0x20,0x00,0x03,0x07,0x0D,0xCC,0xCF,0x07,0x00,0x00,0x00,0x00,0x80,0xDF,0x1F,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xC0,
0xE0,0x63,0x37,0xBE,0xF8,0xF0,0xE0,0xC0,0x81,0x83,0x07,0x06,0x1E,0x3E,0x77,0x63,
0x63,0x77,0x3E,0x1E,0x06,0x07,0x83,0x81,0xC0,0xE0,0xF0,0xF8,0xBC,0x3F,0x63,0xE0,
0xC0,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xC0,0xE0,0x70,0x38,0x1C,0x0E,0x07,0x03,0x01,
0x00,0x00,0x00,0xFF,0xFF,0x03,0x07,0x0C,0x1D,0x1B,0x33,0x33,0x66,0x66,0x66,0xF6,
0xF6,0x66,0x66,0x66,0x33,0x33,0x1B,0x19,0x0D,0x06,0x03,0xFF,0xFF,0x00,0x00,0x00,
0x01,0x03,0x07,0x0E,0x1C,0x38,0x70,0xE0,0xC0,0x80,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x7E,0xFF,0x83,0x01,0x00,0x00,0x00,0x00,0xC0,0xF0,0x78,0xFC,
0xCE,0x06,0x06,0x3F,0x3F,0x06,0x06,0xEC,0xFC,0x78,0xF0,0xC0,0x00,0x1C,0x3E,0x37,
0x37,0x3E,0x1C,0x00,0x00,0xE0,0xF0,0x78,0xFC,0xCE,0x06,0x07,0x7F,0x3E,0x06,0x06,
0xEC,0x7C,0x78,0xE0,0x80,0x00,0x00,0x00,0x01,0x83,0xFF,0x7E,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x03,0x03,0x03,0xFF,0xFF,0x00,0x00,0x80,
0xF0,0x78,0x0C,0x0C,0x0C,0x1C,0xF8,0xF0,0x00,0x00,0x83,0xFF,0x7C,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0xC0,0xF0,0x38,0x0C,0x0C,0x0C,0x1C,0xF8,
0xE0,0x00,0x00,0xC7,0xFF,0x17,0x03,0x03,0x03,0x01,0x01,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x0F,0x38,0x70,
0x67,0xCF,0xDC,0xD8,0xD8,0xDC,0xEF,0x63,0x38,0x1C,0x0F,0x07,0x06,0x06,0x06,0x06,
0x06,0x06,0x06,0x04,0x07,0x0F,0x1E,0x38,0x71,0x67,0xCE,0xDC,0xD8,0xD8,0xCC,0x6F,
0x63,0x38,0x1E,0x0F,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};

//长128，高40
const unsigned char Like[]={
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,
0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xE0,0xC0,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xC0,0xC0,0xC0,0x00,0x00,
0x00,0xE0,0xF0,0xFC,0xFE,0xFF,0xFF,0xFF,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,
0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x80,0xE0,0xF8,0xFC,0xFE,0xFE,0xE7,0x47,0x47,0x47,0x07,
0x07,0x07,0x47,0x47,0x47,0xE7,0xFE,0xFE,0xFC,0xF8,0xE0,0x80,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x60,0xE0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF8,0xFC,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0xF8,
0xF0,0xF0,0xF0,0xF0,0xF0,0xE0,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0x00,0x00,
0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x1F,
0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x3F,0xFF,0xFF,0xFF,0xFF,0x81,0x80,0xC0,0xFC,0xFE,0x00,
0x00,0x00,0xFE,0xFC,0xC0,0x80,0x81,0xFF,0xFF,0xFF,0xFF,0x3F,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x01,0x03,0x07,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0x07,0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x0F,0x0F,0x0F,0x00,0x00,
0x00,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x07,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x07,0x0F,0x0F,0x1F,0x1F,0x3F,0x3F,0x3C,
0x3C,0x3C,0x3F,0x3F,0x1F,0x1F,0x0F,0x0F,0x07,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x0F,0x0F,0x0F,0x07,0x07,0x03,0x03,0x01,0x03,0x03,0x03,0x07,
0x0F,0x0F,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};

//128*64
const unsigned char Panda[]=
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0x7F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x7F,
0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0x7F,0x7F,0x7F,0x7F,
0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0x07,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xE0,
0xF0,0xF8,0xF9,0xFD,0xFD,0xFF,0x7E,0x7E,0x3E,0x3F,0x7F,0x7F,0x7F,0x7F,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7E,0x7F,0x7D,0x7D,0xFB,0xFB,0xF7,
0xE3,0xC1,0x81,0x00,0x00,0x00,0x01,0x01,0x01,0x03,0x07,0x07,0x1F,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFC,0xF0,0xE0,0xC0,0x80,0x00,0x80,0xF0,0xFC,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFC,0xF8,0xF0,0xF1,0xF1,0xF1,0xC0,0xCC,0xCC,0xC0,0xE0,0xE0,0xF0,
0xF9,0xFD,0xFC,0xFF,0xF3,0xE1,0xE1,0xC0,0xC0,0xCC,0xCC,0xFC,0xF0,0xFB,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFE,0xF0,0xC0,0x00,0x80,0x80,0xC0,0xE0,0xF0,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB,0xE1,
0x01,0x03,0x07,0x3F,0x01,0x01,0x03,0xFF,0xFF,0x3F,0x1F,0x1F,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0xCF,0xF8,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xE7,0xEF,0xCF,
0xCF,0x8F,0x9F,0x9F,0x9F,0x9F,0xCF,0xDF,0xFF,0xFF,0xFF,0xEF,0xEF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xF3,0x1F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x3F,0x07,0x01,0xE1,0xF0,0xF8,0xF8,0xF0,0xC1,0x03,
0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x1E,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xC0,0x3F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xF9,0xE1,0x81,0x11,0x18,0x18,0x18,0x19,0x31,0x31,
0x31,0x39,0x3B,0x33,0x33,0x33,0x33,0x33,0x33,0x03,0x87,0xC7,0xEF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0xE0,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0xF0,0xF0,0xE0,0xE1,0xE1,0xE1,0xE1,0x80,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xE0,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0xFB,0x77,0x2F,0x1F,0x1F,0x3F,
0x3F,0x7F,0x7F,0x7F,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xEE,0xE6,0xC6,0xCE,0xCE,
0xCE,0xCE,0xCE,0xEE,0xFE,0xFC,0xFC,0xFE,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,
0x7F,0x7F,0x3F,0x1F,0x1F,0x0F,0x07,0x01,0x02,0x07,0x0F,0x1F,0x3F,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x0F,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x1F,0x07,0x03,0x01,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,
0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x03,
0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x03,0x03,0x01,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xF0,0xE0,0xE0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,
0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,
0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,
0xC0,0xC0,0xC0,0x80,0x80,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,
0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,
0xF0,0xF0,0xF0,0xF8,0xF8,0xFC,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

