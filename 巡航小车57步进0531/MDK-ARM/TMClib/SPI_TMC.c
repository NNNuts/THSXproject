//#include "SPI_TMC.h"
//#include "stm32f1xx_hal_gpio.h"
//#include "Delay.h"
//#include "main.h"
#include "TMC5160_Register.h"
#include "TMC5160_Mask_Shift.h"
#include "SPI_TMC.h"
#include "TMC5160.h"
#include "main.h"

uint8_t tmc_sdo_data[5];

//#define __NOP()     delay_nop()

void delay_nop(void)
{		
	for(uint32_t i = 0;i<1;i++) {
         for(uint32_t j = 0;j<1;j++);
    }        
}

void tmc_spi_init(void)
{
// 	  GPIO_InitTypeDef GPIO_InitStructure;
// 		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC,ENABLE);
//		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//		GPIO_Init(GPIOB,&GPIO_InitStructure); 

//		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//		GPIO_Init(GPIOC,&GPIO_InitStructure); 

//		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14 ;
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//		GPIO_Init(GPIOB,&GPIO_InitStructure);  	
//		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//		GPIO_Init(GPIOA,&GPIO_InitStructure);  					
				
}
//=====================================================
/*------------------------------------------------------
 * @brief Send a byte via SPI and read the received
 * @param	data		to transmitted byte
 * @return	received byte
 * 
  * @brief通过SPI发送一个字节并读取收到的信息
  * @param数据到传输的字节
  * @return收到字节
------------------------------------------------------*/
uint8_t tmc_spi_readwritebyte(uint8_t data)
{
	return tmc_spi_sendbyte(data);
	
}
//=======================================================

 

//模拟SPI
/************************************************************************** 
 * 函数名：SPI_SendByte 
 * 描述  ：SPI模块发送函数 
 * 输入  ：发送数据 
 * 返回  ：返回数据 
 *************************************************************************/   
uint8_t tmc_spi_sendbyte(uint8_t byte)
{
    uint8_t i,Temp=0;
		tmc_sdo_data[0] = 0x00;
		tmc_sdo_data[1] = 0x00;
		tmc_sdo_data[2] = 0x00;
		tmc_sdo_data[3] = 0x00;
		tmc_sdo_data[4] = 0x00;
    for(i=0;i<8;i++)                         // 循环8次
    {
        TMC_SCLK_L;
        __NOP();  __NOP(); __NOP(); __NOP(); __NOP();__NOP(); __NOP(); __NOP();                          //拉低时钟
        if ((byte&0x80)==0x80)	//总是发送最高位
        {
            TMC_SDI_H;
        }
        else
        {
            TMC_SDI_L;
        }
        byte <<= 1;                           // 低一位移位到最高位
        TMC_SCLK_H;__NOP(); __NOP();__NOP(); __NOP(); __NOP();__NOP(); __NOP(); __NOP();                            //拉高时钟
				
        tmc_sdo_data[0] <<= 1;                           //数据左移
				tmc_sdo_data[1] <<= 1;                           //数据左移
				tmc_sdo_data[2] <<= 1;                           //数据左移
				tmc_sdo_data[3] <<= 1;                           //数据左移
				tmc_sdo_data[4] <<= 1;                           //数据左移
        if(TMC_SDO1_HorL)tmc_sdo_data[0]++;                      //若从从机接收到高电平，数据自加一
				if(TMC_SDO2_HorL)tmc_sdo_data[1]++;                      //若从从机接收到高电平，数据自加一
				if(TMC_SDO3_HorL)tmc_sdo_data[2]++;                      //若从从机接收到高电平，数据自加一
				if(TMC_SDO4_HorL)tmc_sdo_data[3]++;                      //若从从机接收到高电平，数据自加一
				if(TMC_SDO5_HorL)tmc_sdo_data[4]++;                      //若从从机接收到高电平，数据自加一
				
//        TMC_SCLK_L;                     //拉低时钟  
    }
		TMC_SCLK_L;
    return (Temp);                              //返回数据
}
//unsigned char SPI_Read(void)
//{

//	unsigned char i,rxdata;
//	rxdata = 0x00;
//	for (i = 0;i < 8;i++)
//	{
//		rxdata = rxdata<<1;
//		TMC_SCLK_L;
//		if (TMC_SDO_HorL)	//读取最高位，保存至最末尾，通过左移位完成整个字节
//		{
//			rxdata |= 0x01;
//		}
//		__nop();__nop();__nop();		
//		TMC_SCLK_H;
//		__nop();__nop();__nop();		
//	 }
//	 return rxdata;

//}
//********************************************************************************//
//* 函数名:				SPI_Write(unsigned char txdata)					          //
//* 函数功能：                SPI单字节写入函数                                   //
//********************************************************************************//
void SPI_Write(unsigned char txdata)
{
	unsigned char i;
	for (i = 0;i < 8;i++)
	{
		TMC_SCLK_L; __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop(); 
		if ((txdata&0x80)==0x80)	//总是发送最高位
		{
		    TMC_SDI_H;
		}
		else
		{
	  	    TMC_SDI_L;
		}
		txdata = txdata<<1;
		__nop();__nop();__nop();__nop();__nop();__nop(); __nop();__nop();__nop(); 	
		TMC_SCLK_H;
		__nop();__nop();__nop();__nop();__nop();__nop(); __nop();__nop();__nop(); 
	}
}
