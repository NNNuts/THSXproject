#include "bsp_init.h"

/* Button bsp init dependency function */

uint8_t read_button_gpio(uint8_t button_id)
{
	/*
    // you can share the gpio read function with multiple buttons
    switch(button_id)
    {
    case btn1_id:
        return !(hal_gpio_readpin(key1_gpio_port, key1_pin));

//			break;

    case btn2_id:
        return !(hal_gpio_readpin(key2_gpio_port, key2_pin));

//			break;

    default:
        return 0;
//			break;
    }
	*/
	        return 0;

}


void bsp_tmc_init(void)
{

    HAL_GPIO_WritePin(TMC5160_SPI_MODE_GPIO_Port, TMC5160_SPI_MODE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TMC5160_SD_MODE_GPIO_Port, TMC5160_SD_MODE_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(TMC5160_EN_GPIO_Port, TMC5160_EN_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);

//    MotorTmc5160[motor1].Axis.speed = 5; //�������ò�Ҫ����12


//	tmc5160_SetPara(&MotorTmc5160[motor1],256,4,10,\
//	5,0,0,10000,MotorTmc5160[motor1].Axis.speed,\
//	10000,500,10,motor1);
//	tmc5160_InitConfig(&MotorTmc5160[motor1],motor1);

    tmc_spi_writeregister(motor1, 0xEC - 0x80, 0x000100C3);
    tmc_spi_writeregister(motor1, 0x90 - 0x80, 0x0006060A);
    tmc_spi_writeregister(motor1, 0x91 - 0x80, 0x0000000A);
    tmc_spi_writeregister(motor1, 0x80 - 0x80, 0x00000004);
    tmc_spi_writeregister(motor1, 0x93 - 0x80, 0x000000C8);    //tmc_spi_writeregister(motor1, 0x93 - 0x80, 0x000001F4);
		
		tmc_spi_writeregister(motor1, 0x8A - 0x80, 0x000D0400); // 1101 0000 0100 0000 0000

    tmc_spi_writeregister(motor1, 0xA4 - 0x80, 1000 ); 			//A1
    tmc_spi_writeregister(motor1, 0xA5 - 0x80, 0    ); 			//V1
    tmc_spi_writeregister(motor1, 0xA6 - 0x80, 3000 ); 			//AMAX
    tmc_spi_writeregister(motor1, 0xA7 - 0x80, 0    ); 	    	//VMAX
    tmc_spi_writeregister(motor1, 0xA8 - 0x80, 3000 ); 			//DMAX
    tmc_spi_writeregister(motor1, 0xAA - 0x80, 1400 ); 			//D1
    tmc_spi_writeregister(motor1, 0xAB - 0x80, 10   ); 			//VSTOP
//tmc_spi_writeregister(motor1,0xA0-0x80,0x00000000); 			//λ��ģʽ
    tmc_spi_writeregister(motor1, 0xA0 - 0x80, 0x00000001);     //�ٶ�ģʽ

    tmc_spi_writeregister(motor1,TMC5160_XACTUAL,0x00000000);

}

void bsp_button_init(void)
{

    button_init(&btn1, read_button_gpio, 0, btn1_id);
    button_init(&btn2, read_button_gpio, 0, btn2_id);

    button_attach(&btn1, PRESS_DOWN,       BTN1_PRESS_DOWN_Handler);
    button_attach(&btn1, PRESS_UP,         BTN1_PRESS_UP_Handler);
    button_attach(&btn1, PRESS_REPEAT,     BTN1_PRESS_REPEAT_Handler);
    button_attach(&btn1, SINGLE_CLICK,     BTN1_SINGLE_Click_Handler);
    button_attach(&btn1, DOUBLE_CLICK,     BTN1_DOUBLE_Click_Handler);
    button_attach(&btn1, LONG_PRESS_START, BTN1_LONG_PRESS_START_Handler);
    button_attach(&btn1, LONG_PRESS_HOLD,  BTN1_LONG_PRESS_HOLD_Handler);

    button_attach(&btn2, PRESS_DOWN,       BTN2_PRESS_DOWN_Handler);
    button_attach(&btn2, PRESS_UP,         BTN2_PRESS_UP_Handler);
    button_attach(&btn2, PRESS_REPEAT,     BTN2_PRESS_REPEAT_Handler);
    button_attach(&btn2, SINGLE_CLICK,     BTN2_SINGLE_Click_Handler);
    button_attach(&btn2, DOUBLE_CLICK,     BTN2_DOUBLE_Click_Handler);
    button_attach(&btn2, LONG_PRESS_START, BTN2_LONG_PRESS_START_Handler);
    button_attach(&btn2, LONG_PRESS_HOLD,  BTN2_LONG_PRESS_HOLD_Handler);

    button_start(&btn1);
    button_start(&btn2);

}


void bsp_oled_init(void)
{

    // OLED_START();
    // HAL_Delay(100);
    // InitGraph();
    // HAL_Delay(100);

}
