#ifndef _BRT38R0_h
#define _BRT38R0_h

/* RS485 Lowside Encoder Resiters Addresses */
#define BRT38R0_REG_ADDR_SINGLE_CYCLE_VALUE               0x0000
#define BRT38R0_REG_ADDR_VIRTUAL_MULTI_CYCLE_VALUE        0x0001
#define BRT38R0_REG_ADDR_VIRTUAL_CYCLE_VALUE              0x0002
#define BRT38R0_REG_ADDR_ANGULAR_SPEED_VALUE              0x0003
#define BRT38R0_REG_ADDR_ADDRESS                          0x0004
#define BRT38R0_REG_ADDR_BAUDRATE                         0x0005
#define BRT38R0_REG_ADDR_MODE                             0x0006
#define BRT38R0_REG_ADDR_AUTO_REPLY_PERIOD                0x0007
#define BRT38R0_REG_ADDR_RESET_ZERO_FLAG_BIT              0x0008
#define BRT38R0_REG_ADDR_VALUE_INCRE_DIRECTION            0x0008
#define BRT38R0_REG_ADDR_ANGULAR_SPEED_SAMPLING_PERIOD    0x000A  
#define BRT38R0_REG_ADDR_SET_CURRENT_VALUE                0x000B
#define BRT38R0_REG_ADDR_SET_MIDPOINT_FLAG_BIT            0x000E
// This value occupies multiple register from 0x0025-0x0026
#define BRT38R0_VALUE                                     0x0025
#define BRT38R0_VALUE_1                                   0x0026

#define BRT38R0_ACTION_READ                               0x03
#define BRT38R0_ACTION_WRITE                              0x06

#define BRT38R0_RESIGER_COUNT_1                           0x01
#define BRT38R0_RESIGER_COUNT_2                           0x02

// Default values 
#define BRT38R0_SLAVE_DEFAULT_ADDR 		                  0x01
#define BRT38R0_REG_DEFAULT_HIGHER_ADDR 	              0x00
#define BRT38R0_REG_DEFAULT_HIGHER_CNT	                  0x00

// Enumeration Types for wite instructions 

enum BRT38R0_BAUD_RATE{
    BRT38R0_BAUD_RATE_9600,
    BRT38R0_BAUD_RATE_19200,
    BRT38R0_BAUD_RATE_38400,
    BRT38R0_BAUD_RATE_57600,
    BRT38R0_BAUD_RATE_115200
};

enum BRT38R0_SENDBACK_MODE{
    BRT38R0_SENDBACK_MODE_QUERY,
    BRT38R0_SENDBACK_MODE_AUTO
};

enum BRT38R0_ASCENDING_DIRECTION{
    BRT38R0_ASCENDING_DIRECTION_CLOCKWISE,
    BRT38R0_ASCENDING_DIRECTION_COUNTER_CLOCKWISE
};

/*************************CRC calculation function*************************/
uint16_t BRT38R0_CRC_CALC(uint8_t pbuf[],uint8_t num);

void _BRT38R0_UART_Handler_5B(void);
void _BRT38R0_UART_Handler_6B(void);
void _BRT38R0_UART_Handler_7B(void);


/** Low side 485 Encoder read function set , 
 *  _1 for 16bit or below (16 bit return data)
 *  _2 for 17bit or above (32 bit return data)
 **/
uint16_t BRT38R0_READ_VALUE_1(void);
uint32_t BRT38R0_READ_VALUE_2(void);
uint16_t BRT38R0_READ_VIRTUAL_CYCLE_COUNT_1(void);
uint32_t BRT38R0_READ_VIRTUAL_CYCLE_COUNT_2(void);
uint16_t BRT38R0_READ_ANGULAR_VELOCITY(void);

/*** Low side 485 Encoder write function set ***/
uint8_t BRT38R0_SET_SLAVE_ADDRESS(uint8_t addr);
uint8_t BRT38R0_SET_BAUD_RATE(uint8_t baudRt);
uint8_t BRT38R0_SET_DATA_MODE(uint8_t mode);
uint8_t BRT38R0_SET_AUTO_SENDBACK_DELAY_MS(uint8_t delay);
uint8_t BRT38R0_SET_ZERO_POINT(void);
uint8_t BRT38R0_SET_VALUE_INCREMENT_DIRECTION(uint8_t dir);
/* set current point as the middle point of the encoder */
uint8_t BRT38R0_SET_MIDDLE_POINT(void);
uint8_t BRT38R0_SET_ANGULAR_VELOCITY_SAMPLING_RATE_MS(uint16_t ms);
uint8_t BRT38R0_SET_CURRENT_POSITION_VALUE(uint16_t pos);


uint8_t _BRT38R0_UART_SEND_CMD(uint8_t action , uint8_t regAddr , uint8_t regCnt);
uint32_t __UART_RS485_DATAPACK_PROCESS(uint8_t dataBits);
uint16_t _g_DATA_EXTRACT_5B(void);
uint32_t _g_DATA_EXTRACT_7B(void);

#endif
