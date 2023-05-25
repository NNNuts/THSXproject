#include "common.h"
#include "BRT38R0.h"

/* 
    Author : MosesChan

    Descriptions : 
        Driver APIs for the encoder "BTR38R0".
    Note :
        All APIs' purpose can be simply get form the function name.

    Driver Implementation Loigc :
        Since RS485 is a UART based transceiver, the data frame reception process
        is accomplished by the Interrupt handlers (USART1_IRQHandler) which is defined in stm32f1xx_it.c.
        And for different type of instructions for this driver the reception data lenght is differed.
        There are 3 types of different reception data length the vendor protocol has defined :
        (since UART dont usually know how much data chunks it'll receive in a single transmission , 
                three different types of reception handler has to be defined) 
            
            5 + 2(CRC code)     ->     _BRT38R0_UART_Handler_5B 
            7 + 2               ->     _BRT38R0_UART_Handler_7B
            6 + 2               ->     _BRT38R0_UART_Handler_6B
       Each instruction sending process will be : 
            1. Replace the function pointer in the USART1_IRQHandler to the coorsponding one.
            2. Send command frame through the UART tx line.
            3. Extract and return the data received by URAT Interrupt handler which is then stored in a 
                global array g_pRS485_dataReceivePack[].

    TODOs : 
        The wirte instruction is implemented but still not been tested.
        The write instruction typically will return the same data as it sent,
            thus there are chances to validate the instruction by implement some loigc in the return 
            section of each functions. 
    
*/


/* Read Functions for the BTR38R0 Encoder */

/// @brief Read Encoder Value (16bit)
/// @param  None
/// @return 16bit position value from the encoder.
uint16_t BRT38R0_READ_VALUE_1(void) {
    g_fpUART_Handler = _BRT38R0_UART_Handler_5B;

    _BRT38R0_UART_SEND_CMD(BRT38R0_ACTION_READ,\
                        BRT38R0_REG_ADDR_SINGLE_CYCLE_VALUE,\
                        BRT38R0_RESIGER_COUNT_1);

    return _g_DATA_EXTRACT_5B();
}


/// @brief Read Encoder Value (32bit)
/// @param None
/// @return 32bit position value from the encoder.
uint32_t BRT38R0_READ_VALUE_2(void) {
    g_fpUART_Handler = _BRT38R0_UART_Handler_7B;

    _BRT38R0_UART_SEND_CMD(BRT38R0_ACTION_READ,\
                        BRT38R0_REG_ADDR_SINGLE_CYCLE_VALUE,\
                        BRT38R0_RESIGER_COUNT_2);
   
    return _g_DATA_EXTRACT_7B();
}

/// @brief Read Encoder virtual multiple cycle counts(16bit).
/// @param  None
/// @return 16bit cycle counts values.
uint16_t BRT38R0_READ_VIRTUAL_CYCLE_COUNT_1(void) {
    g_fpUART_Handler = _BRT38R0_UART_Handler_5B;
   
    _BRT38R0_UART_SEND_CMD(BRT38R0_ACTION_READ,\
                        BRT38R0_REG_ADDR_VIRTUAL_CYCLE_VALUE,\
                        BRT38R0_RESIGER_COUNT_1);

    return _g_DATA_EXTRACT_5B();
}

/// @brief Read Encoder virtual multiple cycle counts(32bit).
/// @param  None
/// @return 32bit cycle counts values.
uint32_t BRT38R0_READ_VIRTUAL_CYCLE_COUNT_2(void) {
    g_fpUART_Handler = _BRT38R0_UART_Handler_7B;

    _BRT38R0_UART_SEND_CMD(BRT38R0_ACTION_READ,\
                        BRT38R0_REG_ADDR_VIRTUAL_CYCLE_VALUE,\
                        BRT38R0_RESIGER_COUNT_2);

    return _g_DATA_EXTRACT_7B();
}

/// @brief Read Encoder angular velocity.
/// @param  None
/// @return 16bit angular velocity values.
uint16_t BRT38R0_READ_ANGULAR_VELOCITY(void) {
    g_fpUART_Handler = _BRT38R0_UART_Handler_5B;

    _BRT38R0_UART_SEND_CMD(BRT38R0_ACTION_READ,\
                        BRT38R0_REG_ADDR_ANGULAR_SPEED_VALUE,\
                        BRT38R0_RESIGER_COUNT_1);
  
    return _g_DATA_EXTRACT_5B();
}


/* Read Functions for the BTR38R0 Encoder */
/* TODOs : Return Statment handling (data itegrity check) */

uint8_t BRT38R0_SET_SLAVE_ADDRESS(uint8_t addr) {
    g_fpUART_Handler = _BRT38R0_UART_Handler_6B;

    _BRT38R0_UART_SEND_CMD(BRT38R0_ACTION_WRITE,\
                       BRT38R0_REG_ADDR_ADDRESS,\
                        addr);

    return 0;
}
/// @brief Set baudrate of the encoder.
/// @param baudRt Baudrate from 9600 to 1115200 
/// @return (TODO)
uint8_t BRT38R0_SET_BAUD_RATE(uint8_t baudRt) {
    g_fpUART_Handler = _BRT38R0_UART_Handler_6B;

    _BRT38R0_UART_SEND_CMD(BRT38R0_ACTION_WRITE,\
                       BRT38R0_REG_ADDR_BAUDRATE,\
                        baudRt);

    return 0;
}

/// @brief Set the current encoder data sendback mode. (default is query sendback)
/// @param mode 0x01 as auto sendback
/// @return (TODO)
uint8_t BRT38R0_SET_DATA_MODE(uint8_t mode) {
    g_fpUART_Handler = _BRT38R0_UART_Handler_6B;

    _BRT38R0_UART_SEND_CMD(BRT38R0_ACTION_WRITE,\
                       BRT38R0_REG_ADDR_MODE,\
                        mode);

    return 0;
}

/// @brief Set encoder auto send back time intervals in ms. (Once the setting time interval is less than 20 ms,
///  the encoder will not be able to set other parameters,caution to use !)
/// @param ms time interval in milliseconds (default is 20mS)
/// @return (TODO)
uint8_t BRT38R0_SET_AUTO_SENDBACK_DELAY_MS(uint8_t ms) {
    g_fpUART_Handler = _BRT38R0_UART_Handler_6B;

    _BRT38R0_UART_SEND_CMD(BRT38R0_ACTION_WRITE,\
                       BRT38R0_REG_ADDR_AUTO_REPLY_PERIOD,\
                        ms);
    return 0;
}

/// @brief Set current position value of the encoder as zero point.
/// @param  
/// @return (TODO)
uint8_t BRT38R0_SET_ZERO_POINT(void) {
    g_fpUART_Handler = _BRT38R0_UART_Handler_6B;

    _BRT38R0_UART_SEND_CMD(BRT38R0_ACTION_WRITE,\
                        BRT38R0_REG_ADDR_RESET_ZERO_FLAG_BIT,\
                        0x01);
    return 0;
}

/// @brief Set encoder value incrementation direction ( default counter clockwise)
/// @param dir 0x00 as clockwise
/// @return (TODO)
uint8_t BRT38R0_SET_VALUE_INCREMENT_DIRECTION(uint8_t dir) {
    g_fpUART_Handler = _BRT38R0_UART_Handler_6B;

    _BRT38R0_UART_SEND_CMD(BRT38R0_ACTION_WRITE,\
                       BRT38R0_REG_ADDR_VALUE_INCRE_DIRECTION,\
                        dir);

    return 0;
}

/// @brief Set current position value as the middle point of the encoder.
/// @param None
/// @return (TODO)
uint8_t BRT38R0_SET_MIDDLE_POINT(void) {
    g_fpUART_Handler = _BRT38R0_UART_Handler_6B;

    _BRT38R0_UART_SEND_CMD(BRT38R0_ACTION_WRITE,\
                       BRT38R0_REG_ADDR_SET_MIDPOINT_FLAG_BIT,\
                        0x01);

    return 0;
}

/// @brief Set encoder angular sampling time intervals in ms.
/// @param ms time interval in milliseconds
/// @return (TODO)
uint8_t BRT38R0_SET_ANGULAR_VELOCITY_SAMPLING_RATE_MS(uint16_t ms) {
    g_fpUART_Handler = _BRT38R0_UART_Handler_6B;

    _BRT38R0_UART_SEND_CMD(BRT38R0_ACTION_WRITE,\
                       BRT38R0_REG_ADDR_ANGULAR_SPEED_SAMPLING_PERIOD,\
                        ms);
    
    return 0;
}

/// @brief Specify the current position value of the encoder.
/// @param val 16 bit postion value.
/// @return (TODO)
uint8_t BRT38R0_SET_CURRENT_POSITION_VALUE(uint16_t val) {
    g_fpUART_Handler = _BRT38R0_UART_Handler_6B;

    _BRT38R0_UART_SEND_CMD(BRT38R0_ACTION_WRITE,\
                       BRT38R0_REG_ADDR_SET_CURRENT_VALUE,\
                        val);

    return 0;
}


/** 
    Flexible UART handlers for different data length (sentback) 
**/
void _BRT38R0_UART_Handler_5B(void){

// UART receive buffer empty check ,ready to pull out data.
  if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_RXNE) != RESET)
  {
    // RS485 harware reception start.
    RS485_EN_RESET();
    g_pRS485_dataReceivePack[g_iDataReceive++] = (uint8_t)(huart1.Instance->DR & 0x00FF);

    if(g_iDataReceive >= (5+2))
    {
        if(__UART_RS485_DATAPACK_PROCESS(5))
            HardFault_Handler();  
        g_iDataReceive = 0;

    }   
  }
  __HAL_UART_CLEAR_FLAG(&huart1,UART_FLAG_RXNE);

}

void _BRT38R0_UART_Handler_6B(void){

// UART receive buffer empty check ,ready to pull out data.
  if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_RXNE) != RESET)
  {
    // RS485 harware reception start.
    RS485_EN_RESET();
    g_pRS485_dataReceivePack[g_iDataReceive++] = (uint8_t)(huart1.Instance->DR & 0x00FF);

    if(g_iDataReceive >= 6+2) 
    {
        if(__UART_RS485_DATAPACK_PROCESS(6))
            HardFault_Handler();  
        g_iDataReceive = 0;

    }   

  }
  __HAL_UART_CLEAR_FLAG(&huart1,UART_FLAG_RXNE);

}

void _BRT38R0_UART_Handler_7B(void){

// UART receive buffer empty check ,ready to pull out data.
  if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_RXNE) != RESET)
  {
    // RS485 harware reception start.
    RS485_EN_RESET();
    g_pRS485_dataReceivePack[g_iDataReceive++] = (uint8_t)(huart1.Instance->DR & 0x00FF);

    if(g_iDataReceive >= 7+2) 
    {
        if(__UART_RS485_DATAPACK_PROCESS(7))
            HardFault_Handler();  
        g_iDataReceive = 0;

    }   
  }
  __HAL_UART_CLEAR_FLAG(&huart1,UART_FLAG_RXNE);

}

uint32_t __UART_RS485_DATAPACK_PROCESS(uint8_t dataBits){

    volatile uint16_t CRC_x = 0;

      CRC_x = BRT38R0_CRC_CALC(g_pRS485_dataReceivePack,dataBits);
    if( (uint8_t)CRC_x == g_pRS485_dataReceivePack[dataBits] &&
        (uint8_t)(CRC_x>>8) == g_pRS485_dataReceivePack[dataBits+1] )
    {
        return 0;
      

    }else
    {
        
        SEGGER_RTT_printf(0,"%s!!!Encoder CRC Mismatch!!!%s\n",RTT_CTRL_BG_RED,RTT_CTRL_RESET);
        SEGGER_RTT_printf(0,"%sOriginal:\t[%d]%x  , [%d]%x%s\n",\
        RTT_CTRL_BG_BLUE, \
        dataBits+1, \
        g_pRS485_dataReceivePack[dataBits], \
        dataBits+2, \
        g_pRS485_dataReceivePack[dataBits+1], \
        RTT_CTRL_RESET);
        SEGGER_RTT_printf(0,"%sCalculated:[%d]%x  , [%d]%x%s\n\n\n",\
        RTT_CTRL_BG_BLUE,\
        dataBits+1, \
        (uint8_t)(CRC_x>>8), \
        dataBits+2, \
        (uint8_t)(CRC_x), \
        RTT_CTRL_RESET);
        return 1;

    }

}


/// @brief Inner callback funtion for sending command to Encoder BRT38R0
/// @param action MAROS defined in coorsponding header file.
/// @param regAddr MAROS defined in coorsponding header file.
/// @param regCnt MAROS defined in coorsponding header file.
/// @return Should be blank.
uint8_t _BRT38R0_UART_SEND_CMD(uint8_t action , uint8_t regAddr , uint8_t regCnt){

   volatile uint16_t CRC_6 = 0;

/*
 Set : Driver Output Enable 
 Reset  : Receiver Output Enable 
*/  
    RS485_EN_SET();

// 0x03 Read Register (Keep)
    // Byte 1 : ADR , Slave Encoder Address, defaultl 01;
    g_pRS485_dataSendPack[0] = (uint8_t)BRT38R0_SLAVE_DEFAULT_ADDR;
    // Byte 2 : 0x03(read) or 0x06(write)
    g_pRS485_dataSendPack[1] = (uint8_t)action;
    // Byte 3 : Start Register High Bytes
    g_pRS485_dataSendPack[2] = (uint8_t)BRT38R0_REG_DEFAULT_HIGHER_ADDR;
    // Byte 4 : Start Register Low Bytes  || TO MODIFY
    g_pRS485_dataSendPack[3] = (uint8_t)regAddr;
    // Byte 5 : Register Count High Bytes
    g_pRS485_dataSendPack[4] = (uint8_t)BRT38R0_REG_DEFAULT_HIGHER_CNT;
    // Byte 6 : Register Count Low Bytes  || TO MODITY
    g_pRS485_dataSendPack[5] = (uint8_t)regCnt;

      // CRC bytes padding at trailing of the frame.
      CRC_6 = BRT38R0_CRC_CALC(g_pRS485_dataSendPack,6);

    // Byte 7 : CRC High Bytes
    g_pRS485_dataSendPack[6] = CRC_6;
    // Byte 8 : CRC Low Bytes
    g_pRS485_dataSendPack[7] = CRC_6>>8;
    

    HAL_UART_Transmit(&huart1,g_pRS485_dataSendPack,8,0xFF);

    // Wati till transfer buffer empty. (These're optional)
    while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) != SET);
    RS485_EN_RESET();


    /* 
    To avoid data collisions while sending and
     receiving almostly at the same time 
    */
		Delay(50);

    return 0;

		
}
uint16_t _g_DATA_EXTRACT_5B(void){

return (uint16_t)\
                    ( (((uint16_t)g_pRS485_dataReceivePack[5-2])<<8) + \
                      (uint16_t)g_pRS485_dataReceivePack[5-1]);

}

uint32_t _g_DATA_EXTRACT_7B(void){

            return (uint32_t) \
                       ((((uint32_t)(g_pRS485_dataReceivePack[7-4]))<<24) + \
                        (((uint32_t)(g_pRS485_dataReceivePack[7-3]))<<16) + \
                        (((uint32_t)(g_pRS485_dataReceivePack[7-2]))<<8)  + \
                        (((uint32_t)(g_pRS485_dataReceivePack[7-1]))<<0) );
}


/*************************CRC calculation function*************************/
uint16_t BRT38R0_CRC_CALC(uint8_t pbuf[],uint8_t num)
{
   int i,j;
   uint16_t wcrc=0xffff;
   for(i=0;i<num;i++)
   {
     wcrc^=(uint16_t)(pbuf[i]);
		 for (j=0;j<8;j++)
		 {
				if(wcrc&0x0001)
			{
				 wcrc>>=1;
				 wcrc^=0xa001;
			}
			else wcrc>>=1;		
		 }
   }   
   return wcrc;
}
