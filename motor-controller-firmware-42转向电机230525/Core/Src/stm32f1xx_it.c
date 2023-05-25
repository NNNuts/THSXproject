/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */


#include "main.h"
#include "common.h"
#include "stm32f1xx_it.h"

void CAN1_RX0_IRQHandler(void) {
//HAL_CAN_IRQHandler(&hcan);
//}
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
//{
///* Get RX message */
//if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, can_msg.rx_msg) != HAL_OK)
//{
//Error_Handler(); }
//can_msg.ID = RxHeader.StdId; // ???????
//can_msg.length = RxHeader.DLC; // ??????
//CAN_SetRxFlag(1); // ???????
}
void USART1_IRQHandler_DataReceiveTest(void)
{
    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET)
    {
    RS485_EN_RESET();
              SEGGER_RTT_printf(0,"%sUSRAT1 Data Register Content : %x : \n%s",\
              RTT_CTRL_BG_RED,\
              (uint8_t)(huart1.Instance->DR & 0x00FF ),\
              RTT_CTRL_RESET);
              __HAL_UART_CLEAR_FLAG(&huart1,UART_FLAG_RXNE);
    }
    

}

void USART1_IRQHandler(void){

    g_fpUART_Handler(); 

}

void USART1_IRQHandler_Default(void)
{
    uint8_t USART1_i = 0;
    uint16_t CRC_7 = 0;
    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET)
    {

    RS485_EN_RESET();
        g_RS485_dataReceive = (uint8_t)(huart1.Instance->DR & 0x00FF);
        for(USART1_i=0;USART1_i<8;USART1_i++){
            g_pRS485_dataReceivePack[USART1_i]=g_pRS485_dataReceivePack[USART1_i+1];
        }
        g_pRS485_dataReceivePack[8]=g_RS485_dataReceive;

        CRC_7 = BRT38R0_CRC_CALC(g_pRS485_dataReceivePack, 7); // Calculate out CRC code.

        if(CRC_7 == (g_pRS485_dataReceivePack[6] + (g_pRS485_dataReceivePack[7] << 8)))// CRC matching check
        {
            if(g_pRS485_dataReceivePack[0] == 0x01)// Check If Encoder ID is correct.
            {
                //Get encoder position data.
                g_RS485_encoderData = 
                        (g_pRS485_dataReceivePack[3] << 24) + 
                        (g_pRS485_dataReceivePack[4] << 16) + 
                        (g_pRS485_dataReceivePack[5] << 8) + 
                        g_pRS485_dataReceivePack[6];
            }
        }else{

              SEGGER_RTT_printf(0,"%sReceived Data Pack : %s",RTT_CTRL_BG_RED,\
              RTT_CTRL_RESET);
              for(USART1_i=0;USART1_i<8;USART1_i++){
              SEGGER_RTT_printf(0,"%s%x%s",RTT_CTRL_BG_RED,\
                  g_pRS485_dataReceivePack[USART1_i],\
                  RTT_CTRL_RESET);
              }
              SEGGER_RTT_printf(0,"\n");
              SEGGER_RTT_printf(0,"%s!!!Encoder CRC Mismatch!!!%s\n",RTT_CTRL_BG_RED,RTT_CTRL_RESET);
              SEGGER_RTT_printf(0,"%sOriginal:\t[6]%x  , [7]%x%s\n",\
              RTT_CTRL_BG_BLUE, \
              g_pRS485_dataReceivePack[6], \
              g_pRS485_dataReceivePack[7], \
              RTT_CTRL_RESET);
              SEGGER_RTT_printf(0,"%sCalculated:[6]%x  , [7]%x%s\n\n\n",\
              RTT_CTRL_BG_BLUE,\
              (uint8_t)(CRC_7>>8), \
              (uint8_t)(CRC_7), \
              RTT_CTRL_RESET);

        }
        //Delay(100);
        __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
    }

    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET)
    {
        g_RS485_dataReceive = (uint8_t)(huart1.Instance->DR & 0x00FF);
        __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_ORE);
    }

    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_NE) != RESET)
    {
        __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_NE);
    }

    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE) != RESET)
    {
        __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_FE);
    }

    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_PE) != RESET)
    {
        __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_PE);
    }
}





/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim4;
/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{

  while (1);
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
    SEGGER_RTT_printf(0,"%sHard Fault Trigged.%s\n",RTT_CTRL_TEXT_RED,RTT_CTRL_RESET);
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  while (1);
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  while (1) {}
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{

  while (1) {}
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{

}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{

}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{

}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{

  HAL_IncTick();
	TimingDelay_Decrement();


}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */

 /* TO supress compilation error 
 static void USB_LP_CAN1_RX0_IRQHandler(void)
{

  HAL_CAN_IRQHandler(&hcan);

}
 */

/**
  * @brief This function handles CAN RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{

  HAL_CAN_IRQHandler(&hcan);

}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{

  HAL_TIM_IRQHandler(&htim4);

}


void Delayms(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}
void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

