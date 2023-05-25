#include "common.h"
#include "user_callback.h"


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint16_t i, ii, iii, iiii;					//06.23

    if(htim->Instance == htim4.Instance)
    {
        g_bTime_1ms = 1;
        g_Time_ms++;
        i++;
        ii++;
        iii++;
        iiii++;

        if(i == 5)
        {
            i = 0;
            g_bTime_5ms		=	1;
        }

        if(ii == 100)
        {
            ii = 0;
            g_bTime_100ms	=	1;
        }

        if(iii == 100)
        {
            iii = 0;
            g_bTime_500ms	=	1;
        }

        if(iiii == 1000)
        {
            iiii = 0;
            g_bTime_1s		=	1;
        }

   //     OledTimeMsFunc();	//ʵʱʱ��-- ��0Ϊֹ
//        if(i==2000) {
//            i=0;
//            if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == 0)Sfl_temp3 = Sfl_temp3+20000;
//            if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == 0)Sfl_temp3 = Sfl_temp3-20000;
//            if(Sfl_temp3==40000)Sfl_temp3=300000;
//            if(Sfl_temp3==-40000)Sfl_temp3=-300000;
//            if(Sfl_temp3==280000)Sfl_temp3=20000;
//            if(Sfl_temp3==-280000)Sfl_temp3=-20000;

//            if(Sfl_temp3>=0) {
//                tmc_spi_writeregister(motor1,0xA0-0x80,1); //�ٶ�ģʽ
//                tmc_spi_writeregister(motor1,0xA7-0x80,Sfl_temp3); //VMAX
//            } else if(Sfl_temp3<0) {
//                tmc_spi_writeregister(motor1,0xA0-0x80,2); //�ٶ�ģʽ
//                tmc_spi_writeregister(motor1,0xA7-0x80,-Sfl_temp3); //VMAX
//            }
//        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &RXHeader, RXmessage); //��ȡ����
				g_CANRxFlag = 1;
//        if( RXHeader.StdId ==  DEVICE_ID)
//        {
//            g_CANRxFlag = 1;
//        }
//        HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);//�ٴο��������ж�
    }
}



/** Button lib callbacks **/

void BTN1_PRESS_DOWN_Handler(void* btn) {}
void BTN1_PRESS_UP_Handler(void* btn) {}
void BTN1_PRESS_REPEAT_Handler(void* btn) {}
void BTN1_SINGLE_Click_Handler(void* btn) {}
void BTN1_DOUBLE_Click_Handler(void* btn) {}
void BTN1_LONG_PRESS_START_Handler(void* btn) {}
void BTN1_LONG_PRESS_HOLD_Handler(void* btn) {}

void BTN2_PRESS_DOWN_Handler(void* btn) {}
void BTN2_PRESS_UP_Handler(void* btn) {}
void BTN2_PRESS_REPEAT_Handler(void* btn) {}
void BTN2_SINGLE_Click_Handler(void* btn) {}
void BTN2_DOUBLE_Click_Handler(void* btn) {}
void BTN2_LONG_PRESS_START_Handler(void* btn) {}
void BTN2_LONG_PRESS_HOLD_Handler(void* btn) {}
