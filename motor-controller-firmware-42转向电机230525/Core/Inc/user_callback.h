#ifndef _USER_CALLBACK_
#define _USER_CALLBACK_


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);


#endif
