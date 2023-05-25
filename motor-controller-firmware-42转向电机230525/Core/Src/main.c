
/* moses added */
#include "bsp_init.h"
#include "msp_init.h"
#include "inner_algorithm.h"
#include "multi_task.h" 
#include "user_callback.h"

extern ADC_HandleTypeDef hadc1 = {0};

extern CAN_HandleTypeDef hcan = {0};

extern SPI_HandleTypeDef hspi1 = {0};
extern SPI_HandleTypeDef hspi2 = {0};

extern TIM_HandleTypeDef htim4 = {0};

extern UART_HandleTypeDef huart1 = {0};

extern uint16_t g_bFindZero = 0;
extern uint16_t g_bTime_1ms = 0;
extern uint16_t g_bTime_5ms = 0;
extern uint16_t g_bTime_100ms = 0;
extern uint16_t g_bTime_500ms = 0;
extern uint16_t g_bTime_1s = 0;

extern uint16_t g_Time_ms = 0;
extern uint16_t g_bInput[2] = {0};
extern uint16_t g_bKey[2] = {0};
extern int16_t g_Speed[8] = {0};
//new add-----
extern uint8_t g_AngleHighSide_Circle = 60;
extern uint16_t g_AngleHighSide_New = 8000;
extern int16_t g_AngleHighSide_Threshold = 1000;
extern int32_t g_AngleHighSide_All = 0;
extern uint16_t g_AngleHighSide_Old = 0;
extern int16_t g_MotorID = 2; 
extern uint32_t g_CanResiveNum = 0;
extern uint16_t g_AngleLowSide_New = 0;
//------------

extern uint16_t g_CANRxFlag = 0;
extern CAN_TxHeaderTypeDef TXHeader = {0};
extern CAN_RxHeaderTypeDef RXHeader = {0};

extern uint8_t TXmessage[8] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};
extern uint8_t RXmessage[8] = {0};
extern uint8_t MotorErr= 0;
extern uint32_t pTxMailbox = 0;
extern uint32_t g_TMC5160_XACTUAL = 0;
extern uint32_t g_TMC5160_VACTUAL = 0;
extern uint32_t g_TMC5160_TSTEP = 0;

extern uint32_t addr = 0x08008000;

extern Register_TypDef g_RxData = {0};
extern Register_TypDef g_Register = {0};
extern Pid_TypDef P_PID = {0};
extern Pid_TypDef V_PID = {0};
extern Pid_TypDef C_PID = {0};

extern Limit_TypDef P_LIMIT = {0};
extern Limit_TypDef V_LIMIT = {0};
extern Limit_TypDef C_LIMIT = {0};

extern MotorControl_TypDef g_MotorControlRegister = {0,0,0,14};

extern int Sfl_temp1, Sfl_temp2 = 0;
extern int32_t Sfl_temp3 = 0;

extern struct Button btn1 = {0};
extern struct Button btn2 = {0};

/* Low Side Encode global scope data */
extern uint8_t g_RS485_dataSend = 0;
extern uint8_t g_pRS485_dataSendPack[] =
            {0x00 , 0x00 , 0x00 , 0x00 ,
             0x00 , 0x00 , 0x00 , 0x00 ,
             0x00 , 0x00 , 0x00 , 0x00};
            
extern uint8_t g_RS485_dataReceive = 0;
extern uint8_t g_pRS485_dataReceivePack[] = 
            {0x00 , 0x00 , 0x00 , 0x00 ,
             0x00 , 0x00 , 0x00 , 0x00 ,
             0x00 , 0x00 , 0x00 , 0x00};
            
extern uint8_t g_iDataReceive = 0;

extern uint32_t g_RS485_encoderData = 0;

extern void (*g_fpUART_Handler)(void) = NULL;

#include "common.h"
uint16_t g_AngleLowSide;
int main(void)
{
	

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
//x  MX_ADC1_Init();
//x  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
//x  MX_SPI1_Init();
//x  MX_SPI2_Init();
  
    /* User defined HAL initializations */
    //////////TIMER//////////////
    HAL_TIM_Base_Start_IT(&htim4);

    //////////CAN//////////////
    CAN_FilterTypeDef can_Filter = {0};

    can_Filter.FilterIdHigh = 0;
    can_Filter.FilterIdLow = 0;
    can_Filter.FilterMaskIdHigh = 0;
    can_Filter.FilterMaskIdLow = 0;
    can_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_Filter.FilterBank = 0;
    can_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_Filter.FilterActivation = CAN_FILTER_ENABLE;
    HAL_CAN_ConfigFilter(&hcan, &can_Filter);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan);

    
    bsp_oled_init();
    bsp_button_init();
    bsp_tmc_init();
    ////////////////////////////
    
    /** motor settings on-the-fly **/
    g_MotorControlRegister.Speed = 30000;
		g_MotorControlRegister.State = 1;

//    HAL_Delay(DEVICE_ID * 1000);
		g_MotorID = 2;
//x		ReadEncoder_SPI_HighSide();
		P_PID_Init();
		//P_PID.Target = BRT38R0_READ_VALUE_1();

//  TASK_RTT_PRINT_TEST();

    while (1)
    {
//        if ((0==HAL_GPIO_ReadPin(INPUT1_GPIO_Port,INPUT1_Pin))&&( 0 != g_MotorControlRegister.State)) {
//            g_MotorControlRegister.Position = 250000;
//            tmc_spi_writeregister(motor1, TMC5160_XTARGET, g_MotorControlRegister.Position);
//        }
        if(g_bTime_1ms == 1)
        {
					g_bTime_1ms = 0;
					g_AngleLowSide = BRT38R0_READ_VALUE_1();
					Task_CAN();
					Task_Control();
					
					
//					  g_MotorControlRegister.Speed = 40000;
//						tmc_spi_writeregister(motor1, 0xA7 - 0x80, g_MotorControlRegister.Speed); //VMAX
//            tmc_spi_writeregister(motor1, 0xA0 - 0x80, 1); //�ٶ�ģʽ
					
//x            ReadEncoder_SPI_HighSide();
            //ReadEncoder_SPI_LowSide();
//x            ReadEncoder_RS485_LowSide();

//x            Task_CAN();
//x            Task_Control();

//x            g_TMC5160_XACTUAL = tmc_spi_readregister(motor1,TMC5160_XACTUAL);
//					  g_TMC5160_TSTEP = tmc_spi_readregister(motor1,TMC5160_TSTEP);
        }
//				if(g_bTime_1s == 1)
//				{
//						g_bTime_1s = 0;
//            
//            SEGGER_RTT_printf(0,\
//                            "%x\n",\
//                            BRT38R0_READ_ANGULAR_VELOCITY()\
//                            );
//            SEGGER_RTT_printf(0,\
//                            "%x\n",\
//                            BRT38R0_READ_VALUE_2()\
//                            );
//				}

//OLED_FULL();
//OLED_CLS();
//HAL_Delay(1000);
//demo();
//motor_zeroing(motor1,reversedir,nowait);//�ȴ����ִ�е�λ
//motor_wait_zeroing_ok(motor1);
//while(1){
//Sfl_temp1 =tmc_spi_readregister(motor1,TMC5160_XACTUAL);
//HAL_Delay(100);
//tmc_spi_writeregister(motor1,TMC5160_XTARGET,Sfl_temp2);
//ShowStars();
//HAL_Delay(100);
//	}

    }

}






#define READ_ANGLE_WITHOUT_SAFETYWORD 0x8020

void ReadEncoder_SPI_HighSide(void)
{
    static uint16_t s_AngleHighSide_Old,i;
    static int32_t s_Speed[4],s_Temp;
    uint16_t cmd = 0,result_highside_original;
    cmd = READ_ANGLE_WITHOUT_SAFETYWORD;
    SPI_CS_HIGH_SIDE_ENCODER_ENABLE();
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&cmd, 1, 0xff);
    __HAL_SPI_DISABLE(&hspi2);
    HAL_SPI_Receive(&hspi2, (uint8_t *)&result_highside_original, 1, 0xff);
    SPI_CS_HIGH_SIDE_ENCODER_DISABLE();
		if(g_AngleHighSide_New < g_AngleHighSide_Threshold)
		{
			g_AngleHighSide_New = result_highside_original& 0x7fff;
			if(g_AngleHighSide_New > 32768 - g_AngleHighSide_Threshold)
				g_AngleHighSide_Circle --;
		}
		else if(g_AngleHighSide_New > 32768 - g_AngleHighSide_Threshold)
		{
			g_AngleHighSide_New = result_highside_original& 0x7fff;
			if(g_AngleHighSide_New < g_AngleHighSide_Threshold)
				g_AngleHighSide_Circle ++;
		}
		else
			g_AngleHighSide_New = result_highside_original& 0x7fff;
		g_AngleHighSide_All = ((int)g_AngleHighSide_Circle) * 32768 + g_AngleHighSide_New;
			

}


extern __IO uint32_t TimingDelay =0;





/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();

    while (1)
    {
    }

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

