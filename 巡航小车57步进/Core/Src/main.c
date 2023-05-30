/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled_driver.h"
#include "oled_config.h"
#include "oled_basic.h"
#include "oled_font.h"
#include "oled_draw.h"
#include "test.h"
#include "TMC5160_Register.h"
#include "TMC5160_Mask_Shift.h"
#include "SPI_TMC.h"
#include "TMC5160.h"
#include "multi_button.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CAN_ID                  0x123
#define DEVICE_ID               0x01
#define GEAR                    1
#define RAD_TO_PUl              8153
typedef enum
{
    fsm_rt_err          = -1,    //!< fsm error, error code can be get from other interface
    fsm_rt_cpl          = 0,     //!< fsm complete
    fsm_rt_on_going     = 1,     //!< fsm on-going
} fsm_rt_t;

typedef struct
{
    uint16_t		State;    	//0stop   1speed   2 position
    int32_t			Speed;
    int32_t  		Position;
    uint16_t		Gear;
    int32_t         Torque;
} MotorControl_TypDef;

typedef struct
{
    float		Kp;
    float		Ki;
    float  	Kd;
		uint32_t Target;
		uint32_t Input;
    int32_t Err;
    float ErrLimit_Max;
    int32_t ErrNext;
    float ErrLast;
    float IntegralValue;
    int32_t ResultValue;
    int32_t ResultLimit_Max;
    int32_t ResultLimit_Min;
		int32_t ZeroThreshold;
		int32_t ResultUpLimit;
		int32_t ResultLast;
} Pid_TypDef;

typedef struct
{
    int16_t		Max;
    int16_t		Min;
} Limit_TypDef;

typedef union
{
    uint8_t  	U8[80];
    uint16_t 	U16[40];
    int16_t 	I16[40];
    uint32_t 	U32[20];
    int32_t 	I32[20];
    double  	D64[10];
    uint64_t  U64[10];
} Register_TypDef;
#define SPI_CS_HIGH_SIDE_ENCODER_ENABLE()  			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
#define SPI_CS_HIGH_SIDE_ENCODER_DISABLE() 			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RESET_FSM()             do { s_tState = START; } while(0)
#define FALSE 			        0
#define TRUE 				    1
#define ABS(__N)                ((__N)<0?-(__N):(__N))
#define OLED_STOP               do { HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_RESET); } while(0)
#define OLED_START()            do { HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_SET); } while(0)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t g_bTime_1ms = 0;
uint16_t g_bTime_5ms = 0;
uint16_t g_bTime_100ms = 0;
uint16_t g_bTime_500ms = 0;
uint16_t g_bTime_1s;
uint16_t g_bFindZero = 0;

uint16_t g_Time_ms = 0;
uint16_t g_bInput[2] = {0};
uint16_t g_bKey[2] = {0};
//uint16_t g_AngleHighSide_New = 0;
uint16_t g_AngleHighSide_Old = 0;
int16_t g_Speed[8];

//new add-----
uint16_t g_AngleHighSide_Circle = 1000;
uint16_t g_AngleHighSide_New = 16000;
int16_t g_AngleHighSide_Threshold = 1000;
int32_t g_AngleHighSide_All;
int16_t g_MotorID; 
uint32_t g_CanResiveNum = 0;
int8_t g_SpeedMod = 0;
int16_t g_SpeedRecord = 0;
//------------

uint16_t g_CANRxFlag = 0;
CAN_TxHeaderTypeDef TXHeader;
CAN_RxHeaderTypeDef RXHeader;

uint8_t TXmessage[8] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};
uint8_t RXmessage[8];
uint8_t MotorErr;
uint32_t pTxMailbox = 0;
uint32_t g_TMC5160_XACTUAL;
uint32_t g_TMC5160_VACTUAL;

uint32_t addr = 0x08008000;

Register_TypDef g_RxData = {0};
Register_TypDef g_Register = {0};
Pid_TypDef P_PID;
Pid_TypDef V_PID;
Pid_TypDef C_PID;

Limit_TypDef P_LIMIT;
Limit_TypDef V_LIMIT;
Limit_TypDef C_LIMIT;

MotorControl_TypDef g_MotorControlRegister = {0,0,0,14};

int Sfl_temp1, Sfl_temp2;
int32_t Sfl_temp3 = 0;

enum Button_IDs
{
    btn1_id,
    btn2_id,
};
struct Button btn1;
struct Button btn2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void Task_CAN(void);
void Task_Control(void);
void Task_OLED(void);
void Task_Key(void);
void Task_FindZero(void);
void Task_Detection(void);
uint8_t read_button_GPIO(uint8_t button_id);
void BTN1_PRESS_DOWN_Handler(void* btn);
void BTN1_PRESS_UP_Handler(void* btn);
void BTN1_PRESS_REPEAT_Handler(void* btn);
void BTN1_SINGLE_Click_Handler(void* btn);
void BTN1_DOUBLE_Click_Handler(void* btn);
void BTN1_LONG_PRESS_START_Handler(void* btn);
void BTN1_LONG_PRESS_HOLD_Handler(void* btn);

void BTN2_PRESS_DOWN_Handler(void* btn);
void BTN2_PRESS_UP_Handler(void* btn);
void BTN2_PRESS_REPEAT_Handler(void* btn);
void BTN2_SINGLE_Click_Handler(void* btn);
void BTN2_DOUBLE_Click_Handler(void* btn);
void BTN2_LONG_PRESS_START_Handler(void* btn);
void BTN2_LONG_PRESS_HOLD_Handler(void* btn);

void ReadEncoder_SPI_HighSide(void);

//new add-----
void P_PID_Init(void);
int32_t Rad2Motor(float Rad);
float Motor2Rad(int32_t motor);
//------------

//void BTN2_PRESS_DOWN_Handler(void* btn)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
	
int testNum1 = 0;
int testNum2 = 0;
int main(void)
{
		
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_CAN_Init();
    MX_USART1_UART_Init();
    MX_TIM4_Init();
    MX_SPI1_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start_IT(&htim4);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    OLED_START();

    HAL_Delay(100);
    InitGraph();
    HAL_Delay(100);

    HAL_GPIO_WritePin(TMC5160_SPI_MODE_GPIO_Port, TMC5160_SPI_MODE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TMC5160_SD_MODE_GPIO_Port, TMC5160_SD_MODE_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(TMC5160_EN_GPIO_Port, TMC5160_EN_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);

//    MotorTmc5160[motor1].Axis.speed = 5; //尽量设置不要超过12


//	tmc5160_SetPara(&MotorTmc5160[motor1],256,4,10,\
//	5,0,0,10000,MotorTmc5160[motor1].Axis.speed,\
//	10000,500,10,motor1);
//	tmc5160_InitConfig(&MotorTmc5160[motor1],motor1);

    tmc_spi_writeregister(motor1, 0xEC - 0x80, 0x000100C3);
    tmc_spi_writeregister(motor1, 0x90 - 0x80, 0x00060C0A);
    tmc_spi_writeregister(motor1, 0x91 - 0x80, 0x0000000A);
    tmc_spi_writeregister(motor1, 0x80 - 0x80, 0x00000004);
    tmc_spi_writeregister(motor1, 0x93 - 0x80, 0x000001A4);    //tmc_spi_writeregister(motor1, 0x93 - 0x80, 0x000001F4);

    tmc_spi_writeregister(motor1, 0xA4 - 0x80, 1000 ); 			//A1
    tmc_spi_writeregister(motor1, 0xA5 - 0x80, 0    ); 			//V1
    tmc_spi_writeregister(motor1, 0xA6 - 0x80, 30000 ); 			//AMAX
    tmc_spi_writeregister(motor1, 0xA7 - 0x80, 0    ); 	    	//VMAX
    tmc_spi_writeregister(motor1, 0xA8 - 0x80, 30000 ); 			//DMAX
    tmc_spi_writeregister(motor1, 0xAA - 0x80, 1400 ); 			//D1
    tmc_spi_writeregister(motor1, 0xAB - 0x80, 10   ); 			//VSTOP
//tmc_spi_writeregister(motor1,0xA0-0x80,0x00000000); 			//位置模式
    tmc_spi_writeregister(motor1, 0xA0 - 0x80, 0x00000001);     //速度模式

    tmc_spi_writeregister(motor1,TMC5160_XACTUAL,0x00000000);
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
/////////////////////////////////////////////////////////////////////

    button_init(&btn1, read_button_GPIO, 0, btn1_id);
    button_init(&btn2, read_button_GPIO, 0, btn2_id);

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
//    g_MotorControlRegister.Speed = 200000;
//////////////////////////////////////////////////////////////////////

//    HAL_Delay(DEVICE_ID * 1000);
		g_MotorID = 6;
		P_PID_Init();
//		P_PID.Target = g_AngleHighSide_All+100000;
		P_PID.Target = g_AngleHighSide_All;
//		g_MotorControlRegister.State = 1;
//		g_MotorControlRegister.Speed = 300000;
//		tmc_spi_writeregister(motor1, 0xA7 - 0x80, 10000); //VMAX
//    tmc_spi_writeregister(motor1, 0xA0 - 0x80, 2); //速度模式
    while (1)
    {
        if(g_bTime_1ms == 1)
        {
            g_bTime_1ms = 0;
						
//						g_MotorControlRegister.Speed -= 30;
//					  if(g_MotorControlRegister.Speed<50)
//							g_MotorControlRegister.State = 0;
					

//		g_MotorControlRegister.State = 2;
//		g_MotorControlRegister.Position = 151200;

					
					
					
            ReadEncoder_SPI_HighSide();
					
            Task_CAN();

            Task_Control();

            g_TMC5160_XACTUAL = tmc_spi_readregister(motor1,TMC5160_XACTUAL);
//		HAL_Delay(2000);	
//					
//		g_MotorControlRegister.State = 2;
//		g_MotorControlRegister.Position = -151200;
//		
//            Task_Control();
//						 Task_Control();

//		HAL_Delay(2000);	
		
					
					
					
					
        }
    }

    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */
    /** Common config
    */
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Regular Channel
    */
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

    /* USER CODE BEGIN CAN_Init 0 */

    /* USER CODE END CAN_Init 0 */

    /* USER CODE BEGIN CAN_Init 1 */

    /* USER CODE END CAN_Init 1 */
    hcan.Instance = CAN1;
		hcan.Init.Prescaler = 9;
		hcan.Init.Mode = CAN_MODE_NORMAL;
		hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
		hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
		hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
		hcan.Init.TimeTriggeredMode = DISABLE;
		hcan.Init.AutoBusOff = ENABLE;
		hcan.Init.AutoWakeUp = ENABLE;
		hcan.Init.AutoRetransmission = DISABLE;
		hcan.Init.ReceiveFifoLocked = DISABLE;
		hcan.Init.TransmitFifoPriority = DISABLE;
		if (HAL_CAN_Init(&hcan) != HAL_OK)
		{
			Error_Handler();
		}
    /* USER CODE BEGIN CAN_Init 2 */
    __HAL_CAN_ENABLE_IT(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);//使能can中断
    //  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.
    //CAN_ITConfig(CAN1, CAN_IT_FMP1, ENABLE);

    //buffer init
//  Can_FifoBuf_Init(&gpmag_rx, mag_rxbuffer, MAG_RXBUFFER_SIZE);

    /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_1LINE;
    hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 72-1;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 1000-1;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, TMC5160_EN_Pin|TMC5160_SPI_MODE_Pin|TMC5160_SD_MODE_Pin|GPIO_PIN_11
                      |STEP_Pin|TMC5160_SDI_Pin|SPI1_CS_Pin|OLED_SDA_Pin
                      |OLED_SCL_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(TMC5160_SCK_GPIO_Port, TMC5160_SCK_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, TMC5160_CSN_Pin|OLED_RES_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : LED1_Pin */
    GPIO_InitStruct.Pin = LED1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : KEY2_Pin KEY1_Pin */
    GPIO_InitStruct.Pin = KEY2_Pin|KEY1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : INPUT1_Pin INPUT2_Pin */
    GPIO_InitStruct.Pin = INPUT1_Pin|INPUT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : TMC5160_EN_Pin TMC5160_SPI_MODE_Pin TMC5160_SD_MODE_Pin PB11
                             STEP_Pin TMC5160_SCK_Pin TMC5160_SDI_Pin SPI1_CS_Pin */
    GPIO_InitStruct.Pin = TMC5160_EN_Pin|TMC5160_SPI_MODE_Pin|TMC5160_SD_MODE_Pin|GPIO_PIN_11
                          |STEP_Pin|TMC5160_SCK_Pin|TMC5160_SDI_Pin|SPI1_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : TMC5160_SDO_Pin */
    GPIO_InitStruct.Pin = TMC5160_SDO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(TMC5160_SDO_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : TMC5160_CSN_Pin */
    GPIO_InitStruct.Pin = TMC5160_CSN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TMC5160_CSN_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : OLED_RES_Pin */
    GPIO_InitStruct.Pin = OLED_RES_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(OLED_RES_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : OLED_SDA_Pin OLED_SCL_Pin */
    GPIO_InitStruct.Pin = OLED_SDA_Pin|OLED_SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
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

        OledTimeMsFunc();	//实时时间-- 到0为止
//        if(i==2000) {
//            i=0;
//            if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == 0)Sfl_temp3 = Sfl_temp3+20000;
//            if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == 0)Sfl_temp3 = Sfl_temp3-20000;
//            if(Sfl_temp3==40000)Sfl_temp3=300000;
//            if(Sfl_temp3==-40000)Sfl_temp3=-300000;
//            if(Sfl_temp3==280000)Sfl_temp3=20000;
//            if(Sfl_temp3==-280000)Sfl_temp3=-20000;

//            if(Sfl_temp3>=0) {
//                tmc_spi_writeregister(motor1,0xA0-0x80,1); //速度模式
//                tmc_spi_writeregister(motor1,0xA7-0x80,Sfl_temp3); //VMAX
//            } else if(Sfl_temp3<0) {
//                tmc_spi_writeregister(motor1,0xA0-0x80,2); //速度模式
//                tmc_spi_writeregister(motor1,0xA7-0x80,-Sfl_temp3); //VMAX
//            }
//        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &RXHeader, RXmessage); //获取数据
				g_CANRxFlag = 1;
//        if( RXHeader.StdId ==  DEVICE_ID)
//        {
//            g_CANRxFlag = 1;
//        }
        HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);//再次开启接收中断
    }
}


//void CAN1_RX0_IRQHandler(void) {
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
//}

void P_PID_Init(void)
{
	P_PID.Kp = 30;
	P_PID.Ki = 0;
	P_PID.Kd = 0;
	
	ReadEncoder_SPI_HighSide();
	//P_PID.Target = g_AngleLowSide_New;
	P_PID.Target = g_AngleHighSide_All;
	P_PID.ResultLimit_Max = 100000;
	P_PID.ResultLimit_Min = -P_PID.ResultLimit_Max;
	P_PID.ErrNext = 0;
	P_PID.ZeroThreshold = 50;//5
	P_PID.ResultUpLimit = 10000;
	P_PID.ResultLast = 0;
}

int32_t Rad2Motor(float Rad)
{
	int32_t motor;
	motor = Rad * 16384 / 3.1415926535 ;
	return motor;
}

float Motor2Rad(int32_t motor)
{
	float Rad;
	Rad = motor * 3.1415926535 / 16384;
	return Rad;
}

void Position2Speed_PID(void)
{
//		P_PID.Input = g_AngleLowSide_New;
		P_PID.Input = g_AngleHighSide_All;
		P_PID.Err = P_PID.Input - P_PID.Target;
		if(P_PID.Err < P_PID.ZeroThreshold && P_PID.Err > -P_PID.ZeroThreshold)
		{
			P_PID.Err = 0;
			g_MotorControlRegister.State = 0;
			g_MotorControlRegister.Speed = 0;
			return;
		}
		P_PID.IntegralValue += P_PID.Err;

    
    P_PID.ResultValue = P_PID.Kp*P_PID.Err+ \
                      P_PID.Ki* P_PID.IntegralValue - \
                      P_PID.Kd*(P_PID.Err-P_PID.ErrNext);
    
    if(P_PID.ResultValue>P_PID.ResultLimit_Max)
        P_PID.ResultValue = P_PID.ResultLimit_Max;
    else if(P_PID.ResultValue<P_PID.ResultLimit_Min)
        P_PID.ResultValue = P_PID.ResultLimit_Min;

		if(P_PID.ResultValue < 0)
		{
			if(P_PID.ResultValue - P_PID.ResultLast < -P_PID.ResultUpLimit)
			{
				P_PID.ResultValue = P_PID.ResultLast - P_PID.ResultUpLimit;
			}
		}
		else
		{
			if(P_PID.ResultValue - P_PID.ResultLast > P_PID.ResultUpLimit)
			{
				P_PID.ResultValue = P_PID.ResultLast + P_PID.ResultUpLimit;
			}
		}
			
    //P_PID.ErrLast=P_PID.ErrNext;
		P_PID.ResultLast = P_PID.ResultValue;
    P_PID.ErrNext    = P_PID.Err;
		
		g_MotorControlRegister.State = 1;
		g_MotorControlRegister.Speed = P_PID.ResultValue;
}

void Task_CAN(void)
{
		uint16_t u16 = 0;
		int16_t int16 = 0; 
		if(g_CANRxFlag == 1)
		{
				
			g_CanResiveNum ++;
			g_CANRxFlag = 0;
			if(RXHeader.StdId == 0x0600 + g_MotorID)
			{
				g_AngleHighSide_Old += 1;
				switch( RXmessage[2] )
				{
					case 0x60:
						switch( RXmessage[1] )
						{
							case 0x70:
								switch( RXmessage[3] )
								{
									case 0x00:
										switch( RXmessage[0] )
										{
											case 0x2B:
												int16 = (RXmessage[4] + RXmessage[5]*256);
												P_PID.Target = ((((double)int16)*20/1000)/3.14+2000)*32768/2;
												TXmessage[0] = 0x60;
												TXmessage[1] = RXmessage[1];
												TXmessage[2] = RXmessage[2];
												TXmessage[3] = RXmessage[3];
												TXmessage[4] = 00;
												TXmessage[5] = 00;
												TXmessage[6] = 00;
												TXmessage[7] = 00;
												TXHeader.ExtId = 0X580 + g_MotorID;
												break;
										}
										break;
								}
								break;
							case 0x71:
								switch( RXmessage[3] )
								{
									case 0x00:
										switch( RXmessage[0] )
										{
											case 0x40:
												TXmessage[0] = 0x43;
												TXmessage[1] = RXmessage[1];
												TXmessage[2] = RXmessage[2];
												TXmessage[3] = RXmessage[3];
												if(g_MotorControlRegister.State == 0)
													TXmessage[5] = 01;
												else
													TXmessage[5] = 00;
												TXmessage[5] = 00;
												TXmessage[6] = 00;
												TXmessage[7] = 00;
												TXHeader.ExtId = 0X580 + g_MotorID;
												break;
										}
										break;
								}
								break;
						}
						break;
				}
				TXHeader.DLC = 8;
				TXHeader.IDE = CAN_ID_STD;
				TXHeader.RTR = CAN_RTR_DATA;
				TXHeader.TransmitGlobalTime = DISABLE;
				HAL_CAN_AddTxMessage(&hcan,&TXHeader,TXmessage,&pTxMailbox);
			}

    }
}

void Task_Control(void)
{
		if(!g_SpeedMod)
		{
			Position2Speed_PID();
		}
    static enum
    {
        START  = 0,
        WRITE,
        RESET,
    } s_tState = START;

    switch( s_tState )
    {
    case START:
        if(TRUE)
        {
            s_tState = WRITE;
        }

    case WRITE:
        if(1)
        {

            switch( g_MotorControlRegister.State )
            {
            case 0:
                tmc_spi_writeregister(motor1, 0xA7 - 0x80, 0); //VMAX
                tmc_spi_writeregister(motor1, 0xA0 - 0x80, 1); //速度模式
                break;

            case 1:
                if(g_MotorControlRegister.Speed >= 0)
                {
                    tmc_spi_writeregister(motor1, 0xA7 - 0x80, g_MotorControlRegister.Speed); //VMAX
                    tmc_spi_writeregister(motor1, 0xA0 - 0x80, 2); //速度模式
                }
                else
                {
                    tmc_spi_writeregister(motor1, 0xA7 - 0x80, -g_MotorControlRegister.Speed); //VMAX
                    tmc_spi_writeregister(motor1, 0xA0 - 0x80, 1); //速度模式
                }

                break;

            case 2:

                g_MotorControlRegister.Speed = 10000*g_MotorControlRegister.Gear;
                if((DEVICE_ID == 1)&&(g_MotorControlRegister.Position>465000)) {
                    g_MotorControlRegister.Position = 465000;
                }
                if((DEVICE_ID == 2)&&(g_MotorControlRegister.Position>250000)) {
                    g_MotorControlRegister.Position = 250000;
                }
                tmc_spi_writeregister(motor1, TMC5160_XTARGET, g_MotorControlRegister.Position);
                tmc_spi_writeregister(motor1, 0xA7 - 0x80, g_MotorControlRegister.Speed);
                tmc_spi_writeregister(motor1, 0xA0 - 0x80, 0); //位置模式
                break;
            case 3:
                g_bFindZero = 1;
                break;
            case 4:

                break;
            }

            s_tState = RESET;
        }

        break;

    case RESET:

        RESET_FSM() ;

    }
}

void Task_OLED(void) {
//    static uint16_t s_whTime1, s_whTime2, s_whDelay;
    static enum
    {
        START  = 0,
        WRITE,
        UPDATA,
        RESET,
    } s_tState = START;
    switch( s_tState )
    {
    case START:
        if(TRUE) {
            s_tState = WRITE;
        }
    case WRITE:
        if(TRUE) {
            ClearScreen	(	);
            SetFontSize	(1);
            DrawString	(0, 0, "Pst:");
            //DrawNum			(24, 0, g_MotorControlRegister.Position, 10);
            DrawNum			(24, 0, tmc_spi_readregister(motor1,TMC5160_XACTUAL), 10);
            DrawString	(0, 8, "Spd:");
            DrawNum			(24, 8, tmc_spi_readregister(motor1,TMC5160_VACTUAL), 10);
            tmc_spi_readregister(motor1,TMC5160_XACTUAL);
            s_tState = UPDATA;
        }
        break;
    case UPDATA:
        if(TRUE) {
            UpdateScreen();
            s_tState = RESET;
        }
        break;
    case RESET:
        RESET_FSM() ;
        break;
    }
}
void Task_FindZero(void)
{
    static uint16_t s_whTime1, s_whTime2, s_whDelay,s_whAngleDelay;
    static enum
    {
        START  = 0,
        Delay1,
        Delay2,
        Delay3,
        RESET,
    } s_tState = START;

    switch( s_tState )
    {
    case START:
        if(TRUE == g_bFindZero)
        {
            g_bFindZero = 0;
            g_MotorControlRegister.State = 1;
            g_MotorControlRegister.Speed = -20000;
            tmc_spi_writeregister(motor1, 0xA6 - 0x80, 20000 ); 			//AMAX
            tmc_spi_writeregister(motor1, 0xA8 - 0x80, 20000 ); 			//DMAX
            s_whTime1 = g_Time_ms;
            s_tState = Delay1;
        }
        break;
    case Delay1:
        if(1)
        {
            s_whTime2 = g_Time_ms;

            if(s_whTime2 > s_whTime1) {
                s_whDelay = s_whTime2-s_whTime1;
            }
            else {
                s_whDelay = 65535+s_whTime2-s_whTime1;
            }

            if(s_whDelay > 100) {
                s_tState = Delay2;
                s_whTime1 = g_Time_ms;
                g_AngleHighSide_Old = g_AngleHighSide_New ;
            }
        }
        break;
    case Delay2:
        if(1)
        {
            s_whTime2 = g_Time_ms;

            if(s_whTime2 > s_whTime1) {
                s_whDelay = s_whTime2-s_whTime1;
            }
            else {
                s_whDelay = 65535 + s_whTime2-s_whTime1;
            }

            if(s_whDelay > 12) {

                if(g_AngleHighSide_New > g_AngleHighSide_Old) {
                    s_whAngleDelay = g_AngleHighSide_New-g_AngleHighSide_Old;
                }
                else {
                    s_whAngleDelay = 32768 + g_AngleHighSide_New - g_AngleHighSide_Old;
                }
                if(g_Speed[0] < 5) {

                    g_MotorControlRegister.State = 0;
//                    tmc_spi_writeregister(motor1, TMC5160_XACTUAL,0);
                    s_whTime1 = g_Time_ms;
                    s_tState = Delay3;
                } else {
                    s_whTime1 = g_Time_ms;
                }
                g_AngleHighSide_Old = g_AngleHighSide_New  ;
            }
        }

        break;
    case Delay3:
        if(1)
        {
            s_whTime2 = g_Time_ms;

            if(s_whTime2 > s_whTime1) {
                s_whDelay = s_whTime2-s_whTime1;
            }
            else {
                s_whDelay = 65535 + s_whTime2-s_whTime1;
            }

            if(s_whDelay > 200) {
                tmc_spi_writeregister(motor1, 0xA6 - 0x80, 3000 ); 			//AMAX
                tmc_spi_writeregister(motor1, 0xA8 - 0x80, 3000 ); 			//DMAX
                tmc_spi_writeregister(motor1, TMC5160_XACTUAL,0);
                s_tState = RESET;
            }
        }
        break;
    case RESET:

        RESET_FSM() ;

    }
}

float PIDCalculate(Pid_TypDef PID)
{
    if(PID.Err>PID.ErrLimit_Max)
        PID.Err = PID.ErrLimit_Max;
    
    PID.IntegralValue += PID.Err;

//    PID.ResultValue = PID.Kp*(PID.Err-PID.ErrNext)+\
//                      (PID.Ki*PID.Err)+ \
//                      PID.Kd*(PID.Err-2*PID.ErrNext+PID.ErrLast);
    
    PID.ResultValue = PID.Kp*PID.Err+ \
                      PID.Ki* PID.IntegralValue+ \
                      PID.Kd*(PID.Err-PID.ErrNext);
    
    if(PID.ResultValue>PID.ResultLimit_Max)
        PID.ResultValue = PID.ResultLimit_Max;
    else if(PID.ResultValue<PID.ResultLimit_Min)
        PID.ResultValue = PID.ResultLimit_Min;

    PID.ErrLast=PID.ErrNext;
    PID.ErrNext=PID.Err;

}







void ReadEncoder_SPI_HighSide(void)
{
    static uint16_t s_AngleHighSide_Old,i;
    static int32_t s_Speed[4],s_Temp;
    uint16_t cmd = 0,result_highside_original;
    cmd = 0x8020;
    SPI_CS_HIGH_SIDE_ENCODER_ENABLE();
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&cmd, 1, 0xff);
    __HAL_SPI_DISABLE(&hspi1);
    HAL_SPI_Receive(&hspi1, (uint8_t *)&result_highside_original, 1, 0xff);
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

uint8_t read_button_GPIO(uint8_t button_id)
{
    // you can share the GPIO read function with multiple Buttons
    switch(button_id)
    {
    case btn1_id:
        return !(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin));

//			break;

    case btn2_id:
        return !(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin));

//			break;

    default:
        return 0;
//			break;
    }
}

void Task_Detection(void)
{
    static uint16_t s_whTime1, s_whTime2, s_whDelay;
    static enum
    {
        START  = 0,
        Delay1,
        Delay2,
        Delay3,
        RESET,
    } s_tState = START;

    switch( s_tState )
    {
    case START:
        if(TRUE == g_MotorControlRegister.State)
        {
            s_whTime1 = g_Time_ms;
            s_tState = Delay1;
        }
        break;
    case Delay1:
        if(1)
        {
            s_whTime2 = g_Time_ms;

            if(s_whTime2 > s_whTime1) {
                s_whDelay = s_whTime2-s_whTime1;
            }
            else {
                s_whDelay = 65535+s_whTime2-s_whTime1;
            }

            if(s_whDelay > 100) {
                s_tState = Delay2;
                s_whTime1 = g_Time_ms;
            }
        }
        break;
    case Delay2:
        if(1)
        {
            s_whTime2 = g_Time_ms;

            if(s_whTime2 > s_whTime1) {
                s_whDelay = s_whTime2-s_whTime1;
            }
            else {
                s_whDelay = 65535 + s_whTime2-s_whTime1;
            }

            if(s_whDelay > 4) {
                if(g_Speed[0] < 12) {
                    s_whTime1 = g_Time_ms;
                    s_tState = Delay3;
                } else {
                    s_whTime1 = g_Time_ms;
                }
            }
        }

        break;
    case Delay3:
        if(1)
        {
            s_whTime2 = g_Time_ms;

            if(s_whTime2 > s_whTime1) {
                s_whDelay = s_whTime2-s_whTime1;
            }
            else {
                s_whDelay = 65535 + s_whTime2-s_whTime1;
            }
            if(s_whDelay > 4) {
                if(g_Speed[0] < 12) {

                    g_MotorControlRegister.State = 0;
                    s_tState = RESET;
                } else {
                    s_whTime1 = g_Time_ms;
                }
            }
        }
        break;
    case RESET:

        RESET_FSM() ;

    }
}

void Task_Key(void)
{
//    static uint16_t s_whTime1, s_whTime2, s_whDelay;
    static enum
    {
        START  = 0,
        MISSION,
        RESET,
    } s_tState = START;

    switch( s_tState )
    {
    case START:
        if(TRUE)
        {
            s_tState = MISSION;
        }

    case MISSION:
        if(TRUE)
        {
            button_ticks();
            s_tState = RESET;
        }

        break;

    case RESET:
        RESET_FSM() ;
    }
}
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
/* USER CODE END 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
