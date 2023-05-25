#ifndef _COMMON_H
#define _COMMON_H

#include "main.h"
#include "stm32f1xx_it.h"

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
#include "user_callback.h"
#include "BRT38R0.h"

/* Segger RTT support */
#include "SEGGER_RTT.h"


#define CAN_ID                  0x123
#define DEVICE_ID               0x01
#define GEAR                    1
#define RAD_TO_PUl              8153

#define RESET_FSM()             do { s_tState = START; } while(0)
#define FALSE 			        0
#define TRUE 				    1
#define ABS(__N)                ((__N)<0?-(__N):(__N))
#define OLED_STOP               do { HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_RESET); } while(0)
#define OLED_START()            do { HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_SET); } while(0)
#define CAN_RES_ON           		do { HAL_GPIO_WritePin(CAN_SLC_GPIO_Port, CAN_SLC_Pin, GPIO_PIN_RESET); } while(0)   

#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define KEY2_Pin GPIO_PIN_14
#define KEY2_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_15
#define KEY1_GPIO_Port GPIOC
#define INPUT1_Pin GPIO_PIN_2
#define INPUT1_GPIO_Port GPIOA
#define INPUT2_Pin GPIO_PIN_3
#define INPUT2_GPIO_Port GPIOA
#define CAN_SLC_Pin GPIO_PIN_4
#define CAN_SLC_GPIO_Port GPIOA
#define DM_A_Pin GPIO_PIN_5
#define DM_A_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_6
#define PWM2_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_7
#define PWM1_GPIO_Port GPIOA
#define TMC5160_SCK_Pin GPIO_PIN_0
#define TMC5160_SCK_GPIO_Port GPIOB
#define TMC5160_SDO_Pin GPIO_PIN_1
#define TMC5160_SDO_GPIO_Port GPIOB
#define TMC5160_SDI_Pin GPIO_PIN_2
#define TMC5160_SDI_GPIO_Port GPIOB
#define TMC5160_SD_MODE_Pin GPIO_PIN_10
#define TMC5160_SD_MODE_GPIO_Port GPIOB
#define TMC5160_REFR_DIR_Pin GPIO_PIN_11
#define TMC5160_REFR_DIR_GPIO_Port GPIOB
#define TMC5160_REFL_STEP_Pin GPIO_PIN_12
#define TMC5160_REFL_STEP_GPIO_Port GPIOB
#define SPI5012_CSQ_Pin GPIO_PIN_14
#define SPI5012_CSQ_GPIO_Port GPIOB
#define TMC5160_CSN_Pin GPIO_PIN_8
#define TMC5160_CSN_GPIO_Port GPIOA
#define TMC5160_EN_Pin GPIO_PIN_11
#define TMC5160_EN_GPIO_Port GPIOA
#define TMC5160_SPI_MODE_Pin GPIO_PIN_12
#define TMC5160_SPI_MODE_GPIO_Port GPIOA
#define OLED_RES_Pin GPIO_PIN_15
#define OLED_RES_GPIO_Port GPIOA
#define OLED_SDA_Pin GPIO_PIN_6
#define OLED_SDA_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_7
#define OLED_SCL_GPIO_Port GPIOBdefine CAN_RES_OFF        			do { HAL_GPIO_WritePin(CAN_SLC_GPIO_Port, CAN_SLC_Pin, GPIO_PIN_SET); } while(0)

#define RS485_EN_PORT GPIOB
#define RS485_EN_PIN GPIO_PIN_4

/* Special Handling for PB4-JRESET pin*/
#define AFIO_MAPR_ADDRESS 0x40010004


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
#define SPI_CS_HIGH_SIDE_ENCODER_ENABLE()  			HAL_GPIO_WritePin(SPI5012_CSQ_GPIO_Port, SPI5012_CSQ_Pin, GPIO_PIN_RESET);
#define SPI_CS_HIGH_SIDE_ENCODER_DISABLE() 			HAL_GPIO_WritePin(SPI5012_CSQ_GPIO_Port, SPI5012_CSQ_Pin, GPIO_PIN_SET);

// Refer to the project schematic connections.
#define RS485_EN_SET()    HAL_GPIO_WritePin(RS485_EN_PORT,RS485_EN_PIN,GPIO_PIN_SET);
#define RS485_EN_RESET()  HAL_GPIO_WritePin(RS485_EN_PORT,RS485_EN_PIN,GPIO_PIN_RESET);

extern ADC_HandleTypeDef hadc1;

extern CAN_HandleTypeDef hcan;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart1;

extern uint16_t g_bFindZero;
extern uint16_t g_bTime_1ms;
extern uint16_t g_bTime_5ms;
extern uint16_t g_bTime_100ms;
extern uint16_t g_bTime_500ms;
extern uint16_t g_bTime_1s;

extern uint16_t g_Time_ms;
extern uint16_t g_bInput[2];
extern uint16_t g_bKey[2];
extern int16_t g_Speed[8];
//new add-----
extern uint8_t g_AngleHighSide_Circle;
extern uint16_t g_AngleHighSide_New;
extern int16_t g_AngleHighSide_Threshold;
extern int32_t g_AngleHighSide_All;
extern uint16_t g_AngleHighSide_Old;
extern int16_t g_MotorID;
extern uint32_t g_CanResiveNum;
extern uint16_t g_AngleLowSide_New;
//------------

extern uint16_t g_CANRxFlag;
extern CAN_TxHeaderTypeDef TXHeader;
extern CAN_RxHeaderTypeDef RXHeader;

extern uint8_t TXmessage[8];
extern uint8_t RXmessage[8];
extern uint8_t MotorErr;
extern uint32_t pTxMailbox;
extern uint32_t g_TMC5160_XACTUAL;
extern uint32_t g_TMC5160_VACTUAL;
extern uint32_t g_TMC5160_TSTEP;

extern uint32_t addr;

extern Register_TypDef g_RxData;
extern Register_TypDef g_Register;
extern Pid_TypDef P_PID;
extern Pid_TypDef V_PID;
extern Pid_TypDef C_PID;

extern Limit_TypDef P_LIMIT;
extern Limit_TypDef V_LIMIT;
extern Limit_TypDef C_LIMIT;

extern MotorControl_TypDef g_MotorControlRegister;

extern int Sfl_temp1, Sfl_temp2;
extern int32_t Sfl_temp3;

enum Button_IDs
{
    btn1_id,
    btn2_id,
};
extern struct Button btn1;
extern struct Button btn2;


/* Low side encoder global scope data */
extern uint8_t g_RS485_dataSend;
extern uint8_t g_pRS485_dataSendPack[];


extern uint8_t g_RS485_dataReceive;
extern uint8_t g_pRS485_dataReceivePack[];

extern uint8_t g_iDataReceive;
extern uint32_t g_RS485_encoderData;

/* Exported function pointers */
extern void (*g_fpUART_Handler)(void);

/* Private function prototypes -----------------------------------------------*/
void Task_Control(void);
void Task_OLED(void);
void Task_Key(void);
void Task_FindZero(void);
void Task_Detection(void);

uint8_t read_button_gpio(uint8_t button_id);
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
void ReadEncoder_SPI_LowSide(void);
void ReadEncoder_RS485_LowSide(void);
//new add-----
void P_PID_Init(void);
int32_t Rad2Motor(float Rad);
float Motor2Rad(int32_t motor);
//------------
extern uint16_t g_AngleLowSide;

extern __IO uint32_t TimingDelay;

void Delayms(__IO uint32_t nCount);
void Delay(__IO uint32_t nTime);
void TimingDelay_Decrement(void);


#endif
