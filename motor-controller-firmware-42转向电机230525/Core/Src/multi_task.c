#include "common.h"
#include "multi_task.h"


void Task_Control(void)
{
//    static uint16_t s_whTime1, s_whTime2, s_whDelay;
		Position2Speed_PID();
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
                tmc_spi_writeregister(motor1, 0xA0 - 0x80, 1); //�ٶ�ģʽ
                break;

            case 1:
                if(g_MotorControlRegister.Speed >= 0)
                {
                    tmc_spi_writeregister(motor1, 0xA7 - 0x80, g_MotorControlRegister.Speed); //VMAX
                    tmc_spi_writeregister(motor1, 0xA0 - 0x80, 1); //�ٶ�ģʽ
                }
                else
                {
                    tmc_spi_writeregister(motor1, 0xA7 - 0x80, -g_MotorControlRegister.Speed); //VMAX
                    tmc_spi_writeregister(motor1, 0xA0 - 0x80, 2); //�ٶ�ģʽ
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
                tmc_spi_writeregister(motor1, 0xA0 - 0x80, 0); //λ��ģʽ
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
    /*
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
    */
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

void Task_CAN(void)
{
		if(g_CANRxFlag == 1)
		{
			g_CanResiveNum ++;
			g_CANRxFlag = 0;
			
			if(RXHeader.ExtId == 0x0600 + g_MotorID)
			{
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
												P_PID.Target = (RXmessage[4] + RXmessage[4]*256)/1000/3.14*32768;
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
			
			
//				if(RXmessage[0] == g_MotorID)
//        {
//            switch( RXmessage[1] )
//            {
//            case 0:
//                switch( RXmessage[2] )
//                {
//                case 5:
//                    P_PID.Kp = RXmessage[3];
//                    P_PID.Ki = RXmessage[4];
//                    P_PID.Kd = RXmessage[5];
//										P_PID.ResultLimit_Max = (RXmessage[6] * 256 + RXmessage[7])*1000;
//										P_PID.ResultLimit_Min = -P_PID.ResultLimit_Max;
//                    break;
//                }
//								TXmessage[0] = 0x10 + g_MotorID;
//								TXmessage[1] = RXmessage[1];
//								TXmessage[2] = RXmessage[2];
//								TXmessage[3] = RXmessage[3];
//								TXmessage[4] = RXmessage[4];
//								TXmessage[5] = RXmessage[5];
//								TXmessage[6] = RXmessage[6];
//								TXmessage[7] = RXmessage[7];
//                break;
//            case 1:
//                switch( RXmessage[2] )
//                {
//                case 1:
//										
//                    //g_MotorControlRegister.Position = -1 * (RXmessage[3]>>7) * ( (0x7f&RXmessage[3]<<8)+(RXmessage[4]) ) * RAD_TO_PUl ;
//										if(RXmessage[3] & 0x80)
//												P_PID.Target = Rad2Motor(-((RXmessage[3]&0x7f)*256 + RXmessage[4])*0.001) + 60 * 32768;
//										else
//												P_PID.Target = Rad2Motor( ((RXmessage[3]&0x7f)*256 + RXmessage[4])*0.001) + 60 * 32768;
//                    //break;
//                }
//								TXmessage[0] = 0x10 + g_MotorID;
//								TXmessage[1] = RXmessage[1];
//								TXmessage[2] = RXmessage[2];
//								TXmessage[3] = RXmessage[3];
//								TXmessage[4] = RXmessage[4];
//								TXmessage[5] = RXmessage[5];
//								TXmessage[6] = g_CanResiveNum / 256;
//								TXmessage[7] = g_CanResiveNum % 256;
//                break;
//            case 2:

//                switch( RXmessage[2] )
//                {
//                case 2:
//										
//                    TXmessage[2] = 2;
//                    TXmessage[3] = Motor2Rad(g_AngleHighSide_All - 60*32768)*1000/256;
//                    TXmessage[4] = Motor2Rad(g_AngleHighSide_All - 60*32768)*1000;
//										TXmessage[5] = 0x00;
//										TXmessage[6] = 0x00;
//										TXmessage[7] = 0x00;
//                    break;
//                }
//								RXmessage[0] = 0x20 + g_MotorID;
//                RXmessage[1] = 2;
//                break;
//            }
//						TXHeader.ExtId = 0X12345678;
//            TXHeader.DLC = 8;
//            TXHeader.IDE = CAN_ID_STD;
//            TXHeader.RTR = CAN_RTR_DATA;
//            TXHeader.TransmitGlobalTime = DISABLE;
//            HAL_CAN_AddTxMessage(&hcan,&TXHeader,TXmessage,&pTxMailbox);
//        }
//    }
	
	
//    static int32_t s_Temp;

//    static enum
//    {
//        START  = 0,
//        UPDATE,
//        REPLY,
//        RESET,
//    } s_tState = START;

//    switch( s_tState )
//    {
//    case START:
//        if(g_CANRxFlag == 1)
//        {
//            g_CANRxFlag = 0;
//            s_tState = UPDATE;
//        }
////        break;
//    case UPDATE:
//        if(RXmessage[0] == g_MotorID)
//        {
//            switch( RXmessage[1] )
//            {
//            case 0:
//                switch( RXmessage[2] )
//                {
////                case 0:
////                    g_MotorControlRegister.State = 0;
////                    TXmessage[2] = 0;
////                    break;
////                case 1:
////                    TXmessage[2] = 1;
////                    break;
////                case 2:
////                    g_MotorControlRegister.State = 2;
////                    TXmessage[2] = 2;
////                    break;
////                case 3:
////                    g_MotorControlRegister.State = 1;
////                    TXmessage[2] = 3;
////                    break;
////                case 4:
////                    g_MotorControlRegister.State = 4;
////                    TXmessage[2] = 4;
////                    break;
//                case 5:
//                    P_PID.Kp = RXmessage[3];
//                    P_PID.Ki = RXmessage[4];
//                    P_PID.Kd = RXmessage[5];
//										P_PID.ResultLimit_Max = (RXmessage[6] * 256 + RXmessage[7])*1000;
//										P_PID.ResultLimit_Min = -P_PID.ResultLimit_Max;
//                    break;
////                case 6:
////                    V_PID.Kp = RXmessage[3];
////                    V_PID.Ki = RXmessage[4];
////                    V_PID.Kd = RXmessage[5];
////                    TXmessage[2] = 6;
////                    TXmessage[3] = RXmessage[3];
////                    TXmessage[4] = RXmessage[4];
////                    TXmessage[5] = RXmessage[5];
////                    break;
////                case 7:
////                    C_PID.Kp = RXmessage[3];
////                    C_PID.Ki = RXmessage[4];
////                    C_PID.Kd = RXmessage[5];
////                    TXmessage[2] = 7;
////                    TXmessage[3] = RXmessage[3];
////                    TXmessage[4] = RXmessage[4];
////                    TXmessage[5] = RXmessage[5];
////                    break;
//                }
//								TXmessage[0] = 0x10 + g_MotorID;
//								TXmessage[1] = RXmessage[1];
//								TXmessage[2] = RXmessage[2];
//								TXmessage[3] = RXmessage[3];
//								TXmessage[4] = RXmessage[4];
//								TXmessage[5] = RXmessage[5];
//								TXmessage[6] = RXmessage[6];
//								TXmessage[7] = RXmessage[7];
//                //TXHeader.StdId = 0x1000 + DEVICE_ID;
//                //RXmessage[1] = 0;
//                break;
//            case 1:
//                switch( RXmessage[2] )
//                {
////                case 0:
////                    g_MotorControlRegister.State = 1;
////                    g_MotorControlRegister.Speed = 0;
////                    TXmessage[2] = 0;
////                    break;
//                case 1:
//                    //g_MotorControlRegister.Position = -1 * (RXmessage[3]>>7) * ( (0x7f&RXmessage[3]<<8)+(RXmessage[4]) ) * RAD_TO_PUl ;
//										if(RXmessage[3] & 0x80)
//												P_PID.Target = Rad2Motor(-((RXmessage[3]&0x7f)*256 + RXmessage[4])*0.001) + 60 * 32768;
//										else
//												P_PID.Target = Rad2Motor(((RXmessage[3]&0x7f)*256 + RXmessage[4])*0.001) + 60 * 32768;

//                    break;
////                case 2:
////                    g_MotorControlRegister.Speed = -1 * (RXmessage[3]>>7) * ( (0x7f&RXmessage[3]<<8)+(RXmessage[4]) ) * RAD_TO_PUl ;
////                    TXmessage[2] = 2;
////                    TXmessage[3] = RXmessage[3];
////                    TXmessage[4] = RXmessage[4];

////                    break;
////                case 3:
////                    g_MotorControlRegister.Torque = -1 * (RXmessage[3]>>7) * (0x7f&RXmessage[3]<<8)+(RXmessage[4]);
////                    TXmessage[2] = 3;
////                    TXmessage[3] = RXmessage[3];
////                    TXmessage[4] = RXmessage[4];

////                    break;
////                case 4:
////                    P_LIMIT.Max = -1 * (RXmessage[3]>>7) * (0x7f&RXmessage[3]<<8)+(RXmessage[4])*0.001;
////                    P_LIMIT.Min = -1 * (RXmessage[5]>>7) * (0x7f&RXmessage[5]<<8)+(RXmessage[6])*0.001;
////                    TXmessage[2] = 4;
////                    TXmessage[3] = RXmessage[3];
////                    TXmessage[4] = RXmessage[4];
////                    TXmessage[5] = RXmessage[5];
////                    TXmessage[6] = RXmessage[6];
////                    break;
////                case 5:
////                    V_LIMIT.Max = -1 * (RXmessage[3]>>7) * (0x7f&RXmessage[3]<<8)+(RXmessage[4])*0.001;
////                    V_LIMIT.Min = -1 * (RXmessage[5]>>7) * (0x7f&RXmessage[5]<<8)+(RXmessage[6])*0.001;
////                    TXmessage[2] = 5;
////                    TXmessage[3] = RXmessage[3];
////                    TXmessage[4] = RXmessage[4];
////                    TXmessage[5] = RXmessage[5];
////                    TXmessage[6] = RXmessage[6];
////                    break;
////                case 6:
////                    C_LIMIT.Max = -1 * (RXmessage[3]>>7) * (0x7f&RXmessage[3]<<8)+(RXmessage[4])*0.001;
////                    C_LIMIT.Min = -1 * (RXmessage[5]>>7) * (0x7f&RXmessage[5]<<8)+(RXmessage[6])*0.001;
////                    TXmessage[2] = 6;
////                    TXmessage[3] = RXmessage[3];
////                    TXmessage[4] = RXmessage[4];
////                    TXmessage[5] = RXmessage[5];
////                    TXmessage[6] = RXmessage[6];
////                    break;
////                case 7:
////                    break;
//                }
//								TXmessage[0] = 0x10 + g_MotorID;
//								TXmessage[1] = RXmessage[1];
//								TXmessage[2] = RXmessage[2];
//								TXmessage[3] = RXmessage[3];
//								TXmessage[4] = RXmessage[4];
//								TXmessage[5] = RXmessage[5];
//								TXmessage[6] = RXmessage[6];
//								TXmessage[7] = RXmessage[7];
////                TXHeader.StdId = 0x1000 + DEVICE_ID;
////                RXmessage[1] = 1;
//                break;
//            case 2:

//                switch( RXmessage[2] )
//                {
////                case 0:
////                    TXmessage[2] = 0;

////                    break;
////                case 1:
////                    TXmessage[2] = 1;
////                    TXmessage[3] = MotorErr;
////                    break;
//                case 2:
//										
//                    TXmessage[2] = 2;
//                    TXmessage[3] = Motor2Rad(g_AngleHighSide_All - 60*32768)*1000/256;
//                    TXmessage[4] = Motor2Rad(g_AngleHighSide_All - 60*32768)*1000;
//										TXmessage[5] = 0x00;
//										TXmessage[6] = 0x00;
//										TXmessage[7] = 0x00;
//                    break;
////                case 3:
////                    TXmessage[2] = 3;
////                    TXmessage[3] = g_TMC5160_VACTUAL/RAD_TO_PUl/256;
////                    TXmessage[4] = g_TMC5160_VACTUAL/RAD_TO_PUl;
////                    break;
////                case 4:
////                    TXmessage[2] = 4;
////                    break;
////                case 5:
////                    TXmessage[2] = 5;

////                    TXmessage[3] =  P_LIMIT.Max>>8;
////                    TXmessage[4] =  P_LIMIT.Max;
////                    TXmessage[5] =  P_LIMIT.Min>>8;
////                    TXmessage[6] =  P_LIMIT.Min;
////                    break;
////                case 6:
////                    TXmessage[2] = 6;
////                    TXmessage[3] =  V_LIMIT.Max>>8;
////                    TXmessage[4] =  V_LIMIT.Max;
////                    TXmessage[5] =  V_LIMIT.Min>>8;
////                    TXmessage[6] =  V_LIMIT.Min;
////                    break;
////                case 7:
////                    TXmessage[2] = 7;
////                    TXmessage[3] =  C_LIMIT.Max>>8;
////                    TXmessage[4] =  C_LIMIT.Max;
////                    TXmessage[5] =  C_LIMIT.Min>>8;
////                    TXmessage[6] =  C_LIMIT.Min;
////                    break;
//                }
////                TXHeader.StdId = 0x2000 + DEVICE_ID;
//								RXmessage[0] = 0x20 + g_MotorID;
//                RXmessage[1] = 2;
//                break;
//            }
//            s_tState = REPLY;
//        }

////        break;
//    case REPLY:
//        if(TRUE)
//        {
////            TXHeader.StdId = 0x123;
//            TXHeader.ExtId = 0X12345678;
//            TXHeader.DLC = 8;
//            TXHeader.IDE = CAN_ID_STD;
//            TXHeader.RTR = CAN_RTR_DATA;
//            TXHeader.TransmitGlobalTime = DISABLE;
//            HAL_CAN_AddTxMessage(&hcan,&TXHeader,TXmessage,&pTxMailbox);
//            s_tState = RESET;
//        }

////        break;
//    case RESET:

//        RESET_FSM() ;

//    }
//}

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

/* Segger RTT Print Utility Test */
void TASK_RTT_PRINT_TEST(){


  SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

  SEGGER_RTT_WriteString(0, "SEGGER Real-Time-Terminal Sample\r\n\r\n");
  SEGGER_RTT_WriteString(0, "###### Testing SEGGER_printf() ######\r\n");

  SEGGER_RTT_printf(0, "printf Test: %%c,         'S' : %c.\r\n", 'S');
  SEGGER_RTT_printf(0, "printf Test: %%5c,        'E' : %5c.\r\n", 'E');
  SEGGER_RTT_printf(0, "printf Test: %%-5c,       'G' : %-5c.\r\n", 'G');
  SEGGER_RTT_printf(0, "printf Test: %%5.3c,      'G' : %-5c.\r\n", 'G');
  SEGGER_RTT_printf(0, "printf Test: %%.3c,       'E' : %-5c.\r\n", 'E');
  SEGGER_RTT_printf(0, "printf Test: %%c,         'R' : %c.\r\n", 'R');

  SEGGER_RTT_printf(0, "printf Test: %%s,      \"RTT\" : %s.\r\n", "RTT");
  SEGGER_RTT_printf(0, "printf Test: %%s, \"RTT\\r\\nRocks.\" : %s.\r\n", "RTT\r\nRocks.");

  SEGGER_RTT_printf(0, "printf Test: %%u,       12345 : %u.\r\n", 12345);
  SEGGER_RTT_printf(0, "printf Test: %%+u,      12345 : %+u.\r\n", 12345);
  SEGGER_RTT_printf(0, "printf Test: %%.3u,     12345 : %.3u.\r\n", 12345);
  SEGGER_RTT_printf(0, "printf Test: %%.6u,     12345 : %.6u.\r\n", 12345);
  SEGGER_RTT_printf(0, "printf Test: %%6.3u,    12345 : %6.3u.\r\n", 12345);
  SEGGER_RTT_printf(0, "printf Test: %%8.6u,    12345 : %8.6u.\r\n", 12345);
  SEGGER_RTT_printf(0, "printf Test: %%08u,     12345 : %08u.\r\n", 12345);
  SEGGER_RTT_printf(0, "printf Test: %%08.6u,   12345 : %08.6u.\r\n", 12345);
  SEGGER_RTT_printf(0, "printf Test: %%0u,      12345 : %0u.\r\n", 12345);
  SEGGER_RTT_printf(0, "printf Test: %%-.6u,    12345 : %-.6u.\r\n", 12345);
  SEGGER_RTT_printf(0, "printf Test: %%-6.3u,   12345 : %-6.3u.\r\n", 12345);
  SEGGER_RTT_printf(0, "printf Test: %%-8.6u,   12345 : %-8.6u.\r\n", 12345);
  SEGGER_RTT_printf(0, "printf Test: %%-08u,    12345 : %-08u.\r\n", 12345);
  SEGGER_RTT_printf(0, "printf Test: %%-08.6u,  12345 : %-08.6u.\r\n", 12345);
  SEGGER_RTT_printf(0, "printf Test: %%-0u,     12345 : %-0u.\r\n", 12345);

  SEGGER_RTT_printf(0, "printf Test: %%u,      -12345 : %u.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%+u,     -12345 : %+u.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%.3u,    -12345 : %.3u.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%.6u,    -12345 : %.6u.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%6.3u,   -12345 : %6.3u.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%8.6u,   -12345 : %8.6u.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%08u,    -12345 : %08u.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%08.6u,  -12345 : %08.6u.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%0u,     -12345 : %0u.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%-.6u,   -12345 : %-.6u.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%-6.3u,  -12345 : %-6.3u.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%-8.6u,  -12345 : %-8.6u.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%-08u,   -12345 : %-08u.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%-08.6u, -12345 : %-08.6u.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%-0u,    -12345 : %-0u.\r\n", -12345);

  SEGGER_RTT_printf(0, "printf Test: %%d,      -12345 : %d.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%+d,     -12345 : %+d.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%.3d,    -12345 : %.3d.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%.6d,    -12345 : %.6d.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%6.3d,   -12345 : %6.3d.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%8.6d,   -12345 : %8.6d.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%08d,    -12345 : %08d.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%08.6d,  -12345 : %08.6d.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%0d,     -12345 : %0d.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%-.6d,   -12345 : %-.6d.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%-6.3d,  -12345 : %-6.3d.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%-8.6d,  -12345 : %-8.6d.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%-08d,   -12345 : %-08d.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%-08.6d, -12345 : %-08.6d.\r\n", -12345);
  SEGGER_RTT_printf(0, "printf Test: %%-0d,    -12345 : %-0d.\r\n", -12345);

  SEGGER_RTT_printf(0, "printf Test: %%x,      0x1234ABC : %x.\r\n", 0x1234ABC);
  SEGGER_RTT_printf(0, "printf Test: %%+x,     0x1234ABC : %+x.\r\n", 0x1234ABC);
  SEGGER_RTT_printf(0, "printf Test: %%.3x,    0x1234ABC : %.3x.\r\n", 0x1234ABC);
  SEGGER_RTT_printf(0, "printf Test: %%.6x,    0x1234ABC : %.6x.\r\n", 0x1234ABC);
  SEGGER_RTT_printf(0, "printf Test: %%6.3x,   0x1234ABC : %6.3x.\r\n", 0x1234ABC);
  SEGGER_RTT_printf(0, "printf Test: %%8.6x,   0x1234ABC : %8.6x.\r\n", 0x1234ABC);
  SEGGER_RTT_printf(0, "printf Test: %%08x,    0x1234ABC : %08x.\r\n", 0x1234ABC);
  SEGGER_RTT_printf(0, "printf Test: %%08.6x,  0x1234ABC : %08.6x.\r\n", 0x1234ABC);
  SEGGER_RTT_printf(0, "printf Test: %%0x,     0x1234ABC : %0x.\r\n", 0x1234ABC);
  SEGGER_RTT_printf(0, "printf Test: %%-.6x,   0x1234ABC : %-.6x.\r\n", 0x1234ABC);
  SEGGER_RTT_printf(0, "printf Test: %%-6.3x,  0x1234ABC : %-6.3x.\r\n", 0x1234ABC);
  SEGGER_RTT_printf(0, "printf Test: %%-8.6x,  0x1234ABC : %-8.6x.\r\n", 0x1234ABC);
  SEGGER_RTT_printf(0, "printf Test: %%-08x,   0x1234ABC : %-08x.\r\n", 0x1234ABC);
  SEGGER_RTT_printf(0, "printf Test: %%-08.6x, 0x1234ABC : %-08.6x.\r\n", 0x1234ABC);
  SEGGER_RTT_printf(0, "printf Test: %%-0x,    0x1234ABC : %-0x.\r\n", 0x1234ABC);


  SEGGER_RTT_WriteString(0, "###### SEGGER_printf() Tests done. ######\r\n");
}