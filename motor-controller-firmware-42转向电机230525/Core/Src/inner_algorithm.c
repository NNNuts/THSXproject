#include "common.h"
#include "inner_algorithm.h"


void P_PID_Init(void)
{
	P_PID.Kp = 80;
	P_PID.Ki = 0;
	P_PID.Kd = 0;
	
//	ReadEncoder_SPI_HighSide();
//	P_PID.Target = g_AngleHighSide_All;
	P_PID.Target = BRT38R0_READ_VALUE_1();
	P_PID.ResultLimit_Max = 40000;
	P_PID.ResultLimit_Min = -P_PID.ResultLimit_Max;
	P_PID.ErrNext = 0;
	P_PID.ZeroThreshold = 20;
	P_PID.ResultUpLimit = 10000;
	P_PID.ResultLast = 0;
}

int32_t Rad2Motor(float Rad)
{
	int32_t motor;
	motor = Rad * 16383 * 10 / 3.1415926535 ;
	return motor;
}

float Motor2Rad(int32_t motor)
{
	float Rad;
	Rad = motor * 3.1415926535 / 10 / 16383;
	return Rad;
}

void Position2Speed_PID(void)
{
		P_PID.Input = g_AngleLowSide;
		P_PID.Err = P_PID.Input - P_PID.Target;
		if(P_PID.Err < P_PID.ZeroThreshold && P_PID.Err > -P_PID.ZeroThreshold)
		{
			P_PID.Err = 0;
			g_MotorControlRegister.State = 0;
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

void PIDCalculate(Pid_TypDef PID)
{
////    if(PID.Err>PID.ErrLimit_Max)
////        PID.Err = PID.ErrLimit_Max;
//    
//    PID.IntegralValue += PID.Err;

////    PID.ResultValue = PID.Kp*(PID.Err-PID.ErrNext)+\
////                      (PID.Ki*PID.Err)+ \
////                      PID.Kd*(PID.Err-2*PID.ErrNext+PID.ErrLast);
//    
//    PID.ResultValue = PID.Kp*PID.Err+ \
//                      PID.Ki* PID.IntegralValue+ \
//                      PID.Kd*(PID.Err-PID.ErrNext);
//    
//    if(PID.ResultValue>PID.ResultLimit_Max)
//        PID.ResultValue = PID.ResultLimit_Max;
//    else if(PID.ResultValue<PID.ResultLimit_Min)
//        PID.ResultValue = PID.ResultLimit_Min;

//    PID.ErrLast=PID.ErrNext;
//    PID.ErrNext=PID.Err;
}
