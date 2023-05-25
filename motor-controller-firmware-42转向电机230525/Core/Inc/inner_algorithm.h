#ifndef _INNER_ALGORITHM_
#define _INNER_ALGORITHM_

#include "common.h"

void P_PID_Init(void);

int32_t Rad2Motor(float Rad);

float Motor2Rad(int32_t motor);

void Position2Speed_PID(void);

void PIDCalculate(Pid_TypDef PID);

#endif
