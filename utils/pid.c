#include <math.h>
#include "utils.h"
// #define PID_NO_D

__weak float pid_process(PID *pid, float aim, float current)
{
    pid->error = aim - current;
    pid->sum_error += pid->error;
    pid->change_error = pid->error - pid->last_error; // 不知道对不对
    if (fabsf(pid->sum_error) > 500)
        pid->sum_error = pid->sum_error > 0 ? 500 : -500;

#ifdef PID_NO_D
    float out = pid->k_p * pid->error + pid->k_i * pid->sum_error;
#else
    float out = pid->k_p * pid->error + pid->k_i * pid->sum_error + pid->k_d * pid->change_error;
#endif

//    if (fabsf(out) > 8)
//        out = (out > 0) ? 8 : -8;
    return out;
}

float pid_process_with_limit(PID *pid, float aim, float current, float max, float min)
{
    float out = pid_process(pid, aim, current);
    if (out > max)
        out = max;
    if (out < min)
        out = min;
    return min;
}


//example
/* PID Voltage_pid={
        .k_p=0,
        .k_i=0,
        .k_d=0,
        .error=0,
        .last_error=0,
        .sum_error=0,
        .change_error=0
}; */