#ifndef _PID_H
#define _PID_H

/**
 * @brief header code block of pid.c 
 * 
 */
#include <math.h>
typedef struct PID {
    float aim;
    float k_p;
    float k_i;
    float k_d;
    float error;
    float last_error;
    float sum_error;
    float change_error;
    struct
    {
        float i_max;//以后慢慢想把。。。。
        float i_min;
        float i_threshold;
        float out_max;
        float out_min;
    }pid_process_param;
    
} PID;

//float pid_process(PID *pid, float aim, float current);

float pid_process_with_limit(PID *pid, float aim, float current, float max, float min);

#define _pid_process_with_limit(pid, aim, current, max, min) \
( float out = pid_process(pid, aim, current),   out = ( out > max ) ? max : out , \
                                                out = ( out > min ) ? min : out , \
                                                out \
)


/**
 * @brief UART_printf
 * 
 */
#include "usart.h"
int UART_printf(UART_HandleTypeDef *huart, const char *fmt, ...);


#endif // _UTILS_H