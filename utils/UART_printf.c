#include "stdarg.h"
#include "stdio.h"
#include "usart.h"
int UART_printf(UART_HandleTypeDef *huart, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int length;
    char buffer[128];
    length = vsnprintf(buffer, 128, fmt, ap);
    HAL_UART_Transmit(huart, buffer, length, HAL_MAX_DELAY);//HAL_MAX_DELAY
//    CDC_Transmit_FS((uint8_t*)buffer,length);
    va_end(ap);
    return length;
}