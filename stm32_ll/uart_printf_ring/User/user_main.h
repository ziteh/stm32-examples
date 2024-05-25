/// @file user_main.c
/// @author ZiTe <honmonoh@gmail.com>
/// @copyright SPDX-License-Identifier: MIT

#ifndef USER_MAIN_H
#define USER_MAIN_H

#include "stm32g4xx.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_utils.h" // For LL_mDelay()

// Define instances and parameters

#define MY_UART          (USART2)
#define MY_UART_BAUDRATE (115200U)

void user_init(void);
void user_loop(void);
void user_uart_isr(void);

#endif
