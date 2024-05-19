/**
 * @file user_main.h
 * @author ZiTe (honmonoh@gmail.com)
 * @copyright MIT
 */

#ifndef USER_MAIN_H_
#define USER_MAIN_H_

#include "stm32g4xx.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_utils.h" // For LL_mDelay().

// Define instances and parameters.

#define MY_UART (USART2)
#define MY_UART_BAUDRATE (115200U)

void User_Init(void);
void User_Loop(void);
void User_UART_ISR(void);

#endif /* USER_MAIN_H_ */
