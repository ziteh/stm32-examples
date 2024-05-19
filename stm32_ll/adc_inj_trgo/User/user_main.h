/**
 * @file user_main.h
 * @author ZiTe (honmonoh@gmail.com)
 * @copyright MIT
 */

#ifndef USER_MAIN_H_
#define USER_MAIN_H_

#include "stm32g4xx.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_utils.h" // For LL_mDelay().

// Define instances and parameters.

#define MY_UART (USART2)
#define MY_UART_BAUDRATE (115200U)

#define MY_TIM (TIM1)
#define MY_TIM_PSC (170 - 1)
#define MY_TIM_ARR (4000 - 1)

#define MY_ADC (ADC2)

#define MY_LED_PORT (GPIOB)
#define MY_LED_PIN (LL_GPIO_PIN_8)

void User_Init(void);
void User_Loop(void);
void User_ADC_ISR(void);

#endif /* USER_MAIN_H_ */
