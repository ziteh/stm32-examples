/**
 * @file   main.h
 * @brief  ADC external trigger by timer example for LibOpenCM3 with STM32.
 * @author ZiTe (honmonoh@gmail.com)
 * @copyright MIT License
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h> /* For printf(). */
#include <errno.h> /* For printf(). */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#define USART_BAUDRATE (9600)

#define GOAL_FREQUENCY (1) /* f_goal, goal frequency in Hz. */

/**
 * @brief f_timer.
 * @note If APBx_Presc = /1 than f_timer = APBx_Clock, else f_timer = 2* APBx_Clock.
 */
#define TIMER_CLOCK (rcc_apb1_frequency * 2)

/**
 * @brief f_counter (CK_CNT).
 */
#define COUNTER_CLOCK (10000)

/**
 * @brief PSC (Prescaler), the value of TIMx_PSC register.
 * @note f_counter = f_timer / (PSC + 1)
 *       so,
 *       PSC = f_timer / f_counter - 1
 */
#define TIMER_PRESCALER (TIMER_CLOCK / COUNTER_CLOCK - 1)

/**
 * @brief ARR (Auto-Reload), the value of TIMx_ARR register.
 * @note f_goal = f_timer / [(PSC + 1) * (ARR + 1)]
 *       so,
 *       ARR = {f_timer / [(PSC + 1) * f_goal]} - 1
 */
#define TIMER_PERIOD (((TIMER_CLOCK) / ((TIMER_PRESCALER + 1) * GOAL_FREQUENCY)) - 1)

#if defined(NUCLEO_F103RB)
  #define ADC_SIMPLE_TIME (ADC_SMPR_SMP_55DOT5CYC)
  #define RCC_ADC_GPIO (RCC_GPIOA)
  #define GPIO_ADC_PORT (GPIOA)
  #define GPIO_ADC_IN0_PIN (GPIO0) /* Arduino-A0. */
  #define ADC_IRQ (NVIC_ADC1_2_IRQ)

  #define RCC_USART_TX_GPIO (RCC_GPIOA)
  #define GPIO_USART_TX_PORT (GPIOA)
  #define GPIO_USART_TX_PIN (GPIO2) /* ST-Link (Arduino-D1). */
#elif defined(NUCLEO_F446RE)
  #define ADC_SIMPLE_TIME (ADC_SMPR_SMP_56CYC)
  #define RCC_ADC_GPIO (RCC_GPIOA)
  #define GPIO_ADC_PORT (GPIOA)
  #define GPIO_ADC_IN0_PIN (GPIO0) /* Arduino-A0. */
  #define ADC_IRQ (NVIC_ADC_IRQ)

  #define RCC_USART_TX_GPIO (RCC_GPIOA)
  #define GPIO_USART_TX_PORT (GPIOA)
  #define GPIO_USART_TX_PIN (GPIO2) /* ST-Link (Arduino-D1). */
  #define GPIO_USART_AF (GPIO_AF7)  /* Ref: Table-11 in DS10693. */
#else
  #error "STM32 board not defined."
#endif

  static void rcc_setup(void);
  static void usart_setup(void);
  static void adc_setup(void);
  static void timer_setup(void);
  static void delay(uint32_t value);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H. */