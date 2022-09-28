/**
 * @file   main.h
 * @brief  ADC injected multi channel example for LibOpenCM3 with STM32.
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

#define USART_BAUDRATE (9600)

#if defined(NUCLEO_F103RB)
  #error "Not working, need to fix." /* TODO. */

  #define ADC_SIMPLE_TIME (ADC_SMPR_SMP_55DOT5CYC)
  #define RCC_ADC_GPIO (RCC_GPIOA)
  #define GPIO_ADC_PORT (GPIOA)
  #define GPIO_ADC_IN0_PIN (GPIO0) /* Arduino-A0. */
  #define GPIO_ADC_IN1_PIN (GPIO1) /* Arduino-A1. */
  #define GPIO_ADC_IN4_PIN (GPIO4) /* Arduino-A2. */

  #define RCC_USART_TX_GPIO (RCC_GPIOA)
  #define GPIO_USART_TX_PORT (GPIOA)
  #define GPIO_USART_TX_PIN (GPIO2) /* ST-Link (Arduino-D1). */
#elif defined(NUCLEO_F446RE)
  #define ADC_SIMPLE_TIME (ADC_SMPR_SMP_56CYC)
  #define RCC_ADC_GPIO (RCC_GPIOA)
  #define GPIO_ADC_PORT (GPIOA)
  #define GPIO_ADC_IN0_PIN (GPIO0) /* Arduino-A0. */
  #define GPIO_ADC_IN1_PIN (GPIO1) /* Arduino-A1. */
  #define GPIO_ADC_IN4_PIN (GPIO4) /* Arduino-A2. */

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
  static void delay(uint32_t value);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H. */