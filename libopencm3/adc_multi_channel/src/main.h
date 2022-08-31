/**
 * @file   main.h
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Multi channel ADC example for Nucleo-F103RB and F446RE.
 */

#ifndef __MAIN_H
#define __MAIN_H

#include <stdio.h>
#include <errno.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define USART_BAUDRATE (9600)

#ifdef NUCLEO_F103RB
#define ADC_SIMPLE_TIME (ADC_SMPR_SMP_55DOT5CYC)
#define RCC_ADC (RCC_ADC1)
#define RCC_ADC_PORT (RCC_GPIOA)
#define GPIO_ADC_PORT (GPIOA)
#define GPIO_ADC_A0_PIN (GPIO0)
#define GPIO_ADC_A1_PIN (GPIO1)
#define GPIO_ADC_A2_PIN (GPIO4)

#define RCC_USART (RCC_USART2)
#define RCC_USART_TX_PORT (RCC_GPIOA)
#define GPIO_USART_TX_PORT (GPIOA)
#define GPIO_USART_TX_PIN (GPIO2)

#define RCC_LED_PORT (RCC_GPIOA)
#define GPIO_LED_PORT (GPIOA)
#define GPIO_LED_PIN (GPIO5)
#elif NUCLEO_F446RE
#define ADC_SIMPLE_TIME (ADC_SMPR_SMP_56CYC)
#define RCC_ADC (RCC_ADC1)
#define RCC_ADC_PORT (RCC_GPIOA)
#define GPIO_ADC_PORT (GPIOA)
#define GPIO_ADC_A0_PIN (GPIO0)
#define GPIO_ADC_A1_PIN (GPIO1)
#define GPIO_ADC_A2_PIN (GPIO4)

#define RCC_USART (RCC_USART2)
#define RCC_USART_TX_PORT (RCC_GPIOA)
#define GPIO_USART_TX_PORT (GPIOA)
#define GPIO_USART_TX_PIN (GPIO2)

#define RCC_LED_PORT (RCC_GPIOA)
#define GPIO_LED_PORT (GPIOA)
#define GPIO_LED_PIN (GPIO5)
#else
#error
#endif

  void usart_setup(void);
  void rcc_setup(void);
  void led_setup(void);
  void adc_setup(void);
  uint16_t get_adc_value(int channel);
  int _write(int file, char *ptr, int len);
  void delay(uint32_t value);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H. */