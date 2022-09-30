/**
 * @file   main.h
 * @brief  I2C EEPROM (24C256) example for STM32 based on LibOpenCM3.
 * @author ZiTe (honmonoh@gmail.com)
 * @copyright MIT License
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#define I2C_SLAVE_ADDRESS ((uint8_t)0x50)
#define USART_BAUDRATE (9600)

#if defined(NUCLEO_F103RB)
  #define RCC_I2C_GPIO (RCC_GPIOB)
  #define GPIO_I2C_PORT (GPIOB)
  #define GPIO_I2C_SCL_PIN (GPIO8) /* D15. */
  #define GPIO_I2C_SDA_PIN (GPIO9) /* D14. */

  #define RCC_USART_TXRX_GPIO (RCC_GPIOA)
  #define GPIO_USART_TXRX_PORT (GPIOA)
  #define GPIO_USART_TX_PIN (GPIO2) /* D1. */
  #define GPIO_USART_RX_PIN (GPIO3) /* D0. */
#elif defined(NUCLEO_F446RE)
  #define RCC_I2C_GPIO (RCC_GPIOB)
  #define GPIO_I2C_PORT (GPIOB)
  #define GPIO_I2C_SCL_PIN (GPIO8) /* D15. */
  #define GPIO_I2C_SDA_PIN (GPIO9) /* D14. */
  #define GPIO_I2C_AF (GPIO_AF4)   /* Ref: Table-11 in DS10693. */

  #define RCC_USART_TXRX_GPIO (RCC_GPIOA)
  #define GPIO_USART_TXRX_PORT (GPIOA)
  #define GPIO_USART_TX_PIN (GPIO2) /* D1. */
  #define GPIO_USART_RX_PIN (GPIO3) /* D0. */
  #define GPIO_USART_AF (GPIO_AF7)  /* Ref: Table-11 in DS10693. */
#else
  #error "STM32 board not defined."
#endif

  static void rcc_setup(void);
  static void i2c_setup(void);
  static void delay(uint32_t value);
  static void usart_setup(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H. */