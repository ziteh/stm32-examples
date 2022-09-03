/**
 * @file   main.h
 * @brief  I2C example with LCD1602(PCF8574T) for LibOpenCM3 with STM32.
 * @author ZiTe (honmonoh@gmail.com)
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
#include "PCF8574T_LCD_Driver.h"

#if defined(NUCLEO_F103RB)
  #define RCC_I2C_GPIO (RCC_GPIOB)
  #define GPIO_I2C_PORT (GPIOB)
  #define GPIO_I2C_SCL_PIN (GPIO8) /* D15. */
  #define GPIO_I2C_SDA_PIN (GPIO9) /* D14. */
#elif defined(NUCLEO_F446RE)
  #define RCC_I2C_GPIO (RCC_GPIOB)
  #define GPIO_I2C_PORT (GPIOB)
  #define GPIO_I2C_SCL_PIN (GPIO8) /* D15. */
  #define GPIO_I2C_SDA_PIN (GPIO9) /* D14. */
  #define GPIO_I2C_AF (GPIO_AF4)   /* Ref: Table-11 in DS10693. */
#else
  #error "STM32 board not defined."
#endif

  static void rcc_setup(void);
  static void i2c_setup(void);
  static void delay(uint32_t value);

  static void lcd_delay(uint16_t ms);
  static void lcd_i2c_write(uint8_t address, uint8_t *data, uint8_t data_length);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H. */