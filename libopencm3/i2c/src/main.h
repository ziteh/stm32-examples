/**
 * @file   main.h
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  I2C example for STM32 Nucleo-F103RB and F446RE.
 */

#ifndef __MAIN_H
#define __MAIN_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include "PCF8574T_LCD_Driver.h"

#ifdef NUCLEO_F103RB
#define RCC_I2C_GPIO (RCC_GPIOB)
#define GPIO_I2C_PORT (GPIOB)
#define GPIO_I2C_SCL_PIN (GPIO8) /* D15. */
#define GPIO_I2C_SDA_PIN (GPIO9) /* D14. */

#define DELAY_VALUE (10000000)
#define DELAY_CONST (5000)
#elif NUCLEO_F446RE
#define RCC_I2C_GPIO (RCC_GPIOB)
#define GPIO_I2C_PORT (GPIOB)
#define GPIO_I2C_SCL_PIN (GPIO8) /* D15. */
#define GPIO_I2C_SDA_PIN (GPIO9) /* D14. */

#define DELAY_VALUE (50000000)
#define DELAY_CONST (1000)
#else
#error
#endif

#ifdef __cplusplus
extern "C"
{
#endif

  void rcc_setup(void);
  void i2c_setup(void);
  void delay(uint32_t value);

  void lcd_delay(uint16_t ms);
  void lcd_i2c_write(uint8_t address, uint8_t *data, uint8_t data_length);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H. */