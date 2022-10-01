/**
 * @file   main.h
 * @brief  SPI slave mode example for STM32 based on LibOpenCM3.
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
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#define USART_BAUDRATE (9600)

#if defined(NUCLEO_F103RB)
  #define GPIO_SPI_PORT (GPIOA)
  #define GPIO_SPI_SCK_PIN (GPIO5)  /* D13. */
  #define GPIO_SPI_MISO_PIN (GPIO6) /* D12. */
  #define GPIO_SPI_MOSI_PIN (GPIO7) /* D11. */
  #define GPIO_SPI_CS_PIN (GPIO4)   /* A2. */

  #define GPIO_SPI_RQ_PORT (GPIOC)
  #define GPIO_SPI_RQ_PIN (GPIO7) /* D9. */

  #define RCC_USART_TXRX_GPIO (RCC_GPIOA)
  #define GPIO_USART_TXRX_PORT (GPIOA)
  #define GPIO_USART_TX_PIN (GPIO2) /* D1. */
  #define GPIO_USART_RX_PIN (GPIO3) /* D0. */
#elif defined(NUCLEO_F446RE)
  #define GPIO_SPI_PORT (GPIOA)
  #define GPIO_SPI_SCK_PIN (GPIO5)  /* D13. */
  #define GPIO_SPI_MISO_PIN (GPIO6) /* D12. */
  #define GPIO_SPI_MOSI_PIN (GPIO7) /* D11. */
  #define GPIO_SPI_CS_PIN (GPIO4)   /* A2. */
  #define GPIO_SPI_AF (GPIO_AF5)    /* Ref: Table-11 in DS10693. */

  #define GPIO_SPI_RQ_PORT (GPIOC)
  #define GPIO_SPI_RQ_PIN (GPIO7) /* D9. */

  #define GPIO_USART_TXRX_PORT (GPIOA)
  #define GPIO_USART_TX_PIN (GPIO2) /* ST-Link (D1). */
  #define GPIO_USART_RX_PIN (GPIO3) /* ST-Link (D0). */
  #define GPIO_USART_AF (GPIO_AF7)  /* Ref: Table-11 in DS10693. */
#else
  #error "STM32 board not defined."
#endif

  static void rcc_setup(void);
  static void usart_setup(void);
  static void spi_setup(void);
  static void spi_rq_setup(void);

  static void spi_rq_set(void);
  static void spi_rq_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H. */