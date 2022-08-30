/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  USART with printf() function.
 * @remark Reference: https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f1/stm32-h103/usart_printf/usart_printf.c
 */

#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include "printf.h"

#define USART_BAUD_RATE (9600)

void gpio_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOA);

  /* Setup Tx pin. */
  gpio_set_mode(GPIOA,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO2);

  /* Setup User-LED. */
  gpio_set_mode(GPIOA,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO5);
}

void usart_setup(void)
{
  rcc_periph_clock_enable(RCC_USART2);

  /* Setup USART config. */
  usart_set_baudrate(USART2, USART_BAUD_RATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX);

  usart_enable(USART2);
}

void delay(int value)
{
  while (value--)
  {
    __asm__("nop");
  }
}

int main(void)
{
  gpio_setup();
  usart_setup();

  uint32_t i = 0;

  while (1)
  {
    printf("Hello World! %i\r\n", i);
    gpio_toggle(GPIOA, GPIO5);
    i++;

    delay(500000);
  }

  return 0;
}
