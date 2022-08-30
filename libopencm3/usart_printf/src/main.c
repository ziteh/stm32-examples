/**
 * @file   main.c
 * @brief  USART with printf() function for STM32 Nucleo-F103RB and F446RE.
 * @remark Reference: https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f1/stm32-h103/usart_printf/usart_printf.c
 */

#include <stdio.h>
#include <errno.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#define USART_BAUDRATE (9600)

#ifdef NUCLEO_F103RB
  #define RCC_USART (RCC_USART2)
  #define RCC_USART_TX_PORT (RCC_GPIOA)
  #define GPIO_USART_TX_PORT (GPIOA)
  #define GPIO_USART_TX_PIN (GPIO2)
#elif NUCLEO_F446RE
  #define RCC_USART (RCC_USART2)
  #define RCC_USART_TX_PORT (RCC_GPIOA)
  #define GPIO_USART_TX_PORT (GPIOA)
  #define GPIO_USART_TX_PIN (GPIO2)
#else
  #error
#endif

void rcc_setup(void)
{
  rcc_periph_clock_enable(RCC_USART_TX_PORT);
  rcc_periph_clock_enable(RCC_USART);
}

void usart_setup(void)
{
  /* Set to output alternate function. */
#ifdef NUCLEO_F103RB
  gpio_set_mode(GPIOA,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO2);
#else
  gpio_mode_setup(GPIO_USART_TX_PORT,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO_USART_TX_PIN);

  gpio_set_af(GPIO_USART_TX_PORT,
              GPIO_AF7,
              GPIO_USART_TX_PIN);
#endif

  /* Setup USART config. */
  usart_set_baudrate(USART2, USART_BAUDRATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX);

  usart_enable(USART2);
}

void delay(uint32_t value)
{
  while (value--)
  {
    __asm__("nop"); /* Do nothing. */
  }
}

int _write(int file, char *ptr, int len)
{
  int i;

  if (file == 1)
  {
    for (i = 0; i < len; i++)
    {
      usart_send_blocking(USART2, ptr[i]);
    }
    return i;
  }

  errno = EIO;
  return -1;
}

int main(void)
{
  rcc_setup();
  usart_setup();

  int i = 0;
  while (1)
  {
    printf("Hello World! %i\r\n", i++);
    delay(500000);
  }

  return 0;
}
