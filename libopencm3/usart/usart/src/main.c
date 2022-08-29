/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Basic USART with receive interrupt.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#define USART (USART2)
#define RCC_USART (RCC_USART2)

/* USART2-Tx = PA2 */
#define USART_TX_PORT (GPIOA)
#define USART_TX_PIN (GPIO2)

/* USART2-Rx = PA3 */
#define USART_RX_PORT (GPIOA)
#define USART_RX_PIN (GPIO3)

/* User-LED = PA5 */
#define LED_PORT (GPIOA)
#define LED_PIN (GPIO5)

void rcc_setup(void)
{
  /* Both USART-Tx/Rx & User-LED are Port-A */
  rcc_periph_clock_enable(RCC_GPIOA);

  rcc_periph_clock_enable(RCC_USART);
}

/**
 * @brief Setup USART1.
 */
void usart_setup(void)
{
  /* Enable USART IRQ. */
  nvic_enable_irq(NVIC_USART2_IRQ);

  /* Setup Tx pin. */
  gpio_set_mode(USART_TX_PORT,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                USART_TX_PIN);

  /* Setup Rx pin. */
  gpio_set_mode(USART_RX_PORT,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_FLOAT,
                USART_RX_PIN);

  /* Setup USART config with 9600, 8-N-1. */
  usart_set_baudrate(USART, 9600);
  usart_set_databits(USART, 8);
  usart_set_stopbits(USART, USART_STOPBITS_1);
  usart_set_parity(USART, USART_PARITY_NONE);
  usart_set_flow_control(USART, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART, USART_MODE_TX_RX);

  /* Enable Rx interrupt. */
  usart_enable_rx_interrupt(USART);

  /* Enable. */
  usart_enable(USART);
}

/**
 * @brief Setup User-LED.
 */
void led_setup(void)
{
  gpio_set_mode(LED_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                LED_PIN);
}

/**
 * @brief USART2 Interrupt service routine.
 */
void usart2_isr(void)
{
  /* Send message 'Received'. */
  usart_send_blocking(USART, 'R');
  usart_send_blocking(USART, 'e');
  usart_send_blocking(USART, 'c');
  usart_send_blocking(USART, 'e');
  usart_send_blocking(USART, 'i');
  usart_send_blocking(USART, 'v');
  usart_send_blocking(USART, 'e');
  usart_send_blocking(USART, 'd');

  /* Newline(CRLF). */
  usart_send_blocking(USART, '\r');
  usart_send_blocking(USART, '\n');

  gpio_toggle(LED_PORT, LED_PIN);

  /* 
   * Clear RXNE(Read data register not empty) flag of
   * USART SR(Status register).
   */
  USART_SR(USART) &= ~USART_SR_RXNE;
}

void delay(int value)
{
  for (int i = 0; i < value; i++)
  {
    __asm__("nop");
  }
}

int main(void)
{
  rcc_setup();
  led_setup();
  usart_setup();

  while (1)
  {
    /* Send message. */
    usart_send_blocking(USART, 'H');
    usart_send_blocking(USART, 'i');
    usart_send_blocking(USART, '!');

    /* Newline(CRLF). */
    usart_send_blocking(USART, '\r');
    usart_send_blocking(USART, '\n');

    gpio_toggle(LED_PORT, LED_PIN);

    delay(800000);
  }

  return 0;
}