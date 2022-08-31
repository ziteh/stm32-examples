/**
 * @file   main.c
 * @brief  USART with receive interrupt for STM32 Nucleo-F103RB and F446RE.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#define USART_BAUDRATE (9600)

#ifdef NUCLEO_F103RB
  #define NVIC_USART_IRQ (NVIC_USART2_IRQ)
  #define RCC_USART (RCC_USART2)
  #define RCC_USART_TXRX_PORT (RCC_GPIOA)
  #define GPIO_USART_TXRX_PORT (GPIOA)
  #define GPIO_USART_TX_PIN (GPIO2)
  #define GPIO_USART_RX_PIN (GPIO3)

  #define RCC_LED_PORT (RCC_GPIOA)
  #define GPIO_LED_PORT (GPIOA)
  #define GPIO_LED_PIN (GPIO5)
#elif NUCLEO_F446RE
  #define NVIC_USART_IRQ (NVIC_USART2_IRQ)
  #define RCC_USART (RCC_USART2)
  #define RCC_USART_TXRX_PORT (RCC_GPIOA)
  #define GPIO_USART_TXRX_PORT (GPIOA)
  #define GPIO_USART_TX_PIN (GPIO2)
  #define GPIO_USART_RX_PIN (GPIO3)

  #define RCC_LED_PORT (RCC_GPIOA)
  #define GPIO_LED_PORT (GPIOA)
  #define GPIO_LED_PIN (GPIO5)
#else
  #error
#endif

void delay(uint32_t value);
void rcc_setup(void);
void usart_setup(void);
void led_setup(void);

int main(void)
{
  rcc_setup();
  led_setup();
  usart_setup();

  /* Halt. */
  while (1)
  {
    __asm__("nop"); /* Do nothing. */
  }

  return 0;
}

void rcc_setup(void)
{
  rcc_periph_clock_enable(RCC_LED_PORT);
  rcc_periph_clock_enable(RCC_USART_TXRX_PORT);
  rcc_periph_clock_enable(RCC_USART);
}

void usart_setup(void)
{
  /* Set to alternate function. */
#ifdef NUCLEO_F103RB
  /* Tx. */
  gpio_set_mode(GPIO_USART_TXRX_PORT,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_USART_TX_PIN);

  /* Rx. */
  gpio_set_mode(GPIO_USART_TXRX_PORT,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_FLOAT,
                GPIO_USART_RX_PIN);
#elif NUCLEO_F446RE
  gpio_mode_setup(GPIO_USART_TXRX_PORT,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO_USART_TX_PIN | GPIO_USART_RX_PIN);

  gpio_set_af(GPIO_USART_TXRX_PORT,
              GPIO_AF7, /* Ref: Table-11 in DS10693. */
              GPIO_USART_TX_PIN | GPIO_USART_RX_PIN);
#endif

  /* Interrupt. */
  nvic_enable_irq(NVIC_USART_IRQ);
  usart_enable_rx_interrupt(USART2); /* Enable receive interrupt. */

  /* Setup USART config. */
  usart_set_baudrate(USART2, USART_BAUDRATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX_RX);

  usart_enable(USART2);
}

void led_setup(void)
{
  /* Set to output Push-Pull. */
#ifdef NUCLEO_F103RB
  gpio_set_mode(GPIO_LED_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO_LED_PIN);
#elif NUCLEO_F446RE
  gpio_mode_setup(GPIO_LED_PORT,
                  GPIO_MODE_OUTPUT,
                  GPIO_PUPD_NONE,
                  GPIO_LED_PIN);

  gpio_set_output_options(GPIO_LED_PORT,
                          GPIO_OTYPE_PP,
                          GPIO_OSPEED_2MHZ,
                          GPIO_LED_PIN);
#endif
}

void delay(uint32_t value)
{
  while (value--)
  {
    __asm__("nop"); /* Do nothing. */
  }
}

/**
 * @brief USART2 Interrupt service routine.
 */
void usart2_isr(void)
{
  gpio_set(GPIO_LED_PORT, GPIO_LED_PIN); /* LED on. */

  uint8_t indata = usart_recv(USART2); /* Read received data. */
  usart_send_blocking(USART2, indata);

  delay(100000);
  gpio_clear(GPIO_LED_PORT, GPIO_LED_PIN); /* LED off. */

  /* Clear RXNE(Read data register not empty) flag at SR(Status register). */
  USART_SR(USART2) &= ~USART_SR_RXNE;
}