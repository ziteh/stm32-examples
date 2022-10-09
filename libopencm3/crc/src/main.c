/**
 * @file main.c
 * @brief CRC example for STM32 based on LibOpenCM3
 * @author ZiTe (honmonoh@gmail.com)
 * @note CRC parameteization: CRC-32/MPEG-2 (Poly: 0x04C1 1DB7, Init: 0xFFFF FFFF).
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/cm3/nvic.h>

#define USART_BAUDRATE (9600)

#if defined(NUCLEO_F103RB)
  #define RCC_USART_TXRX_GPIO (RCC_GPIOA)
  #define GPIO_USART_TXRX_PORT (GPIOA)
  #define GPIO_USART_TX_PIN (GPIO2) /* ST-Link (Arduino-D1). */
  #define GPIO_USART_RX_PIN (GPIO3) /* ST-Link (Arduino-D0). */
#elif defined(NUCLEO_F446RE)
  #define RCC_USART_TXRX_GPIO (RCC_GPIOA)
  #define GPIO_USART_TXRX_PORT (GPIOA)
  #define GPIO_USART_TX_PIN (GPIO2) /* ST-Link (Arduino-D1). */
  #define GPIO_USART_RX_PIN (GPIO3) /* ST-Link (Arduino-D0). */
  #define GPIO_USART_AF (GPIO_AF7)  /* Table-11 in DS10693. */
#else
  #error "STM32 board not defined."
#endif

static void rcc_setup(void)
{
  rcc_periph_clock_enable(RCC_USART_TXRX_GPIO);
  rcc_periph_clock_enable(RCC_USART2);
  rcc_periph_clock_enable(RCC_CRC);
}

static void usart_setup(void)
{
  /* Set USART-Tx and Rx pin to alternate function. */
#if defined(STM32F1)
  gpio_set_mode(GPIO_USART_TXRX_PORT,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_USART_TX_PIN);

  gpio_set_mode(GPIO_USART_TXRX_PORT,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_FLOAT,
                GPIO_USART_RX_PIN);
#else
  gpio_mode_setup(GPIO_USART_TXRX_PORT,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO_USART_TX_PIN | GPIO_USART_RX_PIN);

  gpio_set_af(GPIO_USART_TXRX_PORT,
              GPIO_USART_AF,
              GPIO_USART_TX_PIN | GPIO_USART_RX_PIN);
#endif

  /* Config USART params. */
  usart_set_baudrate(USART2, USART_BAUDRATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX_RX);

  /* Setup interrupt. */
  usart_enable_rx_interrupt(USART2); /* Enable receive interrupt. */
  nvic_enable_irq(NVIC_USART2_IRQ);

  usart_enable(USART2);
}

int main(void)
{
  rcc_setup();
  usart_setup();

  usart_send_blocking(USART2, 'C');
  usart_send_blocking(USART2, 'R');
  usart_send_blocking(USART2, 'C');
  usart_send_blocking(USART2, '\r');
  usart_send_blocking(USART2, '\n');

  /* Halt. */
  while (1)
  {
  }

  return 0;
}

/**
 * @brief USART2 Interrupt service routine.
 */
void usart2_isr(void)
{
  usart_disable_rx_interrupt(USART2);
  crc_reset(); /* Resets the CRC calculation unit and sets the data register to 0xFFFF FFFF. */

  uint8_t data[4];
  for (int i = 0; i < 4; i++)
  {
    data[i] = usart_recv_blocking(USART2);
  }

  uint32_t comb = data[3] + (data[2] << 8) + (data[1] << 16) + (data[0] << 24);
  uint32_t result = crc_calculate(comb);

  usart_send_blocking(USART2, (result >> 24) & 0xFF);
  usart_send_blocking(USART2, (result >> 16) & 0xFF);
  usart_send_blocking(USART2, (result >> 8) & 0xFF);
  usart_send_blocking(USART2, result & 0xFF);

  USART_SR(USART2) &= ~USART_SR_RXNE; /* Clear 'Read data register not empty' flag. */
  usart_enable_rx_interrupt(USART2);
}