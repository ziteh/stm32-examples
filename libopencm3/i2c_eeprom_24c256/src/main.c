/**
 * @file   main.c
 * @brief  I2C EEPROM (24C256) example for STM32 based on LibOpenCM3.
 * @author ZiTe (honmonoh@gmail.com)
 * @copyright MIT License
 */

#include "main.h"

int main(void)
{
  rcc_setup();
  i2c_setup();
  usart_setup();

  usart_send_blocking(USART2, 'O');
  usart_send_blocking(USART2, 'K');
  usart_send_blocking(USART2, '\r');
  usart_send_blocking(USART2, '\n');

  while (1)
  {
  }

  return 0;
}

static void rcc_setup(void)
{
#if defined(STM32F1)
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
  rcc_periph_clock_enable(RCC_AFIO);
#elif defined(STM32F4)
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
#endif

  rcc_periph_clock_enable(RCC_I2C_GPIO);
  rcc_periph_clock_enable(RCC_I2C1);
  rcc_periph_clock_enable(RCC_USART_TXRX_GPIO);
  rcc_periph_clock_enable(RCC_USART2);
}

static void i2c_setup(void)
{
  /* Set SCL & SDA pin to open-drain alternate function. */
#if defined(STM32F1)
  gpio_set_mode(GPIO_I2C_PORT,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
                GPIO_I2C_SCL_PIN | GPIO_I2C_SDA_PIN);

  /*
   * Alternate function remap is required for
   * using I2C1_SCL & SDA on PB8 & PB9.
   * Refer to Table-5 in DS5319.
   */
  gpio_primary_remap(AFIO_MAPR_SWJ_CFG_FULL_SWJ,
                     AFIO_MAPR_I2C1_REMAP);
#else
  gpio_mode_setup(GPIO_I2C_PORT,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO_I2C_SCL_PIN | GPIO_I2C_SDA_PIN);

  gpio_set_output_options(GPIO_I2C_PORT,
                          GPIO_OTYPE_OD,
                          GPIO_OSPEED_50MHZ,
                          GPIO_I2C_SCL_PIN | GPIO_I2C_SDA_PIN);

  gpio_set_af(GPIO_I2C_PORT,
              GPIO_I2C_AF,
              GPIO_I2C_SCL_PIN | GPIO_I2C_SDA_PIN);
#endif
  uint32_t i2c = I2C1;

  i2c_peripheral_disable(i2c);
  i2c_reset(i2c);

  i2c_set_speed(i2c,
                i2c_speed_fm_400k,         /* 400 kHz Fast mode. */
                rcc_apb1_frequency / 1e6); /* I2C clock in MHz. */

  i2c_peripheral_enable(i2c);
}

static void usart_setup(void)
{
  /* Set USART-Tx & Rx pin to alternate function. */
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

  /* Setup interrupt. */
  nvic_enable_irq(NVIC_USART2_IRQ);
  usart_enable_rx_interrupt(USART2); /* Enable receive interrupt. */

  /* Config USART params. */
  usart_set_baudrate(USART2, USART_BAUDRATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX_RX);

  usart_enable(USART2);
}

static void delay(uint32_t value)
{
  for (uint32_t i = 0; i < value; i++)
  {
    __asm__("nop"); /* Do nothing. */
  }
}

/**
 * @brief USART2 Interrupt service routine.
 */
void usart2_isr(void)
{
  usart_disable_rx_interrupt(USART2);

  uint8_t data = usart_recv(USART2);
  if (data == 0x00) /* Write command. */
  {
    uint8_t i2c_rx_data[1];
    uint8_t i2c_tx_data[3];
    i2c_tx_data[0] = usart_recv_blocking(USART2); /* Address 1. */
    i2c_tx_data[1] = usart_recv_blocking(USART2); /* Address 2. */
    i2c_tx_data[2] = usart_recv_blocking(USART2); /* Data. */

    i2c_transfer7(I2C1,
                  I2C_SLAVE_ADDRESS,
                  i2c_tx_data,
                  3,
                  i2c_rx_data,
                  0);

    usart_send_blocking(USART2, 0xF0); /* Write done. */
  }
  else if (data == 0x01) /* Read command. */
  {
    uint8_t i2c_rx_data[1];
    uint8_t i2c_tx_data[2];
    i2c_tx_data[0] = usart_recv_blocking(USART2); /* Address 1. */
    i2c_tx_data[1] = usart_recv_blocking(USART2); /* Address 2. */

    i2c_transfer7(I2C1,
                  I2C_SLAVE_ADDRESS,
                  i2c_tx_data,
                  2,
                  i2c_rx_data,
                  1);

    usart_send_blocking(USART2, i2c_rx_data[0]);
  }
  else
  {
    /* Unknown command. */
    usart_send_blocking(USART2, 0xFF);
  }

  /* Clear RXNE(Read data register not empty) flag at SR(Status register). */
  USART_SR(USART2) &= ~USART_SR_RXNE;
  usart_enable_rx_interrupt(USART2);
}
