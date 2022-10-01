/**
 * @file   main.c
 * @brief  SPI slave mode example for STM32 based on LibOpenCM3.
 * @author ZiTe (honmonoh@gmail.com)
 * @copyright MIT License
 */

#include "main.h"

int main(void)
{
  rcc_setup();
  usart_setup();
  spi_setup();
  spi_rq_setup();

  usart_send_blocking(USART2, 's');
  usart_send_blocking(USART2, 'l');
  usart_send_blocking(USART2, 'a');
  usart_send_blocking(USART2, 'v');
  usart_send_blocking(USART2, 'e');
  usart_send_blocking(USART2, '\r');
  usart_send_blocking(USART2, '\n');

  /* Halt. */
  while (1)
  {
    __asm__("nop"); /* Do nothing. */
  }

  return 0;
}

static void rcc_setup(void)
{
#if defined(NUCLEO_F103RB)
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
#elif defined(NUCLEO_F446RE)
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
#endif

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_USART2);
  rcc_periph_clock_enable(RCC_SPI1);
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

  /* Setup USART config. */
  usart_set_baudrate(USART2, USART_BAUDRATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX_RX);

  usart_enable(USART2);
}

static void spi_setup(void)
{
  /* Set SPI pins to alternate function. */
#if defined(STM32F1)
  gpio_set_mode(GPIO_SPI_PORT,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_SPI_SCK_PIN | GPIO_SPI_MISO_PIN | GPIO_SPI_MOSI_PIN | GPIO_SPI_CS_PIN);
#else
  gpio_mode_setup(GPIO_SPI_PORT,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO_SPI_SCK_PIN | GPIO_SPI_MISO_PIN | GPIO_SPI_MOSI_PIN | GPIO_SPI_CS_PIN);

  gpio_set_output_options(GPIO_SPI_PORT,
                          GPIO_OTYPE_PP,
                          GPIO_OSPEED_50MHZ,
                          GPIO_SPI_MISO_PIN);

  gpio_set_af(GPIO_SPI_PORT,
              GPIO_SPI_AF,
              GPIO_SPI_SCK_PIN | GPIO_SPI_MISO_PIN | GPIO_SPI_MOSI_PIN | GPIO_SPI_CS_PIN);
#endif

  spi_disable(SPI1);
  spi_reset(SPI1);

  /* SPI init. */
  spi_init_master(SPI1,
                  SPI_CR1_BAUDRATE_FPCLK_DIV_64,   /* Clock baudrate. */
                  SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, /* CPOL = 0. */
                  SPI_CR1_CPHA_CLK_TRANSITION_2,   /* CPHA = 1. */
                  SPI_CR1_DFF_8BIT,                /* Data frame format. */
                  SPI_CR1_MSBFIRST);               /* Data frame bit order. */
  spi_set_slave_mode(SPI1);                        /* Set to slave mode. */
  spi_set_full_duplex_mode(SPI1);

  /*
   * Set to hardware NSS management and NSS output disable.
   * The NSS pin works as a standard “chip select” input in slave mode.
   */
  spi_disable_software_slave_management(SPI1); /* SSM = 0. */
  spi_disable_ss_output(SPI1);                 /* SSOE = 0. */

  /* Serup interrupt. */
  spi_enable_rx_buffer_not_empty_interrupt(SPI1);
  nvic_enable_irq(NVIC_SPI1_IRQ);

  spi_enable(SPI1);
}

static void spi_rq_setup(void)
{
  /* Set RQ pin to output push-pull. */
#if defined(STM32F1)
  gpio_set_mode(GPIO_SPI_RQ_PORT,
                GPIO_MODE_OUTPUT_10_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO_SPI_RQ_PIN);
#else
  gpio_mode_setup(GPIO_SPI_RQ_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_SPI_RQ_PIN);
  gpio_set_output_options(GPIO_SPI_RQ_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_SPI_RQ_PIN);
#endif

  spi_rq_reset();
}

static void spi_rq_set(void)
{
  gpio_clear(GPIO_SPI_RQ_PORT, GPIO_SPI_RQ_PIN);
}

static void spi_rq_reset(void)
{
  gpio_set(GPIO_SPI_RQ_PORT, GPIO_SPI_RQ_PIN);
}

/**
 * @brief USART2 Interrupt service routine.
 */
void usart2_isr(void)
{
  uint8_t indata = usart_recv(USART2); /* Read received data. */
  spi_send(SPI1, indata);              /* Put data into buffer. */
  spi_rq_set();                        /* Request master device to select this device. */

  /* Clear 'Read data register not empty' flag. */
  USART_SR(USART2) &= ~USART_SR_RXNE;
}

/**
 * @brief SPI1 Interrupt service routine.
 */
void spi1_isr(void)
{
  /* Wait for 'Busy' flag to reset. */
  while ((SPI_SR(SPI1) & SPI_SR_BSY))
  {
  }

  uint8_t indata = spi_read(SPI1);
  spi_rq_reset();
  usart_send_blocking(USART2, indata);

  /* Clear 'Read data register not empty' flag. */
  SPI_SR(SPI1) &= ~SPI_SR_RXNE;
}