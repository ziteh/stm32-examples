/**
 * @file   main.c
 * @brief  SPI master mode example for STM32 based on LibOpenCM3.
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

  usart_send_blocking(USART2, 'M');
  usart_send_blocking(USART2, 'a');
  usart_send_blocking(USART2, 's');
  usart_send_blocking(USART2, 't');
  usart_send_blocking(USART2, 'e');
  usart_send_blocking(USART2, 'r');
  usart_send_blocking(USART2, '\r');
  usart_send_blocking(USART2, '\n');

  /* Halt. */
  while (1)
  {
    __asm__("nop"); /* Do nothing. */
  }

  return 0;
}

static void spi_select(void)
{
  gpio_clear(GPIO_SPI_PORT, GPIO_SPI_CS_PIN);
}

static void spi_deselect(void)
{
  gpio_set(GPIO_SPI_PORT, GPIO_SPI_CS_PIN);
}

static void rcc_setup(void)
{
#if defined(NUCLEO_F103RB)
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
  rcc_periph_clock_enable(RCC_AFIO); /* For EXTI. */
#elif defined(NUCLEO_F446RE)
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
  rcc_periph_clock_enable(RCC_SYSCFG); /* For EXTI. */
#endif

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_USART2);
  rcc_periph_clock_enable(RCC_SPI1);
}

static void spi_setup(void)
{
  /*
   * Set SPI-SCK & MISO & MOSI pin to alternate function.
   * Set SPI-CS pin to output push-pull (control CS by manual).
   */
#if defined(STM32F1)
  gpio_set_mode(GPIO_SPI_PORT,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_SPI_SCK_PIN | GPIO_SPI_MISO_PIN | GPIO_SPI_MOSI_PIN);

  gpio_set_mode(GPIO_SPI_PORT,
                GPIO_MODE_OUTPUT_10_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO_SPI_CS_PIN);
#else
  gpio_mode_setup(GPIO_SPI_PORT,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO_SPI_SCK_PIN | GPIO_SPI_MISO_PIN | GPIO_SPI_MOSI_PIN);

  gpio_set_output_options(GPIO_SPI_PORT,
                          GPIO_OTYPE_PP,
                          GPIO_OSPEED_50MHZ,
                          GPIO_SPI_SCK_PIN | GPIO_SPI_MOSI_PIN);

  gpio_set_af(GPIO_SPI_PORT,
              GPIO_SPI_AF,
              GPIO_SPI_SCK_PIN | GPIO_SPI_MISO_PIN | GPIO_SPI_MOSI_PIN);

  /* In master mode, control CS by user instead of AF. */
  gpio_mode_setup(GPIO_SPI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_SPI_CS_PIN);
  gpio_set_output_options(GPIO_SPI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_SPI_CS_PIN);
#endif

  spi_disable(SPI1);
  spi_reset(SPI1);

  /* Set up in master mode. */
  spi_init_master(SPI1,
                  SPI_CR1_BAUDRATE_FPCLK_DIV_64,   /* Clock baudrate. */
                  SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, /* CPOL = 0. */
                  SPI_CR1_CPHA_CLK_TRANSITION_2,   /* CPHA = 1. */
                  SPI_CR1_DFF_8BIT,                /* Data frame format. */
                  SPI_CR1_MSBFIRST);               /* Data frame bit order. */
  spi_set_full_duplex_mode(SPI1);

  /*
   * CS pin is not used on master side at standard multi-slave config.
   * It has to be managed internally (SSM=1, SSI=1)
   * to prevent any MODF error.
   */
  spi_enable_software_slave_management(SPI1); /* SSM = 1. */
  spi_set_nss_high(SPI1);                     /* SSI = 1. */

  spi_deselect();
  spi_enable(SPI1);
}

static void spi_rq_setup(void)
{
  /* Set RQ pin to input floating. */
#if defined(STM32F1)
  gpio_set_mode(GPIO_SPI_RQ_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_SPI_RQ_PIN);
#else
  gpio_mode_setup(GPIO_SPI_RQ_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_SPI_RQ_PIN);
#endif

  /* Setup interrupt. */
  exti_select_source(EXTI_SPI_RQ, GPIO_SPI_RQ_PORT);
  exti_set_trigger(EXTI_SPI_RQ, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI_SPI_RQ);
  nvic_enable_irq(NVIC_SPI_RQ_IRQ);
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

/**
 * @brief USART2 Interrupt service routine.
 */
void usart2_isr(void)
{
  uint8_t indata = usart_recv(USART2); /* Read received data. */

  spi_select();
  spi_send(SPI1, indata);

  /*
   * Wait for SPI transmit complete.
   * Ref: https://controllerstech.com/spi-using-registers-in-stm32/.
   */
  while (!(SPI_SR(SPI1) & SPI_SR_TXE)) /* Wait for 'Transmit buffer empty' flag to set. */
  {
  }
  while ((SPI_SR(SPI1) & SPI_SR_BSY)) /* Wait for 'Busy' flag to reset. */
  {
  }

  spi_deselect();

  /* Clear 'Read data register not empty' flag. */
  USART_SR(USART2) &= ~USART_SR_RXNE;
}

/**
 * @brief EXTI9~5 Interrupt service routine.
 */
void exti9_5_isr(void)
{
  exti_reset_request(EXTI_SPI_RQ);

  spi_select();
  spi_send(SPI1, 0x00);               /* Just for beget clock signal. */
  while ((SPI_SR(SPI1) & SPI_SR_BSY)) /* Wait for 'Busy' flag to reset. */
  {
  }
  uint8_t indata = spi_read(SPI1);

  while ((SPI_SR(SPI1) & SPI_SR_BSY)) /* Wait for 'Busy' flag to reset. */
  {
  }
  spi_deselect();

  usart_send_blocking(USART2, indata);
}