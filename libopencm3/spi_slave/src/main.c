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
  rcc_periph_clock_enable(RCC_AFIO); /* For EXTI. */
#elif defined(NUCLEO_F446RE)
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
  rcc_periph_clock_enable(RCC_SYSCFG); /* For EXTI. */
#endif
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
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
  /*
   * Set SPI-SCK & MISO & MOSI pin to alternate function.
   * Set SPI-CS pin to input pull-up.
   */
#if defined(STM32F1)
  gpio_set_mode(GPIO_SPI_SCK_MISO_MOSI_PORT,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_SPI_SCK_PIN | GPIO_SPI_MISO_PIN | GPIO_SPI_MOSI_PIN);

  gpio_set_mode(GPIO_SPI_CS_PORT,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_PULL_UPDOWN,
                GPIO_SPI_CS_PIN);
  GPIO_ODR(GPIO_SPI_CS_PORT) |= GPIO_SPI_CS_PIN; /* Set uo pull-up. */
#else
  gpio_mode_setup(GPIO_SPI_SCK_MISO_MOSI_PORT,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO_SPI_SCK_PIN | GPIO_SPI_MISO_PIN | GPIO_SPI_MOSI_PIN);

  gpio_set_output_options(GPIO_SPI_SCK_MISO_MOSI_PORT,
                          GPIO_OTYPE_PP,
                          GPIO_OSPEED_50MHZ,
                          GPIO_SPI_SCK_PIN | GPIO_SPI_MISO_PIN | GPIO_SPI_MOSI_PIN);

  gpio_set_af(GPIO_SPI_SCK_MISO_MOSI_PORT,
              GPIO_SPI_AF,
              GPIO_SPI_SCK_PIN | GPIO_SPI_MISO_PIN | GPIO_SPI_MOSI_PIN);

  gpio_mode_setup(GPIO_SPI_CS_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_SPI_CS_PIN);
#endif

  nvic_enable_irq(NVIC_SPI_CS_IRQ);
  exti_select_source(EXTI_SPI_CS, GPIO_SPI_CS_PORT);
  exti_set_trigger(EXTI_SPI_CS, EXTI_TRIGGER_BOTH);
  exti_enable_request(EXTI_SPI_CS);

  spi_reset(SPI1);

  /* SPI init. */
  spi_init_master(SPI1,
                  SPI_CR1_BAUDRATE_FPCLK_DIV_64,   /* Clock baudrate. */
                  SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, /* Set clock to low when idle. */
                  SPI_CR1_CPHA_CLK_TRANSITION_2,   /* Data is sampled on the 2nd edge. */
                  SPI_CR1_DFF_8BIT,                /* Data frame format. */
                  SPI_CR1_MSBFIRST);               /* Data frame bit order. */

  spi_set_slave_mode(SPI1); /* Set to slave mode. */
  spi_set_full_duplex_mode(SPI1);

  spi_enable(SPI1);
}

static void spi_rq_setup(void)
{
  /* Set RQ pin to output open-drain. */
#if defined(STM32F1)
  gpio_set_mode(GPIO_SPI_RQ_PORT,
                GPIO_MODE_OUTPUT_10_MHZ,
                GPIO_CNF_OUTPUT_OPENDRAIN,
                GPIO_SPI_RQ_PIN);
#else
  gpio_mode_setup(GPIO_SPI_RQ_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_SPI_RQ_PIN);
  gpio_set_output_options(GPIO_SPI_RQ_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO_SPI_RQ_PIN);
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

  /* Clear RXNE(Read data register not empty) flag at SR(Status register). */
  USART_SR(USART2) &= ~USART_SR_RXNE;
}

/**
 * @brief EXTI9~5 Interrupt service routine.
 */
void exti9_5_isr(void)
{
  exti_reset_request(EXTI_SPI_CS);

  bool spi_selected = gpio_get(GPIO_SPI_CS_PORT, GPIO_SPI_CS_PIN) == 0;
  if (spi_selected)
  {
    while ((SPI_SR(SPI1) & (SPI_SR_BSY))) /* Wait for BSY(Busy) flag to reset. */
    {
    }

    uint8_t indata = spi_read(SPI1);
    spi_rq_reset();
    usart_send_blocking(USART2, indata);
  }
}