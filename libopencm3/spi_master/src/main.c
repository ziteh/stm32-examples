/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  SPI master mode example for STM32 Nucleo-F103RB and F446RE.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

#define USART_BAUDRATE (9600)

#ifdef NUCLEO_F103RB
#define RCC_SPI (RCC_SPI1)
#define RCC_SPI_GPIO (RCC_GPIOA | RCC_GPIOB)
#define GPIO_SPI_SCK_PORT (GPIOA)
#define GPIO_SPI_SCK_PIN (GPIO5)
#define GPIO_SPI_MISO_PORT (GPIOA)
#define GPIO_SPI_MISO_PIN (GPIO6)
#define GPIO_SPI_MOSI_PORT (GPIOA)
#define GPIO_SPI_MOSI_PIN (GPIO7)
#define GPIO_SPI_CS_PORT (GPIOB)
#define GPIO_SPI_CS_PIN (GPIO6)

#define RCC_SPI_RQ_GPIO (RCC_GPIOC)
#define GPIO_SPI_RQ_PORT (GPIOC)
#define GPIO_SPI_RQ_PIN (GPIO7)
#define NVIC_SPI_RQ_IRQ (NVIC_EXTI9_5_IRQ)
#define EXTI_SPI_RQ (EXTI7)

#define RCC_USART (RCC_USART2)
#define RCC_USART_TXRX_GPIO (RCC_GPIOA)
#define GPIO_USART_TXRX_PORT (GPIOA)
#define GPIO_USART_TX_PIN (GPIO2)
#define GPIO_USART_RX_PIN (GPIO3)
#elif NUCLEO_F446RE
#define RCC_SPI (RCC_SPI1)
#define RCC_SPI_GPIO (RCC_GPIOA | RCC_GPIOB)
#define GPIO_SPI_SCK_PORT (GPIOA)
#define GPIO_SPI_SCK_PIN (GPIO5)
#define GPIO_SPI_MISO_PORT (GPIOA)
#define GPIO_SPI_MISO_PIN (GPIO6)
#define GPIO_SPI_MOSI_PORT (GPIOA)
#define GPIO_SPI_MOSI_PIN (GPIO7)
#define GPIO_SPI_CS_PORT (GPIOB)
#define GPIO_SPI_CS_PIN (GPIO6)

#define RCC_SPI_RQ_GPIO (RCC_GPIOC)
#define GPIO_SPI_RQ_PORT (GPIOC)
#define GPIO_SPI_RQ_PIN (GPIO7)
#define NVIC_SPI_RQ_IRQ (NVIC_EXTI9_5_IRQ)
#define EXTI_SPI_RQ (EXTI7)

#define RCC_USART (RCC_USART2)
#define RCC_USART_TXRX_GPIO (RCC_GPIOA)
#define GPIO_USART_TXRX_PORT (GPIOA)
#define GPIO_USART_TX_PIN (GPIO2)
#define GPIO_USART_RX_PIN (GPIO3)
#else
#error
#endif

void rcc_setup(void);
void spi_setup(void);
void usart_setup(void);
void spi_rq_setup(void);
void spi_select(void);
void spi_deselect(void);

int main(void)
{
  rcc_setup();
  usart_setup();
  spi_setup();
  spi_rq_setup();

  usart_send_blocking(USART2, 'O');
  usart_send_blocking(USART2, 'K');

  /* Halt. */
  while (1)
  {
    __asm__("nop"); /* Do nothing. */
  }

  return 0;
}

void spi_select(void)
{
  gpio_clear(GPIO_SPI_CS_PORT, GPIO_SPI_CS_PIN);
}

void spi_deselect(void)
{
  gpio_set(GPIO_SPI_CS_PORT, GPIO_SPI_CS_PIN);
}

void rcc_setup(void)
{
#ifdef NUCLEO_F103RB
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
  rcc_periph_clock_enable(RCC_AFIO); /* For EXTI. */
#elif NUCLEO_F446RE
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
  rcc_periph_clock_enable(RCC_SYSCFG); /* For EXTI. */
#endif

  rcc_periph_clock_enable(RCC_USART_TXRX_GPIO);
  rcc_periph_clock_enable(RCC_USART);
  rcc_periph_clock_enable(RCC_SPI_GPIO);
  rcc_periph_clock_enable(RCC_SPI);
  rcc_periph_clock_enable(RCC_SPI_RQ_GPIO);
}

void spi_setup(void)
{
#ifdef NUCLEO_F103RB
  gpio_set_mode(GPIO_SPI_SCK_PORT,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_SPI_SCK_PIN);

  gpio_set_mode(GPIO_SPI_MISO_PORT,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_SPI_MISO_PIN);

  gpio_set_mode(GPIO_SPI_MOSI_PORT,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_SPI_MOSI_PIN);

  /* Control CS by manual instead of AF. */
  gpio_set_mode(GPIO_SPI_CS_PORT,
                GPIO_MODE_OUTPUT_10_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO_SPI_CS_PIN);
#elif NUCLEO_F446RE
  gpio_mode_setup(GPIO_SPI_SCK_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_SPI_SCK_PIN);
  gpio_mode_setup(GPIO_SPI_MISO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_SPI_MISO_PIN);
  gpio_mode_setup(GPIO_SPI_MOSI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_SPI_MOSI_PIN);

  gpio_set_output_options(GPIO_SPI_SCK_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_SPI_SCK_PIN);
  gpio_set_output_options(GPIO_SPI_MOSI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_SPI_MOSI_PIN);

  /* Ref: Table-11 in DS10693. */
  gpio_set_af(GPIO_SPI_SCK_PORT, GPIO_AF5, GPIO_SPI_SCK_PIN);
  gpio_set_af(GPIO_SPI_MISO_PORT, GPIO_AF5, GPIO_SPI_MISO_PIN);
  gpio_set_af(GPIO_SPI_MOSI_PORT, GPIO_AF5, GPIO_SPI_MOSI_PIN);

  /* Control CS by manual instead of AF. */
  gpio_mode_setup(GPIO_SPI_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_SPI_CS_PIN);
  gpio_set_output_options(GPIO_SPI_CS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_SPI_CS_PIN);
#endif

  spi_reset(SPI1);

  /* Set up in master mode. */
  spi_init_master(SPI1,
                  SPI_CR1_BAUDRATE_FPCLK_DIV_64,   /* Clock baudrate. */
                  SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, /* Set clock to low when idle. */
                  SPI_CR1_CPHA_CLK_TRANSITION_2,   /* Data is sampled on the 2nd edge. */
                  SPI_CR1_DFF_8BIT,                /* Data frame format. */
                  SPI_CR1_MSBFIRST);               /* Data frame bit order. */

  /* Control CS by manual instead of AF. */
  spi_disable_software_slave_management(SPI1);
  spi_deselect();

  spi_enable(SPI1);
}

void spi_rq_setup(void)
{
  /* Set to input pull-up. */
#ifdef NUCLEO_F103RB
  gpio_set_mode(GPIO_SPI_RQ_PORT,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_PULL_UPDOWN,
                GPIO_SPI_RQ_PIN);

  GPIO_ODR(GPIO_SPI_RQ_PORT) |= GPIO_SPI_RQ_PIN; /* Set to Pull-Up */
#else
  gpio_mode_setup(GPIO_SPI_RQ_PORT,
                  GPIO_MODE_INPUT,
                  GPIO_PUPD_PULLUP,
                  GPIO_SPI_RQ_PIN);
#endif

  /* Set up interrupt. */
  nvic_enable_irq(NVIC_SPI_RQ_IRQ);
  exti_select_source(EXTI_SPI_RQ, GPIO_SPI_RQ_PORT);
  exti_set_trigger(EXTI_SPI_RQ, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI_SPI_RQ);
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
   * TXE: Transmit register empty.
   * BSY: Busy.
   *
   * Ref: https://controllerstech.com/spi-using-registers-in-stm32/.
   */
  while (!(SPI_SR(SPI1) & (SPI_SR_TXE))) /* Wait for TXE flag to set. */
  {
  }
  while ((SPI_SR(SPI1) & (SPI_SR_BSY))) /* Wait for BSY flag to reset. */
  {
  }

  spi_deselect();

  /* Clear RXNE(Read data register not empty) flag at SR(Status register). */
  USART_SR(USART2) &= ~USART_SR_RXNE;
}

/**
 * @brief EXTI9~5 Interrupt service routine.
 */
void exti9_5_isr(void)
{
  exti_reset_request(EXTI_SPI_RQ);

  spi_select();
  spi_send(SPI1, 0x00); /* Just for beget clock signal. */
  uint8_t indata = spi_read(SPI1);

  /*
   * Wait for SPI receive complete.
   * BSY: Busy.
   * RXNE: Receive register not empty.
   *
   * Ref: https://controllerstech.com/spi-using-registers-in-stm32/.
   */
  while ((SPI_SR(SPI1) & (SPI_SR_BSY))) /* Wait for BSY flag to reset. */
  {
  }
  while (!(SPI_SR(SPI1) & (SPI_SR_RXNE))) /* Wait for RXNE flag to set. */
  {
  }

  spi_deselect();

  usart_send_blocking(USART2, indata);
}