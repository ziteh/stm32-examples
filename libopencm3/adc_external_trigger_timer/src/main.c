/**
 * @file   main.c
 * @brief  ADC external trigger by timer example for LibOpenCM3 with STM32.
 * @author ZiTe (honmonoh@gmail.com)
 * @copyright MIT License
 */

#include "main.h"

int main(void)
{
  rcc_setup();
  usart_setup();
  adc_setup();
  timer_setup();

  printf("ADC External Trigger by Timer.\r\n");

  /* Halt. */
  while (1)
  {
  }
  return 0;
}

static void rcc_setup(void)
{
#if defined(STM32F1)
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
#elif defined(STM32F4)
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
#endif

  rcc_periph_clock_enable(RCC_USART_TX_GPIO);
  rcc_periph_clock_enable(RCC_USART2);
  rcc_periph_clock_enable(RCC_ADC_GPIO);
  rcc_periph_clock_enable(RCC_ADC1);

  rcc_periph_clock_enable(RCC_TIM3);
  rcc_periph_reset_pulse(RST_TIM3);
}

static void adc_setup(void)
{
/* Set to input analog. */
#if defined(STM32F1)
  gpio_set_mode(GPIO_ADC_PORT,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_ANALOG,
                GPIO_ADC_IN0_PIN);
#else
  gpio_mode_setup(GPIO_ADC_PORT,
                  GPIO_MODE_ANALOG,
                  GPIO_PUPD_NONE,
                  GPIO_ADC_IN0_PIN);
#endif
  uint32_t adc = ADC1;

  /* Setup ADC. */
  adc_power_off(adc);

  adc_disable_scan_mode(adc);
  adc_set_single_conversion_mode(adc);
  adc_set_right_aligned(adc);
  adc_set_sample_time_on_all_channels(adc, ADC_SIMPLE_TIME);

#if defined(STM32F1)
  adc_enable_external_trigger_regular(adc,
                                      ADC_CR2_EXTSEL_TIM3_TRGO);
#else
  adc_enable_external_trigger_regular(adc,
                                      ADC_CR2_EXTSEL_TIM3_TRGO,
                                      ADC_CR2_EXTEN_RISING_EDGE);
#endif

  /* Setup interrupt. */
  adc_enable_eoc_interrupt(adc);
  nvic_enable_irq(ADC_IRQ);

  uint8_t channels[16];
  channels[0] = 0;
  adc_set_regular_sequence(adc, 1, channels);

  adc_power_on(adc);
  delay(800000); /* Wait a bit. */

#if defined(STM32F1)
  adc_reset_calibration(adc);
  adc_calibrate(adc);
#endif
}

static void timer_setup(void)
{
  uint32_t timer = TIM3;

  timer_set_mode(timer,
                 TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE,
                 TIM_CR1_DIR_UP);
  timer_set_prescaler(timer, TIMER_PRESCALER); /* PSC register. */
  timer_set_period(timer, TIMER_PERIOD);       /* ARR register. */

  /* The update event is selected as trigger output (TRGO). */
  timer_set_master_mode(timer, TIM_CR2_MMS_UPDATE);

  timer_enable_counter(timer);
}

static void usart_setup(void)
{
  /* Set USART-Tx pin to alternate function. */
#if defined(NUCLEO_F103RB)
  gpio_set_mode(GPIO_USART_TX_PORT,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_USART_TX_PIN);
#else
  gpio_mode_setup(GPIO_USART_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_USART_TX_PIN);
  gpio_set_af(GPIO_USART_TX_PORT, GPIO_USART_AF, GPIO_USART_TX_PIN);
#endif

  /* Config USART params. */
  usart_set_baudrate(USART2, USART_BAUDRATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX);

  usart_enable(USART2);
}

static void delay(uint32_t value)
{
  for (uint32_t i = 0; i < value; i++)
  {
    __asm__("nop"); /* Do nothing. */
  }
}

/* For printf(). */
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

/**
 * @brief ADC Interrupt service routine.
 */
#if defined(STM32F1)
void adc1_2_isr(void)
#else
void adc_isr(void)
#endif
{
  /* Clear regular end of conversion flag. */
  ADC_SR(ADC1) &= ~ADC_SR_EOC;

  uint16_t value = adc_read_regular(ADC1);
  printf("%4d\r\n", value);
}