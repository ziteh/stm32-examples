/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Basic ADC example.
 */

#include <stdio.h>
#include <errno.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>

#define USART_BAUD_RATE (9600)

/**
 * @brief For printf().
 * 
 * @param file 
 * @param ptr 
 * @param len 
 * @return int 
 */
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

static void delay(int value)
{
  for (uint32_t i = 0; i < value; i++)
  {
    __asm__("nop");
  }
}

static void gpio_setup(void)
{
  /* All the used GPIO are Port-A. */
  rcc_periph_clock_enable(RCC_GPIOA);

  /* Setup PA0 for ADC1-In0 analog input. */
  gpio_set_mode(GPIOA,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_ANALOG,
                GPIO0);

  /* Setup GPIO for USART-Tx. */
  gpio_set_mode(GPIO_BANK_USART2_TX,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_USART2_TX);

  /* Setup User-LED. */
  gpio_set_mode(GPIOA,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO5);
}

static void adc_setup(void)
{
  rcc_periph_clock_enable(RCC_ADC1);

  adc_power_off(ADC1);

  adc_disable_scan_mode(ADC1);
  adc_disable_external_trigger_regular(ADC1);
  adc_set_single_conversion_mode(ADC1);
  adc_set_right_aligned(ADC1);
  adc_set_sample_time(ADC1, ADC_CHANNEL0, ADC_SMPR_SMP_55DOT5CYC);

  adc_power_on(ADC1);
  delay(800000); /* Wait a bit. */
  adc_reset_calibration(ADC1);
  adc_calibrate(ADC1);
}

static void usart_setup(void)
{
  rcc_periph_clock_enable(RCC_USART2);

  usart_set_baudrate(USART2, USART_BAUD_RATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX);

  usart_enable(USART2);
}

int main(void)
{
  rcc_clock_setup_in_hsi_out_48mhz();

  gpio_setup();
  usart_setup();
  adc_setup();

  uint8_t adc_channel[16];
  adc_channel[0] = 0;
  adc_set_regular_sequence(ADC1, 1, adc_channel);
  adc_start_conversion_direct(ADC1);

  printf("Ready.\r\n");

  while (1)
  {
    /* Wait for ADC convert complete. */
    if (adc_get_flag(ADC1, ADC_SR_EOC))
    {
      uint16_t adc_value = ADC_DR(ADC1);
      printf("ADC Value: %i \r\n", adc_value);

      gpio_toggle(GPIOA, GPIO5);
      delay(100000); /* Wait a bit. */
      adc_start_conversion_direct(ADC1);
    }
  }

  return 0;
}
