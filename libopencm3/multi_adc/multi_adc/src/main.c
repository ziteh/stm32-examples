/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Multi channel ADC example.
 */

#include "printf.h"
#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>

#define USART_BAUD_RATE (9600)

#define LED_PORT (GPIOA)
#define LED_PIN (GPIO5)

uint16_t get_adc_value(int channel)
{
  uint8_t adc_channel[16];
  adc_channel[0] = channel;
  adc_set_regular_sequence(ADC1, 1, adc_channel);

  adc_start_conversion_direct(ADC1);

  /* Wait for ADC. */
  while (!adc_get_flag(ADC1, ADC_SR_EOC))
  {
    __asm__("nop"); /* Do nothing. */
  }

  return ADC_DR(ADC1);
}

void delay(unsigned int value)
{
  while (value--)
  {
    __asm__("nop"); /* Do nothing. */
  }
}

void setup_rcc(void)
{
  rcc_clock_setup_in_hsi_out_48mhz();

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_ADC1);
  rcc_periph_clock_enable(RCC_USART2);
}

void setup_led(void)
{
  gpio_set_mode(LED_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                LED_PIN);
}

void setup_adc(void)
{
  /* Setup PA0 for ADC1-IN0 analog input. */
  gpio_set_mode(GPIOA,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_ANALOG,
                GPIO0);

  /* Setup PA1 for ADC1-IN1 analog input. */
  gpio_set_mode(GPIOA,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_ANALOG,
                GPIO1);

  /* Setup PA4 for ADC1-IN4 analog input. */
  gpio_set_mode(GPIOA,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_ANALOG,
                GPIO4);

  adc_power_off(ADC1);

  adc_disable_scan_mode(ADC1);
  adc_disable_external_trigger_regular(ADC1);
  adc_set_single_conversion_mode(ADC1);
  adc_set_right_aligned(ADC1);
  adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_55DOT5CYC);

  adc_power_on(ADC1);
  delay(800000); /* Wait a bit. */
  adc_reset_calibration(ADC1);
  adc_calibrate(ADC1);
}

void setup_usart(void)
{
  /* Setup GPIO for USART-Tx. */
  gpio_set_mode(GPIO_BANK_USART2_TX,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_USART2_TX);

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
  setup_rcc();
  setup_led();
  setup_adc();
  setup_usart();

  uint8_t channel = 0;

  printf("Ready\r\n");

  while (1)
  {
    uint16_t adc_value = get_adc_value(channel);
    printf("CH%d: %4d ", channel, adc_value);

    switch (channel)
    {
    case 0:
      channel = 1;
      break;

    case 1:
      channel = 4;
      break;

    case 4:
    default:
      channel = 0;
      printf("\r\n");
      break;
    }

    gpio_toggle(LED_PORT, LED_PIN); /* LED on/off. */
    delay(100000);
  }

  return 0;
}