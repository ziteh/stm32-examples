/**
 * @file user_main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @copyright MIT
 */

#include "user_main.h"
#include "user_print.h"
#include <stdio.h>

#define ADC_CALIBRATION // Comment out the line to skip ADC calibration.

volatile float adc_value = 0.0F;

void User_Init(void)
{
  printf("Init... ");

  // Setup Timer.
  LL_TIM_EnableAllOutputs(MY_TIM);
  LL_TIM_CC_EnableChannel(MY_TIM, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(MY_TIM, LL_TIM_CHANNEL_CH1N);
  LL_TIM_CC_EnableChannel(MY_TIM, LL_TIM_CHANNEL_CH4);
  LL_TIM_EnableCounter(MY_TIM);

#ifdef ADC_CALIBRATION
  // ADC calibration.
  LL_mDelay(10);
  LL_ADC_StartCalibration(MY_ADC, LL_ADC_SINGLE_ENDED);
  while (LL_ADC_IsCalibrationOnGoing(MY_ADC))
  {
    __NOP();
  }
  LL_mDelay(10);
#endif

  // Setup ADC.
  LL_ADC_EnableIT_JEOC(MY_ADC);
  LL_ADC_Enable(MY_ADC);
  LL_ADC_INJ_StartConversion(MY_ADC);

  printf("Start\r\n");
}

void User_Loop(void)
{
  // Wait for ISR.
  LL_mDelay(250);
  LL_GPIO_TogglePin(MY_LED_PORT, MY_LED_PIN); // LED blinking.
}

void User_ADC_ISR(void)
{
  if (LL_ADC_IsActiveFlag_JEOC(MY_ADC)) // Check end of conversion.
  {
    uint32_t raw = LL_ADC_INJ_ReadConversionData32(MY_ADC, LL_ADC_INJ_RANK_1);
    adc_value = (raw & 0xFFF) * (3.3F / 4095.0F); // Convert, 3.3V, 12-bits, 0xFFF: mask.

    printf("Raw: %4d, Val: %4d mV\r\n", (int)raw, (int)(adc_value * 1000));

    LL_ADC_ClearFlag_JEOC(MY_ADC);
    LL_ADC_INJ_StartConversion(MY_ADC); // Wait for next conversion.
  }
}

/**
 * @note A blocking implementation.
 */
PUTCHAR_PROTOTYPE
{
  uint16_t timeout = 0xFFFFU;
  while (!LL_USART_IsActiveFlag_TXE(MY_UART)) // Wait for Tx buffer empty.
  {
    if (--timeout == 0) return ch; // Comment out this line for infinite timeout.
  }
  LL_USART_TransmitData8(MY_UART, (uint8_t)ch);
  return ch;
}
