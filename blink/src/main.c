/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Basic LED blinking example.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

// The User-LED is PA5(Port-A, Pin-5).
#define RCC_LED_PORT (RCC_GPIOA)
#define LED_PORT (GPIOA)
#define LED_PIN (GPIO5)

void gpio_setup(void)
{
  // Enable RCC.
  rcc_periph_clock_enable(RCC_LED_PORT);

  // Setup GPIO mode.
  gpio_set_mode(LED_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                LED_PIN);
}

void delay(int value)
{
  for (int i = 0; i < value; i++)
  {
    __asm__("nop");
  }
}

int main(void)
{
  gpio_setup();

  while (1)
  {
    gpio_toggle(LED_PORT, LED_PIN);
    delay(1000000);
  }

  return 0;
}
