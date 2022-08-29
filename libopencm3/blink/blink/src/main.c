/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Blinking LED example.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

/* The User-LED was on PA5 (Port-A, Pin-5) in my case (NUCLEO-F103RB board). */
#define RCC_LED_PORT (RCC_GPIOA)
#define LED_PORT     (GPIOA)
#define LED_PIN      (GPIO5)

void delay(unsigned int value)
{
  while(value--)
  {
    __asm__("nop"); /* Do nothing. */
  }
}

int main(void)
{
  /* Enable clock. */
  rcc_periph_clock_enable(RCC_LED_PORT);

  /* Configure GPIO as push-pull output and maximum speed of 2 MHz. */
  gpio_set_mode(LED_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                LED_PIN);

  /* Start blinking. */
  while (1)
  {
    gpio_toggle(LED_PORT, LED_PIN); /* LED on/off. */
    delay(500000);                  /* Wait a bit. */
  }

  return 0;
}
