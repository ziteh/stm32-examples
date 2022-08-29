/**
  *  @file  main.c
  *  @brief Blinking LED example for STM32 Nucleo-F103RB and F446RE.
  */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

/* User LED */
#ifdef NUCLEO_F103RB
  #define RCC_LED_PORT    (RCC_GPIOA)
  #define GPIO_LED_PORT   (GPIOA)
  #define GPIO_LED_PIN    (GPIO5)
#elif NUCLEO_F446RE
  #define RCC_LED_PORT    (RCC_GPIOA)
  #define GPIO_LED_PORT   (GPIOA)
  #define GPIO_LED_PIN    (GPIO5)
#else
  #error
#endif

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
  
  /* Setup LED pin. */
#ifdef NUCLEO_F103RB
  gpio_set_mode(GPIO_LED_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO_LED_PIN);
#else
  gpio_mode_setup(GPIO_LED_PORT,
                  GPIO_MODE_OUTPUT,
                  GPIO_PUPD_NONE,
                  GPIO_LED_PIN);

  gpio_set_output_options(GPIO_LED_PORT,
                          GPIO_OTYPE_PP,
                          GPIO_OSPEED_2MHZ,
                          GPIO_LED_PIN);
#endif

  /* Start blinking. */
  while (1)
  {
    gpio_toggle(GPIO_LED_PORT, GPIO_LED_PIN); /* LED on/off. */
    delay(500000);
  }

  return 0;
}
