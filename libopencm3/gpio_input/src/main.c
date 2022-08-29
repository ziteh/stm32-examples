/**
 *  @file  main.c
 *  @brief Button input example for STM32 Nucleo-F103RB and F446RE.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#ifdef NUCLEO_F103RB
  #define RCC_BUTTON_PORT (RCC_GPIOC)
  #define GPIO_BUTTON_PORT (GPIOC)
  #define GPIO_BUTTON_PIN (GPIO13)

  #define RCC_LED_PORT (RCC_GPIOA)
  #define GPIO_LED_PORT (GPIOA)
  #define GPIO_LED_PIN (GPIO5)
#elif NUCLEO_F446RE
  #define RCC_BUTTON_PORT (RCC_GPIOC)
  #define GPIO_BUTTON_PORT (GPIOC)
  #define GPIO_BUTTON_PIN (GPIO13)

  #define RCC_LED_PORT (RCC_GPIOA)
  #define GPIO_LED_PORT (GPIOA)
  #define GPIO_LED_PIN (GPIO5)
#else
  #error
#endif

int main(void)
{
  /* Enable clock. */
  rcc_periph_clock_enable(RCC_LED_PORT);
  rcc_periph_clock_enable(RCC_BUTTON_PORT);

#ifdef NUCLEO_F103RB
  /* Setup LED pin. */
  gpio_set_mode(GPIO_LED_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO_LED_PIN);

  /* Setup button pin. */
  gpio_set_mode(GPIO_BUTTON_PORT,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_FLOAT,
                GPIO_BUTTON_PIN);
#else
  /* Setup LED pin. */
  gpio_mode_setup(GPIO_LED_PORT,
                  GPIO_MODE_OUTPUT,
                  GPIO_PUPD_NONE,
                  GPIO_LED_PIN);
  gpio_set_output_options(GPIO_LED_PORT,
                          GPIO_OTYPE_PP,
                          GPIO_OSPEED_2MHZ,
                          GPIO_LED_PIN);

  /* Setup button pin. */
  gpio_mode_setup(GPIO_BUTTON_PORT,
                  GPIO_MODE_INPUT,
                  GPIO_PUPD_NONE,
                  GPIO_BUTTON_PIN);
#endif

  while (1)
  {
    /* Read input value. */
    bool pressed = gpio_get(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN) == 0;

    if (pressed)
    {
      gpio_set(GPIO_LED_PORT, GPIO_LED_PIN); /* LED on. */
    }
    else
    {
      gpio_clear(GPIO_LED_PORT, GPIO_LED_PIN); /* LED off. */
    }
  }

  return 0;
}
