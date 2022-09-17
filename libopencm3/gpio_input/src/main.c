/**
 * @file   main.c
 * @brief  Polling button example for STM32 based on LibOpenCM3.
 * @author ZiTe (honmonoh@gmail.com)
 * @copyright MIT License
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

/* User LED connected to Arduino-D13 pin. */
#if defined(NUCLEO_F103RB) || \ 
    defined(NUCLEO_F401RE) || \
    defined(NUCLEO_F446RE)
  #define RCC_LED_GPIO (RCC_GPIOA)
  #define GPIO_LED_PORT (GPIOA)
  #define GPIO_LED_PIN (GPIO5)
#elif defined(NUCLEO_F302R8)
  #define RCC_LED_GPIO (RCC_GPIOB)
  #define GPIO_LED_PORT (GPIOB)
  #define GPIO_LED_PIN (GPIO13)
#elif defined(NUCLEO_L432KC)
  #define RCC_LED_GPIO (RCC_GPIOB)
  #define GPIO_LED_PORT (GPIOB)
  #define GPIO_LED_PIN (GPIO3)
#else
  #error "STM32 board not defined."
#endif

/* User button. */
#if defined(NUCLEO_L432KC)
  #define RCC_BUTTON_GPIO (RCC_GPIOA)
  #define GPIO_BUTTON_PORT (GPIOA)
  #define GPIO_BUTTON_PIN (GPIO12)
#else
  #define RCC_BUTTON_GPIO (RCC_GPIOC)
  #define GPIO_BUTTON_PORT (GPIOC)
  #define GPIO_BUTTON_PIN (GPIO13)
#endif

int main(void)
{
  /* Enable clock. */
  rcc_periph_clock_enable(RCC_LED_GPIO);
  rcc_periph_clock_enable(RCC_BUTTON_GPIO);

  /*
   * Set LED pin to output push-pull,
   * and set button pin to input floating.
   */
#if defined(STM32F1)
  gpio_set_mode(GPIO_LED_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO_LED_PIN);

  gpio_set_mode(GPIO_BUTTON_PORT,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_FLOAT,
                GPIO_BUTTON_PIN);
#else
  gpio_mode_setup(GPIO_LED_PORT,
                  GPIO_MODE_OUTPUT,
                  GPIO_PUPD_NONE,
                  GPIO_LED_PIN);
  gpio_set_output_options(GPIO_LED_PORT,
                          GPIO_OTYPE_PP,
                          GPIO_OSPEED_2MHZ,
                          GPIO_LED_PIN);

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
