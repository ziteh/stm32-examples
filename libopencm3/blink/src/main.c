/**
 * @file   main.c
 * @brief  Blinking LED example for STM32 based on LibOpenCM3.
 * @author ZiTe (honmonoh@gmail.com)
 * @copyright MIT License
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

/* User LED (LD2) connected to Arduino-D13 pin. */
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
#elif defined(NUCLEO_G431KB)
  #define RCC_LED_GPIO (RCC_GPIOB)
  #define GPIO_LED_PORT (GPIOB)
  #define GPIO_LED_PIN (GPIO8)
#else
  #error "STM32 board not defined."
#endif

static void delay(uint32_t value)
{
  for (uint32_t i = 0; i < value; i++)
  {
    __asm__("nop"); /* Do nothing. */
  }
}

int main(void)
{
  /* Enable clock. */
  rcc_periph_clock_enable(RCC_LED_GPIO);

  /* Set LED pin to output push-pull. */
#if defined(STM32F1)
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
