/**
 * @file   main.c
 * @brief  EXTI button example for STM32 based on LibOpenCM3.
 * @author ZiTe (honmonoh@gmail.com)
 * @copyright MIT License
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

#if defined(NUCLEO_F103RB)
  #define RCC_BUTTON_GPIO (RCC_GPIOC)
  #define GPIO_BUTTON_PORT (GPIOC)
  #define GPIO_BUTTON_PIN (GPIO13)
  #define NVIC_BUTTON_IRQ (NVIC_EXTI15_10_IRQ)
  #define EXTI_BUTTON_SOURCE (EXTI13)

  #define RCC_LED_GPIO (RCC_GPIOA)
  #define GPIO_LED_PORT (GPIOA)
  #define GPIO_LED_PIN (GPIO5) /* D13. */
#elif defined(NUCLEO_F446RE)
  #define RCC_BUTTON_GPIO (RCC_GPIOC)
  #define GPIO_BUTTON_PORT (GPIOC)
  #define GPIO_BUTTON_PIN (GPIO13)
  #define NVIC_BUTTON_IRQ (NVIC_EXTI15_10_IRQ)
  #define EXTI_BUTTON_SOURCE (EXTI13)

  #define RCC_LED_GPIO (RCC_GPIOA)
  #define GPIO_LED_PORT (GPIOA)
  #define GPIO_LED_PIN (GPIO5) /* D13. */
#else
  #error "STM32 board not defined."
#endif

#define DELAY_VALUE_A ((uint32_t)500000)
#define DELAY_VALUE_B ((uint32_t)200000)

uint32_t delay_value = DELAY_VALUE_A;

static void delay(uint32_t value)
{
  for (uint32_t i = 0; i < value; i++)
  {
    __asm__("nop"); /* Do nothing. */
  }
}

static void led_setup(void)
{
  /* Set LED pin to output push-pull. */
#if defined(STM32F1)
  gpio_set_mode(GPIO_LED_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO_LED_PIN);
#else
  gpio_mode_setup(GPIO_LED_PORT, GPIO_MODE_OUTPUT,GPIO_PUPD_NONE, GPIO_LED_PIN);
  gpio_set_output_options(GPIO_LED_PORT,GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_LED_PIN);
#endif
}

static void button_setup(void)
{
  /* Set button pin to input floating. */
#if defined(STM32F1)
  gpio_set_mode(GPIO_BUTTON_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_BUTTON_PIN);
#else
  gpio_mode_setup(GPIO_BUTTON_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_BUTTON_PIN);
#endif

  /* Set up interrupt. */
  nvic_enable_irq(NVIC_BUTTON_IRQ);
  exti_select_source(EXTI_BUTTON_SOURCE, GPIO_BUTTON_PORT);
  exti_set_trigger(EXTI_BUTTON_SOURCE, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI_BUTTON_SOURCE);
}

static void rcc_setup(void)
{
  rcc_periph_clock_enable(RCC_LED_GPIO);
  rcc_periph_clock_enable(RCC_BUTTON_GPIO);

  /* For EXTI. */
#if defined(STM32F1)
  rcc_periph_clock_enable(RCC_AFIO);
#else
  rcc_periph_clock_enable(RCC_SYSCFG);
#endif
}

int main(void)
{
  rcc_setup();
  led_setup();
  button_setup();

  /* Start blinking. */
  while (1)
  {
    gpio_toggle(GPIO_LED_PORT, GPIO_LED_PIN); /* LED on/off. */
    delay(delay_value);
  }

  return 0;
}

/**
 * @brief EXTI15~10 Interrupt service routine.
 */
void exti15_10_isr(void)
{
  exti_reset_request(EXTI13);

  if (delay_value == DELAY_VALUE_A)
  {
    delay_value = DELAY_VALUE_B;
  }
  else
  {
    delay_value = DELAY_VALUE_A;
  }
}