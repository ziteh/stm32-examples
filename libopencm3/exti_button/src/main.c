/**
 *  @file  main.c
 *  @brief EXTI example for STM32 Nucleo-F103RB and F446RE.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

#ifdef NUCLEO_F103RB
  #define RCC_BUTTON_PORT (RCC_GPIOC)
  #define GPIO_BUTTON_PORT (GPIOC)
  #define GPIO_BUTTON_PIN (GPIO13)
  #define NVIC_BUTTON_IRQ (NVIC_EXTI15_10_IRQ)
  #define EXTI_BUTTON_SOURCE (EXTI13)

  #define RCC_LED_PORT (RCC_GPIOA)
  #define GPIO_LED_PORT (GPIOA)
  #define GPIO_LED_PIN (GPIO5)
#elif NUCLEO_F446RE
  #define RCC_BUTTON_PORT (RCC_GPIOC)
  #define GPIO_BUTTON_PORT (GPIOC)
  #define GPIO_BUTTON_PIN (GPIO13)
  #define NVIC_BUTTON_IRQ (NVIC_EXTI15_10_IRQ)
  #define EXTI_BUTTON_SOURCE (EXTI13)

  #define RCC_LED_PORT (RCC_GPIOA)
  #define GPIO_LED_PORT (GPIOA)
  #define GPIO_LED_PIN (GPIO5)
#else
  #error
#endif

#define DELAY_VALUE_A ((uint32_t)500000)
#define DELAY_VALUE_B ((uint32_t)200000)

uint32_t delay_value = DELAY_VALUE_A;

void delay(uint32_t value)
{
  while (value--)
  {
    __asm__("nop"); /* Do nothing. */
  }
}

void led_setup(void)
{
  /* Set up output Push-Pull. */
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
}

void button_setup(void)
{
  /* Set to input floating. */
#ifdef NUCLEO_F103RB
  gpio_set_mode(GPIO_BUTTON_PORT,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_FLOAT,
                GPIO_BUTTON_PIN);
#else
  gpio_mode_setup(GPIO_BUTTON_PORT,
                  GPIO_MODE_INPUT,
                  GPIO_PUPD_NONE,
                  GPIO_BUTTON_PIN);
#endif

  /* Set up interrupt. */
  nvic_enable_irq(NVIC_BUTTON_IRQ);
  exti_select_source(EXTI_BUTTON_SOURCE, GPIO_BUTTON_PORT);
  exti_set_trigger(EXTI_BUTTON_SOURCE, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI_BUTTON_SOURCE);
}

void rcc_setup(void)
{
  rcc_periph_clock_enable(RCC_LED_PORT);
  rcc_periph_clock_enable(RCC_BUTTON_PORT);

  /* For EXTI. */
#ifdef NUCLEO_F103RB
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