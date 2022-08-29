/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Basic button external interrupt(EXTI).
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

/**
 * @brief Setup User-LED(PA5). 
 */
void led_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_set_mode(GPIOA,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO5);
}

/**
 * @brief Setup User-button(PC13). 
 */
void button_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_AFIO);

  nvic_enable_irq(NVIC_EXTI15_10_IRQ);

  gpio_set_mode(GPIOC,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_FLOAT,
                GPIO13);

  exti_select_source(EXTI13, GPIOC);
  exti_set_trigger(EXTI13, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI13);
}

/**
 * @brief EXTI15~10 Interrupt service routine.
 */
void exti15_10_isr(void)
{
  exti_reset_request(EXTI13);
  gpio_toggle(GPIOA, GPIO5);
}

int main(void)
{
  led_setup();
  button_setup();

  while (1)
  {
    __asm__("nop");
  }

  return 0;
}
