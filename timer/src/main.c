/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Basic timer interrupt.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

/* Timer */
#define TIMER_PRESCALER (rcc_apb1_frequency / 1000)
#define TIMER_PERIOD (500)

/* User-LED */
#define RCC_LED_PORT (RCC_GPIOA)
#define LED_PORT (GPIOA)
#define LED_PIN (GPIO5)

void led_setup(void)
{
  rcc_periph_clock_enable(RCC_LED_PORT);
  gpio_set_mode(LED_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                LED_PIN);
}

void timer_setup(void)
{
  rcc_periph_clock_enable(RCC_TIM2);
  rcc_periph_reset_pulse(RST_TIM2);

  nvic_enable_irq(NVIC_TIM2_IRQ);

  timer_set_mode(TIM2,
                 TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE,
                 TIM_CR1_DIR_UP);
  timer_disable_preload(TIM2);
  timer_continuous_mode(TIM2);

  timer_set_prescaler(TIM2, TIMER_PRESCALER);
  timer_set_period(TIM2, TIMER_PERIOD);

  timer_enable_counter(TIM2);
  timer_enable_irq(TIM2, TIM_DIER_CC1IE);
}

/**
 * @brief Timer2 Interrupt service routine.
 */
void tim2_isr(void)
{
  /*
   * SR: Status register.
   * CC1IF: Capture/Compare 1 interrupt flag.
   */

  if (timer_get_flag(TIM2, TIM_SR_CC1IF))
  {
    timer_clear_flag(TIM2, TIM_SR_CC1IF);

    gpio_toggle(LED_PORT, LED_PIN);
  }
}

int main(void)
{
  led_setup();
  timer_setup();

  /* Halt. */
  while (1)
  {
    __asm__("nop");
  }

  return 0;
}
