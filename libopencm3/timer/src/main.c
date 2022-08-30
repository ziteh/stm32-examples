/**
 * @file   main.c
 * @brief  Timer example for STM32 Nucleo-F103RB and F446RE.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#define GOAL_FREQUENCY (5) /* Goal frequency in Hz. */

/* If apb_presc = /1 than timer_clock = apb_clock, else timer_clock = 2* apb_clock. */
#define TIMER_CLOCK (rcc_apb1_frequency * 2)
#define TIMER_PRESCALER (TIMER_CLOCK / 100000 - 1)

/*
 * f_goal = f_timer / [(Prescaler + 1) * (Period + 1)]
 * so
 * Period = {f_timer / [(Prescaler + 1) * f_goal]} - 1
 */
#define TIMER_PERIOD ((TIMER_CLOCK / ((TIMER_PRESCALER + 1) * GOAL_FREQUENCY)) - 1)

#ifdef NUCLEO_F103RB
#define NVIC_TIM_IRQ (NVIC_TIM2_IRQ)
#define RCC_TIM (RCC_TIM2)

#define RCC_LED_PORT (RCC_GPIOA)
#define GPIO_LED_PORT (GPIOA)
#define GPIO_LED_PIN (GPIO5)

#elif NUCLEO_F446RE
#define NVIC_TIM_IRQ (NVIC_TIM2_IRQ)
#define RCC_TIM (RCC_TIM2)

#define RCC_LED_PORT (RCC_GPIOA)
#define GPIO_LED_PORT (GPIOA)
#define GPIO_LED_PIN (GPIO5)
#else
#error
#endif

void rcc_setup(void);
void timer_setup(void);
void led_setup(void);

int main(void)
{
  rcc_setup();
  led_setup();
  timer_setup();

  /* Halt. */
  while (1)
  {
    __asm__("nop"); /* Do nothing. */
  }

  return 0;
}

void rcc_setup(void)
{
  /* Setup system clock. */
#ifdef NUCLEO_F103RB
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
#elif NUCLEO_F446RE
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
#endif

  rcc_periph_clock_enable(RCC_LED_PORT);
  rcc_periph_clock_enable(RCC_TIM);
}

void led_setup(void)
{
  /* Set to output Push-Pull. */
#ifdef NUCLEO_F103RB
  gpio_set_mode(GPIO_LED_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO_LED_PIN);
#elif NUCLEO_F446RE
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

void timer_setup(void)
{
  /* Interrupt. */
  nvic_enable_irq(NVIC_TIM_IRQ);
  timer_enable_irq(TIM2, TIM_DIER_CC1IE);

  /* Setup timer. */
  timer_set_mode(TIM2,
                 TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE,
                 TIM_CR1_DIR_UP);
  timer_disable_preload(TIM2);
  timer_continuous_mode(TIM2);

  timer_set_prescaler(TIM2, TIMER_PRESCALER);
  timer_set_period(TIM2, TIMER_PERIOD);

  timer_enable_counter(TIM2);
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

    gpio_toggle(GPIO_LED_PORT, GPIO_LED_PIN); /* LED on/off. */
  }
}
