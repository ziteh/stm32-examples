/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  PWM(Pulse-width modulation) example for STM32 Nucleo-F103RB and F446RE.
 * @remark Reference: https://bdebyl.net/post/stm32-part1/
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#define PWM_GOAL_FREQUENCY (1000)  /* PWM goal frequency in Hz. */
#define PWM_GOAL_DUTY_CYCLE (72.5) /* PWM goal duty-cycle in %. */

/* If apb_presc = /1 than timer_clock = apb_clock, else timer_clock = 2* apb_clock. */
#define PWM_TIMER_CLOCK (rcc_apb1_frequency * 2)
#define PWM_TIMER_PRESCALER (PWM_TIMER_CLOCK / 1000000 - 1)

/*
 * f_goal = f_timer / [(Prescaler + 1) * (Period + 1)]
 * so
 * Period = {f_timer / [(Prescaler + 1) * f_goal]} - 1
 */
#define PWM_TIMER_PERIOD (((PWM_TIMER_CLOCK) / ((PWM_TIMER_PRESCALER + 1) * PWM_GOAL_FREQUENCY)) - 1)

#define PWM_TIMER (TIM3)
#define PWM_CHANNEL (TIM_OC2)
#define RCC_PWM_TIMER (RCC_TIM3)

#ifdef NUCLEO_F103RB
/* PA7 = D11. */
#define RCC_PWM_PORT (RCC_GPIOA)
#define GPIO_PWM_PORT (GPIOA)
#define GPIO_PWM_PIN (GPIO7)
#elif NUCLEO_F446RE
/* PA7 = D11. */
#define RCC_PWM_PORT (RCC_GPIOA)
#define GPIO_PWM_PORT (GPIOA)
#define GPIO_PWM_PIN (GPIO7)
#else
#error
#endif

void rcc_setup(void)
{
#ifdef NUCLEO_F103RB
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
#elif NUCLEO_F446RE
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
#endif

  rcc_periph_clock_enable(RCC_PWM_PORT);
  rcc_periph_clock_enable(RCC_PWM_TIMER);
}

void gpio_setup(void)
{
  /* Set to alternate function. */
#ifdef NUCLEO_F103RB
  gpio_set_mode(GPIO_PWM_PORT,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_PWM_PIN);
#elif NUCLEO_F446RE
  gpio_mode_setup(GPIO_PWM_PORT,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO_PWM_PIN);

  gpio_set_af(GPIO_PWM_PORT,
              GPIO_AF2, /* Ref: DS10693 Table-11. */
              GPIO_PWM_PIN);
#endif
}

void pwm_setup(void)
{
  timer_set_mode(PWM_TIMER,
                 TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE,
                 TIM_CR1_DIR_UP);
  timer_disable_preload(PWM_TIMER);
  timer_continuous_mode(PWM_TIMER);
  timer_set_prescaler(PWM_TIMER, PWM_TIMER_PRESCALER);
  timer_set_period(PWM_TIMER, PWM_TIMER_PERIOD);
  timer_set_oc_mode(PWM_TIMER, PWM_CHANNEL, TIM_OCM_PWM1);

  uint32_t ccr_value = PWM_TIMER_PERIOD * PWM_GOAL_DUTY_CYCLE / 100.0;
  timer_set_oc_value(PWM_TIMER, PWM_CHANNEL, ccr_value);

  timer_enable_oc_output(PWM_TIMER, PWM_CHANNEL);
  timer_enable_counter(PWM_TIMER);
}

int main(void)
{
  rcc_setup();
  gpio_setup();
  pwm_setup();

  /* Halt. */
  while (1)
  {
    __asm__("nop"); /* Do nothing. */
  }

  return 0;
}
