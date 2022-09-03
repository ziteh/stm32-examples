/**
 * @file   main.c
 * @brief  PWM(Pulse-width modulation) example for LibOpenCM3 with STM32.
 * @author ZiTe (honmonoh@gmail.com)
 * @copyright MIT License
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

#if defined(NUCLEO_F103RB)
  #define RCC_PWM_GPIO (RCC_GPIOA)
  #define GPIO_PWM_PORT (GPIOA)
  #define GPIO_PWM_PIN (GPIO7) /* D11. */
#elif defined(NUCLEO_F446RE)
  #define RCC_PWM_GPIO (RCC_GPIOA)
  #define GPIO_PWM_PORT (GPIOA)
  #define GPIO_PWM_PIN (GPIO7)   /* D11. */
  #define GPIO_PWM_AF (GPIO_AF2) /* Ref: Table-11 in DS10693. */
#else
  #error "STM32 board not defined."
#endif

static void rcc_setup(void)
{
#if defined(STM32F1)
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
#elif defined(STM32F4)
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
#endif

  rcc_periph_clock_enable(RCC_PWM_GPIO);
  rcc_periph_clock_enable(RCC_TIM3);
}

static void pwm_setup(void)
{
  /* Set PWM pin to alternate function. */
#if defined(NUCLEO_F103RB)
  gpio_set_mode(GPIO_PWM_PORT,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_PWM_PIN);
#else
  gpio_mode_setup(GPIO_PWM_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PWM_PIN);
  gpio_set_af(GPIO_PWM_PORT, GPIO_PWM_AF, GPIO_PWM_PIN);
#endif

  timer_set_mode(TIM3,
                 TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE,
                 TIM_CR1_DIR_UP);
  timer_disable_preload(TIM3);
  timer_continuous_mode(TIM3);
  timer_set_prescaler(TIM3, PWM_TIMER_PRESCALER);
  timer_set_period(TIM3, PWM_TIMER_PERIOD);
  timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);

  uint32_t ccr_value = PWM_TIMER_PERIOD * PWM_GOAL_DUTY_CYCLE / 100.0;
  timer_set_oc_value(TIM3, TIM_OC2, ccr_value);

  timer_enable_oc_output(TIM3, TIM_OC2);
  timer_enable_counter(TIM3);
}

int main(void)
{
  rcc_setup();
  pwm_setup();

  /* Halt. */
  while (1)
  {
    __asm__("nop"); /* Do nothing. */
  }

  return 0;
}
