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

#define PWM_GOAL_FREQUENCY (1000)  /* f_goal, PWM goal frequency in Hz. */
#define PWM_GOAL_DUTY_CYCLE (72.5) /* dc_goal, PWM goal duty-cycle in %. */

/**
 * @brief f_timer.
 * @note If APBx_Presc = /1 than f_timer = APBx_Clock, else f_timer = 2* APBx_Clock.
 */
#define PWM_TIMER_CLOCK (rcc_apb1_frequency * 2)

/**
 * @brief f_counter (CK_CNT).
 */
#define PWM_COUNTER_CLOCK (1000000)

/**
 * @brief PSC (Prescaler), the value of TIMx_PSC register.
 * @note f_counter = f_timer / (PSC + 1)
 *       so,
 *       PSC = f_timer / f_counter - 1
 */
#define PWM_TIMER_PRESCALER (PWM_TIMER_CLOCK / PWM_COUNTER_CLOCK - 1)

/**
 * @brief ARR (Auto-Reload), the value of TIMx_ARR register.
 * @note f_goal = f_timer / [(PSC + 1) * (ARR + 1)]
 *       so,
 *       ARR = {f_timer / [(PSC + 1) * f_goal]} - 1
 */
#define PWM_TIMER_PERIOD (((PWM_TIMER_CLOCK) / ((PWM_TIMER_PRESCALER + 1) * PWM_GOAL_FREQUENCY)) - 1) /* TIMx_ARR value. */

/**
 * @brief CCR (Capture/Compare), the value of TIMx_CCRx register.
 * @note dc_goal% = CCR / (ARR + 1) * 100%
 *       so,
 *       CCR = (ARR + 1) * dc_goal% / 100%
 */
#define PWM_TIMER_OC_VALUE ((PWM_TIMER_PERIOD + 1) * PWM_GOAL_DUTY_CYCLE / 100)

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
  rcc_periph_reset_pulse(RST_TIM3); /* Reset TIM3 to defaults. */
}

static void pwm_setup(void)
{
  /* Set PWM pin to alternate function push-pull. */
#if defined(NUCLEO_F103RB)
  gpio_set_mode(GPIO_PWM_PORT,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_PWM_PIN);
#else
  gpio_mode_setup(GPIO_PWM_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PWM_PIN);
  gpio_set_output_options(GPIO_PWM_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PWM_PIN);
  gpio_set_af(GPIO_PWM_PORT, GPIO_PWM_AF, GPIO_PWM_PIN);
#endif

  timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_disable_preload(TIM3);
  timer_continuous_mode(TIM3);

  timer_set_prescaler(TIM3, PWM_TIMER_PRESCALER);        /* Setup TIMx_PSC register. */
  timer_set_period(TIM3, PWM_TIMER_PERIOD);              /* Setup TIMx_ARR register. */
  timer_set_oc_value(TIM3, TIM_OC2, PWM_TIMER_OC_VALUE); /* Setup TIMx_CCRx register. */
  timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);

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
