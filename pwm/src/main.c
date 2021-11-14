/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Basic PWM(Pulse-width modulation) output example.
 * @remark Reference: https://bdebyl.net/post/stm32-part1/
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#define PWM_FREQUENCY (1000)  /* PWM frequency in Hz. */
#define PWM_DUTY_CYCLE (72.5) /* PWM duty-cycle in %. */

/*
 * f_pwm = f_tim / [(PRS + 1) * (PER + 1)]
 * 
 * So,
 * PER = {f_tim / [(PRS + 1) * f_pwm]} - 1
 * 
 * f_pwm: PWM frequency.
 * f_tim: Timer frequency. The value is 'rcc_apb1_frequency * 2' equal 48MHz in this case.
 * PRS:   PWM timer prescaler.
 * PER:   PWM timer period.
 * 
 * We can get the value of f_tim by 'Clock Tree'.
 * 
 * In my case, I'm using STM32F103RB (NUCLEO-F103RB board),
 * look at datasheet-production data of STM32F103x8/B (DocID 13587, Rev 17),
 * the 'Clock Tree' show on 'Figure 2' at page-12.
 * 
 * I'm using Timer3 (TIM3) for PWM, the TIM3 clock is from APB1,
 * and setup system clock = 48MHz by 'rcc_clock_setup_in_hsi_out_48mhz()' function,
 * it will also set the APB1 prescaler = 2, so APB1 clock is 48MHz / 2 = 24MHz.
 * 
 * But if APB1 prescaler not equal 1, the TIM3 clock given by APB1 will multiply 2.
 * So, the TIM3 clock = APB1 clock * 2
 *                    = 24MHz * 2
 *                    = 48MHz
 */
#define PWM_TIMER_PRESCALER (48 - 1)
#define PWM_TIMER_PERIOD (((rcc_apb1_frequency * 2) / ((PWM_TIMER_PRESCALER + 1) * PWM_FREQUENCY)) - 1)

void gpio_setup(void)
{
  /* Timer3-Channel2 on PA7 (NUCLEO-F103RB). */
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_set_mode(GPIOA,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO7);
}

/**
 * @brief Setup PWM on Timer3-Channel2.
 */
void pwm_setup(void)
{
  rcc_periph_clock_enable(RCC_TIM3);

  timer_set_mode(TIM3,
                 TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE,
                 TIM_CR1_DIR_UP);
  timer_disable_preload(TIM3);
  timer_continuous_mode(TIM3);

  timer_set_prescaler(TIM3, PWM_TIMER_PRESCALER);
  timer_set_period(TIM3, PWM_TIMER_PERIOD);

  timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
  timer_set_oc_value(TIM3,
                     TIM_OC2,
                     PWM_TIMER_PERIOD * (PWM_DUTY_CYCLE / 100.0));

  timer_enable_oc_output(TIM3, TIM_OC2);
  timer_enable_counter(TIM3);
}

int main(void)
{
  /* Setup system clock = 48MHz. */
  rcc_clock_setup_in_hsi_out_48mhz();

  gpio_setup();
  pwm_setup();

  /* Halt. */
  while (1)
  {
    __asm__("nop");
  }

  return 0;
}
