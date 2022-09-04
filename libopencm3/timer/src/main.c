/**
 * @file   main.c
 * @brief  Timer example for LibOpenCM3 with STM32.
 * @author ZiTe (honmonoh@gmail.com)
 * @copyright MIT License
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#define GOAL_FREQUENCY (5) /* f_goal, goal frequency in Hz. */

/**
 * @brief f_timer.
 * @note If APBx_Presc = /1 than f_timer = APBx_Clock, else f_timer = 2* APBx_Clock.
 */
#define TIMER_CLOCK (rcc_apb1_frequency * 2)

/**
 * @brief f_counter (CK_CNT).
 */
#define COUNTER_CLOCK (1000000)

/**
 * @brief PSC (Prescaler), the value of TIMx_PSC register.
 * @note f_counter = f_timer / (PSC + 1)
 *       so,
 *       PSC = f_timer / f_counter - 1
 */
#define TIMER_PRESCALER (TIMER_CLOCK / COUNTER_CLOCK - 1)

/**
 * @brief ARR (Auto-Reload), the value of TIMx_ARR register.
 * @note f_goal = f_timer / [(PSC + 1) * (ARR + 1)]
 *       so,
 *       ARR = {f_timer / [(PSC + 1) * f_goal]} - 1
 */
#define TIMER_PERIOD (((TIMER_CLOCK) / ((TIMER_PRESCALER + 1) * GOAL_FREQUENCY)) - 1)

#if defined(NUCLEO_F103RB)
  #define RCC_LED_GPIO (RCC_GPIOA)
  #define GPIO_LED_PORT (GPIOA)
  #define GPIO_LED_PIN (GPIO5) /* D13. */
#elif defined(NUCLEO_F446RE)
  #define RCC_LED_GPIO (RCC_GPIOA)
  #define GPIO_LED_PORT (GPIOA)
  #define GPIO_LED_PIN (GPIO5) /* D13. */
#else
  #error "STM32 board not defined."
#endif

static void rcc_setup(void)
{
  /* Setup system clock. */
#if defined(STM32F1)
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
#elif defined(STM32F4)
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
#endif

  rcc_periph_clock_enable(RCC_LED_GPIO);
  rcc_periph_clock_enable(RCC_TIM2);
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
  gpio_mode_setup(GPIO_LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_LED_PIN);
  gpio_set_output_options(GPIO_LED_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_LED_PIN);
#endif
}

static void timer_setup(void)
{
  /* Setup interrupt. */
  nvic_enable_irq(NVIC_TIM2_IRQ);
  timer_enable_irq(TIM2, TIM_DIER_CC1IE);

  timer_set_mode(TIM2,
                 TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE,
                 TIM_CR1_DIR_UP);
  timer_disable_preload(TIM2);
  timer_continuous_mode(TIM2);

  timer_set_prescaler(TIM2, TIMER_PRESCALER); /* Setup TIMx_PSC register. */
  timer_set_period(TIM2, TIMER_PERIOD);       /* Setup TIMx_ARR register. */

  timer_enable_counter(TIM2);
}

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
