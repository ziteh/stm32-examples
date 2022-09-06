/**
 * @file   main.c
 * @brief  IWDG (Independent watchdog) example for STM32 based on LibOpenCM3.
 * @author ZiTe (honmonoh@gmail.com)
 * @copyright MIT License
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

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

static volatile uint32_t systick_delay = 0;

static void delay_ms(uint32_t value)
{
  systick_delay = value;
  while (systick_delay != 0)
  {
    /* Wait. */
  }
}

static void systick_setup(void)
{
  /*
   * SysTick interrupt every N clock pulses, set reload to N-1.
   * N = AHB_clock / 8(DIV) / 1000(1000 overflows per second).
   */
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
  systick_set_reload(rcc_ahb_frequency / 8 / 1000 - 1);

  systick_interrupt_enable();
  systick_counter_enable();
}

static void rcc_setup(void)
{
#if defined(STM32F1)
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
#elif defined(STM32F4)
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
#endif

  rcc_periph_clock_enable(RCC_LED_GPIO);
}

static void iwdg_setup(void)
{
  iwdg_reset();
  iwdg_set_period_ms(300);
  iwdg_start();
}

static void led_setup(void)
{
  /* Set LED pin to output push-pull. */
#if defined(STM32F1)
  gpio_set_mode(GPIO_LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO_LED_PIN);
#else
  gpio_mode_setup(GPIO_LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_LED_PIN);
  gpio_set_output_options(GPIO_LED_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_LED_PIN);
#endif
}

int main(void)
{
  rcc_setup();
  systick_setup();
  led_setup();

  gpio_clear(GPIO_LED_PORT, GPIO_LED_PIN);
  delay_ms(10);
  gpio_set(GPIO_LED_PORT, GPIO_LED_PIN);
  delay_ms(2000);

  iwdg_setup(); /* Setup and start IWDG. */

  while (1)
  {
    gpio_toggle(GPIO_LED_PORT, GPIO_LED_PIN); /* LED on/off. */
    delay_ms(200);
    iwdg_reset(); /* Refresh. */
  }

  return 0;
}

/**
 * @brief  SysTick handler.
 */
void sys_tick_handler(void)
{
  if (systick_delay != 0)
  {
    systick_delay--;
  }
}