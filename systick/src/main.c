/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Basic SysTick(System tick timer) example.
 * @remark Reference: https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f1/other/systick/systick.c
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
// #include <libopencm3/cm3/nvic.h>

/* User-LED */
#define RCC_LED_PORT (RCC_GPIOA)
#define LED_PORT (GPIOA)
#define LED_PIN (GPIO5)

uint32_t systick_count = 0;

/**
 * @brief Setup User-LED.
 */
void led_setup(void)
{
  rcc_periph_clock_enable(RCC_LED_PORT);
  gpio_set_mode(LED_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                LED_PIN);
}

void systick_setup(void)
{
  /* 48MHz / 8 = 6MHz -> 6000000 count per second. */
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

  /*
   * 6000000 / 6000 = 1000 overflows per second,
   * every 1ms one interrupt.
   * Systick interrupt every N clock pulses, set reload to N-1.
   */
  systick_set_reload(6000 - 1);

  systick_interrupt_enable();
  systick_counter_enable();
}

/**
 * @brief  SysTick handler.
 * @remark Every 1ms one interrupt.
 */
void sys_tick_handler(void)
{
  systick_count++;

  /* 1000 * 1ms = 1s. */
  if (systick_count == 1000)
  {
    systick_count = 0;
    gpio_toggle(LED_PORT, LED_PIN);
  }
}

int main(void)
{
  /* Set clock to 48MHz. */
  rcc_clock_setup_in_hsi_out_48mhz();

  led_setup();
  systick_setup();

  /* Halt. */
  while (1)
  {
    __asm__("nop");
  }

  return 0;
}