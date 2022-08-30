/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  SysTick example for STM32 Nucleo-F103RB and F446RE.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

#ifdef NUCLEO_F103RB
#define RCC_LED_PORT (RCC_GPIOA)
#define GPIO_LED_PORT (GPIOA)
#define GPIO_LED_PIN (GPIO5)
#elif NUCLEO_F446RE
#define RCC_LED_PORT (RCC_GPIOA)
#define GPIO_LED_PORT (GPIOA)
#define GPIO_LED_PIN (GPIO5)
#else
#error
#endif

volatile uint32_t systick_count = 0;

void rcc_setup(void);
void led_setup(void);
void systick_setup(void);
void delay_ms(uint32_t ms);

int main(void)
{
  rcc_setup();
  systick_setup();
  led_setup();

  while (1)
  {
    gpio_toggle(GPIO_LED_PORT, GPIO_LED_PIN); /* LED on/off. */
    delay_ms(500);
  }

  return 0;
}

void delay_ms(uint32_t ms)
{
  uint32_t wake = systick_count + ms;
  while (wake > systick_count)
  {
    __asm__("nop"); /* Do nothing. */
  }
}

void rcc_setup(void)
{
#ifdef NUCLEO_F103RB
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
#elif NUCLEO_F446RE
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
#endif

  rcc_periph_clock_enable(RCC_LED_PORT);
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

void systick_setup(void)
{
  /*
   * Set to every 1ms one interrupt.
   *
   * SysTick interrupt every N clock pulses, set reload to N-1.
   * N = AHB_clock / 8(Prescaler) / 1000(1000 overflows per second).
   */
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
  systick_set_reload(rcc_ahb_frequency / 8 / 1000 - 1);

  systick_counter_enable();
  systick_interrupt_enable();
}

/**
 * @brief  SysTick handler.
 */
void sys_tick_handler(void)
{
  systick_count++;
}
