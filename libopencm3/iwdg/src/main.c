/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  IWDG (Independent watchdog) example for STM32 Nucleo-F103RB and F446RE.
 */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/iwdg.h>

#ifdef NUCLEO_F103RB
#define RCC_LED_GPIO (RCC_GPIOA)
#define GPIO_LED_PORT (GPIOA)
#define GPIO_LED_PIN (GPIO5)
#elif NUCLEO_F446RE
#define RCC_LED_GPIO (RCC_GPIOA)
#define GPIO_LED_PORT (GPIOA)
#define GPIO_LED_PIN (GPIO5)
#else
#error
#endif

void rcc_setup(void);
void led_setup(void);
void iwdg_setup(void);
void delay(uint32_t value);

int main(void)
{
  rcc_setup();
  led_setup();

  gpio_set(GPIO_LED_PORT, GPIO_LED_PIN);
  delay(20000000);
  gpio_clear(GPIO_LED_PORT, GPIO_LED_PIN);
  delay(2000000);
  gpio_set(GPIO_LED_PORT, GPIO_LED_PIN);
  delay(20000000);

  iwdg_setup(); /* Setup and start IWDG. */

  while (1)
  {
    gpio_toggle(GPIO_LED_PORT, GPIO_LED_PIN); /* LED on/off. */
    delay(2000000);

    iwdg_reset();
  }

  return 0;
}

void rcc_setup(void)
{
#ifdef NUCLEO_F103RB
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
#elif NUCLEO_F446RE
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
#endif

  rcc_periph_clock_enable(RCC_LED_GPIO);
}

void iwdg_setup(void)
{
  iwdg_reset();
  iwdg_set_period_ms(5000);
  iwdg_start();
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

void delay(uint32_t value)
{
  while (value--)
  {
    __asm__("nop"); /* Do nothing. */
  }
}