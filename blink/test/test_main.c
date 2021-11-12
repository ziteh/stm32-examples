/**
 * @file   test_main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  The unit test cases for PlatformIO.
 * @remark Reference: https://docs.platformio.org/en/latest/tutorials/ststm32/stm32cube_debugging_unit_testing.html#writing-unit-tests
 */

#include <unity.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

void delay(int value)
{
  while (value--)
  {
    __asm__("nop");
  }
}

void test_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOA);

  /* User-LED (PA5). */
  gpio_set_mode(GPIOA,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO5);
}

void test_led_output_high(void)
{
  gpio_set(GPIOA, GPIO5);
  TEST_ASSERT_NOT_EQUAL(0, gpio_get(GPIOA, GPIO5));
}

void test_led_output_low(void)
{
  gpio_clear(GPIOA, GPIO5);
  TEST_ASSERT_EQUAL(0, gpio_get(GPIOA, GPIO5));
}

int main(void)
{
  test_setup();
  delay(100000);

  UNITY_BEGIN();

  RUN_TEST(test_led_output_high);
  delay(50000);
  RUN_TEST(test_led_output_low);
  delay(50000);

  UNITY_END();

  /* Halt. */
  while (1)
  {
    __asm__("nop");
  }

  return 0;
}
