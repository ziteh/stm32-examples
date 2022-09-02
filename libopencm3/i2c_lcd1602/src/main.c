/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  I2C example with LCD1602(PCF8574T) for STM32 Nucleo-F103RB and F446RE.
 */

#include "main.h"

int main(void)
{
  rcc_setup();
  i2c_setup();

  PCF8574T_Init(0x27, lcd_delay, lcd_i2c_write);
  PCF8574T_displayString("                ", 1); /* Clear. */
  PCF8574T_displayString("                ", 2); /* Clear. */

  PCF8574T_displayString("  Hello World!  ", 1);
  while (1)
  {
    PCF8574T_displayString("This is NUCLEO, ", 2);
    delay(DELAY_VALUE);
    PCF8574T_displayString("a STM32 board.  ", 2);
    delay(DELAY_VALUE);
  }

  return 0;
}

void delay(uint32_t value)
{
  while (value--)
  {
    __asm__("nop"); /* Do nothing. */
  }
}

void rcc_setup(void)
{
#ifdef NUCLEO_F103RB
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
  rcc_periph_clock_enable(RCC_AFIO);
#elif NUCLEO_F446RE
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
#endif

  rcc_periph_clock_enable(RCC_I2C_GPIO);
  rcc_periph_clock_enable(RCC_I2C1);
}

void i2c_setup(void)
{
  /* Set to alternate function. */
#ifdef NUCLEO_F103RB
  gpio_set_mode(GPIO_I2C_PORT,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
                GPIO_I2C_SCL_PIN | GPIO_I2C_SDA_PIN);

  /*
   * Alternate function remap is required for
   * using I2C1_SCL & SDA on PB8 & PB9.
   * Ref: Table-5 in DS5319.
   */
  gpio_primary_remap(AFIO_MAPR_SWJ_CFG_FULL_SWJ,
                     AFIO_MAPR_I2C1_REMAP);
#elif NUCLEO_F446RE
  gpio_mode_setup(GPIO_I2C_PORT,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO_I2C_SCL_PIN | GPIO_I2C_SDA_PIN);

  gpio_set_output_options(GPIO_I2C_PORT,
                          GPIO_OTYPE_OD,
                          GPIO_OSPEED_50MHZ,
                          GPIO_I2C_SCL_PIN | GPIO_I2C_SDA_PIN);

  gpio_set_af(GPIO_I2C_PORT,
              GPIO_AF4, /* Ref: Table-11 in DS10693. */
              GPIO_I2C_SCL_PIN | GPIO_I2C_SDA_PIN);
#endif

  i2c_peripheral_disable(I2C1);
  i2c_reset(I2C1);

  /* Ref: https://github.com/brabo/stm32f4-i2c-scan/blob/master/i2c.c. */
  i2c_set_fast_mode(I2C1);
  i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_42MHZ);
  i2c_set_ccr(I2C1, 35);
  i2c_set_trise(I2C1, 43);
  i2c_set_own_7bit_slave_address(I2C1, 0x00);

  i2c_peripheral_enable(I2C1);
}

/*
 * Ref: https://github.com/brabo/stm32f4-i2c-scan
 */
void lcd_i2c_write(uint8_t address, uint8_t *data, uint8_t data_length)
{
  i2c_send_start(I2C1);

  while (!((I2C_SR1(I2C1) & I2C_SR1_SB) & (I2C_SR2(I2C1) & (I2C_SR2_MSL | I2C_SR2_BUSY))))
  {
  }

  i2c_send_7bit_address(I2C1, address, I2C_WRITE);

  while (!(I2C_SR1(I2C1) & I2C_SR1_ADDR))
  {
  }

  uint32_t u = I2C_SR2(I2C1);
  (void)u;

  for (int i = 0; i < data_length; i++)
  {
    i2c_send_data(I2C1, data[i]);
    while (!(I2C_SR1(I2C1) & (I2C_SR1_BTF)))
      ;
  }

  i2c_send_stop(I2C1);
}

void lcd_delay(uint16_t ms)
{
  delay(ms * DELAY_CONST);
}
