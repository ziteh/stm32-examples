# STM32 Examples
This repo contains some basic examples for STM32.

# List
### LibOpenCM3
[LibOpenCM3](https://github.com/libopencm3/libopencm3) is an open source ARM Cortex-M microcontroller library.

| examples \ boards                                                |   Nucleo-F103RB    |   Nucleo-F446RE    |
| :--------------------------------------------------------------- | :----------------: | :----------------: |
| [Blinking LED](./libopencm3/blink/)                              | :heavy_check_mark: | :heavy_check_mark: |
| [GPIO input](./libopencm3/gpio_input/)                           | :heavy_check_mark: | :heavy_check_mark: |
| [EXTI button](./libopencm3/exti_button/)                         | :heavy_check_mark: | :heavy_check_mark: |
| [USART with printf()](./libopencm3/usart_printf/)                | :heavy_check_mark: | :heavy_check_mark: |
| [USART receive interrupt](./libopencm3/usart_receive_interrupt/) | :heavy_check_mark: | :heavy_check_mark: |
| [Timer](./libopencm3/timer/)                                     | :heavy_check_mark: | :heavy_check_mark: |
| [SysTick](./libopencm3/systick/)                                 | :heavy_check_mark: | :heavy_check_mark: |
| [PWM](./libopencm3/pwm/)                                         | :heavy_check_mark: | :heavy_check_mark: |
| [IWDG](./libopencm3/iwdg/)                                       | :heavy_check_mark: | :heavy_check_mark: |
| [ADC (Multi channel)](./libopencm3/adc_multi_channel/)           | :heavy_check_mark: | :heavy_check_mark: |
| [SPI (Master mode)](./libopencm3/spi_master/)                    | :heavy_check_mark: | :heavy_check_mark: |
| [SPI (Slave mode)](./libopencm3/spi_slave/)                      | :heavy_check_mark: | :heavy_check_mark: |
| [I2C (LCD 1602)](./libopencm3/i2c_lcd1602/)                      | :heavy_check_mark: | :heavy_check_mark: |

## Usage
1. Install [VSCode](https://code.visualstudio.com) and [PlatformIO IDE for VSCode](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide).
2. Clone this repo (some examples contain submodules):
```git
git clone --recurse-submodules https://github.com/ziteh/stm32-examples.git
```
or
```git
git clone https://github.com/ziteh/stm32-examples.git
git submodule update --init --recursive
```
3. Open example project folder in VSCode, e.g. [blink example](./libopencm3/blink/) .
4. Selecte environment (STM32 board). [Ref](https://docs.platformio.org/en/stable/integration/ide/vscode.html#task-explorer)
5. Build project and upload to STM32 board. [Ref](https://docs.platformio.org/en/stable/integration/ide/vscode.html)

## References
- [libopencm3/libopencm3-examples](https://github.com/libopencm3/libopencm3-examples)
- [platformio/platform-ststm32](https://github.com/platformio/platform-ststm32)
- [STM32 Nucleo-64 board user manual (UM1724)](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf)
- [STM32F446RE datasheet (DS10693)](https://www.st.com/resource/en/datasheet/stm32f446re.pdf)
- [STM32F103RB datasheet (DS5319)](https://www.st.com/resource/en/datasheet/stm32f103rb.pdf)