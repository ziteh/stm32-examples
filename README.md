# STM32 Examples
This repo contains some basic examples for STM32.

## List
### LibOpenCM3
[LibOpenCM3](https://github.com/libopencm3/libopencm3) is an open source ARM Cortex-M microcontroller library.

| examples \ boards                                                |   Nucleo-F103RB    |   Nucleo-F446RE    |   Nucleo-F401RE    |   Nucleo-F302R8    |   Nucleo-G431KB    |
| :--------------------------------------------------------------- | :----------------: | :----------------: | :----------------: | :----------------: | :----------------: |
| [Blinking LED](./libopencm3/blink/)                              | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |
| [GPIO input](./libopencm3/gpio_input/)                           | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |                    |
| [EXTI button](./libopencm3/exti_button/)                         | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |                    |
| [USART with printf()](./libopencm3/usart_printf/)                | :heavy_check_mark: | :heavy_check_mark: |                    | :heavy_check_mark: |                    |
| [USART receive interrupt](./libopencm3/usart_receive_interrupt/) | :heavy_check_mark: | :heavy_check_mark: |                    |                    |                    |
| [Timer](./libopencm3/timer/)                                     | :heavy_check_mark: | :heavy_check_mark: |                    |                    |                    |
| [SysTick delay](./libopencm3/systick/)                           | :heavy_check_mark: | :heavy_check_mark: |                    |                    |                    |
| [PWM](./libopencm3/pwm/)                                         | :heavy_check_mark: | :heavy_check_mark: |                    |                    |                    |
| [IWDG](./libopencm3/iwdg/)                                       | :heavy_check_mark: | :heavy_check_mark: |                    |                    |                    |
| [WWDG](./libopencm3/wwdg/)                                       | :heavy_check_mark: | :heavy_check_mark: |                    |                    |                    |
| [ADC (Multi channel)](./libopencm3/adc_multi_channel/)           | :heavy_check_mark: | :heavy_check_mark: |                    |                    |                    |
| [SPI (Master mode)](./libopencm3/spi_master/)                    | :heavy_check_mark: | :heavy_check_mark: |                    |                    |                    |
| [SPI (Slave mode)](./libopencm3/spi_slave/)                      | :heavy_check_mark: | :heavy_check_mark: |                    |                    |                    |
| [I2C (LCD 1602)](./libopencm3/i2c_lcd1602/)                      | :heavy_check_mark: | :heavy_check_mark: |                    |                    |                    |

### STM32 HAL

| examples \ boards                  | Nucleo-F103RB | Nucleo-F446RE |   Nucleo-G431KB    |
| :--------------------------------- | :-----------: | :-----------: | :----------------: |
| [Blinking LED](./stm32_hal/blink/) |               |               | :heavy_check_mark: |

## Usage
1. Install [VSCode](https://code.visualstudio.com) and [PlatformIO IDE](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide).
2. Clone this repo (some examples contain submodules):
```git
git clone --recurse-submodules https://github.com/ziteh/stm32-examples.git
```
or
```git
git clone https://github.com/ziteh/stm32-examples.git
git submodule update --init --recursive
```
3. Open example project folder (e.g. [blink example](./libopencm3/blink/)) in VSCode.
4. Selecte environment (STM32 board). [Ref](https://docs.platformio.org/en/stable/integration/ide/vscode.html#task-explorer)
5. Build project and upload to STM32 board. [Ref](https://docs.platformio.org/en/stable/integration/ide/vscode.html)

## References
### Code
- [libopencm3/libopencm3-examples](https://github.com/libopencm3/libopencm3-examples)
- [platformio/platform-ststm32](https://github.com/platformio/platform-ststm32)

### STM32 Documentation
- [STM32F103RB](https://www.st.com/en/microcontrollers-microprocessors/stm32f103rb.html)
  - Datasheet: [DS5319](https://www.st.com/resource/en/datasheet/stm32f103rb.pdf)
  - Reference Manual: [RM0008](https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [STM32F446RE](https://www.st.com/en/microcontrollers-microprocessors/stm32f446re.html)
  - Datasheet: [DS10693](https://www.st.com/resource/en/datasheet/stm32f446re.pdf)
  - Reference Manual: [RM0390](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [STM32G431KB](https://www.st.com/en/microcontrollers-microprocessors/stm32g431kb.html)
  - Datasheet: [DS12589](https://www.st.com/resource/en/datasheet/stm32g431kb.pdf)
  - Reference Manual: [RM0440](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- STM32 Nucleo board user manual
  - Nucleo-64 MB1136: [UM1724](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf)
  - Nucleo-32 MB1430: [UM2397](https://www.st.com/resource/en/user_manual/um2397-stm32g4-nucleo32-board-mb1430-stmicroelectronics.pdf)
