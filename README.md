# STM32 Examples

This repo contains some basic examples for STM32.

## LibOpenCM3

[LibOpenCM3](https://github.com/libopencm3/libopencm3) is an open source ARM Cortex-M microcontroller library.

My blog posts: [簡單入門 LibOpenCM3 STM32 嵌入式系統開發](https://ziteh.github.io/series/%E7%B0%A1%E5%96%AE%E5%85%A5%E9%96%80-libopencm3-stm32-%E5%B5%8C%E5%85%A5%E5%BC%8F%E7%B3%BB%E7%B5%B1%E9%96%8B%E7%99%BC/)

| Examples                                                                    | F103RB | F446RE | F401RE | F302R8 | L432KC | G431KB |
| :-------------------------------------------------------------------------- | :----: | :----: | :----: | :----: | :----: | :----: |
| [Blinking LED](./libopencm3/blink/)                                         |   ✔️    |   ✔️    |   ✔️    |   ✔️    |   ✔️    |   ✔️    |
| [GPIO input](./libopencm3/gpio_input/)                                      |   ✔️    |   ✔️    |   ✔️    |   ✔️    |   ✔️    |        |
| [EXTI button](./libopencm3/exti_button/)                                    |   ✔️    |   ✔️    |   ✔️    |   ✔️    |   ✔️    |        |
| [USART with printf()](./libopencm3/usart_printf/)                           |   ✔️    |   ✔️    |        |   ✔️    |   ✔️    |        |
| [USART receive interrupt](./libopencm3/usart_receive_interrupt/)            |   ✔️    |   ✔️    |        |        |        |        |
| [Timer](./libopencm3/timer/)                                                |   ✔️    |   ✔️    |        |        |        |        |
| [SysTick delay](./libopencm3/systick/)                                      |   ✔️    |   ✔️    |        |        |        |        |
| [PWM](./libopencm3/pwm/)                                                    |   ✔️    |   ✔️    |        |        |        |        |
| [IWDG](./libopencm3/iwdg/)                                                  |   ✔️    |   ✔️    |        |        |        |        |
| [WWDG](./libopencm3/wwdg/)                                                  |   ✔️    |   ✔️    |        |        |        |        |
| [ADC (Regular single channel)](./libopencm3/adc_single_channel_regular/)    |   ✔️    |   ✔️    |        |        |        |        |
| [ADC (Injected multi channel)](./libopencm3/adc_multi_channel_injected/)    |   ❌    |   ✔️    |        |        |        |        |
| [ADC (Interrupt)](./libopencm3/adc_interrupt/)                              |   ✔️    |   ✔️    |        |        |        |        |
| [ADC (External trigger by timer)](./libopencm3/adc_external_trigger_timer/) |   ✔️    |   ✔️    |        |        |        |        |
| [SPI (Master mode)](./libopencm3/spi_master/)                               |   ✔️    |   ✔️    |        |        |        |        |
| [SPI (Slave mode)](./libopencm3/spi_slave/)                                 |   ✔️    |   ✔️    |        |        |        |        |
| [I2C (LCD 1602)](./libopencm3/i2c_lcd1602/)                                 |   ✔️    |   ✔️    |        |        |        |        |
| [I2C (EEPROM 24C256)](./libopencm3/i2c_eeprom_24c256/)                      |   ✔️    |   ✔️    |        |        |        |        |
| [CRC](./libopencm3/crc/)                                                    |   ✔️    |   ✔️    |        |        |        |        |

## STM32 LL

STM32 Low Layer.

| Examples                                                        | G431KB |
| :-------------------------------------------------------------- | :----: |
| [ADC (Injected TRGO)](./stm32_ll/adc_inj_trgo/)                 |   ✔️    |
| [UART Printf](./stm32_ll/uart_printf/)                          |   ✔️    |
| [UART Printf (Ring buffer/kfifo)](./stm32_ll/uart_printf_ring/) |   ✔️    |
| [UART Printf (tiny)](./stm32_ll/uart_printf_tiny/)              |   ✔️    |

> Check the code under the `User` folder in each project.

## STM32 HAL

STM32 Hardware Abstraction Layer.

| Examples                           | F103RB | F446RE | G431KB |
| :--------------------------------- | :----: | :----: | :----: |
| [Blinking LED](./stm32_hal/blink/) |        |        |   ✔️    |

## Usage

Clone the repo:

```bash
git clone --recursive https://github.com/ziteh/stm32-examples.git
```

or

```bash
git clone https://github.com/ziteh/stm32-examples.git
git submodule update --init --recursive
```

### LibOpenCM3 & STM32 HAL

1. Install [VSCode](https://code.visualstudio.com) and [PlatformIO IDE](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide).
2. Open example project folder (e.g. [blink example](./libopencm3/blink/)) in VSCode.
3. Selecte environment (STM32 board). [Ref](https://docs.platformio.org/en/stable/integration/ide/vscode.html#task-explorer)
4. Build project and upload to STM32 board. [Ref](https://docs.platformio.org/en/stable/integration/ide/vscode.html)

### STM32 LL

Download STM32CubeIDE and open the project.

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
- [STM32L432KC](https://www.st.com/en/microcontrollers-microprocessors/stm32l432kc.html)
    - Datasheet: [DS11451](https://www.st.com/resource/en/datasheet/stm32l432kc.pdf)
    - Reference Manual: [RM0394](https://www.st.com/resource/en/reference_manual/rm0394-stm32l41xxx42xxx43xxx44xxx45xxx46xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [STM32G431KB](https://www.st.com/en/microcontrollers-microprocessors/stm32g431kb.html)
    - Datasheet: [DS12589](https://www.st.com/resource/en/datasheet/stm32g431kb.pdf)
    - Reference Manual: [RM0440](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- STM32 Nucleo board user manual
    - Nucleo-64 MB1136: [UM1724](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf)
    - Nucleo-32 MB1180: [UM1956](https://www.st.com/resource/en/user_manual/um1956-stm32-nucleo32-boards-mb1180-stmicroelectronics.pdf)
    - Nucleo-32 MB1430: [UM2397](https://www.st.com/resource/en/user_manual/um2397-stm32g4-nucleo32-board-mb1430-stmicroelectronics.pdf)

## License

My code is licensed under the [MIT License](./LICENSE). Other code is licensed under their own licenses (e.g., CMSIS under Apache 2.0, STM32 HAL/LL under BSD-3-Clause), which can be identified at each level of the folder or at the beginning of each file.
