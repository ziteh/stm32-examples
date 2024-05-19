/**
 * @file user_print.h
 * @brief Implement printf() on your device.
 * @note
 * Usage:
 * Define your PUTCHAR_PROTOTYPE function in the user code, like this:
 *
 * PUTCHAR_PROTOTYPE
 * {
 *   UART_SEND_1BYTE((uint8_t)ch);  // Print to UART.
 *   return ch;
 * }
 *
 * Refer to: https://github.com/STMicroelectronics/STM32CubeH7/blob/799e0591d3fd8b80cdf96c73230676672077d653/Projects/STM32H743I-EVAL/Examples/UART/UART_Printf/Src/main.c#L40-L46
 *
 */

#ifndef USER_PRINT_H_
#define USER_PRINT_H_

#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#endif /* USER_PRINT_H_ */
