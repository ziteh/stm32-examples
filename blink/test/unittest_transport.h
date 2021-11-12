/**
 * @file   unittest_transport.h
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  The custom unit test transport head file for PlatformIO.
 * @remark Reference: https://docs.platformio.org/en/latest/tutorials/ststm32/stm32cube_debugging_unit_testing.html#writing-unit-tests
 */

#ifndef UNITTEST_TRANSPORT_H_
#define UNITTEST_TRANSPORT_H_

#ifdef __cplusplus
extern "C"
{
#endif

  void unittest_uart_begin(void);
  void unittest_uart_putchar(char c);
  void unittest_uart_flush(void);
  void unittest_uart_end(void);

#ifdef __cplusplus
}
#endif

#endif /* UNITTEST_TRANSPORT_H_ */