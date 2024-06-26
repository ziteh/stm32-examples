/**
 * @file user_main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @copyright MIT
 */

#include "user_main.h"
#include "user_print.h"
#include <stdio.h>
#include <string.h>

#define MODE 0

#if (MODE == 0) // Interrupt-driven, use printf(), copy entire buffer at once.
  #define OVERRIDE_WRITE
  #undef SNPRINTF
#elif (MODE == 1) // Interrupt-driven, use snprintf(), copy entire buffer at once.
  #undef OVERRIDE_WRITE
  #define TX_INTERRUPT
  #define SNPRINTF
#elif (MODE == 2) // Interrupt-driven, use printf(), put 1 byte into buffer each time.
  #undef OVERRIDE_WRITE
  #define TX_INTERRUPT
  #undef SNPRINTF
#elif (MODE == 3) // Blocking, use printf(), send 1 byte each time.
  #undef OVERRIDE_WRITE
  #undef TX_INTERRUPT
  #undef SNPRINTF
#else
  #error MODE = 0~3
#endif

#define TX_BUF_SIZE (100U)

uint8_t tx_buffer[TX_BUF_SIZE] = {0};
uint16_t tx_in_index = 0;  // Tx buffer input index.
uint16_t tx_out_index = 0; // Tx buffer output index.

#if defined(TX_INTERRUPT) && defined(SNPRINTF) // Print by snprintf().
  #define PRINTF(format, ...)                                      \
    do                                                             \
    {                                                              \
      uint8_t str[TX_BUF_SIZE] = {0};                              \
      int len = snprintf(str, TX_BUF_SIZE, format, ##__VA_ARGS__); \
      PutBuffer(str, len);                                         \
    } while (0);
#else // Print by printf().
  #define PRINTF(format, ...) printf(format, ##__VA_ARGS__)
#endif

#ifdef TX_INTERRUPT

// Interrupt-driven implementation.
PUTCHAR_PROTOTYPE
{
  if (tx_in_index < TX_BUF_SIZE)
  {
    tx_buffer[tx_in_index] = (uint8_t)ch;
    tx_in_index++;
  }

  if (!LL_USART_IsEnabledIT_TXE_TXFNF(MY_UART))
  {
    LL_USART_EnableIT_TXE_TXFNF(MY_UART);
  }

  return ch;
}

#else

// Blocking implementation.
PUTCHAR_PROTOTYPE
{
  uint16_t timeout = 0xFFFFU;
  while (!LL_USART_IsActiveFlag_TXE(MY_UART)) // Wait for Tx buffer empty.
  {
    if (--timeout == 0) return ch; // Comment out this line for infinite timeout.
  }
  LL_USART_TransmitData8(MY_UART, (uint8_t)ch);
  return ch;
}

#endif

#ifdef OVERRIDE_WRITE

int _write(int file, char *ptr, int len)
{
  (void)file; // Stream.

  if (tx_in_index + len > TX_BUF_SIZE)
  {
    return 0;
  }

  memcpy(tx_buffer + tx_in_index, ptr, len);
  tx_in_index += len;

  if (!LL_USART_IsEnabledIT_TXE_TXFNF(MY_UART))
  {
    LL_USART_EnableIT_TXE_TXFNF(MY_UART);
  }
  return len;
}

#endif

void PutBuffer(const uint8_t *data, uint16_t len);

void User_Init(void)
{
  PRINTF("Start\r\n");
}

void User_Loop(void)
{
  static uint16_t count = 0;
  PRINTF("Hello World! %d\r\n", count++);
  LL_mDelay(250);
}

void User_UART_ISR(void)
{
  if (LL_USART_IsActiveFlag_TXE_TXFNF(MY_UART) && tx_in_index > 0)
  {
    LL_USART_TransmitData8(MY_UART, tx_buffer[tx_out_index]); // Send 1 byte from buffer.
    tx_out_index++;

    if (tx_in_index == tx_out_index) // All data has been sent.
    {
      LL_USART_DisableIT_TXE_TXFNF(MY_UART);
      tx_in_index = 0;
      tx_out_index = 0;
      // memset(tx_buffer, 0, TX_BUF_SIZE); // Optional buffer clear.
    }
  }
}

void PutBuffer(const uint8_t *data, uint16_t len)
{
  if ((tx_in_index + len) <= TX_BUF_SIZE)
  {
    memcpy(tx_buffer + tx_in_index, data, len);
    tx_in_index += len;

    if (!LL_USART_IsEnabledIT_TXE_TXFNF(MY_UART))
    {
      LL_USART_EnableIT_TXE_TXFNF(MY_UART);
    }
  }
}
