/**
 * @file user_main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @copyright MIT
 */

#include "user_main.h"
#include "user_print.h"
#include <stdio.h>
#include <string.h>

#define TX_INTERRUPT // Comment out this line for blocking send.

#define TX_BUF_SIZE (100U)

uint8_t tx_buffer[TX_BUF_SIZE] = {0};
uint16_t tx_in_index = 0;
uint16_t tx_out_index = 0;

uint16_t count = 0;

void User_Init(void)
{
  printf("Start\r\n");
}

void User_Loop(void)
{
  printf("Hello World! %d\r\n", count++);
  LL_mDelay(250);
}

void User_UART_ISR(void)
{
  if (LL_USART_IsActiveFlag_TXE_TXFNF(MY_UART) && tx_in_index > 0)
  {
    LL_USART_TransmitData8(MY_UART, tx_buffer[tx_out_index]); // Send 1 byte from buffer.
    tx_out_index++;

    if (tx_in_index == tx_out_index) // All data has been sended.
    {
      LL_USART_DisableIT_TXE_TXFNF(MY_UART);
      tx_in_index = 0;
      tx_out_index = 0;
      // memset(tx_buffer, 0, TX_BUF_SIZE); // Optional buffer clear.
    }
  }
}

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
