/**
 * @file user_main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @copyright MIT
 */

#include "user_main.h"
#include "../Printf/src/printf/printf.h"
//#include <stdio.h>
#include <string.h>

#define TX_BUF_SIZE (100U)

uint8_t tx_buffer[TX_BUF_SIZE] = {0};
uint16_t tx_in_index = 0;  // Tx buffer input index.
uint16_t tx_out_index = 0; // Tx buffer output index.

void User_Init(void)
{
  printf_("Start\r\n");
}

void User_Loop(void)
{
  static uint16_t count = 0;
  printf_("Hello World! %d\r\n", count++);
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

void putchar_(char c)
{
  if (tx_in_index >= TX_BUF_SIZE - 1)
  {
    return;
  }

  tx_buffer[tx_in_index] = (uint8_t)c;
  tx_in_index++;

  if (!LL_USART_IsEnabledIT_TXE_TXFNF(MY_UART))
  {
    LL_USART_EnableIT_TXE_TXFNF(MY_UART);
  }
}
