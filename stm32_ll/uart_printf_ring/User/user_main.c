/**
 * @file user_main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @copyright MIT
 */

#include "user_main.h"
#include <stdio.h>
#include <string.h>

#define MIN(a, b) (a) < (b) ? (a) : (b)

// #define MIN(a, b) ({  \
//   typeof(a) _a = (a); \
//   typeof(b) _b = (b); \
//   _a < _b ? _a : _b;  \
// })

#define TX_BUF_SIZE (256U)

/**
 * @brief Linux kfifo v2.6
 */
typedef struct
{
  uint8_t *buffer;
  const uint16_t size;
  uint16_t in;
  uint16_t out;
} ring_buf;

uint8_t tx_buf[TX_BUF_SIZE] = {0};

ring_buf tx_ring = {
    .buffer = tx_buf,
    .size = TX_BUF_SIZE,
    .in = 0,
    .out = 0,
};

uint16_t ring_put(ring_buf *ring, const uint8_t *data, uint16_t len)
{
  // uint8_t lock = 0;
  // if (LL_USART_IsEnabledIT_TXE_TXFNF(MY_UART))
  // {
  //   LL_USART_DisableIT_TXE_TXFNF(MY_UART);
  //   lock = 1;
  // }

  len = MIN(len, ring->size - ring->in + ring->out);

  uint16_t l = MIN(len, ring->size - (ring->in & (ring->size - 1)));

  memcpy(ring->buffer + (ring->in & (ring->size - 1)), data, l);

  memcpy(ring->buffer, data + l, len - l);

  ring->in += len;

  // if (lock == 1)
  // {
  //   LL_USART_EnableIT_TXE_TXFNF(MY_UART);
  // }

  return len;
}

uint16_t ring_get(ring_buf *ring, uint8_t *data, uint16_t len)
{
  // uint8_t lock = 0;
  // if (LL_USART_IsEnabledIT_TXE_TXFNF(MY_UART))
  // {
  //   LL_USART_DisableIT_TXE_TXFNF(MY_UART);
  //   lock = 1;
  // }

  len = MIN(len, ring->in - ring->out);

  uint16_t l = MIN(len, ring->size - (ring->out & (ring->size - 1)));

  memcpy(data, ring->buffer + (ring->out & (ring->size - 1)), l);

  memcpy(data + l, ring->buffer, len - l);

  ring->out += len;

  // if (lock == 1)
  // {
  //   LL_USART_EnableIT_TXE_TXFNF(MY_UART);
  // }
  return len;
}

int _write(int file, char *ptr, int len)
{
  (void)file; // Stream.

  int l = ring_put(&tx_ring, (uint8_t *)ptr, len);

  if (!LL_USART_IsEnabledIT_TXE_TXFNF(MY_UART))
  {
    LL_USART_EnableIT_TXE_TXFNF(MY_UART);
  }
  return l;
}

void User_Init(void)
{
  printf("Start\r\n");
}

void User_Loop(void)
{
  static uint16_t count = 0;

  if (count % 15 == 0)
  {
    __NOP();
  }

  printf("Hello World! %d\r\n", count++);
  LL_mDelay(250);
}

void User_UART_ISR(void)
{
  if (LL_USART_IsActiveFlag_TXE_TXFNF(MY_UART))
  {
    uint8_t data[1];
    ring_get(&tx_ring, data, 1);
    LL_USART_TransmitData8(MY_UART, data[0]); // Send 1 byte from buffer.

    if (tx_ring.in == tx_ring.out) // All data has been sent.
    {
      LL_USART_DisableIT_TXE_TXFNF(MY_UART);
    }
  }
}
