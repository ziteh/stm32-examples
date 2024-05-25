/// @file ring_buf.c
/// @brief Ring buffer from Linux kernel kfifo v2.6.24
/// @note https://github.com/torvalds/linux/blob/49914084e797530d9baaf51df9eda77babc98fa8/kernel/kfifo.c
/// @author ZiTe <honmonoh@gmail.com>
/// @copyright SPDX-License-Identifier: MIT

#include "ring_buf.h"
#include "user_main.h"
#include <string.h>

// #define ENABLE_LOCK

#define MIN(a, b) ((a) < (b) ? (a) : (b))

// #define MIN(a, b) ({  \
//   typeof(a) _a = (a); \
//   typeof(b) _b = (b); \
//   _a < _b ? _a : _b;  \
// })

uint16_t ring_put(ring_buf_t *ring, const uint8_t *data, uint16_t len) {
#ifdef ENABLE_LOCK
    uint8_t lock = 0;
    if (LL_USART_IsEnabledIT_TXE_TXFNF(MY_UART)) {
        LL_USART_DisableIT_TXE_TXFNF(MY_UART);
        lock = 1;
    }
#endif

    len = MIN(len, ring->size - ring->in + ring->out);
    uint16_t mir_len = MIN(len, ring->size - (ring->in & (ring->size - 1)));
    memcpy(ring->buffer + (ring->in & (ring->size - 1)), data, mir_len);
    memcpy(ring->buffer, data + mir_len, len - mir_len);
    ring->in += len;

#ifdef ENABLE_LOCK
    if (lock) {
        LL_USART_EnableIT_TXE_TXFNF(MY_UART);
    }
#endif
    return len;
}

uint16_t ring_get(ring_buf_t *ring, uint8_t *data, uint16_t len) {
#ifdef ENABLE_LOCK
    uint8_t lock = 0;
    if (LL_USART_IsEnabledIT_TXE_TXFNF(MY_UART)) {
        LL_USART_DisableIT_TXE_TXFNF(MY_UART);
        lock = 1;
    }
#endif

    len = MIN(len, ring->in - ring->out);
    uint16_t mir_len = MIN(len, ring->size - (ring->out & (ring->size - 1)));
    memcpy(data, ring->buffer + (ring->out & (ring->size - 1)), mir_len);
    memcpy(data + mir_len, ring->buffer, len - mir_len);
    ring->out += len;

#ifdef ENABLE_LOCK
    if (lock) {
        LL_USART_EnableIT_TXE_TXFNF(MY_UART);
    }
#endif
    return len;
}

uint8_t ring_get_1byte(ring_buf_t *ring) {
#ifdef ENABLE_LOCK
    uint8_t lock = 0;
    if (LL_USART_IsEnabledIT_TXE_TXFNF(MY_UART)) {
        LL_USART_DisableIT_TXE_TXFNF(MY_UART);
        lock = 1;
    }
#endif

    uint8_t data = *(ring->buffer + (ring->out & (ring->size - 1)));
    ring->out += 1;

#ifdef ENABLE_LOCK
    if (lock) {
        LL_USART_EnableIT_TXE_TXFNF(MY_UART);
    }
#endif
    return data;
}

uint8_t ring_is_empty(const ring_buf_t *ring) {
    return (ring->in == ring->out);
}
