/// @file user_main.c
/// @author ZiTe <honmonoh@gmail.com>
/// @copyright SPDX-License-Identifier: MIT

#include "user_main.h"
#include "ring_buf.h"
#include <stdio.h>

#define TX_BUF_SIZE (64U) // MUST be a power of 2
uint8_t tx_buf[TX_BUF_SIZE] = {0};

ring_buf_t tx_ring = {
    .buffer = tx_buf,
    .size = TX_BUF_SIZE,
    .in = 0,
    .out = 0,
};

void user_init(void) {
    printf("Start\r\n");
}

void user_loop(void) {
    static uint16_t count = 0;

    printf("Hello World! %d\r\n", count++);
    LL_mDelay(250);
}

void user_uart_isr(void) {
    if (LL_USART_IsActiveFlag_TXE_TXFNF(MY_UART)) {
        uint8_t data = ring_get_1byte(&tx_ring);
        LL_USART_TransmitData8(MY_UART, data); // Send 1 byte from buffer

        if (ring_is_empty(&tx_ring)) {
            LL_USART_DisableIT_TXE_TXFNF(MY_UART); // All data has been sent
        }
    }
}

int _write(int file, char *ptr, int len) {
    (void)file; // Stream. 1 for stdout, 2 for stderr

    int w_len = ring_put(&tx_ring, (uint8_t *)ptr, len);

    if (!LL_USART_IsEnabledIT_TXE_TXFNF(MY_UART)) {
        LL_USART_EnableIT_TXE_TXFNF(MY_UART);
    }
    return w_len;
}
