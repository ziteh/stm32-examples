/// @file ring_buf.h
/// @brief Ring buffer from Linux kernel kfifo v2.6.24
/// @note https://github.com/torvalds/linux/blob/49914084e797530d9baaf51df9eda77babc98fa8/kernel/kfifo.c
/// @author ZiTe <honmonoh@gmail.com>
/// @copyright SPDX-License-Identifier: MIT

#ifndef RING_BUF_H
#define RING_BUF_H

#include <stdint.h>

/// @brief FIFO ring buffer. The `size` MUST be a power of 2.
typedef struct {
    uint8_t *const buffer;
    const uint16_t size;
    uint16_t in;
    uint16_t out;
} ring_buf_t;

uint16_t ring_put(ring_buf_t *ring, const uint8_t *data, uint16_t len);
uint16_t ring_get(ring_buf_t *ring, uint8_t *data, uint16_t len);
uint8_t ring_get_1byte(ring_buf_t *ring);
uint8_t ring_is_empty(const ring_buf_t *ring);

#endif
