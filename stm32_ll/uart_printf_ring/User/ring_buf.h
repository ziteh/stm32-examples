/// @file ring_buf.h
/// @brief Ring buffer from Linux kernel kfifo v2.6.24
/// @note https://github.com/torvalds/linux/blob/49914084e797530d9baaf51df9eda77babc98fa8/kernel/kfifo.c
/// @author ZiTe <honmonoh@gmail.com>
/// @copyright SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

/// @brief FIFO ring buffer.
/// @note The `size` must be power of 2.
typedef struct {
    uint8_t *const buffer; // Pointer to the buffer
    const uint16_t size;   // Size of the buffer, must be power of 2
    uint16_t in;           // Input index
    uint16_t out;          // Output index
} ring_buf_t;

/// @brief Puts data data into the ring buffer.
/// @param ring Pointer to the ring buffer.
/// @param data Pointer to the data to put.
/// @param len Length of the data.
/// @return The number actually put into the buffer.
uint16_t ring_put(ring_buf_t *ring, const uint8_t *data, uint16_t len);

/// @brief Gets data from the ring buffer.
/// @param ring Pointer to the ring buffer.
/// @param data Pointer to the buffer to store the data.
/// @param len Max length of the data to get.
/// @return The number actually read from the buffer.
uint16_t ring_get(ring_buf_t *ring, uint8_t *data, uint16_t len);

/// @brief Gets one byte from the ring buffer.
/// @param ring Pointer to the ring buffer.
/// @return The byte read from the buffer.
uint8_t ring_get_1byte(ring_buf_t *ring);

/// @brief Checks if the ring buffer is empty.
/// @param ring Pointer to the ring buffer.
/// @return 1 if the buffer is empty, 0 otherwise.
uint8_t ring_is_empty(const ring_buf_t *ring);
