#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t *buffer;
    size_t   elem_size;
    size_t   capacity;     // MUST be a power of 2
    size_t   mask;         // capacity - 1
    volatile size_t head;  // Written by producer
    volatile size_t tail;  // Written by consumer
} ring_buffer_t;

/**
 * @brief Initialize ring buffer. Capacity must be a power of 2 (e.g. 16, 32, 64, 128).
 */
bool ring_buffer_init(ring_buffer_t *rb, void *storage, size_t elem_size, size_t capacity);

bool ring_buffer_push(ring_buffer_t *rb, const void *elem);
bool ring_buffer_pop(ring_buffer_t *rb, void *elem);
bool ring_buffer_is_empty(const ring_buffer_t *rb);
bool ring_buffer_is_full(const ring_buffer_t *rb);
size_t ring_buffer_count(const ring_buffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* RING_BUFFER_H */
