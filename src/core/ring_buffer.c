#include "ring_buffer.h"
#include <string.h>

static inline bool is_power_of_two(size_t x) {
    return (x != 0) && ((x & (x - 1)) == 0);
}

bool ring_buffer_init(ring_buffer_t *rb, void *storage, size_t elem_size, size_t capacity) {
    if (!rb || !storage || elem_size == 0 || !is_power_of_two(capacity)) {
        return false;
    }
    rb->buffer    = (uint8_t *)storage;
    rb->elem_size = elem_size;
    rb->capacity  = capacity;
    rb->mask      = capacity - 1;
    rb->head      = 0;
    rb->tail      = 0;
    return true;
}

bool ring_buffer_push(ring_buffer_t *rb, const void *elem) {
    size_t current_head = rb->head;
    size_t current_tail = rb->tail;

    if ((current_head - current_tail) >= rb->capacity) {
        return false; // Queue full
    }

    uint8_t *dest = rb->buffer + ((current_head & rb->mask) * rb->elem_size);
    memcpy(dest, elem, rb->elem_size);

    // Memory barrier: ensure copy is complete before publishing new head
    __asm__ __volatile__("" ::: "memory");
    rb->head = current_head + 1;
    return true;
}

bool ring_buffer_pop(ring_buffer_t *rb, void *elem) {
    size_t current_head = rb->head;
    size_t current_tail = rb->tail;

    if (current_head == current_tail) {
        return false; // Queue empty
    }

    const uint8_t *src = rb->buffer + ((current_tail & rb->mask) * rb->elem_size);
    memcpy(elem, src, rb->elem_size);

    // Memory barrier: ensure copy out completes before advancing tail
    __asm__ __volatile__("" ::: "memory");
    rb->tail = current_tail + 1;
    return true;
}

bool ring_buffer_is_empty(const ring_buffer_t *rb) {
    return rb->head == rb->tail;
}

bool ring_buffer_is_full(const ring_buffer_t *rb) {
    return (rb->head - rb->tail) >= rb->capacity;
}

size_t ring_buffer_count(const ring_buffer_t *rb) {
    return (rb->head - rb->tail);
}
