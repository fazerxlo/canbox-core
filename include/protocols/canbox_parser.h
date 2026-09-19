#ifndef CANBOX_PARSER_H
#define CANBOX_PARSER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define CANBOX_MAX_PAYLOAD 64

typedef enum {
    CANBOX_DIALECT_RAISE = 0,   // 0x2E header, inverted checksum
    CANBOX_DIALECT_HIWORLD,     // 0x5A 0xA5 header, direct 8-bit sum
    CANBOX_DIALECT_BAGOO        // 0xFD header, 2's complement checksum
} canbox_dialect_t;

typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[CANBOX_MAX_PAYLOAD];
} canbox_rx_frame_t;

typedef void (*canbox_frame_cb_t)(const canbox_rx_frame_t *frame);

typedef struct {
    canbox_dialect_t  dialect;
    uint8_t           state;
    uint8_t           expected_len;
    uint8_t           payload_idx;
    uint8_t           running_sum;
    canbox_rx_frame_t rx_frame;
    canbox_frame_cb_t on_frame;
} canbox_parser_t;

void canbox_parser_init(canbox_parser_t *parser, canbox_dialect_t dialect, canbox_frame_cb_t cb);
void canbox_parser_feed_byte(canbox_parser_t *parser, uint8_t byte);
void canbox_parser_feed_buffer(canbox_parser_t *parser, const uint8_t *buf, size_t len);

#endif /* CANBOX_PARSER_H */
