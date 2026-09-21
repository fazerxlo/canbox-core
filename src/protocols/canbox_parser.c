#include "protocols/canbox_parser.h"
#include <string.h>

#define SYNC_RAISE_BAGOO_RAISE  0x2E
#define SYNC_BAGOO              0xFD
#define SYNC_HIWORLD_1          0x5A
#define SYNC_HIWORLD_2          0xA5

enum {
    STATE_SYNC_1 = 0,
    STATE_SYNC_2,
    STATE_CMD,
    STATE_LEN,
    STATE_PAYLOAD,
    STATE_CHECKSUM
};

static inline bool validate_checksum(canbox_dialect_t dialect, uint8_t running_sum, uint8_t rx_cs) {
    uint8_t expected = 0;
    switch (dialect) {
        case CANBOX_DIALECT_RAISE:
            expected = (uint8_t)(~running_sum);
            return expected == rx_cs;

        case CANBOX_DIALECT_HIWORLD:
            return running_sum == rx_cs;

        case CANBOX_DIALECT_BAGOO:
            return running_sum == rx_cs;

        default:
            return false;
    }
}

void canbox_parser_init(canbox_parser_t *parser, canbox_dialect_t dialect, canbox_frame_cb_t cb) {
    if (!parser) return;
    memset(parser, 0, sizeof(canbox_parser_t));
    parser->dialect = dialect;
    parser->on_frame = cb;
    parser->state = STATE_SYNC_1;
}

void canbox_parser_feed_byte(canbox_parser_t *parser, uint8_t byte) {
    if (!parser) return;

    switch (parser->state) {
        case STATE_SYNC_1:
            if (parser->dialect == CANBOX_DIALECT_HIWORLD) {
                if (byte == SYNC_HIWORLD_1) {
                    parser->state = STATE_SYNC_2;
                }
            } else if (parser->dialect == CANBOX_DIALECT_RAISE) {
                if (byte == SYNC_RAISE_BAGOO_RAISE) {
                    parser->running_sum = 0;
                    parser->state = STATE_CMD;
                }
            } else if (parser->dialect == CANBOX_DIALECT_BAGOO) {
                if (byte == SYNC_BAGOO) {
                    parser->running_sum = 0;
                    parser->state = STATE_CMD;
                }
            }
            break;

        case STATE_SYNC_2:
            if (byte == SYNC_HIWORLD_2) {
                parser->running_sum = 0;
                parser->state = STATE_LEN;
            } else if (byte != SYNC_HIWORLD_1) {
                parser->state = STATE_SYNC_1;
            }
            break;

        case STATE_CMD:
            parser->rx_frame.cmd = byte;
            parser->running_sum += byte;

            if (parser->dialect == CANBOX_DIALECT_HIWORLD) {
                if (parser->expected_len == 0) {
                    parser->state = STATE_CHECKSUM;
                } else {
                    parser->payload_idx = 0;
                    parser->state = STATE_PAYLOAD;
                }
            } else {
                parser->state = STATE_LEN;
            }
            break;

        case STATE_LEN:
            if (parser->dialect == CANBOX_DIALECT_HIWORLD) {
                if (byte < 1 || byte > (CANBOX_MAX_PAYLOAD + 1)) {
                    parser->state = STATE_SYNC_1;
                    break;
                }
                parser->running_sum += byte;
                parser->expected_len = byte - 1;
                parser->rx_frame.len = parser->expected_len;
                parser->state = STATE_CMD;
            } else {
                if (byte > CANBOX_MAX_PAYLOAD) {
                    parser->state = STATE_SYNC_1;
                    break;
                }
                parser->running_sum += byte;
                parser->expected_len = byte;
                parser->rx_frame.len = byte;

                if (parser->expected_len == 0) {
                    parser->state = STATE_CHECKSUM;
                } else {
                    parser->payload_idx = 0;
                    parser->state = STATE_PAYLOAD;
                }
            }
            break;

        case STATE_PAYLOAD:
            parser->rx_frame.payload[parser->payload_idx++] = byte;
            parser->running_sum += byte;

            if (parser->payload_idx >= parser->expected_len) {
                parser->state = STATE_CHECKSUM;
            }
            break;

        case STATE_CHECKSUM:
            if (validate_checksum(parser->dialect, parser->running_sum, byte)) {
                if (parser->on_frame) {
                    parser->on_frame(&parser->rx_frame);
                }
                parser->state = STATE_SYNC_1;
            } else {
                if (parser->dialect == CANBOX_DIALECT_HIWORLD && byte == SYNC_HIWORLD_1) {
                    parser->state = STATE_SYNC_2;
                } else if (parser->dialect == CANBOX_DIALECT_RAISE && byte == SYNC_RAISE_BAGOO_RAISE) {
                    parser->running_sum = 0;
                    parser->state = STATE_CMD;
                } else {
                    parser->state = STATE_SYNC_1;
                }
            }
            break;

        default:
            parser->state = STATE_SYNC_1;
            break;
    }
}

void canbox_parser_feed_buffer(canbox_parser_t *parser, const uint8_t *buf, size_t len) {
    if (!parser || !buf) return;
    for (size_t i = 0; i < len; i++) {
        canbox_parser_feed_byte(parser, buf[i]);
    }
}
