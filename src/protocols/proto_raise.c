#include "proto_raise.h"
#include <string.h>

typedef enum {
    STATE_WAIT_SYNC = 0,
    STATE_WAIT_CMD,
    STATE_WAIT_LEN,
    STATE_WAIT_PAYLOAD,
    STATE_WAIT_CHECKSUM
} parser_state_t;

static parser_state_t      s_state = STATE_WAIT_SYNC;
static raise_packet_t      s_rx_packet;
static uint8_t             s_payload_idx = 0;
static uint8_t             s_running_sum = 0;
static raise_rx_callback_t s_rx_callback = NULL;

static inline uint8_t calc_checksum(uint8_t cmd, uint8_t len, const uint8_t *payload) {
    uint8_t sum = cmd + len;
    for (uint8_t i = 0; i < len; i++) {
        sum += payload[i];
    }
    return (uint8_t)(~sum);
}

void proto_raise_init(raise_rx_callback_t rx_cb) {
    s_rx_callback = rx_cb;
    s_state = STATE_WAIT_SYNC;
    s_payload_idx = 0;
    s_running_sum = 0;
    memset(&s_rx_packet, 0, sizeof(s_rx_packet));
}

void proto_raise_feed_byte(uint8_t byte) {
    switch (s_state) {
        case STATE_WAIT_SYNC:
            if (byte == RAISE_SYNC_BYTE) {
                s_running_sum = 0;
                s_payload_idx = 0;
                s_state = STATE_WAIT_CMD;
            }
            break;

        case STATE_WAIT_CMD:
            s_rx_packet.cmd = byte;
            s_running_sum = byte;
            s_state = STATE_WAIT_LEN;
            break;

        case STATE_WAIT_LEN:
            if (byte > RAISE_MAX_PAYLOAD_LEN) {
                // Invalid length: resync immediately
                s_state = (byte == RAISE_SYNC_BYTE) ? STATE_WAIT_CMD : STATE_WAIT_SYNC;
                break;
            }
            s_rx_packet.len = byte;
            s_running_sum += byte;

            if (s_rx_packet.len == 0) {
                s_state = STATE_WAIT_CHECKSUM;
            } else {
                s_state = STATE_WAIT_PAYLOAD;
            }
            break;

        case STATE_WAIT_PAYLOAD:
            s_rx_packet.payload[s_payload_idx++] = byte;
            s_running_sum += byte;

            if (s_payload_idx >= s_rx_packet.len) {
                s_state = STATE_WAIT_CHECKSUM;
            }
            break;

        case STATE_WAIT_CHECKSUM: {
            uint8_t expected_cs = (uint8_t)(~s_running_sum);
            if (byte == expected_cs) {
                if (s_rx_callback) {
                    s_rx_callback(&s_rx_packet);
                }
                s_state = STATE_WAIT_SYNC;
            } else {
                // Checksum mismatch: test if this byte happens to be a new frame header
                s_state = (byte == RAISE_SYNC_BYTE) ? STATE_WAIT_CMD : STATE_WAIT_SYNC;
            }
            break;
        }

        default:
            s_state = STATE_WAIT_SYNC;
            break;
    }
}

size_t proto_raise_serialize(uint8_t cmd, const uint8_t *payload, uint8_t len, uint8_t *out_buf, size_t max_out) {
    if (!out_buf || len > RAISE_MAX_PAYLOAD_LEN) {
        return 0;
    }

    size_t total_len = 4 + (size_t)len; // Sync(1) + Cmd(1) + Len(1) + Payload(N) + Checksum(1)
    if (max_out < total_len) {
        return 0;
    }

    out_buf[0] = RAISE_SYNC_BYTE;
    out_buf[1] = cmd;
    out_buf[2] = len;

    if (len > 0 && payload != NULL) {
        memcpy(&out_buf[3], payload, len);
    }

    out_buf[3 + len] = calc_checksum(cmd, len, payload);

    return total_len;
}
