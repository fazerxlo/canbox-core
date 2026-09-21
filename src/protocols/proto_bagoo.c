#include "proto_bagoo.h"
#include <string.h>

typedef enum {
    STATE_WAIT_SYNC = 0,
    STATE_WAIT_LEN,
    STATE_WAIT_CMD,
    STATE_WAIT_PAYLOAD,
    STATE_WAIT_CHECKSUM
} bagoo_parser_state_t;

static bagoo_parser_state_t s_state = STATE_WAIT_SYNC;
static bagoo_packet_t       s_rx_packet;
static uint8_t              s_payload_idx = 0;
static uint8_t              s_running_sum = 0;
static uint8_t              s_expected_total_len = 0;
static bagoo_rx_callback_t  s_rx_callback = NULL;

static inline uint8_t calc_bagoo_checksum(uint8_t total_len, uint8_t cmd, uint8_t payload_len, const uint8_t *payload) {
    uint8_t sum = total_len + cmd;
    for (uint8_t i = 0; i < payload_len; i++) {
        sum += payload[i];
    }
    return sum;
}

void proto_bagoo_init(bagoo_rx_callback_t rx_cb) {
    s_rx_callback = rx_cb;
    s_state = STATE_WAIT_SYNC;
    s_payload_idx = 0;
    s_running_sum = 0;
    s_expected_total_len = 0;
    memset(&s_rx_packet, 0, sizeof(s_rx_packet));
}

void proto_bagoo_feed_byte(uint8_t byte) {
    switch (s_state) {
        case STATE_WAIT_SYNC:
            if (byte == BAGOO_SYNC_BYTE) {
                s_state = STATE_WAIT_LEN;
            }
            break;

        case STATE_WAIT_LEN:
            // Total length byte represents remaining bytes in frame including Len itself and CS
            // Min total_len = 3 (Len + Cmd + CS, 0-byte payload)
            // Max total_len = 3 + BAGOO_MAX_PAYLOAD_LEN
            if (byte < 3 || byte > (BAGOO_MAX_PAYLOAD_LEN + 3)) {
                s_state = (byte == BAGOO_SYNC_BYTE) ? STATE_WAIT_LEN : STATE_WAIT_SYNC;
                break;
            }
            s_expected_total_len = byte;
            s_rx_packet.len = byte - 3; // Payload byte count
            s_running_sum = byte;
            s_payload_idx = 0;
            s_state = STATE_WAIT_CMD;
            break;

        case STATE_WAIT_CMD:
            s_rx_packet.cmd = byte;
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
            uint8_t expected_cs = s_running_sum;
            if (byte == expected_cs) {
                if (s_rx_callback) {
                    s_rx_callback(&s_rx_packet);
                }
                s_state = STATE_WAIT_SYNC;
            } else {
                // Resync hunt
                s_state = (byte == BAGOO_SYNC_BYTE) ? STATE_WAIT_LEN : STATE_WAIT_SYNC;
            }
            break;
        }

        default:
            s_state = STATE_WAIT_SYNC;
            break;
    }
}

size_t proto_bagoo_serialize(uint8_t cmd, const uint8_t *payload, uint8_t len, uint8_t *out_buf, size_t max_out) {
    if (!out_buf || len > BAGOO_MAX_PAYLOAD_LEN) {
        return 0;
    }

    uint8_t total_len_field = (uint8_t)(len + 3); // Len byte + Cmd + Payload(N) + CS
    size_t total_wire_len = 1 + (size_t)total_len_field; // Sync(1) + Total_len_field
    if (max_out < total_wire_len) {
        return 0;
    }

    out_buf[0] = BAGOO_SYNC_BYTE;
    out_buf[1] = total_len_field;
    out_buf[2] = cmd;

    if (len > 0 && payload != NULL) {
        memcpy(&out_buf[3], payload, len);
    }

    out_buf[3 + len] = calc_bagoo_checksum(total_len_field, cmd, len, payload);

    return total_wire_len;
}

