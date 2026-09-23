#include "proto_hiworld.h"
#include <string.h>

typedef enum {
    HW_STATE_WAIT_SYNC1 = 0,
    HW_STATE_WAIT_SYNC2,
    HW_STATE_WAIT_LEN,
    HW_STATE_WAIT_CMD,
    HW_STATE_WAIT_PAYLOAD,
    HW_STATE_WAIT_CHECKSUM
} hiworld_parser_state_t;

static hiworld_parser_state_t s_state = HW_STATE_WAIT_SYNC1;
static hiworld_packet_t       s_rx_packet;
static uint8_t                s_payload_len = 0;
static uint8_t                s_payload_idx = 0;
static uint8_t                s_running_sum = 0;
static hiworld_rx_callback_t  s_rx_callback = NULL;

static inline uint8_t calc_checksum(uint8_t payload_len, uint8_t cmd, const uint8_t *payload) {
    uint8_t sum = (uint8_t)(payload_len + cmd);
    for (uint8_t i = 0; i < payload_len; i++) {
        sum = (uint8_t)(sum + payload[i]);
    }
    return (uint8_t)((sum - 1) & 0xFF);
}

void proto_hiworld_init(hiworld_rx_callback_t rx_cb) {
    s_rx_callback = rx_cb;
    s_state = HW_STATE_WAIT_SYNC1;
    s_payload_len = 0;
    s_payload_idx = 0;
    s_running_sum = 0;
    memset(&s_rx_packet, 0, sizeof(s_rx_packet));
}

void proto_hiworld_feed_byte(uint8_t byte) {
    switch (s_state) {
        case HW_STATE_WAIT_SYNC1:
            if (byte == HIWORLD_SYNC_1) {
                s_state = HW_STATE_WAIT_SYNC2;
            }
            break;

        case HW_STATE_WAIT_SYNC2:
            if (byte == HIWORLD_SYNC_2) {
                s_state = HW_STATE_WAIT_LEN;
            } else if (byte != HIWORLD_SYNC_1) {
                s_state = HW_STATE_WAIT_SYNC1;
            }
            break;

        case HW_STATE_WAIT_LEN:
            if (byte > HIWORLD_MAX_PAYLOAD_LEN) {
                s_state = (byte == HIWORLD_SYNC_1) ? HW_STATE_WAIT_SYNC2 : HW_STATE_WAIT_SYNC1;
                break;
            }
            s_payload_len = byte;
            s_rx_packet.payload_len = byte;
            s_running_sum = byte;
            s_payload_idx = 0;
            s_state = HW_STATE_WAIT_CMD;
            break;

        case HW_STATE_WAIT_CMD:
            s_rx_packet.cmd = byte;
            s_running_sum = (uint8_t)(s_running_sum + byte);

            if (s_rx_packet.payload_len == 0) {
                s_state = HW_STATE_WAIT_CHECKSUM;
            } else {
                s_state = HW_STATE_WAIT_PAYLOAD;
            }
            break;

        case HW_STATE_WAIT_PAYLOAD:
            s_rx_packet.payload[s_payload_idx++] = byte;
            s_running_sum = (uint8_t)(s_running_sum + byte);

            if (s_payload_idx >= s_rx_packet.payload_len) {
                s_state = HW_STATE_WAIT_CHECKSUM;
            }
            break;

        case HW_STATE_WAIT_CHECKSUM: {
            uint8_t expected_cs = (uint8_t)((s_running_sum - 1) & 0xFF);
            if (byte == expected_cs) {
                if (s_rx_callback) {
                    s_rx_callback(&s_rx_packet);
                }
                s_state = HW_STATE_WAIT_SYNC1;
            } else {
                // Framing slip or corrupt byte: check if sync pattern begins
                s_state = (byte == HIWORLD_SYNC_1) ? HW_STATE_WAIT_SYNC2 : HW_STATE_WAIT_SYNC1;
            }
            break;
        }

        default:
            s_state = HW_STATE_WAIT_SYNC1;
            break;
    }
}

size_t proto_hiworld_serialize(uint8_t cmd, const uint8_t *payload, uint8_t payload_len,
                               uint8_t *out_buf, size_t max_out) {
    if (!out_buf || payload_len > HIWORLD_MAX_PAYLOAD_LEN) {
        return 0;
    }

    size_t total_size = (size_t)(payload_len + 5); // Sync(2) + Len(1) + Cmd(1) + Data(N) + CS(1)

    if (max_out < total_size) {
        return 0;
    }

    out_buf[0] = HIWORLD_SYNC_1;
    out_buf[1] = HIWORLD_SYNC_2;
    out_buf[2] = payload_len;
    out_buf[3] = cmd;

    if (payload_len > 0 && payload != NULL) {
        memcpy(&out_buf[4], payload, payload_len);
    }

    out_buf[4 + payload_len] = calc_checksum(payload_len, cmd, payload);

    return total_size;
}
