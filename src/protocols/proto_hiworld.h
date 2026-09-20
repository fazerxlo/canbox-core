#ifndef PROTO_HIWORLD_H
#define PROTO_HIWORLD_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HIWORLD_SYNC_1           0x5A
#define HIWORLD_SYNC_2           0xA5
#define HIWORLD_MAX_PAYLOAD_LEN  64

// Common Hiworld Command IDs
#define HIWORLD_CMD_WHEEL_KEY    0x11
#define HIWORLD_CMD_AIR_CON      0x12
#define HIWORLD_CMD_DOOR_STATUS  0x21
#define HIWORLD_CMD_RADAR_FRONT  0x22
#define HIWORLD_CMD_RADAR_REAR   0x23
#define HIWORLD_CMD_TRACK_ANGLE  0x26
#define HIWORLD_CMD_HEARTBEAT    0xFF

typedef struct {
    uint8_t cmd;
    uint8_t payload_len; // N = Length - 1
    uint8_t payload[HIWORLD_MAX_PAYLOAD_LEN];
} hiworld_packet_t;

typedef void (*hiworld_rx_callback_t)(const hiworld_packet_t *packet);

void proto_hiworld_init(hiworld_rx_callback_t rx_cb);
void proto_hiworld_feed_byte(uint8_t byte);
size_t proto_hiworld_serialize(uint8_t cmd, const uint8_t *payload, uint8_t payload_len, 
                               uint8_t *out_buf, size_t max_out);

#ifdef __cplusplus
}
#endif

#endif /* PROTO_HIWORLD_H */
