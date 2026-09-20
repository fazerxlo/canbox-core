#ifndef PROTO_RAISE_H
#define PROTO_RAISE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RAISE_SYNC_BYTE        0x2E
#define RAISE_MAX_PAYLOAD_LEN  64

// Command IDs commonly used in Raise definitions
#define RAISE_CMD_WHEEL_KEY    0x01
#define RAISE_CMD_DOOR_STATUS  0x24
#define RAISE_CMD_STEERING_RAD 0x26
#define RAISE_CMD_TELEMETRY    0x29
#define RAISE_CMD_VERSION_REQ  0x7F
#define RAISE_CMD_WHEEL_KEY        0x01
#define RAISE_CMD_STALK_KEY        0x02
#define RAISE_CMD_HVAC             0x21
#define RAISE_CMD_DOOR_STATUS      0x24
#define RAISE_CMD_STEERING_RAD     0x26
#define RAISE_CMD_STEERING_ANGLE   0x29
#define RAISE_CMD_TELEMETRY        0x29
#define RAISE_CMD_RADAR_FRONT      0x30
#define RAISE_CMD_RADAR_REAR       0x32
#define RAISE_CMD_INSTANT_FUEL     0x33
#define RAISE_CMD_TRIP1            0x34
#define RAISE_CMD_TRIP2            0x35
#define RAISE_CMD_OUTSIDE_TEMP     0x36
#define RAISE_CMD_DOORS_BODY       0x38
#define RAISE_CMD_REVERSE          0x40
#define RAISE_CMD_CD_CHANGER       0x54
#define RAISE_CMD_RDS_NAME         0x55
#define RAISE_CMD_AMPLIFIER        0x56
#define RAISE_CMD_VERSION_REQ      0x7F
#define RAISE_CMD_CAR_MODEL_SELECT 0xCA

typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[RAISE_MAX_PAYLOAD_LEN];
} raise_packet_t;

typedef void (*raise_rx_callback_t)(const raise_packet_t *packet);

void proto_raise_init(raise_rx_callback_t rx_cb);
void proto_raise_feed_byte(uint8_t byte);
size_t proto_raise_serialize(uint8_t cmd, const uint8_t *payload, uint8_t len, uint8_t *out_buf, size_t max_out);

#ifdef __cplusplus
}
#endif

#endif /* PROTO_RAISE_H */
