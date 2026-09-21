#ifndef PROTO_BAGOO_H
#define PROTO_BAGOO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BAGOO_SYNC_BYTE        0xFD
#define BAGOO_MAX_PAYLOAD_LEN  64

// Bagoo / RZC Command IDs
#define BAGOO_CMD_WHEEL_KEY        0x01
#define BAGOO_CMD_TPMS_DISCRETE    0x18
#define BAGOO_CMD_HVAC             0x21
#define BAGOO_CMD_STEERING_RAD     0x26
#define BAGOO_CMD_STEERING_ANGLE   0x29
#define BAGOO_CMD_RADAR_FRONT      0x30
#define BAGOO_CMD_RADAR_REAR       0x32
#define BAGOO_CMD_INSTANT_FUEL     0x33
#define BAGOO_CMD_TRIP1            0x34
#define BAGOO_CMD_TRIP2            0x35
#define BAGOO_CMD_OUTSIDE_TEMP     0x36
#define BAGOO_CMD_DOORS_BODY       0x38
#define BAGOO_CMD_REVERSE          0x40
#define BAGOO_CMD_BSI_CONFIG       0x7D
#define BAGOO_CMD_VERSION_REQ      0x7F

typedef struct {
    uint8_t cmd;
    uint8_t len; // Payload length (N bytes)
    uint8_t payload[BAGOO_MAX_PAYLOAD_LEN];
} bagoo_packet_t;

typedef void (*bagoo_rx_callback_t)(const bagoo_packet_t *packet);

void proto_bagoo_init(bagoo_rx_callback_t rx_cb);
void proto_bagoo_feed_byte(uint8_t byte);
size_t proto_bagoo_serialize(uint8_t cmd, const uint8_t *payload, uint8_t len, uint8_t *out_buf, size_t max_out);

#ifdef __cplusplus
}
#endif

#endif /* PROTO_BAGOO_H */

