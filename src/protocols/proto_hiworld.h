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

/* Common Hiworld Command IDs */
#define HIWORLD_CMD_WHEEL_KEY     0x11
#define HIWORLD_CMD_DOORS         0x12
#define HIWORLD_CMD_ECU_P0        0x13
#define HIWORLD_CMD_ECU_P1        0x14
#define HIWORLD_CMD_ECU_P2        0x15
#define HIWORLD_CMD_TPMS_DISCRETE 0x18
#define HIWORLD_CMD_ECU_SETTING_SET 0x1B
#define HIWORLD_CMD_CONTROL_PANEL_KEY 0x21
#define HIWORLD_CMD_DIAGNOSTIC_QUERY 0x2F
#define HIWORLD_CMD_AIR_CON       0x31
#define HIWORLD_CMD_RADAR         0x41
#define HIWORLD_CMD_WARNING_INFO      0x42
#define HIWORLD_CMD_EXTENDED_ALERT    0xEA
#define HIWORLD_CMD_TPMS_NUMERIC      0x66
#define HIWORLD_CMD_SOUND_EFFECT      0x82
#define HIWORLD_CMD_CAR_RADIO_STATE   0x84
#define HIWORLD_CMD_CAR_RADIO_PRESET_FREQ 0x85
#define HIWORLD_CMD_RADIO_TEXT        0x86
#define HIWORLD_CMD_CAR_MEDIA_STATE   0x97
#define HIWORLD_CMD_HEARTBEAT         0xFF

/* Alert transmission compilation parameters:
 * By default, only Cmd 0xEA is transmitted to prevent gluing 0x42 and 0xEA
 * into a single UART write buffer and to prevent UI race conditions on Android.
 * Define HIWORLD_ENABLE_ALERT_0X42=1 to enable legacy Cmd 0x42 transmission.
 */
#ifndef HIWORLD_ENABLE_ALERT_0X42
#define HIWORLD_ENABLE_ALERT_0X42     0
#endif

#ifndef HIWORLD_ENABLE_ALERT_0XEA
#define HIWORLD_ENABLE_ALERT_0XEA     1
#endif

typedef struct {
    uint8_t cmd;
    uint8_t payload_len; /* Pure payload length L */
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
