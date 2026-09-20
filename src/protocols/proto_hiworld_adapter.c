#include "protocols/hu_protocol.h"
#include "protocols/hu_protocol_driver.h"
#include "proto_hiworld.h"
#include "hal/hal_uart.h"

#include <stdbool.h>

static void on_hiworld_packet_received(const hiworld_packet_t *packet) {
    // Android head unit control messages (e.g. HU settings sync)
    (void)packet;
}

static bool s_hiworld_initialized = false;

static void ensure_hiworld_initialized(void) {
    if (!s_hiworld_initialized) {
        proto_hiworld_init(on_hiworld_packet_received);
        s_hiworld_initialized = true;
    }
}

static void hiworld_init(void) {
    proto_hiworld_init(on_hiworld_packet_received);
    s_hiworld_initialized = true;
}

static void hiworld_feed_byte(uint8_t byte) {
    ensure_hiworld_initialized();
    proto_hiworld_feed_byte(byte);
}

static void hiworld_send_wheel_key(const vehicle_wheel_t *wheel) {
    uint8_t hw_key_code = 0x00;

    switch (wheel->active_key) {
        case WHEEL_KEY_VOL_UP:       hw_key_code = 0x01; break;
        case WHEEL_KEY_VOL_DOWN:     hw_key_code = 0x02; break;
        case WHEEL_KEY_MUTE:         hw_key_code = 0x03; break;
        case WHEEL_KEY_SRC:          hw_key_code = 0x04; break;
        case WHEEL_KEY_NEXT:         hw_key_code = 0x07; break;
        case WHEEL_KEY_PREV:         hw_key_code = 0x08; break;
        case WHEEL_KEY_PHONE_ACCEPT: hw_key_code = 0x09; break;
        case WHEEL_KEY_PHONE_REJECT: hw_key_code = 0x0A; break;
        case WHEEL_KEY_VOICE:        hw_key_code = 0x0B; break;
        default:                     hw_key_code = 0x00; break;
    }

    // Hiworld Key Payload: [Key Code, Press Status (1 = pressed, 0 = released)]
    uint8_t payload[2];
    payload[0] = hw_key_code;
    payload[1] = wheel->press_state;

    uint8_t tx_buf[16];
    size_t len = proto_hiworld_serialize(HIWORLD_CMD_WHEEL_KEY, payload, sizeof(payload), 
                                         tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

static void hiworld_send_doors(const vehicle_doors_t *doors) {
    // Hiworld Door Status (Cmd 0x21)
    // Payload byte 0: bit0: Dvr, bit1: Pas, bit2: RL, bit3: RR, bit4: Trunk, bit5: Hood
    uint8_t d_byte = 0;
    if (doors->door_driver)     d_byte |= (1 << 0);
    if (doors->door_passenger)  d_byte |= (1 << 1);
    if (doors->door_rear_left)  d_byte |= (1 << 2);
    if (doors->door_rear_right) d_byte |= (1 << 3);
    if (doors->trunk)           d_byte |= (1 << 4);
    if (doors->hood)            d_byte |= (1 << 5);

    uint8_t payload[1] = { d_byte };
    uint8_t tx_buf[16];
    size_t len = proto_hiworld_serialize(HIWORLD_CMD_DOOR_STATUS, payload, sizeof(payload), 
                                         tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

static void hiworld_send_telemetry(uint16_t speed, uint16_t rpm, int16_t angle) {
    // Hiworld Steering Angle (Cmd 0x26): 2 bytes signed Little-Endian
    uint8_t angle_payload[2];
    angle_payload[0] = (uint8_t)(angle & 0xFF);
    angle_payload[1] = (uint8_t)((angle >> 8) & 0xFF);

    uint8_t tx_buf[16];
    size_t len = proto_hiworld_serialize(HIWORLD_CMD_TRACK_ANGLE, angle_payload, sizeof(angle_payload), 
                                         tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }

    (void)speed;
    (void)rpm;
}

static void hiworld_send_heartbeat(void) {
    static const uint8_t hb[1] = { 0x01 };
    uint8_t tx_buf[8];
    size_t len = proto_hiworld_serialize(HIWORLD_CMD_HEARTBEAT, hb, sizeof(hb), tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

const hu_protocol_driver_t g_hu_protocol_hiworld = {
    .id = HU_PROTOCOL_HIWORLD,
    .name = "Hiworld",
    .init = hiworld_init,
    .feed_byte = hiworld_feed_byte,
    .send_wheel_key = hiworld_send_wheel_key,
    .send_doors = hiworld_send_doors,
    .send_telemetry = hiworld_send_telemetry,
    .send_heartbeat = hiworld_send_heartbeat,
};
