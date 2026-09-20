#include "protocols/hu_protocol.h"
#include "protocols/hu_protocol_driver.h"
#include "proto_raise.h"
#include "hal/hal_uart.h"

#include <stdbool.h>

#define TX_BUFFER_MAX 128

static void on_raise_packet_received(const raise_packet_t *packet) {
    // Process commands originating from the Android Head Unit (e.g., settings, requests)
    if (packet->cmd == RAISE_CMD_VERSION_REQ) {
        // Echo firmware version back: "SW001"
        const uint8_t ver[] = { 'S', 'W', '0', '0', '1' };
        uint8_t tx_buf[16];
        size_t len = proto_raise_serialize(0x7F, ver, sizeof(ver), tx_buf, sizeof(tx_buf));
        hal_uart_write(tx_buf, len);
    }
}

static bool s_raise_initialized = false;

static void ensure_raise_initialized(void) {
    if (!s_raise_initialized) {
        proto_raise_init(on_raise_packet_received);
        s_raise_initialized = true;
    }
}

static void raise_init(void) {
    proto_raise_init(on_raise_packet_received);
    s_raise_initialized = true;
}

static void raise_feed_byte(uint8_t byte) {
    ensure_raise_initialized();
    proto_raise_feed_byte(byte);
}

static void raise_send_wheel_key(const vehicle_wheel_t *wheel) {
    uint8_t raise_key_code = 0x00;

    switch (wheel->active_key) {
        case WHEEL_KEY_VOL_UP:       raise_key_code = 0x01; break;
        case WHEEL_KEY_VOL_DOWN:     raise_key_code = 0x02; break;
        case WHEEL_KEY_NEXT:         raise_key_code = 0x03; break;
        case WHEEL_KEY_PREV:         raise_key_code = 0x04; break;
        case WHEEL_KEY_SRC:          raise_key_code = 0x07; break;
        case WHEEL_KEY_MUTE:         raise_key_code = 0x09; break;
        case WHEEL_KEY_PHONE_ACCEPT: raise_key_code = 0x05; break;
        case WHEEL_KEY_PHONE_REJECT: raise_key_code = 0x06; break;
        default:                     raise_key_code = 0x00; break;
    }

    uint8_t payload[2];
    payload[0] = (wheel->press_state != 0) ? raise_key_code : 0x00; // Key action
    payload[1] = 0x00; // Extra modifier

    uint8_t tx_buf[8];
    size_t len = proto_raise_serialize(RAISE_CMD_WHEEL_KEY, payload, sizeof(payload), tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

static void raise_send_doors(const vehicle_doors_t *doors) {
    // Raise Door Status Payload (Cmd 0x24):
    // Byte 0: [Hood(7), Trunk(6), RR(3), RL(2), Pas(1), Dvr(0)]
    uint8_t d_byte = 0;
    if (doors->door_driver)     d_byte |= (1 << 0);
    if (doors->door_passenger)  d_byte |= (1 << 1);
    if (doors->door_rear_left)  d_byte |= (1 << 2);
    if (doors->door_rear_right) d_byte |= (1 << 3);
    if (doors->trunk)           d_byte |= (1 << 6);
    if (doors->hood)            d_byte |= (1 << 7);

    uint8_t payload[1] = { d_byte };
    uint8_t tx_buf[8];
    size_t len = proto_raise_serialize(RAISE_CMD_DOOR_STATUS, payload, sizeof(payload), tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

static void raise_send_telemetry(uint16_t speed, uint16_t rpm, int16_t angle) {
    // Raise Telemetry Payload (Cmd 0x29)
    uint8_t payload[6];
    payload[0] = (uint8_t)(speed >> 8);
    payload[1] = (uint8_t)(speed & 0xFF);
    payload[2] = (uint8_t)(rpm >> 8);
    payload[3] = (uint8_t)(rpm & 0xFF);
    payload[4] = (uint8_t)(angle >> 8);
    payload[5] = (uint8_t)(angle & 0xFF);

    uint8_t tx_buf[16];
    size_t len = proto_raise_serialize(RAISE_CMD_TELEMETRY, payload, sizeof(payload), tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

static void raise_send_heartbeat(void) {
    // Keep-alive or periodic status query (Cmd 0x20 ping)
    static const uint8_t ping[1] = { 0x01 };
    uint8_t tx_buf[8];
    size_t len = proto_raise_serialize(0x20, ping, sizeof(ping), tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

const hu_protocol_driver_t g_hu_protocol_raise = {
    .id = HU_PROTOCOL_RAISE,
    .name = "Raise",
    .init = raise_init,
    .feed_byte = raise_feed_byte,
    .send_wheel_key = raise_send_wheel_key,
    .send_doors = raise_send_doors,
    .send_telemetry = raise_send_telemetry,
    .send_heartbeat = raise_send_heartbeat,
};
