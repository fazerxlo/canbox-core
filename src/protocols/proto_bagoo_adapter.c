#include "protocols/hu_protocol.h"
#include "protocols/hu_protocol_driver.h"
#include "proto_bagoo.h"
#include "hal/hal_uart.h"

#include <stdbool.h>
#include <string.h>

#define BAGOO_TX_BUF_SIZE 64

static void on_bagoo_packet_received(const bagoo_packet_t *packet) {
    // Process downlink requests originating from the Head Unit
    if (packet->cmd == BAGOO_CMD_VERSION_REQ) {
        const uint8_t ver[] = { 'R', 'Z', 'C', '-', 'P', 'S', 'A' };
        uint8_t tx_buf[16];
        size_t len = proto_bagoo_serialize(BAGOO_CMD_VERSION_REQ, ver, sizeof(ver), tx_buf, sizeof(tx_buf));
        if (len > 0) {
            hal_uart_write(tx_buf, len);
        }
    }
}

static bool s_bagoo_initialized = false;

static void ensure_bagoo_initialized(void) {
    if (!s_bagoo_initialized) {
        proto_bagoo_init(on_bagoo_packet_received);
        s_bagoo_initialized = true;
    }
}

static void bagoo_init(void) {
    proto_bagoo_init(on_bagoo_packet_received);
    s_bagoo_initialized = true;
}

static void bagoo_feed_byte(uint8_t byte) {
    ensure_bagoo_initialized();
    proto_bagoo_feed_byte(byte);
}

static void bagoo_send_wheel_key(const vehicle_wheel_t *wheel) {
    uint8_t key_code = 0x00;

    switch (wheel->active_key) {
        case WHEEL_KEY_VOL_UP:       key_code = 0x08; break; // As observed in 407 dump
        case WHEEL_KEY_VOL_DOWN:     key_code = 0x0F; break;
        case WHEEL_KEY_NEXT:         key_code = 0x0A; break;
        case WHEEL_KEY_PREV:         key_code = 0x01; break;
        case WHEEL_KEY_SRC:          key_code = 0x07; break;
        case WHEEL_KEY_MUTE:         key_code = 0x06; break;
        case WHEEL_KEY_PHONE_ACCEPT: key_code = 0x19; break;
        case WHEEL_KEY_PHONE_HANGUP: key_code = 0x15; break;
        default:                     key_code = 0x00; break;
    }

    // Packet format: [0x00, 0x00, KeyCode]
    uint8_t payload[3] = { 0x00, 0x00, 0x00 };
    if (wheel->press_state != 0) {
        payload[2] = key_code;
    }

    uint8_t tx_buf[BAGOO_TX_BUF_SIZE];
    size_t len = proto_bagoo_serialize(BAGOO_CMD_WHEEL_KEY, payload, sizeof(payload), tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

static void bagoo_send_doors(const vehicle_doors_t *doors) {
    // Bagoo / Peugeot 407 Cmd 0x38 (8 bytes payload)
    uint8_t payload[8];
    memset(payload, 0, sizeof(payload));

    uint8_t d0 = 0;
    if (doors->door_driver)     d0 |= 0x80;
    if (doors->door_passenger)  d0 |= 0x40;
    if (doors->door_rear_left)  d0 |= 0x20;
    if (doors->door_rear_right) d0 |= 0x10;
    if (doors->trunk)           d0 |= 0x08;
    if (doors->hood)            d0 |= 0x04;
    payload[0] = d0;

    // Default BSI settings active in Peugeot 407
    payload[1] = 0x88; // Rear wiper in reverse (0x80) | Park assist active (0x08)
    payload[2] = 0x81; // DRL active (0x80) | Cornering lights (0x01)
    payload[3] = 0x01; // Ambient mood lighting lvl 1
    payload[7] = 0x68; // Stop&Start active (0x40) | Mirror folding (0x08)

    uint8_t tx_buf[BAGOO_TX_BUF_SIZE];
    size_t len = proto_bagoo_serialize(BAGOO_CMD_DOORS_BODY, payload, sizeof(payload), tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

static void bagoo_send_telemetry(uint16_t speed, uint16_t rpm, int16_t angle) {
    (void)speed;
    (void)rpm;

    // Steering Angle (Cmd 0x29)
    uint8_t angle_payload[2];
    angle_payload[0] = (uint8_t)(angle & 0xFF);
    angle_payload[1] = (uint8_t)((angle >> 8) & 0xFF);

    uint8_t tx_buf[BAGOO_TX_BUF_SIZE];
    size_t len = proto_bagoo_serialize(BAGOO_CMD_STEERING_ANGLE, angle_payload, sizeof(angle_payload), tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

static void bagoo_send_heartbeat(void) {
    // Keepalive status packet (Cmd 0x7D subcmd 0x01: active vehicle functions)
    uint8_t payload[2] = { 0x01, 0x80 };
    uint8_t tx_buf[BAGOO_TX_BUF_SIZE];
    size_t len = proto_bagoo_serialize(BAGOO_CMD_BSI_CONFIG, payload, sizeof(payload), tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

const hu_protocol_driver_t g_hu_protocol_bagoo = {
    .id = HU_PROTOCOL_BAGOO,
    .name = "Bagoo",
    .init = bagoo_init,
    .feed_byte = bagoo_feed_byte,
    .send_wheel_key = bagoo_send_wheel_key,
    .send_doors = bagoo_send_doors,
    .send_telemetry = bagoo_send_telemetry,
    .send_heartbeat = bagoo_send_heartbeat,
};

