#include "protocols/hu_protocol.h"
#include "protocols/canbox_parser.h"
#include "hal/hal_uart.h"
#include <string.h>

#define RAISE_SYNC_BYTE 0x2E

static canbox_parser_t s_hu_parser;
static bool s_hu_parser_initialized = false;

static void on_hu_frame_received(const canbox_rx_frame_t *frame) {
    (void)frame;
    // Head Unit command callback (e.g. settings request, car type config)
}

static void ensure_parser_initialized(void) {
    if (!s_hu_parser_initialized) {
        canbox_parser_init(&s_hu_parser, CANBOX_DIALECT_RAISE, on_hu_frame_received);
        s_hu_parser_initialized = true;
    }
}

static void hu_protocol_send_packet_raise(uint8_t cmd, const uint8_t *payload, uint8_t len) {
    uint8_t tx_buf[CANBOX_MAX_PAYLOAD + 4];
    if (len > CANBOX_MAX_PAYLOAD) {
        return;
    }

    tx_buf[0] = RAISE_SYNC_BYTE;
    tx_buf[1] = cmd;
    tx_buf[2] = len;

    uint8_t sum = (uint8_t)(cmd + len);
    for (uint8_t i = 0; i < len; i++) {
        tx_buf[3 + i] = payload[i];
        sum = (uint8_t)(sum + payload[i]);
    }
    tx_buf[3 + len] = (uint8_t)(~sum);

    hal_uart_write(tx_buf, (size_t)(4 + len));
}

void hu_protocol_feed_byte(uint8_t byte) {
    ensure_parser_initialized();
    canbox_parser_feed_byte(&s_hu_parser, byte);
}

void hu_protocol_send_wheel_key(const vehicle_wheel_t *wheel) {
    if (!wheel) return;

    uint8_t payload[2];
    payload[0] = (uint8_t)wheel->active_key;
    payload[1] = wheel->press_state ? 0x01 : 0x00;

    hu_protocol_send_packet_raise(0x01, payload, sizeof(payload));
}

void hu_protocol_send_doors(const vehicle_doors_t *doors) {
    if (!doors) return;

    uint8_t payload[1] = {0};
    if (doors->door_driver)     payload[0] |= (1 << 0);
    if (doors->door_passenger)  payload[0] |= (1 << 1);
    if (doors->door_rear_left)  payload[0] |= (1 << 2);
    if (doors->door_rear_right) payload[0] |= (1 << 3);
    if (doors->trunk)           payload[0] |= (1 << 4);
    if (doors->hood)            payload[0] |= (1 << 5);

    hu_protocol_send_packet_raise(0x24, payload, sizeof(payload));
}

void hu_protocol_send_telemetry(uint16_t speed, uint16_t rpm, int16_t angle) {
    uint8_t payload[6];
    payload[0] = (uint8_t)((speed >> 8) & 0xFF);
    payload[1] = (uint8_t)(speed & 0xFF);
    payload[2] = (uint8_t)((rpm >> 8) & 0xFF);
    payload[3] = (uint8_t)(rpm & 0xFF);
    payload[4] = (uint8_t)(((uint16_t)angle >> 8) & 0xFF);
    payload[5] = (uint8_t)((uint16_t)angle & 0xFF);

    hu_protocol_send_packet_raise(0x29, payload, sizeof(payload));
}

void hu_protocol_send_heartbeat(void) {
    uint8_t payload[2] = {0x00, 0x00};
    hu_protocol_send_packet_raise(0x01, payload, sizeof(payload));
}

