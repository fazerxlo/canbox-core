#include "protocols/hu_protocol.h"
#include "protocols/hu_protocol_driver.h"
#include "protocols/hiworld_connection.h"
#include "protocols/hiworld_car_mapping.h"
#include "proto_hiworld.h"
#include "hal/hal_uart.h"
#include "hal/hal_can.h"
#include <stdbool.h>

static hiworld_connection_ctx_t s_hw_conn_ctx;
static bool s_hiworld_initialized = false;

static void on_can_config_callback(uint8_t car_model_id, uint32_t baud_rate) {
    (void)car_model_id;
    can_baudrate_t baud = CAN_BAUD_125K;
    if (baud_rate == 500000) {
        baud = CAN_BAUD_500K;
    } else if (baud_rate == 250000) {
        baud = CAN_BAUD_250K;
    }
    hal_can_init(baud);
}

static void on_hiworld_packet_received(const hiworld_packet_t *packet) {
    (void)packet;
}

static void uart_tx_adapter(const uint8_t *buf, size_t len) {
    hal_uart_write(buf, len);
}

static void ensure_hiworld_initialized(void) {
    if (!s_hiworld_initialized) {
        proto_hiworld_init(on_hiworld_packet_received);
        hiworld_conn_init(&s_hw_conn_ctx, "HW_PSA_V2.04.01", uart_tx_adapter, on_can_config_callback);
        s_hiworld_initialized = true;
    }
}

static void hiworld_init(void) {
    proto_hiworld_init(on_hiworld_packet_received);
    hiworld_conn_init(&s_hw_conn_ctx, "HW_PSA_V2.04.01", uart_tx_adapter, on_can_config_callback);
    s_hiworld_initialized = true;
}

static void hiworld_feed_byte(uint8_t byte) {
    ensure_hiworld_initialized();
    hiworld_conn_process_rx_byte(&s_hw_conn_ctx, byte);
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

    /* Hiworld Key Payload: [Key Code, Press Status (1 = pressed, 0 = released)] */
    uint8_t payload[2];
    payload[0] = hw_key_code;
    payload[1] = wheel->press_state;

    uint8_t tx_buf[16];
    size_t len = proto_hiworld_serialize(HIWORLD_CMD_CAR_BASE_INFO, payload, sizeof(payload), 
                                         tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

static void hiworld_send_doors(const vehicle_doors_t *doors) {
    /* Hiworld Door Status (Cmd 0x12) */
    /* Payload byte 0: bit0: Dvr, bit1: Pas, bit2: RL, bit3: RR, bit4: Trunk, bit5: Hood */
    uint8_t d_byte = 0;
    if (doors->door_driver)     d_byte |= (1 << 0);
    if (doors->door_passenger)  d_byte |= (1 << 1);
    if (doors->door_rear_left)  d_byte |= (1 << 2);
    if (doors->door_rear_right) d_byte |= (1 << 3);
    if (doors->trunk)           d_byte |= (1 << 4);
    if (doors->hood)            d_byte |= (1 << 5);

    uint8_t payload[1] = { d_byte };
    uint8_t tx_buf[16];
    size_t len = proto_hiworld_serialize(HIWORLD_CMD_DOOR_WINDOW_STATE, payload, sizeof(payload), 
                                         tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

static void hiworld_send_telemetry(uint16_t speed, uint16_t rpm, int16_t angle) {
    /* Hiworld Steering Angle (Cmd 0x11): 2 bytes signed Little-Endian */
    uint8_t angle_payload[2];
    angle_payload[0] = (uint8_t)(angle & 0xFF);
    angle_payload[1] = (uint8_t)((angle >> 8) & 0xFF);

    uint8_t tx_buf[16];
    size_t len = proto_hiworld_serialize(HIWORLD_CMD_CAR_BASE_INFO, angle_payload, sizeof(angle_payload), 
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
