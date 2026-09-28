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
        hiworld_conn_init(&s_hw_conn_ctx, "H1H2PA123A-240717", uart_tx_adapter, on_can_config_callback);
        s_hiworld_initialized = true;
    }
}

static void hiworld_init(void) {
    proto_hiworld_init(on_hiworld_packet_received);
    hiworld_conn_init(&s_hw_conn_ctx, "H1H2PA123A-240717", uart_tx_adapter, on_can_config_callback);
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
    /* Hiworld Door Status (Cmd 0x12): 10 payload bytes as captured from real PSA Hiworld Canbox
     * Byte 0: 0x00
     * Byte 1: 0x04
     * Byte 2: bit7: Driver (0x80), bit6: Pass (0x40), bit5: RL (0x20), bit4: RR (0x10), bit3: Trunk (0x08), bit2: Hood/Active (0x04)
     * Byte 3..8: 0x00
     * Byte 9: 0x03
     */
    uint8_t b2 = 0x04;
    if (doors->door_driver)     b2 |= 0x80;
    if (doors->door_passenger)  b2 |= 0x40;
    if (doors->door_rear_left)  b2 |= 0x20;
    if (doors->door_rear_right) b2 |= 0x10;
    if (doors->trunk)           b2 |= 0x08;
    if (doors->hood)            b2 |= 0x04;

    uint8_t payload[10] = { 0x00, 0x04, b2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03 };
    uint8_t tx_buf[20];
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

static void hiworld_send_climate(const vehicle_climate_t *climate) {
    uint8_t payload[12] = {0};
    
    if (climate->power_on)         payload[0] |= 0x40;
    if (climate->ac_max)           payload[0] |= 0x20;
    if (climate->dual_mode)        payload[0] |= 0x04; // Hiworld Dual mode is Bit 2 (0x04)
    if (climate->auto_mode)        payload[0] |= 0x08;
    if (climate->ac_on)            payload[0] |= 0x01;

    payload[1] = 0x08; // Baseline AQS auto as observed from real Canbox
    if (climate->recirculate)      payload[1] |= 0x10;
    if (climate->aqs_auto)         payload[1] |= 0x08;

    if (climate->rear_defrost)      payload[2] |= 0x20;
    if (climate->front_max_defrost) payload[2] |= 0x10;

    payload[3] = 0x03; // Auto blower intensity level (matches OEM Canbox)
    payload[4] = (uint8_t)(((climate->pass_wind_mode & 0x0F) << 4) | (climate->driver_wind_mode & 0x0F));
    payload[5] = climate->fan_speed;
    payload[6] = climate->temp_driver;
    payload[7] = climate->temp_passenger;

    payload[8] = 0x00;
    payload[9] = 0x00;
    payload[10] = 0x00;
    payload[11] = (climate->outdoor_temp_raw != 0) ? climate->outdoor_temp_raw : 0x78;

    uint8_t tx_buf[20];
    size_t len = proto_hiworld_serialize(HIWORLD_CMD_CAR_AC_STATE, payload, sizeof(payload), 
                                         tx_buf, sizeof(tx_buf));
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
    .send_climate = hiworld_send_climate,
    .send_telemetry = hiworld_send_telemetry,
    .send_heartbeat = hiworld_send_heartbeat,
};
