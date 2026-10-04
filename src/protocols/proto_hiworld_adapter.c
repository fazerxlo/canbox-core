#include "protocols/hu_protocol.h"
#include "protocols/hu_protocol_driver.h"
#include "protocols/hiworld_connection.h"
#include "protocols/hiworld_car_mapping.h"
#include "proto_hiworld.h"
#include "core/can_router.h"
#include "hal/hal_uart.h"
#include "hal/hal_can.h"
#include <stdbool.h>
#include <string.h>

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
    if (!packet) return;

    if (packet->cmd == HIWORLD_CMD_ECU_SETTING_SET) {
        /* ForwardEcuSetting:
         * 4-byte variant (standard Android QF/Hiworld APK):
         *   payload[0] = Active Page (1=Instant, 2=Trip1, 3=Trip2)
         *   payload[1] = Reset Target (0=None/Navigation only, 2=Trip1 Reset, 3=Trip2 Reset)
         *   payload[2] = 0x01 (Command validity)
         *   payload[3] = 0xFF (Parameter mask)
         * 2-byte variant:
         *   payload[0] = Trip Page (1=Trip1, 2=Trip2 / 2=Trip1, 3=Trip2)
         *   payload[1] = Action (0=None, 1=Reset)
         */
        if (packet->payload_len >= 4) {
            uint8_t reset_target = packet->payload[1];
            if (reset_target == 0x02) {
                /* Hiworld EcuInfoPage2 (Cmd 0x14) = Trip 1 */
                can_router_reset_trip(1);
            } else if (reset_target == 0x03) {
                /* Hiworld EcuInfoPage3 (Cmd 0x15) = Trip 2 */
                can_router_reset_trip(2);
            }
            /* reset_target == 0x00: tab navigation only, do NOT trigger reset */
        } else if (packet->payload_len >= 2) {
            uint8_t page = packet->payload[0];
            uint8_t action = packet->payload[1];
            if (action == 0x01) {
                if (page == 0x03) {
                    can_router_reset_trip(2);
                } else if (page == 0x01 || page == 0x02) {
                    can_router_reset_trip(1);
                }
            }
        }
    } else if (packet->cmd == HIWORLD_CMD_DIAGNOSTIC_QUERY) {
        /* Head Unit requested alert/diagnostic log refresh (forwardType 0x2F) */
        can_router_query_alert_journal();
    }
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

    payload[1] = 0x00;
    if (climate->recirculate) {
        payload[1] |= 0x10;
    } else if (climate->aqs_auto) {
        payload[1] |= 0x08;
    }

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
    payload[11] = (climate->outdoor_temp_raw != 0) ? climate->outdoor_temp_raw : 0xFF;

    uint8_t tx_buf[20];
    size_t len = proto_hiworld_serialize(HIWORLD_CMD_CAR_AC_STATE, payload, sizeof(payload), 
                                         tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

static void hiworld_send_tpms_numeric(const vehicle_tpms_t *tpms) {
    if (!tpms || !tpms->valid) return;

    /* Hiworld Numeric TPMS (Cmd 0x66): Mode(0x01), FL, FR, RL, RR, Unit(0x00=Bar) */
    uint8_t num_payload[6];
    num_payload[0] = 0x01; /* Live mode */
    num_payload[1] = tpms->pressure_bar_deci[0]; /* FL */
    num_payload[2] = tpms->pressure_bar_deci[1]; /* FR */
    num_payload[3] = tpms->pressure_bar_deci[2]; /* RL */
    num_payload[4] = tpms->pressure_bar_deci[3]; /* RR */
    num_payload[5] = 0x00; /* Bar */

    uint8_t tx_buf[16];
    size_t len = proto_hiworld_serialize(HIWORLD_CMD_TPMS_NUMERIC, num_payload, sizeof(num_payload),
                                         tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

static void hiworld_send_tpms_discrete(const vehicle_tpms_t *tpms) {
    if (!tpms || !tpms->valid) return;

    /* Hiworld Discrete TPMS Alarm (Cmd 0x18): FL, FR, RL, RR alarm states */
    uint8_t disc_payload[4];
    disc_payload[0] = tpms->alarm_state[0]; /* FL */
    disc_payload[1] = tpms->alarm_state[1]; /* FR */
    disc_payload[2] = tpms->alarm_state[2]; /* RL */
    disc_payload[3] = tpms->alarm_state[3]; /* RR */

    uint8_t tx_buf[16];
    size_t len = proto_hiworld_serialize(HIWORLD_CMD_TPMS_DISCRETE, disc_payload, sizeof(disc_payload),
                                         tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

static void hiworld_send_tpms(const vehicle_tpms_t *tpms) {
    hiworld_send_tpms_numeric(tpms);
    hiworld_send_tpms_discrete(tpms);
}

static void hiworld_send_trip_instant(const vehicle_trip_t *trip) {
    if (!trip || !trip->instant_valid) return;

    /* Hiworld Instantaneous Telemetry (Cmd 0x13): 10 payload bytes as captured from real PSA Hiworld Canbox
     * Byte 0..1: Instantaneous Fuel Consumption (0.1 L/100km, Big-Endian)
     * Byte 2..3: Range / DTE (km, Big-Endian)
     * Byte 4..5: Target Mileage / Remaining Destination Distance (Big-Endian)
     * Byte 6..9: Reserved / Padding (0x00)
     */
    uint8_t payload[10] = {0};
    payload[0] = (uint8_t)(trip->instant_fuel_deci >> 8);
    payload[1] = (uint8_t)(trip->instant_fuel_deci & 0xFF);
    payload[2] = (uint8_t)(trip->range_km >> 8);
    payload[3] = (uint8_t)(trip->range_km & 0xFF);
    payload[4] = (uint8_t)(trip->dest_dist_km >> 8);
    payload[5] = (uint8_t)(trip->dest_dist_km & 0xFF);

    uint8_t tx_buf[20];
    size_t len = proto_hiworld_serialize(HIWORLD_CMD_ECU_P0, payload, sizeof(payload),
                                         tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

static void hiworld_send_trip1(const vehicle_trip_t *trip) {
    if (!trip || !trip->trip1_valid) return;

    /* Hiworld Trip 1 Telemetry (Cmd 0x14): 6 payload bytes
     * Byte 0..1: Average Fuel Consumption (0.1 L/100km, Big-Endian)
     * Byte 2: Reserved (0x00)
     * Byte 3: Average Speed (km/h)
     * Byte 4..5: Distance Traveled (km, Big-Endian)
     */
    uint8_t payload[6];
    payload[0] = (uint8_t)(trip->trip1_avg_fuel >> 8);
    payload[1] = (uint8_t)(trip->trip1_avg_fuel & 0xFF);
    payload[2] = 0x00;
    payload[3] = trip->trip1_avg_speed;
    payload[4] = (uint8_t)(trip->trip1_distance_km >> 8);
    payload[5] = (uint8_t)(trip->trip1_distance_km & 0xFF);

    uint8_t tx_buf[16];
    size_t len = proto_hiworld_serialize(HIWORLD_CMD_ECU_P1, payload, sizeof(payload),
                                         tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

static void hiworld_send_trip2(const vehicle_trip_t *trip) {
    if (!trip || !trip->trip2_valid) return;

    /* Hiworld Trip 2 Telemetry (Cmd 0x15): 6 payload bytes
     * Byte 0..1: Average Fuel Consumption (0.1 L/100km, Big-Endian)
     * Byte 2: Reserved (0x00)
     * Byte 3: Average Speed (km/h)
     * Byte 4..5: Distance Traveled (km, Big-Endian)
     */
    uint8_t payload[6];
    payload[0] = (uint8_t)(trip->trip2_avg_fuel >> 8);
    payload[1] = (uint8_t)(trip->trip2_avg_fuel & 0xFF);
    payload[2] = 0x00;
    payload[3] = trip->trip2_avg_speed;
    payload[4] = (uint8_t)(trip->trip2_distance_km >> 8);
    payload[5] = (uint8_t)(trip->trip2_distance_km & 0xFF);

    uint8_t tx_buf[16];
    size_t len = proto_hiworld_serialize(HIWORLD_CMD_ECU_P2, payload, sizeof(payload),
                                         tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

static void hiworld_send_radar(const vehicle_radar_t *radar) {
    if (!radar || !radar->valid) return;

    uint8_t payload[12];
    payload[0] = radar->rear_left_outer;
    payload[1] = radar->rear_left_center;
    payload[2] = radar->rear_right_center;
    payload[3] = radar->rear_right_outer;
    payload[4] = radar->front_left_outer;
    payload[5] = radar->front_left_center;
    payload[6] = radar->front_right_center;
    payload[7] = radar->front_right_outer;
    payload[8] = 0x01; /* Radar active / enabled flag */
    payload[9] = 0x00; /* Reserved */
    payload[10] = 0x3F; /* 6-sensor config mask */
    payload[11] = 0x05; /* Distance scale / max zone steps */

    uint8_t tx_buf[20];
    size_t len = proto_hiworld_serialize(HIWORLD_CMD_CAR_RADAR_STATE, payload, sizeof(payload),
                                         tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

static uint16_t s_cached_alert_codes[CANBOX_MAX_ACTIVE_ALERTS];
static uint8_t  s_cached_alert_count = 0;

static void hiworld_send_alert_single(uint16_t alert_code) {
    uint8_t payload[2];
    payload[0] = (uint8_t)(alert_code >> 8);
    payload[1] = (uint8_t)(alert_code & 0xFF);

    uint8_t tx_buf[16];
    size_t len = proto_hiworld_serialize(HIWORLD_CMD_WARNING_INFO, payload, sizeof(payload),
                                         tx_buf, sizeof(tx_buf));
    if (len > 0) {
        hal_uart_write(tx_buf, len);
    }
}

static void hiworld_send_alerts_summary(const uint16_t *alert_codes, uint8_t count) {
    if (alert_codes != NULL) {
        if (count > CANBOX_MAX_ACTIVE_ALERTS) {
            count = CANBOX_MAX_ACTIVE_ALERTS;
        }
        s_cached_alert_count = count;
        if (count > 0) {
            memcpy(s_cached_alert_codes, alert_codes, count * sizeof(uint16_t));
        }
    } else {
        /* Retransmit cached summary */
        count = s_cached_alert_count;
        alert_codes = s_cached_alert_codes;
    }

    uint8_t payload[24] = {0};
    /* D0..D2: Reserved = 0x00 */
    /* D3: mNumber = count */
    payload[3] = count;

    for (uint8_t i = 0; i < count; i++) {
        uint8_t offset = (uint8_t)(4 + (i * 2));
        payload[offset]     = (uint8_t)(alert_codes[i] >> 8);
        payload[offset + 1] = (uint8_t)(alert_codes[i] & 0xFF);
    }

    uint8_t tx_buf[36];
    size_t len = proto_hiworld_serialize(HIWORLD_CMD_WARNING_INFO, payload, sizeof(payload),
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
    .send_tpms = hiworld_send_tpms,
    .send_tpms_numeric = hiworld_send_tpms_numeric,
    .send_tpms_discrete = hiworld_send_tpms_discrete,
    .send_trip_instant = hiworld_send_trip_instant,
    .send_trip1 = hiworld_send_trip1,
    .send_trip2 = hiworld_send_trip2,
    .send_radar = hiworld_send_radar,
    .send_reverse = NULL,
    .send_alert_single = hiworld_send_alert_single,
    .send_alerts_summary = hiworld_send_alerts_summary,
    .send_heartbeat = hiworld_send_heartbeat,
};
