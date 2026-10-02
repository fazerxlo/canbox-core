#ifndef PEUGEOT_407_H
#define PEUGEOT_407_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "core/can_router.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PSA CAN Identifiers (Comfort CAN @ 125 kbps) */
#define PSA_CAN_ID_REVERSE_IGNITION 0x036
#define PSA_CAN_ID_FAST_DYNAMIC     0x0B6
#define PSA_CAN_ID_RADAR_0E1        0x0E1
#define PSA_CAN_ID_STEERING_ANGLE   0x0E6
#define PSA_CAN_ID_BSI_SLOW_DATA    0x0F6
#define PSA_CAN_ID_STALK_BUTTONS    0x0F6
#define PSA_CAN_ID_ALERT_JOURNAL    0x120
#define PSA_CAN_ID_ALERT_QUERY      0x39B
#define PSA_CAN_ID_BSI_GAUGES       0x161
#define PSA_CAN_ID_FUEL_RANGE_TEMP  0x165
#define PSA_CAN_ID_ALERTS_INDICATORS 0x168
#define PSA_CAN_ID_JBL_AMPLIFIER    0x1A0
#define PSA_CAN_ID_ALERT_MESSAGE    0x1A1
#define PSA_CAN_ID_TRIP1            0x1A5
#define PSA_CAN_ID_CRUISE_CONTROL   0x1A8
#define PSA_CAN_ID_CLIMATE_HVAC     0x1D0
#define PSA_CAN_ID_DOORS_BODY_220   0x220
#define PSA_CAN_ID_DOORS_BODY       0x220
#define PSA_CAN_ID_TRIP_INSTANT     0x221
#define PSA_CAN_ID_REAR_RADAR_AAS   0x260
#define PSA_CAN_ID_TRIP2_ODB        0x261
#define PSA_CAN_ID_FRONT_RADAR_AAS  0x270
#define PSA_CAN_ID_TRIP1_ODB        0x2A1
#define PSA_CAN_ID_TRIP2            0x2A5
#define PSA_CAN_ID_TPMS_STATUS_1E1  0x1E1
#define PSA_CAN_ID_TPMS_DIRECT_361  0x361
#define PSA_CAN_ID_TPMS_PRESSURES_3A1 0x3A1
#define PSA_CAN_ID_RDS_NAME         0x396
#define PSA_CAN_ID_CD_CHANGER       0x3A6

/* --------------------------------------------------------------------------
 * 1.1 Steering Column Stalk & Buttons
 * -------------------------------------------------------------------------- */
typedef enum {
    PSA_STALK_KEY_VOL_UP       = 0x14,
    PSA_STALK_KEY_VOL_DOWN     = 0x15,
    PSA_STALK_KEY_PREV         = 0x17,
    PSA_STALK_KEY_NEXT         = 0x18,
    PSA_STALK_KEY_SRC          = 0x11,
    PSA_STALK_KEY_OK           = 0x19,
    PSA_STALK_KEY_DARK         = 0x20,
    PSA_STALK_KEY_TEL_HANGUP   = 0x31,
    PSA_STALK_KEY_TEL_ANSWER   = 0x32,
    PSA_STALK_KEY_SCROLL_UP    = 0x42,
    PSA_STALK_KEY_SCROLL_DOWN  = 0x43,
    PSA_STALK_KEY_MENU         = 0x54,
    PSA_STALK_KEY_ESC          = 0x60
} psa_stalk_key_id_t;

typedef struct {
    uint8_t prev_b0;
    uint8_t prev_b1;
    uint8_t prev_b2;
} psa_stalk_state_t;

typedef void (*psa_stalk_key_callback_t)(uint8_t key_id, uint8_t state);

void psa_stalk_init(psa_stalk_state_t *st);
void psa_stalk_process_can_ex(psa_stalk_state_t *st, const uint8_t *data, uint8_t dlc, psa_stalk_key_callback_t send_key);
void psa_stalk_process_can(const uint8_t *data, uint8_t dlc, psa_stalk_key_callback_t send_key);
size_t build_raise_stalk_key(uint8_t key_id, uint8_t state, uint8_t *out_buf, size_t max_len);

/* --------------------------------------------------------------------------
 * 1.2 Dual-Zone Climate Control (HVAC)
 * -------------------------------------------------------------------------- */
typedef struct {
    bool    power;
    bool    ac_compressor;
    bool    recirculation;
    bool    aqs_auto;
    bool    auto_mode;
    bool    dual_mode;
    bool    rear_defrost;
    bool    front_max_defrost;
    bool    ac_max;
    uint8_t fan_speed;       /* 0..8 */
    uint8_t driver_temp_raw; /* 0x00=LO, 0xFF=HI, 28..60 in 0.5 deg C */
    uint8_t pass_temp_raw;   /* 0x00=LO, 0xFF=HI, 28..60 in 0.5 deg C */
    bool    driver_wind_up;
    bool    driver_wind_face;
    bool    driver_wind_down;
    bool    pass_wind_up;
    bool    pass_wind_face;
    bool    pass_wind_down;
} hvac_state_t;

typedef void (*canbox_uart_tx_fn)(const uint8_t *buf, size_t len);

#define HIWORLD_CMD_CAR_AC_STATE 0x31

void psa_decode_hvac_0x1d0(const uint8_t *data, uint8_t dlc, hvac_state_t *st);

void psa_hvac_process_can_0x1d0(vehicle_climate_t *climate, const uint8_t *data, uint8_t dlc);
void psa_hvac_process_can_0x1e3(vehicle_climate_t *climate, const uint8_t *data, uint8_t dlc);
void psa_hvac_process_can_0x12d(vehicle_climate_t *climate, const uint8_t *data, uint8_t dlc);
size_t build_raise_hvac_packet(const hvac_state_t *st, uint8_t *out_buf, size_t max_len);

/* --------------------------------------------------------------------------
 * 1.3 Ultrasonic Parking Sensors (Front & Rear AAS)
 * -------------------------------------------------------------------------- */
#define PSA_RADAR_DIST_OVER_120CM 0x00
#define PSA_RADAR_DIST_90_120CM   0x01
#define PSA_RADAR_DIST_60_90CM    0x02
#define PSA_RADAR_DIST_30_60CM    0x03
#define PSA_RADAR_DIST_UNDER_30CM 0x04
#define PSA_RADAR_DIST_INACTIVE   0xFF

void psa_decode_aas_rear_0x260(const uint8_t *data, uint8_t dlc, uint8_t *rl, uint8_t *rc, uint8_t *rr);
void psa_decode_aas_front_0x270(const uint8_t *data, uint8_t dlc, uint8_t *fl, uint8_t *fc, uint8_t *fr);
size_t build_raise_rear_radar(uint8_t rl, uint8_t rc, uint8_t rr,
                              uint8_t fl, uint8_t fc, uint8_t fr,
                              uint8_t *out_buf, size_t max_len);
size_t build_raise_front_radar(uint8_t fl, uint8_t fc, uint8_t fr,
                               uint8_t *out_buf, size_t max_len);

#ifndef HIWORLD_CMD_RADAR_STATE
#define HIWORLD_CMD_RADAR_STATE  0x41
#endif

typedef struct {
    uint8_t rear_left_outer;
    uint8_t rear_left_center;
    uint8_t rear_right_center;
    uint8_t rear_right_outer;
    uint8_t front_left_outer;
    uint8_t front_left_center;
    uint8_t front_right_center;
    uint8_t front_right_outer;
    bool    rear_active;
    bool    front_active;
    bool    display_active;
    bool    system_fault;
} psa_radar_state_t;

typedef struct {
    psa_radar_state_t state;
    canbox_uart_tx_fn uart_tx;
} psa_radar_ctx_t;

void psa_radar_init(psa_radar_ctx_t *ctx, canbox_uart_tx_fn uart_tx);
void psa_radar_send_hiworld(psa_radar_ctx_t *ctx);
uint8_t psa_radar_map_zone(uint8_t raw3bit);
void psa_radar_process_can_0x0e1(psa_radar_ctx_t *ctx, const uint8_t *data, uint8_t dlc);
void psa_radar_process_can_0x260(psa_radar_ctx_t *ctx, const uint8_t *data, uint8_t dlc);
void psa_radar_process_can_0x270(psa_radar_ctx_t *ctx, const uint8_t *data, uint8_t dlc);
size_t build_hiworld_radar(const psa_radar_state_t *radar, uint8_t *out, size_t max_len);

/* --------------------------------------------------------------------------
 * 1.4 Trip Computer & Engine Telemetry
 * -------------------------------------------------------------------------- */
void psa_decode_trip_0x165(const uint8_t *data, uint8_t dlc,
                           uint16_t *instant_fuel, uint16_t *range_dte,
                           uint16_t *dest_dist, uint8_t *ext_temp_raw);
void psa_decode_trip1_0x1a5(const uint8_t *data, uint8_t dlc,
                            uint16_t *dist, uint16_t *avg_fuel, uint16_t *avg_speed);
void psa_decode_trip2_0x2a5(const uint8_t *data, uint8_t dlc,
                            uint16_t *dist, uint16_t *avg_fuel, uint16_t *avg_speed);

size_t build_raise_instant_fuel(uint16_t instant_fuel_dkl, uint16_t dte_range_km,
                                uint16_t dest_dist_km, uint8_t *out, size_t max_len);
size_t build_raise_trip1(uint16_t avg_fuel_dkl, uint16_t avg_spd_kmh,
                         uint16_t dist_dkm, uint8_t *out);
size_t build_raise_trip2(uint16_t avg_fuel_dkl, uint16_t avg_spd_kmh,
                         uint16_t dist_dkm, uint8_t *out);
size_t build_raise_outside_temp(uint8_t temp_raw, uint8_t *out, size_t max_len);
size_t build_raise_reverse_state(bool reverse_active, uint8_t *out, size_t max_len);
void psa_decode_reverse_0x036(const uint8_t *data, uint8_t dlc, bool *reverse_active);
void psa_decode_reverse_0x0f6(const uint8_t *data, uint8_t dlc, bool *reverse_active);
void psa_reverse_set_hardware_trigger(bool reverse_active);

#ifndef HIWORLD_SOF1
#define HIWORLD_SOF1        0x5A
#define HIWORLD_SOF2        0xA5
#endif
#define HIWORLD_CMD_ECU_P0  0x13
#define HIWORLD_CMD_ECU_P1  0x14
#define HIWORLD_CMD_ECU_P2  0x15

typedef struct {
    uint16_t rpm;
    uint16_t speed_kmh;
    int8_t   coolant_c;
    int8_t   ambient_c;
    bool     reverse_active;
    
    uint16_t instant_fuel_deci; /* 0.1 L/100km */
    uint16_t range_km;          /* Distance to Empty */
    uint16_t dest_dist_km;      /* Remaining Destination Distance / Target Mileage */
    
    uint16_t trip1_avg_fuel;    /* 0.1 L/100km */
    uint8_t  trip1_avg_speed;   /* km/h */
    uint16_t trip1_distance_km; /* km */

    uint16_t trip2_avg_fuel;    /* 0.1 L/100km */
    uint8_t  trip2_avg_speed;   /* km/h */
    uint16_t trip2_distance_km; /* km */
} psa_trip_state_t;

typedef struct {
    psa_trip_state_t  state;
    canbox_uart_tx_fn uart_tx;
} psa_trip_ctx_t;

void psa_trip_init(psa_trip_ctx_t *ctx, canbox_uart_tx_fn uart_tx);
void psa_trip_send_instant(psa_trip_ctx_t *ctx);
void psa_trip_send_trip1(psa_trip_ctx_t *ctx);
void psa_trip_send_trip2(psa_trip_ctx_t *ctx);

size_t build_hiworld_trip_instant(const psa_trip_state_t *trip, uint8_t *out, size_t max_len);
size_t build_hiworld_trip1(const psa_trip_state_t *trip, uint8_t *out, size_t max_len);
size_t build_hiworld_trip2(const psa_trip_state_t *trip, uint8_t *out, size_t max_len);

void psa_trip_process_can_0x0b6(psa_trip_ctx_t *ctx, const uint8_t *data, uint8_t dlc);
void psa_trip_process_can_0x221(psa_trip_ctx_t *ctx, const uint8_t *data, uint8_t dlc);
void psa_trip_process_can_0x2a1(psa_trip_ctx_t *ctx, const uint8_t *data, uint8_t dlc);
void psa_trip_process_can_0x261(psa_trip_ctx_t *ctx, const uint8_t *data, uint8_t dlc);
void psa_trip_process_can_0x0f6(psa_trip_ctx_t *ctx, const uint8_t *data, uint8_t dlc);
bool build_psa_trip_reset_frame(uint8_t trip_index, can_frame_t *out_frame);
hal_status_t psa_trip_send_reset(uint8_t trip_index);

/* --------------------------------------------------------------------------
 * 1.5 Doors & Body Status
 * -------------------------------------------------------------------------- */
#ifndef HIWORLD_CMD_DOOR
#define HIWORLD_CMD_DOOR    0x12
#endif

typedef struct {
    bool driver_door;
    bool pass_door;
    bool rear_left_door;
    bool rear_right_door;
    bool trunk;
    bool hood;
    bool handbrake;
    bool rear_window;
    bool fuel_flap;
    bool auto_rear_wiper;
    bool auto_locking;
    bool parking_radar_enabled;
} psa_doors_body_t;

typedef struct {
    bool door_front_left;
    bool door_front_right;
    bool door_rear_left;
    bool door_rear_right;
    bool trunk_open;
    bool hood_open;
    bool handbrake_pulled;
    bool rear_window_open;
    bool fuel_flap_open;
    bool auto_rear_wiper_active;
    bool auto_locking_active;
    bool parking_radar_enabled;
} psa_doors_state_t;

typedef struct {
    psa_doors_state_t state;
    canbox_uart_tx_fn uart_tx;
} psa_doors_ctx_t;

void psa_doors_init(psa_doors_ctx_t *ctx, canbox_uart_tx_fn uart_tx);
void psa_doors_send_hiworld(psa_doors_ctx_t *ctx);
void psa_doors_process_can_0x220(psa_doors_ctx_t *ctx, const uint8_t *data, uint8_t dlc);

void psa_decode_doors_0x220(const uint8_t *data, uint8_t dlc, psa_doors_body_t *doors);
void psa_decode_doors_0x221(const uint8_t *data, uint8_t dlc, psa_doors_body_t *doors);
size_t build_raise_doors(const psa_doors_body_t *doors, uint8_t *out, size_t max_len);
size_t build_hiworld_doors(const psa_doors_body_t *doors, uint8_t *out, size_t max_len);
size_t build_hiworld_doors_state(const psa_doors_state_t *state, uint8_t *out, size_t max_len);

/* --------------------------------------------------------------------------
 * 1.6 Steering Wheel Angle & Dynamic Trajectory
 * -------------------------------------------------------------------------- */
void psa_decode_steering_angle_0x0e6(const uint8_t *data, uint8_t dlc, int16_t *angle_deci_deg);
size_t build_raise_steering_angle(int16_t angle_deci_deg, uint8_t *out);

/* --------------------------------------------------------------------------
 * 1.7 OEM JBL Sound Amplifier (DSP)
 * -------------------------------------------------------------------------- */
typedef struct {
    uint8_t bass;           /* 0..14, Neutral=7 (-7..0..+7) */
    uint8_t treble;         /* 0..14, Neutral=7 (-7..0..+7) */
    uint8_t balance;        /* 0..14, 7=Center, <7=Left, >7=Right */
    uint8_t fader;          /* 0..14, 7=Center, <7=Rear, >7=Front */
    uint8_t eq_preset;      /* 0=Custom/Off, 1=Pop, 2=Classic, 3=Electronic, 4=Jazz, 5=Vocal */
    bool    loudness;       /* Bit 4: 0x10 */
    uint8_t speed_vol_comp; /* Bits 3..0: 0..3 */
    uint8_t master_volume;  /* 0..30 */
} jbl_amplifier_state_t;

void psa_decode_amplifier_0x1a0(const uint8_t *data, uint8_t dlc, jbl_amplifier_state_t *amp);
size_t build_raise_amplifier(const jbl_amplifier_state_t *st, uint8_t *out, size_t max_len);

/* --------------------------------------------------------------------------
 * 1.8 RD4 Radio & CD Changer Media Data
 * -------------------------------------------------------------------------- */
typedef struct {
    uint8_t disc_slot;      /* 1..6 */
    uint8_t track_num;      /* 1..99 */
    uint8_t total_tracks;   /* 1..99 */
    uint8_t elapsed_min;    /* 0..59 */
    uint8_t elapsed_sec;    /* 0..59 */
    uint8_t play_flags;     /* 0x01=Random, 0x02=Scan, 0x04=Repeat */
} cd_changer_state_t;

void psa_decode_cd_changer_0x3a6(const uint8_t *data, uint8_t dlc, cd_changer_state_t *cdc);
size_t build_raise_cd_changer(const cd_changer_state_t *st, uint8_t *out, size_t max_len);

void psa_decode_rds_name_0x396(const uint8_t *data, uint8_t dlc, char out_name[9]);
size_t build_raise_rds_name(const char *name, uint8_t *out, size_t max_len);

/* --------------------------------------------------------------------------
 * 2.1 Direct TPMS Numeric Readings & Fault Classification
 * -------------------------------------------------------------------------- */
#define HIWORLD_CMD_TPMS_NUMERIC  0x66
#define HIWORLD_CMD_TPMS_DISCRETE 0x18

typedef struct {
    uint8_t fl_press_bar_deci; /* 0.1 Bar: 24 = 2.4 Bar */
    uint8_t fr_press_bar_deci;
    uint8_t rl_press_bar_deci;
    uint8_t rr_press_bar_deci;
    uint8_t fl_state;          /* 0=OK, 1=LOW, 2=PUNCTURE, 3=FAULT */
    uint8_t fr_state;
    uint8_t rl_state;
    uint8_t rr_state;
} psa_tpms_state_t;

typedef struct {
    psa_tpms_state_t  state;
    canbox_uart_tx_fn uart_tx;
} psa_tpms_ctx_t;

void psa_tpms_init(psa_tpms_ctx_t *ctx, canbox_uart_tx_fn uart_tx);
void psa_tpms_send_numeric(psa_tpms_ctx_t *ctx);
void psa_tpms_send_discrete(psa_tpms_ctx_t *ctx);
void psa_tpms_process_can_0x361(psa_tpms_ctx_t *ctx, const uint8_t *data, uint8_t dlc);
void psa_tpms_process_can_0x3a1(psa_tpms_ctx_t *ctx, const uint8_t *data, uint8_t dlc);
void psa_tpms_process_can_0x1e1(psa_tpms_ctx_t *ctx, const uint8_t *data, uint8_t dlc);

size_t build_hiworld_tpms_numeric(const psa_tpms_state_t *tpms, uint8_t *out, size_t max_len);
size_t build_hiworld_tpms_discrete(const psa_tpms_state_t *tpms, uint8_t *out, size_t max_len);

typedef struct {
    uint16_t pressure_dbar[4]; /* FL, FR, RL, RR in 0.1 Bar */
    int16_t  temperature_c[4]; /* FL, FR, RL, RR in deg C (-40..+215) */
    uint8_t  alarm_code[4];    /* 0=OK, 1=Low, 2=Puncture, 3=Lost, 4=LowBat */
} canbox_tpms_state_t;

size_t build_raise_tpms_numeric(const canbox_tpms_state_t *tpms, uint8_t *out);
size_t build_raise_tpms_temp_alarms(const canbox_tpms_state_t *tpms, uint8_t *out);
size_t build_raise_tpms_discrete(const uint8_t alarms[4], uint8_t *out);

/* --------------------------------------------------------------------------
 * 2.2 Stop & Start (S&S) Telemetry & Timer
 * -------------------------------------------------------------------------- */
size_t build_raise_start_stop(bool is_active, uint32_t stop_time_sec, uint8_t *out);

/* --------------------------------------------------------------------------
 * 2.3 Cruise Control & Speed Memory Presets
 * -------------------------------------------------------------------------- */
size_t build_raise_cruise_memory(bool active, uint8_t target_spd, const uint8_t presets[5], uint8_t *out);
void psa_decode_cruise_0x1a8(const uint8_t *data, uint8_t dlc, bool *active, uint8_t *set_speed_kmh, uint32_t *partial_odo_m);

/* --------------------------------------------------------------------------
 * 2.4 Driver Assistance & ADAS Features
 * -------------------------------------------------------------------------- */
typedef struct {
    bool    blind_spot_warning;
    bool    fatigue_coffee_cup;
    uint8_t lane_departure_state; /* 0=None, 1=Left, 2=Right */
    uint8_t speed_limit_tsr;      /* km/h */
    bool    esp_active;
    uint8_t aeb_risk_level;       /* 0=None, 1=Risk, 2=Braking */
} canbox_adas_state_t;

size_t build_raise_adas(const canbox_adas_state_t *adas, uint8_t *out);
void psa_decode_alerts_0x168(const uint8_t *data, uint8_t dlc,
                             bool *tpms_fault, bool *tpms_underinflation,
                             bool *tpms_puncture, bool *esp_fault);

/* --------------------------------------------------------------------------
 * 2.5 Vehicle Alerts & Diagnostic Journal (Hiworld Command 0x42)
 * -------------------------------------------------------------------------- */
#define HIWORLD_CMD_WARNING_INFO     0x42
#define HIWORLD_CMD_DIAGNOSTIC_QUERY 0x2F

#define PSA_JOURNAL_PAYLOAD_BYTES    21
#define PSA_JOURNAL_TOTAL_BITS       168

/* Canonical Hiworld Alert Presets (mOriginalType) */
#define PSA_HIWORLD_ALERT_GENERAL             0x0000
#define PSA_HIWORLD_ALERT_LOW_FUEL            0x0001
#define PSA_HIWORLD_ALERT_ENGINE_TEMP         0x0003
#define PSA_HIWORLD_ALERT_OIL_PRESSURE        0x0004
#define PSA_HIWORLD_ALERT_BRAKE_FLUID         0x0005
#define PSA_HIWORLD_ALERT_HANDBRAKE           0x0008
#define PSA_HIWORLD_ALERT_KEY_BATTERY         0x000A
#define PSA_HIWORLD_ALERT_DIRECTIONAL_LIGHTS  0x000B
#define PSA_HIWORLD_ALERT_BATTERY_CHARGE      0x000D
#define PSA_HIWORLD_ALERT_ESP                 0x000F
#define PSA_HIWORLD_ALERT_DOOR_FL             0x0011
#define PSA_HIWORLD_ALERT_DOOR_FR             0x0012
#define PSA_HIWORLD_ALERT_DOOR_REAR           0x0013
#define PSA_HIWORLD_ALERT_BOOT                0x0014
#define PSA_HIWORLD_ALERT_SERVICE_DUE         0x0061
#define PSA_HIWORLD_ALERT_DPF                 0x0064
#define PSA_HIWORLD_ALERT_GEARBOX             0x0067
#define PSA_HIWORLD_ALERT_ANTIPOLLUTION       0x0068
#define PSA_HIWORLD_ALERT_ABS                 0x0069
#define PSA_HIWORLD_ALERT_EBD                 0x006A
#define PSA_HIWORLD_ALERT_SUSPENSION_90KMH    0x006B
#define PSA_HIWORLD_ALERT_SUSPENSION_SYSTEM   0x006C
#define PSA_HIWORLD_ALERT_AUTO_LIGHTS         0x0081
#define PSA_HIWORLD_ALERT_AUTO_WIPERS         0x0083
#define PSA_HIWORLD_ALERT_TPMS_UNDER_FL       0x009A
#define PSA_HIWORLD_ALERT_TPMS_UNDER_FR       0x009B
#define PSA_HIWORLD_ALERT_TPMS_UNDER_RR       0x009C
#define PSA_HIWORLD_ALERT_TPMS_UNDER_RL       0x009D
#define PSA_HIWORLD_ALERT_TPMS_PUNCTURE_FL    0x009E
#define PSA_HIWORLD_ALERT_TPMS_PUNCTURE_FR    0x009F
#define PSA_HIWORLD_ALERT_TPMS_PUNCTURE_REAR  0x00A0
#define PSA_HIWORLD_ALERT_BULB_SIDELIGHT      0x00E3
#define PSA_HIWORLD_ALERT_BULB_DIPPED         0x00E5
#define PSA_HIWORLD_ALERT_BULB_BRAKE          0x00E7
#define PSA_HIWORLD_ALERT_BULB_REVERSE        0x00E8
#define PSA_HIWORLD_ALERT_AIRBAG              0x00F0
#define PSA_HIWORLD_ALERT_SEATBELT_FL         0x012F
#define PSA_HIWORLD_ALERT_SEATBELT_FR         0x0130
#define PSA_HIWORLD_ALERT_IMMOBILIZER         0x01FA

typedef struct {
    uint8_t buffer[PSA_JOURNAL_PAYLOAD_BYTES];
    uint8_t bytes_rx;
    uint8_t expected_seq;
    bool    in_progress;
    uint8_t blocks_received;
} psa_journal_iso_tp_t;

void psa_journal_iso_tp_init(psa_journal_iso_tp_t *ctx);
uint16_t psa_can_alarm_id_to_hiworld_code(uint16_t can_alarm_id, uint8_t door_mask, uint8_t param);
void psa_decode_alert_message_0x1a1(const uint8_t *data, uint8_t dlc, vehicle_alert_item_t *alert);
bool psa_process_journal_0x120(psa_journal_iso_tp_t *ctx, const uint8_t *data, uint8_t dlc,
                               uint16_t *out_codes, uint8_t *out_count);
size_t build_hiworld_alert_single(uint16_t alert_code, uint8_t *out, size_t max_len);
size_t build_hiworld_alerts_summary(const uint16_t *codes, uint8_t count, uint8_t *out, size_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* PEUGEOT_407_H */

