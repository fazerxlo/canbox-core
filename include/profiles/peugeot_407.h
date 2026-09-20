#ifndef PEUGEOT_407_H
#define PEUGEOT_407_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PSA CAN Identifiers (Comfort CAN @ 125 kbps) */
#define PSA_CAN_ID_REVERSE_IGNITION 0x036
#define PSA_CAN_ID_STEERING_ANGLE   0x0E6
#define PSA_CAN_ID_STALK_BUTTONS    0x0F6
#define PSA_CAN_ID_FUEL_RANGE_TEMP  0x165
#define PSA_CAN_ID_JBL_AMPLIFIER    0x1A0
#define PSA_CAN_ID_TRIP1            0x1A5
#define PSA_CAN_ID_CLIMATE_HVAC     0x1D0
#define PSA_CAN_ID_DOORS_BODY       0x221
#define PSA_CAN_ID_REAR_RADAR_AAS   0x260
#define PSA_CAN_ID_FRONT_RADAR_AAS  0x270
#define PSA_CAN_ID_TRIP2            0x2A5
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

void psa_decode_hvac_0x1d0(const uint8_t *data, uint8_t dlc, hvac_state_t *st);
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

/* --------------------------------------------------------------------------
 * 1.5 Doors & Body Status
 * -------------------------------------------------------------------------- */
typedef struct {
    bool driver_door;
    bool pass_door;
    bool rear_left_door;
    bool rear_right_door;
    bool trunk;
    bool hood;
    bool handbrake;
} psa_doors_body_t;

void psa_decode_doors_0x221(const uint8_t *data, uint8_t dlc, psa_doors_body_t *doors);
size_t build_raise_doors(const psa_doors_body_t *doors, uint8_t *out, size_t max_len);

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

#ifdef __cplusplus
}
#endif

#endif /* PEUGEOT_407_H */

