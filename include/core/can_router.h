#ifndef CAN_ROUTER_H
#define CAN_ROUTER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "hal/hal_can.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WHEEL_KEY_NONE = 0x00,
    WHEEL_KEY_VOL_UP,
    WHEEL_KEY_VOL_DOWN,
    WHEEL_KEY_NEXT,
    WHEEL_KEY_PREV,
    WHEEL_KEY_SRC,
    WHEEL_KEY_MUTE,
    WHEEL_KEY_VOICE,
    WHEEL_KEY_PHONE_ACCEPT,
    WHEEL_KEY_PHONE_HANGUP,
    WHEEL_KEY_PHONE_REJECT = WHEEL_KEY_PHONE_HANGUP,
    WHEEL_KEY_SCROLL_UP,
    WHEEL_KEY_SCROLL_DOWN,
    WHEEL_KEY_TRIP
} wheel_key_t;

typedef wheel_key_t steering_key_t;

typedef struct {
    wheel_key_t active_key;
    uint8_t     press_state; // 0: Released, 1: Pressed
} vehicle_wheel_t;

typedef vehicle_wheel_t wheel_state_t;

typedef struct {
    uint8_t key_code;    // Fascia / Console panel key code (0x00 when idle/released)
    uint8_t press_state; // 0: Released, 1: Pressed
} vehicle_panel_key_t;

typedef vehicle_panel_key_t panel_key_state_t;

typedef struct {
    bool door_driver;
    bool door_passenger;
    bool door_rear_left;
    bool door_rear_right;
    bool trunk;
    bool hood;
} vehicle_doors_t;

typedef vehicle_doors_t door_state_t;

typedef struct {
    bool    power_on;
    bool    ac_on;
    bool    ac_max;
    bool    auto_mode;
    bool    dual_mode;
    bool    recirculate;
    bool    aqs_auto;
    bool    front_max_defrost;
    bool    rear_defrost;
    uint8_t fan_speed;       // 0 - 7 (or up to 15)
    uint8_t driver_wind_mode;
    uint8_t pass_wind_mode;
    uint8_t temp_driver;     // Raw scale for Android
    uint8_t temp_passenger;  // Raw scale for Android
    uint8_t outdoor_temp_raw;
} vehicle_climate_t;

typedef struct {
    bool side_light;      // Side lights / Parking / Position lights
    bool headlights;      // Headlights / Low beam / Dipped beam
    bool high_beam;       // High beam / Main beam
    bool front_fog;       // Front fog lights
    bool rear_fog;        // Rear fog lights
    bool illumination;    // Instrument cluster / interior night illumination active
    uint8_t brightness;   // Cluster / Backlight brightness level (0..15)
} vehicle_lights_t;

typedef vehicle_lights_t lights_state_t;

typedef enum {
    VEHICLE_IGNITION_OFF   = 0x00,
    VEHICLE_IGNITION_ON    = 0x01,
    VEHICLE_IGNITION_ACC   = 0x02,
    VEHICLE_IGNITION_CRANK = 0x03
} vehicle_ignition_state_t;

typedef struct {
    uint8_t pressure_bar_deci[4]; /* 0.1 Bar: FL, FR, RL, RR */
    uint8_t alarm_state[4];       /* 0=OK, 1=Low, 2=Puncture, 3=Fault: FL, FR, RL, RR */
    bool    valid;
} vehicle_tpms_t;

typedef vehicle_tpms_t tpms_state_t;

typedef struct {
    uint16_t instant_fuel_deci; /* 0.1 L/100km */
    uint16_t range_km;          /* Distance to Empty */
    uint16_t dest_dist_km;      /* Remaining Destination Distance / Target Mileage */
    
    uint16_t trip1_avg_fuel;    /* 0.1 L/100km */
    uint8_t  trip1_avg_speed;   /* km/h */
    uint16_t trip1_distance_km; /* km */

    uint16_t trip2_avg_fuel;    /* 0.1 L/100km */
    uint8_t  trip2_avg_speed;   /* km/h */
    uint16_t trip2_distance_km; /* km */

    uint8_t  updated_page;      /* 0: none, 1: instant (0x13), 2: trip1 (0x14), 3: trip2 (0x15) */
    bool     instant_valid;
    bool     trip1_valid;
    bool     trip2_valid;
} vehicle_trip_t;

typedef vehicle_trip_t trip_state_t;

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
    bool    valid;
    bool    updated;
} vehicle_radar_t;

typedef vehicle_radar_t radar_state_t;

#define CANBOX_MAX_ACTIVE_ALERTS 10

typedef struct {
    uint16_t alert_code;    /* 16-bit Hiworld alert code (mOriginalType) */
    uint16_t can_alarm_id;  /* 15-bit PSA CAN alarm ID */
    uint8_t  severity;      /* 0=Info, 1=Minor, 2=Service, 3=Stop */
    uint8_t  chime_id;      /* Acoustic chime index (0..15) */
    uint8_t  door_mask;     /* Door / opening bitfield */
    uint8_t  param_detail;  /* Parameter detail (wheel index, bulb index, etc.) */
    bool     is_active;     /* true if alert currently triggered */
    bool     display_req;   /* true if modal dialog popup requested */
} vehicle_alert_item_t;

typedef struct {
    vehicle_alert_item_t realtime_alert;
    vehicle_alert_item_t active_items[CANBOX_MAX_ACTIVE_ALERTS];
    uint8_t              active_count;     /* 0..10 */
    bool                 realtime_updated; /* Single alert state changed */
    bool                 journal_updated;  /* Summary list changed */
} vehicle_alerts_t;

typedef vehicle_alerts_t alerts_state_t;

typedef struct {
    vehicle_doors_t          doors;
    vehicle_wheel_t          wheel;
    vehicle_panel_key_t      panel_key;
    vehicle_climate_t        climate;
    vehicle_lights_t         lights;
    vehicle_tpms_t           tpms;
    vehicle_trip_t           trip;
    vehicle_radar_t          radar;
    vehicle_alerts_t         alerts;
    vehicle_ignition_state_t ignition_state;
    uint16_t                 speed_kmh;
    uint16_t                 rpm;
    int16_t                  steering_angle_deg;
    bool                     reverse_gear;
    bool                     handbrake;
    bool                     economy_mode;
} vehicle_state_t;

typedef void (*can_msg_handler_t)(const can_frame_t *frame, vehicle_state_t *state);

typedef struct {
    uint32_t          can_id;
    can_msg_handler_t handler;
} can_router_rule_t;

void can_router_init(void);
void can_router_process_can(const can_frame_t *frame);
void can_router_process_uart_byte(uint8_t byte);
void can_router_periodic_100ms(void);
const vehicle_state_t *can_router_get_state(void);
bool can_router_reset_trip(uint8_t trip_index);
bool can_router_query_alert_journal(void);
bool can_router_is_bus_sleeping(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_ROUTER_H */
