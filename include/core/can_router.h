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
    WHEEL_KEY_PHONE_REJECT = WHEEL_KEY_PHONE_HANGUP
} wheel_key_t;

typedef wheel_key_t steering_key_t;

typedef struct {
    wheel_key_t active_key;
    uint8_t     press_state; // 0: Released, 1: Pressed
} vehicle_wheel_t;

typedef vehicle_wheel_t wheel_state_t;

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
    bool    auto_mode;
    bool    recirculate;
    uint8_t fan_speed;       // 0 - 7
    uint8_t temp_driver;     // Raw scale: (val * 0.5) deg C
    uint8_t temp_passenger;  // Raw scale: (val * 0.5) deg C
} vehicle_climate_t;

typedef enum {
    VEHICLE_IGNITION_OFF   = 0x00,
    VEHICLE_IGNITION_ON    = 0x01,
    VEHICLE_IGNITION_ACC   = 0x02,
    VEHICLE_IGNITION_CRANK = 0x03
} vehicle_ignition_state_t;

typedef struct {
    vehicle_doors_t          doors;
    vehicle_wheel_t          wheel;
    vehicle_climate_t        climate;
    vehicle_ignition_state_t ignition_state;
    uint16_t                 speed_kmh;
    uint16_t                 rpm;
    int16_t                  steering_angle_deg;
    bool                     reverse_gear;
    bool                     handbrake;
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

#ifdef __cplusplus
}
#endif

#endif /* CAN_ROUTER_H */
