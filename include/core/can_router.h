#ifndef CAN_ROUTER_H
#define CAN_ROUTER_H

#include <stdint.h>
#include <stdbool.h>
#include "hal/hal_can.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WHEEL_KEY_NONE = 0,
    WHEEL_KEY_VOL_UP,
    WHEEL_KEY_VOL_DOWN,
    WHEEL_KEY_NEXT,
    WHEEL_KEY_PREV,
    WHEEL_KEY_SRC,
    WHEEL_KEY_MUTE,
    WHEEL_KEY_PHONE_ACCEPT,
    WHEEL_KEY_PHONE_HANGUP,
    WHEEL_KEY_VOICE
} wheel_key_t;

typedef struct {
    wheel_key_t active_key;
    uint8_t     press_state;
} wheel_state_t;

typedef struct {
    bool door_driver;
    bool door_passenger;
    bool door_rear_left;
    bool door_rear_right;
    bool trunk;
    bool hood;
} door_state_t;

typedef struct {
    wheel_state_t wheel;
    door_state_t  doors;
    bool          reverse_gear;
    bool          handbrake;
    uint16_t      rpm;
    uint16_t      speed_kmh;
    int16_t       steering_angle_deg;
} vehicle_state_t;

void can_router_init(void);
void can_router_process_can(const can_frame_t *frame);
void can_router_process_uart_byte(uint8_t byte);
void can_router_periodic_100ms(void);
const vehicle_state_t *can_router_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_ROUTER_H */

