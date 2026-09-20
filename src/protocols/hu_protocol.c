#include "protocols/hu_protocol.h"
#include "protocols/hu_protocol_driver.h"
#include <stddef.h>

extern const hu_protocol_driver_t g_hu_protocol_raise;
extern const hu_protocol_driver_t g_hu_protocol_hiworld;

static const hu_protocol_driver_t *s_available_protocols[] = {
    [HU_PROTOCOL_RAISE]   = &g_hu_protocol_raise,
    [HU_PROTOCOL_HIWORLD] = &g_hu_protocol_hiworld,
    [HU_PROTOCOL_BAGOO]   = NULL,
};

static const hu_protocol_driver_t *s_active_driver = &g_hu_protocol_raise;

void hu_protocol_init(void) {
    if (s_active_driver && s_active_driver->init) {
        s_active_driver->init();
    }
}

bool hu_protocol_set_active(hu_protocol_id_t protocol_id) {
    if (protocol_id >= HU_PROTOCOL_COUNT) {
        return false;
    }
    if (s_available_protocols[protocol_id] == NULL) {
        return false;
    }
    s_active_driver = s_available_protocols[protocol_id];
    if (s_active_driver->init) {
        s_active_driver->init();
    }
    return true;
}

const hu_protocol_driver_t *hu_protocol_get_active(void) {
    return s_active_driver;
}

void hu_protocol_feed_byte(uint8_t byte) {
    if (s_active_driver && s_active_driver->feed_byte) {
        s_active_driver->feed_byte(byte);
    }
}

void hu_protocol_send_wheel_key(const vehicle_wheel_t *wheel) {
    if (s_active_driver && s_active_driver->send_wheel_key) {
        s_active_driver->send_wheel_key(wheel);
    }
}

void hu_protocol_send_doors(const vehicle_doors_t *doors) {
    if (s_active_driver && s_active_driver->send_doors) {
        s_active_driver->send_doors(doors);
    }
}

void hu_protocol_send_telemetry(uint16_t speed, uint16_t rpm, int16_t angle) {
    if (s_active_driver && s_active_driver->send_telemetry) {
        s_active_driver->send_telemetry(speed, rpm, angle);
    }
}

void hu_protocol_send_heartbeat(void) {
    if (s_active_driver && s_active_driver->send_heartbeat) {
        s_active_driver->send_heartbeat();
    }
}

