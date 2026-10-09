#include "protocols/hu_protocol.h"
#include "protocols/hu_protocol_driver.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

extern const hu_protocol_driver_t g_hu_protocol_raise;
extern const hu_protocol_driver_t g_hu_protocol_hiworld;
extern const hu_protocol_driver_t g_hu_protocol_bagoo;

static const hu_protocol_driver_t *s_available_protocols[] = {
    [HU_PROTOCOL_RAISE]   = &g_hu_protocol_raise,
    [HU_PROTOCOL_HIWORLD] = &g_hu_protocol_hiworld,
    [HU_PROTOCOL_BAGOO]   = &g_hu_protocol_bagoo,
};

static const hu_protocol_driver_t *s_active_driver = &g_hu_protocol_raise;

void hu_protocol_init(void) {
#if defined(PLATFORM_LINUX)
    const char *proto_env = getenv("CANBOX_HU_PROTOCOL");
    if (proto_env && proto_env[0] != '\0') {
        if (strcasecmp(proto_env, "raise") == 0 || strcasecmp(proto_env, "rzc") == 0) {
            s_active_driver = &g_hu_protocol_raise;
        } else if (strcasecmp(proto_env, "hiworld") == 0) {
            s_active_driver = &g_hu_protocol_hiworld;
        } else if (strcasecmp(proto_env, "bagoo") == 0 || strcasecmp(proto_env, "psa15") == 0) {
            s_active_driver = &g_hu_protocol_bagoo;
        }
    }
#endif
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

void hu_protocol_send_panel_key(const vehicle_panel_key_t *panel_key) {
    if (s_active_driver && s_active_driver->send_panel_key) {
        s_active_driver->send_panel_key(panel_key);
    }
}

void hu_protocol_send_doors(const vehicle_doors_t *doors) {
    if (s_active_driver && s_active_driver->send_doors) {
        s_active_driver->send_doors(doors);
    }
}

void hu_protocol_send_climate(const vehicle_climate_t *climate) {
    if (s_active_driver && s_active_driver->send_climate) {
        s_active_driver->send_climate(climate);
    }
}

void hu_protocol_send_telemetry(uint16_t speed, uint16_t rpm, int16_t angle) {
    if (s_active_driver && s_active_driver->send_telemetry) {
        s_active_driver->send_telemetry(speed, rpm, angle);
    }
}

void hu_protocol_send_tpms(const vehicle_tpms_t *tpms) {
    if (s_active_driver && s_active_driver->send_tpms) {
        s_active_driver->send_tpms(tpms);
    }
}

void hu_protocol_send_tpms_numeric(const vehicle_tpms_t *tpms) {
    if (s_active_driver && s_active_driver->send_tpms_numeric) {
        s_active_driver->send_tpms_numeric(tpms);
    }
}

void hu_protocol_send_tpms_discrete(const vehicle_tpms_t *tpms) {
    if (s_active_driver && s_active_driver->send_tpms_discrete) {
        s_active_driver->send_tpms_discrete(tpms);
    }
}

void hu_protocol_send_trip_instant(const vehicle_trip_t *trip) {
    if (s_active_driver && s_active_driver->send_trip_instant) {
        s_active_driver->send_trip_instant(trip);
    }
}

void hu_protocol_send_trip1(const vehicle_trip_t *trip) {
    if (s_active_driver && s_active_driver->send_trip1) {
        s_active_driver->send_trip1(trip);
    }
}

void hu_protocol_send_trip2(const vehicle_trip_t *trip) {
    if (s_active_driver && s_active_driver->send_trip2) {
        s_active_driver->send_trip2(trip);
    }
}

void hu_protocol_send_radar(const vehicle_radar_t *radar) {
    if (s_active_driver && s_active_driver->send_radar) {
        s_active_driver->send_radar(radar);
    }
}

void hu_protocol_send_reverse(bool reverse_active) {
    if (s_active_driver && s_active_driver->send_reverse) {
        s_active_driver->send_reverse(reverse_active);
    }
}

void hu_protocol_send_alert_single(const vehicle_alert_item_t *alert) {
    if (s_active_driver && s_active_driver->send_alert_single) {
        s_active_driver->send_alert_single(alert);
    }
}

void hu_protocol_send_alerts_summary(const vehicle_alert_item_t *alerts, uint8_t count) {
    if (s_active_driver && s_active_driver->send_alerts_summary) {
        s_active_driver->send_alerts_summary(alerts, count);
    }
}

void hu_protocol_send_radio_state(const vehicle_radio_t *radio) {
    if (s_active_driver && s_active_driver->send_radio_state) {
        s_active_driver->send_radio_state(radio);
    }
}

void hu_protocol_send_radio_text(const char *text, uint8_t len) {
    if (s_active_driver && s_active_driver->send_radio_text) {
        s_active_driver->send_radio_text(text, len);
    }
}

void hu_protocol_send_media_state(const vehicle_cdc_t *cdc) {
    if (s_active_driver && s_active_driver->send_media_state) {
        s_active_driver->send_media_state(cdc);
    }
}

void hu_protocol_send_heartbeat(void) {
    if (s_active_driver && s_active_driver->send_heartbeat) {
        s_active_driver->send_heartbeat();
    }
}

