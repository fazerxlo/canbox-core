#ifndef HU_PROTOCOL_DRIVER_H
#define HU_PROTOCOL_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "core/can_router.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HU_PROTOCOL_RAISE = 0,
    HU_PROTOCOL_HIWORLD,
    HU_PROTOCOL_BAGOO,
    HU_PROTOCOL_COUNT
} hu_protocol_id_t;

typedef struct {
    hu_protocol_id_t id;
    const char      *name;
    void           (*init)(void);
    void           (*feed_byte)(uint8_t byte);
    void           (*send_wheel_key)(const vehicle_wheel_t *wheel);
    void           (*send_panel_key)(const vehicle_panel_key_t *panel_key);
    void           (*send_doors)(const vehicle_doors_t *doors);
    void           (*send_climate)(const vehicle_climate_t *climate);
    void           (*send_telemetry)(uint16_t speed, uint16_t rpm, int16_t angle);
    void           (*send_tpms)(const vehicle_tpms_t *tpms);
    void           (*send_tpms_numeric)(const vehicle_tpms_t *tpms);
    void           (*send_tpms_discrete)(const vehicle_tpms_t *tpms);
    void           (*send_trip_instant)(const vehicle_trip_t *trip);
    void           (*send_trip1)(const vehicle_trip_t *trip);
    void           (*send_trip2)(const vehicle_trip_t *trip);
    void           (*send_radar)(const vehicle_radar_t *radar);
    void           (*send_reverse)(bool reverse_active);
    void           (*send_alert_single)(const vehicle_alert_item_t *alert);
    void           (*send_alerts_summary)(const vehicle_alert_item_t *alerts, uint8_t count);
    void           (*send_radio_state)(const vehicle_radio_t *radio);
    void           (*send_radio_text)(const char *text, uint8_t len);
    void           (*send_media_state)(const vehicle_cdc_t *cdc);
    void           (*send_heartbeat)(void);
} hu_protocol_driver_t;

bool hu_protocol_set_active(hu_protocol_id_t protocol_id);
const hu_protocol_driver_t *hu_protocol_get_active(void);
void hu_protocol_init(void);

#ifdef __cplusplus
}
#endif

#endif /* HU_PROTOCOL_DRIVER_H */

