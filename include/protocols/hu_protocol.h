#ifndef HU_PROTOCOL_H
#define HU_PROTOCOL_H

#include <stdint.h>
#include "core/can_router.h"

#ifdef __cplusplus
extern "C" {
#endif

void hu_protocol_init(void);
void hu_protocol_feed_byte(uint8_t byte);
void hu_protocol_send_heartbeat(void);
void hu_protocol_send_wheel_key(const vehicle_wheel_t *wheel);
void hu_protocol_send_panel_key(const vehicle_panel_key_t *panel_key);
void hu_protocol_send_doors(const vehicle_doors_t *doors);
void hu_protocol_send_climate(const vehicle_climate_t *climate);
void hu_protocol_send_telemetry(uint16_t speed, uint16_t rpm, int16_t angle);
void hu_protocol_send_tpms(const vehicle_tpms_t *tpms);
void hu_protocol_send_tpms_numeric(const vehicle_tpms_t *tpms);
void hu_protocol_send_tpms_discrete(const vehicle_tpms_t *tpms);
void hu_protocol_send_trip_instant(const vehicle_trip_t *trip);
void hu_protocol_send_trip1(const vehicle_trip_t *trip);
void hu_protocol_send_trip2(const vehicle_trip_t *trip);
void hu_protocol_send_radar(const vehicle_radar_t *radar);
void hu_protocol_send_reverse(bool reverse_active);
void hu_protocol_send_alert_single(const vehicle_alert_item_t *alert);
void hu_protocol_send_alerts_summary(const vehicle_alert_item_t *alerts, uint8_t count);
void hu_protocol_send_radio_state(const vehicle_radio_t *radio);
void hu_protocol_send_radio_text(const char *text, uint8_t len);
void hu_protocol_send_media_state(const vehicle_cdc_t *cdc);

#ifdef __cplusplus
}
#endif

#endif /* HU_PROTOCOL_H */

