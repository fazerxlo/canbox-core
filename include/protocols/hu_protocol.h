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
void hu_protocol_send_alert_single(uint16_t alert_code);
void hu_protocol_send_alerts_summary(const uint16_t *alert_codes, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif /* HU_PROTOCOL_H */

