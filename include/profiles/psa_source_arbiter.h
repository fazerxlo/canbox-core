#ifndef PSA_SOURCE_ARBITER_H
#define PSA_SOURCE_ARBITER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "hal/hal_can.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SRC_SYNC_IDLE = 0,          /* In sync, normal operation */
    SRC_SYNC_SEEKING,           /* Actively pulsing 0x21F to reach HU target */
    SRC_SYNC_WAIT_CONFIRM,      /* Pulse sent, waiting up to 650ms for 0x165 update */
    SRC_SYNC_PHYSICAL_DEBOUNCE  /* User pressed physical SRC button, waiting to settle */
} psa_src_sync_state_t;

typedef struct {
    psa_src_sync_state_t sync_state;
    uint8_t actual_rd4_source;      /* Ground-truth from CAN 0x165 (1=Tuner, 2=CD, 3=CDC, 4=AUX) */
    uint8_t reported_hu_source;     /* Last stable source sent to HU via Cmd 0x84 */
    uint8_t target_source;          /* Target requested by HU Cmd 0xAD (0 if none) */
    uint8_t last_pulse_source;      /* Source when last pulse was injected */
    uint8_t retry_count;            /* Max retry pulses per request */
    uint32_t state_timer_ms;        /* Timestamp / tick counter for debounce & timeouts */
    bool     pending_station_list_trigger; /* Deferred 0x0F rotary scroll trigger waiting for Tuner */
    bool     user_override_non_hu;  /* True when user manually switched away from AUX via physical RD4/stalk */
} psa_source_arbiter_t;

#define PSA_RD4_SRC_TUNER 0x01
#define PSA_RD4_SRC_CD    0x02
#define PSA_RD4_SRC_CDC   0x03
#define PSA_RD4_SRC_AUX   0x04

#define STALK_PRESS_DURATION_MS     50   /* Hold 0x21F#020000 for 50 ms */
#define STALK_WAIT_CONFIRM_MAX_MS  650   /* Wait up to 650 ms for CAN 0x165 source nibble to change */
#define STALK_POST_CHANGE_SETTLE_MS 150  /* Wait 150 ms AFTER 0x165 changes before next press */

void psa_source_arbiter_init(void);
void psa_source_arbiter_handle_hu_cmd(uint8_t subcmd, uint8_t param);
void psa_source_arbiter_switch_source(uint8_t target_source);
void psa_source_arbiter_trigger_station_list(void);
void psa_source_arbiter_update_stalk_rotary(uint8_t rotary_cnt);
uint8_t psa_source_arbiter_get_stalk_rotary(void);
void psa_source_arbiter_process_can_0x165(uint8_t source_mode, uint8_t power_status);
void psa_source_arbiter_tick_100ms(void);
const psa_source_arbiter_t *psa_source_arbiter_get_context(void);
struct vehicle_radio;
bool psa_source_arbiter_is_switching_in_progress(void);
bool psa_source_arbiter_is_source_filtered(void);
uint8_t psa_source_arbiter_get_target_source(void);
uint8_t psa_source_arbiter_get_reported_source(void);
bool psa_source_arbiter_is_target_source_reached(const struct vehicle_radio *radio);
uint8_t psa_source_mode_to_arbiter(uint8_t source_mode);
void psa_source_arbiter_process_can_0x225(uint8_t preset_slot, uint16_t freq_0_1mhz);
bool psa_source_arbiter_is_station_scroll_in_progress(void);

#ifdef __cplusplus
}
#endif

#endif /* PSA_SOURCE_ARBITER_H */

