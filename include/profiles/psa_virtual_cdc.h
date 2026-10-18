#ifndef PSA_VIRTUAL_CDC_H
#define PSA_VIRTUAL_CDC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "hal/hal_can.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool     enabled;            /* Continuous presence heartbeat active */
    bool     playback_active;    /* Audio actively playing */
    bool     radio_enabled;      /* RD4 reported power on via 0x131 or BSI 0x036 */
    uint8_t  active_disc;        /* 1..6 = Active disc slot */
    uint8_t  track_num;          /* 1..99 */
    uint8_t  total_tracks;       /* Total tracks (default 99 matching CDCEmu TRACK_MAX) */
    uint8_t  elapsed_min;        /* 0..59 */
    uint8_t  elapsed_sec;        /* 0..59 */
    uint8_t  status_icon;        /* 1: STOP, 2: PAUSE, 3: PLAY, 6: NONE */
    bool     intro_mode;
    bool     random_mode;
    bool     repeat_mode;
    uint32_t timer_ticks_100ms;  /* 100ms ticks counter for round-robin scheduler */
} psa_virtual_cdc_t;

typedef enum {
    SRC_SYNC_IDLE = 0,          /* In sync, normal operation */
    SRC_SYNC_SEEKING,           /* Actively pulsing 0x21F to reach HU target */
    SRC_SYNC_WAIT_CONFIRM,      /* Pulse sent, waiting up to 350ms for 0x165 update */
    SRC_SYNC_PHYSICAL_DEBOUNCE  /* User pressed physical SRC button, waiting to settle */
} psa_src_sync_state_t;

typedef struct {
    psa_src_sync_state_t sync_state;
    uint8_t actual_rd4_source;      /* Ground-truth from CAN 0x165 (1=Tuner, 2=CD, 3=CDC, 4=AUX) */
    uint8_t reported_hu_source;     /* Last stable source sent to HU via Cmd 0x84 */
    uint8_t target_source;          /* Target requested by HU Cmd 0xAD (0 if none) */
    uint8_t last_pulse_source;      /* Source when last pulse was injected */
    uint8_t retry_count;            /* Max 5 pulses per request */
    uint32_t state_timer_ms;        /* Timestamp / tick counter for debounce & timeouts */
    bool     pending_station_list_trigger; /* Deferred 0x0F rotary scroll trigger waiting for Tuner */
} psa_source_arbiter_t;

#define PSA_RD4_SRC_TUNER 0x01
#define PSA_RD4_SRC_CD    0x02
#define PSA_RD4_SRC_CDC   0x03
#define PSA_RD4_SRC_AUX   0x04

#define STALK_PRESS_DURATION_MS     50   /* Hold 0x21F#020000 for 50 ms */
#define STALK_WAIT_CONFIRM_MAX_MS  650   /* Wait up to 650 ms for CAN 0x165 source nibble to change */
#define STALK_POST_CHANGE_SETTLE_MS 150  /* Wait 150 ms AFTER 0x165 changes before next press */

void psa_virtual_cdc_init(void);
void psa_virtual_cdc_set_enabled(bool enabled);
void psa_virtual_cdc_set_state(bool active);
void psa_virtual_cdc_handle_hu_cmd(uint8_t subcmd, uint8_t param);
void psa_virtual_cdc_switch_source(uint8_t target_source);
void psa_virtual_cdc_trigger_station_list(void);
void psa_virtual_cdc_update_stalk_rotary(uint8_t rotary_cnt);
uint8_t psa_virtual_cdc_get_stalk_rotary(void);
void psa_virtual_cdc_process_can_0x131(const uint8_t *data, uint8_t dlc);
void psa_virtual_cdc_process_can_0x165(uint8_t source_mode, uint8_t power_status);
void psa_virtual_cdc_tick_100ms(void);
bool psa_virtual_cdc_is_active(void);
bool psa_virtual_cdc_is_presence_enabled(void);
const psa_virtual_cdc_t *psa_virtual_cdc_get_context(void);
const psa_source_arbiter_t *psa_virtual_cdc_get_arbiter(void);
struct vehicle_radio;
bool psa_virtual_cdc_is_switching_in_progress(void);
bool psa_virtual_cdc_is_source_filtered(void);
uint8_t psa_virtual_cdc_get_target_source(void);
uint8_t psa_virtual_cdc_get_reported_source(void);
bool psa_virtual_cdc_is_target_source_reached(const struct vehicle_radio *radio);
uint8_t psa_source_mode_to_arbiter(uint8_t source_mode);

#ifdef __cplusplus
}
#endif

#endif /* PSA_VIRTUAL_CDC_H */
