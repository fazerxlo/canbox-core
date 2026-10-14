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

void psa_virtual_cdc_init(void);
void psa_virtual_cdc_set_enabled(bool enabled);
void psa_virtual_cdc_set_state(bool active);
void psa_virtual_cdc_handle_hu_cmd(uint8_t subcmd, uint8_t param);
void psa_virtual_cdc_process_can_0x131(const uint8_t *data, uint8_t dlc);
void psa_virtual_cdc_tick_100ms(void);
bool psa_virtual_cdc_is_active(void);
bool psa_virtual_cdc_is_presence_enabled(void);
const psa_virtual_cdc_t *psa_virtual_cdc_get_context(void);

#ifdef __cplusplus
}
#endif

#endif /* PSA_VIRTUAL_CDC_H */
