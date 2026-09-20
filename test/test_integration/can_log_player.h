#ifndef CAN_LOG_PLAYER_H
#define CAN_LOG_PLAYER_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "hal/hal_can.h"
#include "core/can_router.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    FILE     *fp;
    uint64_t  start_timestamp_us;
    uint64_t  current_timestamp_us;
    uint64_t  simulated_elapsed_us;
    uint64_t  last_periodic_tick_us;
    uint32_t  frames_processed;
    uint32_t  periodic_ticks_fired;
    bool      has_started;
} can_log_player_t;

typedef struct {
    uint64_t    timestamp_us;
    can_frame_t frame;
} can_log_entry_t;

bool can_log_player_open(can_log_player_t *player, const char *csv_path);
void can_log_player_close(can_log_player_t *player);
bool can_log_player_read_entry(can_log_player_t *player, can_log_entry_t *entry);
bool can_log_player_step(can_log_player_t *player);
uint32_t can_log_player_replay_until_timestamp(can_log_player_t *player, uint64_t target_timestamp_us);
uint32_t can_log_player_replay_until_can_id(can_log_player_t *player, uint32_t can_id);
uint32_t can_log_player_replay_all(can_log_player_t *player);

#ifdef __cplusplus
}
#endif

#endif /* CAN_LOG_PLAYER_H */

