#include "can_log_player.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#define CSV_LINE_MAX_LEN 256

bool can_log_player_open(can_log_player_t *player, const char *csv_path) {
    if (!player || !csv_path) {
        return false;
    }

    memset(player, 0, sizeof(can_log_player_t));
    player->fp = fopen(csv_path, "r");
    if (!player->fp) {
        return false;
    }

    // Skip CSV header line if present
    char line[CSV_LINE_MAX_LEN];
    if (fgets(line, sizeof(line), player->fp)) {
        if (strstr(line, "Time") == NULL && strstr(line, "time") == NULL) {
            // Not a header line, rewind back to beginning
            rewind(player->fp);
        }
    }

    return true;
}

void can_log_player_close(can_log_player_t *player) {
    if (player && player->fp) {
        fclose(player->fp);
        player->fp = NULL;
    }
}

static void trim_whitespace(char *str) {
    if (!str) return;
    char *end = str + strlen(str) - 1;
    while (end >= str && (isspace((unsigned char)*end) || *end == '\r' || *end == '\n')) {
        *end = '\0';
        end--;
    }
}

bool can_log_player_read_entry(can_log_player_t *player, can_log_entry_t *entry) {
    if (!player || !player->fp || !entry) {
        return false;
    }

    char line[CSV_LINE_MAX_LEN];
    while (fgets(line, sizeof(line), player->fp)) {
        trim_whitespace(line);
        if (line[0] == '\0') {
            continue; // Skip empty lines
        }

        // Expected format: Time Stamp,ID,Extended,Dir,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8
        char *saveptr = NULL;
        char *token = strtok_r(line, ",", &saveptr);
        if (!token) continue;
        entry->timestamp_us = (uint64_t)strtoull(token, NULL, 10);

        // ID
        token = strtok_r(NULL, ",", &saveptr);
        if (!token) continue;
        entry->frame.id = (uint32_t)strtoul(token, NULL, 16);

        // Extended
        token = strtok_r(NULL, ",", &saveptr);
        if (!token) continue;
        entry->frame.is_extended = (strstr(token, "true") != NULL || strstr(token, "1") != NULL);

        // Dir (Rx/Tx)
        token = strtok_r(NULL, ",", &saveptr);
        if (!token) continue;

        // Bus
        token = strtok_r(NULL, ",", &saveptr);
        if (!token) continue;

        // LEN (DLC)
        token = strtok_r(NULL, ",", &saveptr);
        if (!token) continue;
        uint8_t dlc = (uint8_t)strtoul(token, NULL, 10);
        if (dlc > 8) dlc = 8;
        entry->frame.dlc = dlc;

        // Clear payload bytes
        memset(entry->frame.data, 0, sizeof(entry->frame.data));

        // Read payload bytes D1..D8
        for (uint8_t i = 0; i < dlc; i++) {
            token = strtok_r(NULL, ",", &saveptr);
            if (token && token[0] != '\0') {
                entry->frame.data[i] = (uint8_t)strtoul(token, NULL, 16);
            }
        }

        return true;
    }

    return false;
}

bool can_log_player_step(can_log_player_t *player) {
    if (!player) return false;

    can_log_entry_t entry;
    if (!can_log_player_read_entry(player, &entry)) {
        return false;
    }

    if (!player->has_started) {
        player->start_timestamp_us = entry.timestamp_us;
        player->last_periodic_tick_us = entry.timestamp_us;
        player->has_started = true;
    }

    player->current_timestamp_us = entry.timestamp_us;
    if (entry.timestamp_us >= player->start_timestamp_us) {
        player->simulated_elapsed_us = entry.timestamp_us - player->start_timestamp_us;
    }

    // Step periodic 100ms timer (100,000 us)
    while ((entry.timestamp_us - player->last_periodic_tick_us) >= 100000ULL) {
        can_router_periodic_100ms();
        player->last_periodic_tick_us += 100000ULL;
        player->periodic_ticks_fired++;
    }

    // Process CAN message through router
    can_router_process_can(&entry.frame);
    player->frames_processed++;

    return true;
}

uint32_t can_log_player_replay_until_timestamp(can_log_player_t *player, uint64_t target_timestamp_us) {
    if (!player || !player->fp) return 0;

    uint32_t count = 0;
    while (1) {
        // Peek or read next entry
        long file_pos = ftell(player->fp);
        can_log_entry_t entry;
        if (!can_log_player_read_entry(player, &entry)) {
            break;
        }

        if (entry.timestamp_us > target_timestamp_us) {
            // Seek back so this entry can be processed in next step
            fseek(player->fp, file_pos, SEEK_SET);
            break;
        }

        if (!player->has_started) {
            player->start_timestamp_us = entry.timestamp_us;
            player->last_periodic_tick_us = entry.timestamp_us;
            player->has_started = true;
        }

        player->current_timestamp_us = entry.timestamp_us;
        if (entry.timestamp_us >= player->start_timestamp_us) {
            player->simulated_elapsed_us = entry.timestamp_us - player->start_timestamp_us;
        }

        while ((entry.timestamp_us - player->last_periodic_tick_us) >= 100000ULL) {
            can_router_periodic_100ms();
            player->last_periodic_tick_us += 100000ULL;
            player->periodic_ticks_fired++;
        }

        can_router_process_can(&entry.frame);
        player->frames_processed++;
        count++;
    }

    return count;
}

uint32_t can_log_player_replay_until_can_id(can_log_player_t *player, uint32_t can_id) {
    if (!player || !player->fp) return 0;

    uint32_t count = 0;
    while (1) {
        can_log_entry_t entry;
        if (!can_log_player_read_entry(player, &entry)) {
            break;
        }

        if (!player->has_started) {
            player->start_timestamp_us = entry.timestamp_us;
            player->last_periodic_tick_us = entry.timestamp_us;
            player->has_started = true;
        }

        player->current_timestamp_us = entry.timestamp_us;
        if (entry.timestamp_us >= player->start_timestamp_us) {
            player->simulated_elapsed_us = entry.timestamp_us - player->start_timestamp_us;
        }

        while ((entry.timestamp_us - player->last_periodic_tick_us) >= 100000ULL) {
            can_router_periodic_100ms();
            player->last_periodic_tick_us += 100000ULL;
            player->periodic_ticks_fired++;
        }

        can_router_process_can(&entry.frame);
        player->frames_processed++;
        count++;

        if (entry.frame.id == can_id) {
            break;
        }
    }

    return count;
}

uint32_t can_log_player_replay_all(can_log_player_t *player) {
    if (!player || !player->fp) return 0;

    uint32_t count = 0;
    while (can_log_player_step(player)) {
        count++;
    }
    return count;
}

