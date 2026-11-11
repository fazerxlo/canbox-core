#include "core/can_sniffer.h"
#include "protocols/hu_protocol.h"
#include "hal/hal_gpio.h"
#include <string.h>

typedef enum {
    SNIFFER_STATE_IDLE = 0,
    SNIFFER_STATE_RUNNING,
    SNIFFER_STATE_SUMMARY_FLUSH,
} sniffer_state_t;

typedef struct {
    uint16_t can_id;
    uint8_t  dlc;
    uint8_t  last_data[8];
    uint16_t last_rx_dt_ms;
    uint16_t repeat_count;
    uint8_t  period_10ms;
    bool     used;
} sniffer_entry_t;

typedef struct {
    uint16_t dt_ms;
    uint16_t id_flags_dlc;
    uint16_t prev_repeat_count;
    uint8_t  period_10ms;
    uint8_t  dlc;
    uint8_t  data[8];
} sniffer_event_t;

static sniffer_state_t s_sniffer_state = SNIFFER_STATE_IDLE;
static uint32_t        s_start_ms = 0;
static uint16_t        s_stop_dt_ms = 0;
static uint8_t         s_duration_s = 30;
static uint16_t        s_dropped_events = 0;

static sniffer_entry_t s_id_table[SNIFFER_MAX_IDS];
static uint8_t         s_num_ids = 0;
static uint8_t         s_summary_idx = 0;

static sniffer_event_t s_event_queue[SNIFFER_EVENT_QUEUE_SIZE];
static uint8_t         s_queue_head = 0;
static uint8_t         s_queue_tail = 0;
static uint8_t         s_queue_count = 0;

static int find_id_entry(uint16_t can_id) {
    for (uint8_t i = 0; i < s_num_ids; i++) {
        if (s_id_table[i].used && s_id_table[i].can_id == can_id) {
            return (int)i;
        }
    }
    return -1;
}

static bool enqueue_event(const sniffer_event_t *ev) {
    if (s_queue_count >= SNIFFER_EVENT_QUEUE_SIZE) {
        if (s_dropped_events < 0xFFFF) {
            s_dropped_events++;
        }
        return false;
    }
    s_event_queue[s_queue_tail] = *ev;
    s_queue_tail = (uint8_t)((s_queue_tail + 1) % SNIFFER_EVENT_QUEUE_SIZE);
    s_queue_count++;
    return true;
}

static bool dequeue_event(sniffer_event_t *ev) {
    if (s_queue_count == 0) {
        return false;
    }
    *ev = s_event_queue[s_queue_head];
    s_queue_head = (uint8_t)((s_queue_head + 1) % SNIFFER_EVENT_QUEUE_SIZE);
    s_queue_count--;
    return true;
}

void can_sniffer_init(void) {
    s_sniffer_state = SNIFFER_STATE_IDLE;
    s_start_ms = 0;
    s_stop_dt_ms = 0;
    s_duration_s = 30;
    s_dropped_events = 0;
    s_num_ids = 0;
    s_summary_idx = 0;
    s_queue_head = 0;
    s_queue_tail = 0;
    s_queue_count = 0;
    memset(s_id_table, 0, sizeof(s_id_table));
}

void can_sniffer_start(uint8_t mode, uint8_t duration_s, uint32_t now_ms) {
    (void)mode;
    /* Hard ceiling: sniffer must never run for more than 30 seconds */
    if (duration_s == 0 || duration_s > SNIFFER_MAX_DURATION_S) {
        duration_s = SNIFFER_MAX_DURATION_S;
    }
    s_duration_s = duration_s;
    s_start_ms = now_ms;
    s_stop_dt_ms = 0;
    s_dropped_events = 0;
    s_num_ids = 0;
    s_summary_idx = 0;
    s_queue_head = 0;
    s_queue_tail = 0;
    s_queue_count = 0;
    memset(s_id_table, 0, sizeof(s_id_table));
    s_sniffer_state = SNIFFER_STATE_RUNNING;
    hal_gpio_write(GPIO_PIN_HEADUNIT_POWER, true);
}

void can_sniffer_stop(uint32_t now_ms) {
    if (s_sniffer_state == SNIFFER_STATE_RUNNING) {
        uint32_t elapsed = now_ms - s_start_ms;
        s_stop_dt_ms = (elapsed > 60000) ? 60000 : (uint16_t)elapsed;
        s_summary_idx = 0;
        s_sniffer_state = SNIFFER_STATE_SUMMARY_FLUSH;
    }
}

bool can_sniffer_is_active(void) {
    return (s_sniffer_state != SNIFFER_STATE_IDLE);
}

void can_sniffer_on_can_frame(const can_frame_t *frame, bool is_tx, uint32_t now_ms) {
    if (s_sniffer_state != SNIFFER_STATE_RUNNING || !frame) {
        return;
    }

    uint32_t elapsed = now_ms - s_start_ms;
    uint16_t dt_ms = (elapsed > 60000) ? 60000 : (uint16_t)elapsed;
    uint8_t dlc = (frame->dlc > 8) ? 8 : frame->dlc;

    if (is_tx) {
        sniffer_event_t ev;
        ev.dt_ms = dt_ms;
        ev.id_flags_dlc = (uint16_t)((frame->id & 0x07FF) |
                                     ((dlc & 0x0F) << 11) |
                                     0x8000);
        ev.prev_repeat_count = 0;
        ev.period_10ms = 0;
        ev.dlc = dlc;
        if (dlc > 0) {
            memcpy(ev.data, frame->data, dlc);
        }
        enqueue_event(&ev);
        return;
    }

    /* Incoming CAN RX from vehicle */
    uint16_t can_id = (uint16_t)(frame->id & 0x07FF);
    int idx = find_id_entry(can_id);

    if (idx < 0) {
        /* Discovery of new CAN ID */
        if (s_num_ids >= SNIFFER_MAX_IDS) {
            if (s_dropped_events < 0xFFFF) {
                s_dropped_events++;
            }
            return;
        }

        uint8_t new_idx = s_num_ids++;
        sniffer_entry_t *entry = &s_id_table[new_idx];
        entry->can_id = can_id;
        entry->dlc = dlc;
        if (dlc > 0) {
            memcpy(entry->last_data, frame->data, dlc);
        }
        entry->last_rx_dt_ms = dt_ms;
        entry->repeat_count = 0;
        entry->period_10ms = 0;
        entry->used = true;

        sniffer_event_t ev;
        ev.dt_ms = dt_ms;
        ev.id_flags_dlc = (uint16_t)(can_id | ((dlc & 0x0F) << 11));
        ev.prev_repeat_count = 0;
        ev.period_10ms = 0;
        ev.dlc = dlc;
        if (dlc > 0) {
            memcpy(ev.data, frame->data, dlc);
        }
        enqueue_event(&ev);
    } else {
        sniffer_entry_t *entry = &s_id_table[idx];
        uint32_t delta_ms = (dt_ms >= entry->last_rx_dt_ms) ? (dt_ms - entry->last_rx_dt_ms) : 0;
        uint32_t interval_10ms = (delta_ms + 5) / 10;
        if (interval_10ms > 255) {
            interval_10ms = 255;
        }
        if (interval_10ms == 0 && delta_ms > 0) {
            interval_10ms = 1;
        }

        if (entry->period_10ms == 0) {
            entry->period_10ms = (uint8_t)interval_10ms;
        } else if (interval_10ms > 0) {
            entry->period_10ms = (uint8_t)(((uint16_t)entry->period_10ms * 3 + interval_10ms + 2) / 4);
        }

        bool changed = (entry->dlc != dlc) ||
                       (memcmp(entry->last_data, frame->data, dlc) != 0);

        if (!changed) {
            if (entry->repeat_count < 0xFFFF) {
                entry->repeat_count++;
            }
            entry->last_rx_dt_ms = dt_ms;
        } else {
            sniffer_event_t ev;
            ev.dt_ms = dt_ms;
            ev.id_flags_dlc = (uint16_t)(can_id | ((dlc & 0x0F) << 11));
            ev.prev_repeat_count = entry->repeat_count;
            ev.period_10ms = entry->period_10ms;
            ev.dlc = dlc;
            if (dlc > 0) {
                memcpy(ev.data, frame->data, dlc);
            }
            enqueue_event(&ev);

            entry->dlc = dlc;
            if (dlc > 0) {
                memcpy(entry->last_data, frame->data, dlc);
            }
            entry->last_rx_dt_ms = dt_ms;
            entry->repeat_count = 0;
        }
    }
}

void can_sniffer_poll_10ms(uint32_t now_ms) {
    uint32_t elapsed_ms = (now_ms >= s_start_ms) ? (now_ms - s_start_ms) : 0;

    /* Hard ceiling: automatically trigger stop when duration expires regardless of HU */
    if (s_sniffer_state == SNIFFER_STATE_RUNNING) {
        if (elapsed_ms >= ((uint32_t)s_duration_s * 1000)) {
            can_sniffer_stop(now_ms);
        }
    }

    /* Failsafe timeout: force idle if summary flush takes > 2s past max duration */
    if (s_sniffer_state == SNIFFER_STATE_SUMMARY_FLUSH) {
        if (elapsed_ms >= (((uint32_t)s_duration_s * 1000) + 2000)) {
            uint8_t fin_buf[3];
            fin_buf[0] = s_num_ids;
            fin_buf[1] = (uint8_t)(s_dropped_events & 0xFF);
            fin_buf[2] = (uint8_t)((s_dropped_events >> 8) & 0xFF);
            hu_protocol_send_sniffer_packet(SNIFFER_SUBTYPE_FINISHED, fin_buf, sizeof(fin_buf));
            s_sniffer_state = SNIFFER_STATE_IDLE;
        }
    }

    if (s_sniffer_state == SNIFFER_STATE_IDLE) {
        return;
    }

    /* Pacing rule: Emit at most ONE 0xD1 frame per call with payload <= 52 bytes.
     * Record wire size = 7 + dlc (7 to 15 bytes).
     * Room in payload for records = SNIFFER_MAX_BATCH_PAYLOAD - 1 (subtype byte) = 51 bytes. */
    uint8_t record_buf[SNIFFER_MAX_BATCH_PAYLOAD];
    size_t packed_len = 0;

    if (s_queue_count > 0) {
        /* Drain queued live events */
        while (s_queue_count > 0) {
            sniffer_event_t *head_ev = &s_event_queue[s_queue_head];
            size_t rec_size = 7 + head_ev->dlc;
            if (packed_len + rec_size > (SNIFFER_MAX_BATCH_PAYLOAD - 1)) {
                break;
            }

            sniffer_event_t ev;
            dequeue_event(&ev);

            record_buf[packed_len + 0] = (uint8_t)(ev.dt_ms & 0xFF);
            record_buf[packed_len + 1] = (uint8_t)((ev.dt_ms >> 8) & 0xFF);
            record_buf[packed_len + 2] = (uint8_t)(ev.id_flags_dlc & 0xFF);
            record_buf[packed_len + 3] = (uint8_t)((ev.id_flags_dlc >> 8) & 0xFF);
            record_buf[packed_len + 4] = (uint8_t)(ev.prev_repeat_count & 0xFF);
            record_buf[packed_len + 5] = (uint8_t)((ev.prev_repeat_count >> 8) & 0xFF);
            record_buf[packed_len + 6] = ev.period_10ms;
            if (ev.dlc > 0) {
                memcpy(&record_buf[packed_len + 7], ev.data, ev.dlc);
            }
            packed_len += rec_size;
        }

        if (packed_len > 0) {
            hu_protocol_send_sniffer_packet(SNIFFER_SUBTYPE_LIVE, record_buf, packed_len);
            return;
        }
    }

    /* If in summary flush mode and live event queue is drained */
    if (s_sniffer_state == SNIFFER_STATE_SUMMARY_FLUSH && s_queue_count == 0) {
        while (s_summary_idx < s_num_ids) {
            sniffer_entry_t *entry = &s_id_table[s_summary_idx];
            size_t rec_size = 7 + entry->dlc;
            if (packed_len + rec_size > (SNIFFER_MAX_BATCH_PAYLOAD - 1)) {
                break;
            }

            uint16_t id_flags_dlc = (uint16_t)((entry->can_id & 0x07FF) |
                                               ((entry->dlc & 0x0F) << 11));

            record_buf[packed_len + 0] = (uint8_t)(s_stop_dt_ms & 0xFF);
            record_buf[packed_len + 1] = (uint8_t)((s_stop_dt_ms >> 8) & 0xFF);
            record_buf[packed_len + 2] = (uint8_t)(id_flags_dlc & 0xFF);
            record_buf[packed_len + 3] = (uint8_t)((id_flags_dlc >> 8) & 0xFF);
            record_buf[packed_len + 4] = (uint8_t)(entry->repeat_count & 0xFF);
            record_buf[packed_len + 5] = (uint8_t)((entry->repeat_count >> 8) & 0xFF);
            record_buf[packed_len + 6] = entry->period_10ms;
            if (entry->dlc > 0) {
                memcpy(&record_buf[packed_len + 7], entry->last_data, entry->dlc);
            }
            packed_len += rec_size;
            s_summary_idx++;
        }

        if (packed_len > 0) {
            hu_protocol_send_sniffer_packet(SNIFFER_SUBTYPE_SUMMARY, record_buf, packed_len);
            return;
        }

        /* All summary records drained: emit Sniffer Finished Marker */
        uint8_t fin_buf[3];
        fin_buf[0] = s_num_ids;
        fin_buf[1] = (uint8_t)(s_dropped_events & 0xFF);
        fin_buf[2] = (uint8_t)((s_dropped_events >> 8) & 0xFF);
        hu_protocol_send_sniffer_packet(SNIFFER_SUBTYPE_FINISHED, fin_buf, sizeof(fin_buf));

        s_sniffer_state = SNIFFER_STATE_IDLE;
    }
}

uint16_t can_sniffer_get_dropped_count(void) {
    return s_dropped_events;
}

uint8_t can_sniffer_get_unique_id_count(void) {
    return s_num_ids;
}

