#include "protocols/hiworld_connection.h"
#include "protocols/hiworld_car_mapping.h"
#include "core/can_router.h"
#include "hal/hal_can.h"
#include "hal/hal_system.h"
#include <string.h>

#define HIWORLD_RX_BUF_SIZE 256

typedef enum {
    HW_PARSE_SOF1 = 0,
    HW_PARSE_SOF2,
    HW_PARSE_LEN,
    HW_PARSE_CMD,
    HW_PARSE_DATA,
    HW_PARSE_CS
} hiworld_parse_step_t;

static hiworld_parse_step_t s_step = HW_PARSE_SOF1;
static uint8_t              s_rx_buf[HIWORLD_RX_BUF_SIZE];
static uint8_t              s_payload_len = 0;
static uint8_t              s_cmd_id = 0;
static uint8_t              s_data_idx = 0;
static uint8_t              s_024_rr_step = 0;

size_t hiworld_conn_build_frame(uint8_t cmd_id, const uint8_t *payload, uint8_t len, uint8_t *out_buf, size_t max_out) {
    if (!out_buf || (size_t)(len + 5) > max_out) {
        return 0;
    }

    out_buf[0] = HIWORLD_SOF1;
    out_buf[1] = HIWORLD_SOF2;
    out_buf[2] = len;
    out_buf[3] = cmd_id;
    if (len > 0 && payload != NULL) {
        memcpy(&out_buf[4], payload, len);
    }
    out_buf[len + 4] = hiworld_calc_checksum(out_buf, (size_t)(len + 5));

    return (size_t)(len + 5);
}

static void send_hiworld_frame(hiworld_connection_ctx_t *ctx, uint8_t cmd_id, const uint8_t *payload, uint8_t len) {
    if (!ctx || !ctx->uart_tx) {
        return;
    }

    uint8_t tx[HIWORLD_RX_BUF_SIZE];
    size_t total = hiworld_conn_build_frame(cmd_id, payload, len, tx, sizeof(tx));
    if (total > 0) {
        ctx->uart_tx(tx, total);
    }
}

void hiworld_conn_init(hiworld_connection_ctx_t *ctx,
                       const char *fw_version_str,
                       hiworld_uart_tx_fn uart_tx,
                       hiworld_can_config_fn can_config) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(hiworld_connection_ctx_t));
    ctx->state = HIWORLD_LINK_WAIT_MODEL;
    ctx->uart_tx = uart_tx;
    ctx->can_config = can_config;
    if (fw_version_str) {
        strncpy(ctx->fw_version, fw_version_str, sizeof(ctx->fw_version) - 1);
        ctx->fw_version[sizeof(ctx->fw_version) - 1] = '\0';
    }
    ctx->feat1[0] = 0x60;
    ctx->feat1[1] = 0x12;
    ctx->feat2[0] = 0x00;
    ctx->feat2[1] = 0x01;
    ctx->feat2[2] = 0x80;
    ctx->first_024_received = false;
    ctx->last_024_rx_ms = 0;
    ctx->rr_step = 0;
    s_024_rr_step = 0;
    s_step = HW_PARSE_SOF1;
    s_payload_len = 0;
    s_cmd_id = 0;
    s_data_idx = 0;
}

void hiworld_conn_send_version(hiworld_connection_ctx_t *ctx) {
    if (!ctx) return;
    size_t len = strlen(ctx->fw_version);
    send_hiworld_frame(ctx, HIWORLD_CMD_VERSION_REPORT, (const uint8_t *)ctx->fw_version, (uint8_t)len);
}

void hiworld_conn_set_feature_enables(hiworld_connection_ctx_t *ctx, const uint8_t *feat1, const uint8_t *feat2) {
    if (!ctx) return;
    if (feat1) {
        memcpy(ctx->feat1, feat1, 2);
    }
    if (feat2) {
        memcpy(ctx->feat2, feat2, 8);
    }
}

void hiworld_conn_send_feature_enables(hiworld_connection_ctx_t *ctx) {
    if (!ctx) return;

    /* Feature Enable 1 (Cmd 0x71): 2 bytes */
    send_hiworld_frame(ctx, HIWORLD_CMD_FEATURE_ENABLE1, ctx->feat1, 2);

    /* Feature Enable 2 (Cmd 0x72): 8 bytes */
    send_hiworld_frame(ctx, HIWORLD_CMD_FEATURE_ENABLE2, ctx->feat2, 8);

    /* Unit Info (Cmd 0xC1): 3 bytes (0x28, 0x20, 0x00) - distance/temp/fuel units */
    uint8_t units[3] = { 0x28, 0x20, 0x00 };
    send_hiworld_frame(ctx, HIWORLD_CMD_UNIT_INFO, units, 3);
}

uint8_t hiworld_conn_get_024_rr_step(const hiworld_connection_ctx_t *ctx) {
    return ctx ? ctx->rr_step : s_024_rr_step;
}

void hiworld_conn_reset_024_state(hiworld_connection_ctx_t *ctx) {
    s_024_rr_step = 0;
    if (ctx) {
        ctx->first_024_received = false;
        ctx->last_024_rx_ms = 0;
        ctx->rr_step = 0;
    }
}

static void handle_parsed_command(hiworld_connection_ctx_t *ctx, uint8_t cmd_id, const uint8_t *payload, uint8_t len) {
    if (!ctx) return;

    switch (cmd_id) {
        case HIWORLD_CMD_CAR_TYPE_SET: {
            if (len >= 1) {
                uint8_t new_model_id = payload[0];
                uint8_t new_variant  = (len >= 2) ? payload[1] : 0;

                /* Respond immediately with ACK frame: 5A A5 01 FF 24 23 */
                const uint8_t ack_payload[1] = { HIWORLD_CMD_CAR_TYPE_SET };
                send_hiworld_frame(ctx, HIWORLD_CMD_HEARTBEAT, ack_payload, 1);

                uint32_t now_ms = hal_get_tick_ms();
                bool model_changed = (ctx->state != HIWORLD_LINK_ACTIVE) ||
                                     (ctx->car_model_id != new_model_id) ||
                                     (ctx->car_variant != new_variant);
                bool gap_exceeded = ctx->first_024_received &&
                                    ((uint32_t)(now_ms - ctx->last_024_rx_ms) > 4500U);

                if (!ctx->first_024_received || model_changed || gap_exceeded) {
                    /* Case A — Initial Boot, Model Change, or Detected HU Reboot (> 4500 ms) */
                    ctx->car_model_id = new_model_id;
                    ctx->car_variant  = new_variant;
                    ctx->state = HIWORLD_LINK_INITIALIZED;

                    /* Look up car profile and baud rate */
                    vehicle_profile_id_t profile_id;
                    if (hiworld_car_mapping_get_profile(ctx->car_model_id, &profile_id)) {
                        vehicle_profile_set_active(profile_id);
                    }

                    uint32_t baud = hiworld_car_mapping_get_baud_rate(ctx->car_model_id);
                    if (ctx->can_config) {
                        ctx->can_config(ctx->car_model_id, baud);
                    }

                    /* Send Version and Feature Enables (0xF0, 0x71, 0x72, 0xC1) */
                    hiworld_conn_send_version(ctx);
                    hiworld_conn_send_feature_enables(ctx);

                    ctx->state = HIWORLD_LINK_ACTIVE;
                    ctx->first_024_received = true;
                    ctx->last_024_rx_ms = now_ms;
                    s_024_rr_step = 0;
                    ctx->rr_step = 0;

                    can_router_trigger_full_state_push();
                } else {
                    /* Case B — Steady-State 3-Second Heartbeat */
                    ctx->last_024_rx_ms = now_ms;
                    uint8_t step = s_024_rr_step & 0x03U;
                    s_024_rr_step = (uint8_t)((s_024_rr_step + 1U) & 0x03U);
                    ctx->rr_step = s_024_rr_step;

                    switch (step) {
                        case 0:
                            /* Step 0 (t + 3s): Send static discovery frames (0xF0, 0x71, 0x72, 0xC1) */
                            hiworld_conn_send_version(ctx);
                            hiworld_conn_send_feature_enables(ctx);
                            break;
                        case 1:
                            /* Step 1 (t + 6s): Push Central Vehicle Settings (0x76 & 0x79) */
                            can_router_push_central_settings();
                            break;
                        case 2:
                            /* Step 2 (t + 9s): Push Climate (0x31), Doors (0x12), and Audio DSP (0x56) */
                            can_router_push_climate_doors_dsp();
                            break;
                        case 3:
                            /* Step 3 (t + 12s): Push lightweight active Radio/CD header status (0x84 / 0x82) */
                            can_router_push_media_header();
                            break;
                        default:
                            break;
                    }
                }
            }
            break;
        }

        case HIWORLD_CMD_VERSION_QUERY: {
            hiworld_conn_send_version(ctx);
            break;
        }

        case HIWORLD_CMD_DATE_TIME_SET: {
            /* GPS Time received from Android: Year, Month, Day, Hour, Min, 24H */
            if (len >= 6 && !can_router_is_bus_sleeping()) {
                can_frame_t frame;
                memset(&frame, 0, sizeof(frame));
                frame.id = 0x228;
                frame.dlc = 8;
                frame.data[0] = payload[3]; // Hour
                frame.data[1] = payload[4]; // Minute
                frame.data[2] = payload[2]; // Day
                frame.data[3] = payload[1]; // Month
                frame.data[4] = payload[0]; // Year (2-digit offset from 2000)
                frame.data[5] = 0x00;
                frame.data[6] = 0x00;
                frame.data[7] = 0x00;
                if (ctx->can_tx) {
                    ctx->can_tx(&frame);
                } else {
                    hal_can_send(&frame);
                }
            }
            break;
        }

        default:
            break;
    }
}

void hiworld_conn_process_rx_byte(hiworld_connection_ctx_t *ctx, uint8_t byte) {
    switch (s_step) {
        case HW_PARSE_SOF1:
            if (byte == HIWORLD_SOF1) {
                s_step = HW_PARSE_SOF2;
            }
            break;

        case HW_PARSE_SOF2:
            if (byte == HIWORLD_SOF2) {
                s_step = HW_PARSE_LEN;
            } else if (byte != HIWORLD_SOF1) {
                s_step = HW_PARSE_SOF1;
            }
            break;

        case HW_PARSE_LEN:
            s_payload_len = byte;
            s_rx_buf[0] = HIWORLD_SOF1;
            s_rx_buf[1] = HIWORLD_SOF2;
            s_rx_buf[2] = byte;
            s_step = HW_PARSE_CMD;
            break;

        case HW_PARSE_CMD:
            s_cmd_id = byte;
            s_rx_buf[3] = byte;
            s_data_idx = 0;
            if (s_payload_len == 0) {
                s_step = HW_PARSE_CS;
            } else {
                s_step = HW_PARSE_DATA;
            }
            break;

        case HW_PARSE_DATA:
            if (4 + s_data_idx < HIWORLD_RX_BUF_SIZE) {
                s_rx_buf[4 + s_data_idx] = byte;
            }
            s_data_idx++;
            if (s_data_idx >= s_payload_len) {
                s_step = HW_PARSE_CS;
            }
            break;

        case HW_PARSE_CS: {
            uint8_t expected_cs = byte;
            size_t total_len = (size_t)(s_payload_len + 5);
            if (total_len <= HIWORLD_RX_BUF_SIZE) {
                s_rx_buf[4 + s_payload_len] = expected_cs;
                uint8_t actual_cs = hiworld_calc_checksum(s_rx_buf, total_len);

                if (expected_cs == actual_cs) {
                    handle_parsed_command(ctx, s_cmd_id, &s_rx_buf[4], s_payload_len);
                }
            }

            if (byte == HIWORLD_SOF1) {
                s_step = HW_PARSE_SOF2;
            } else {
                s_step = HW_PARSE_SOF1;
            }
            break;
        }

        default:
            s_step = HW_PARSE_SOF1;
            break;
    }
}

void hiworld_conn_task_periodic(hiworld_connection_ctx_t *ctx, uint32_t now_millis) {
    if (!ctx) return;

    if (ctx->state == HIWORLD_LINK_WAIT_MODEL) {
        if (now_millis - ctx->last_handshake_millis >= 1000) {
            ctx->last_handshake_millis = now_millis;
            hiworld_conn_send_version(ctx);
        }
    }
}
