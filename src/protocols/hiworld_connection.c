#include "protocols/hiworld_connection.h"
#include "protocols/hiworld_car_mapping.h"
#include "hal/hal_can.h"
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

void hiworld_conn_send_feature_enables(hiworld_connection_ctx_t *ctx) {
    if (!ctx) return;

    /* Feature Enable 1 (Cmd 0x71): Lighting, Locks, Wipers, Radar available */
    uint8_t feat1[2] = { 0xFF, 0xFF };
    send_hiworld_frame(ctx, HIWORLD_CMD_FEATURE_ENABLE1, feat1, 2);

    /* Feature Enable 2 (Cmd 0x72): TPMS Reset, Mirror Fold, ADAS available */
    uint8_t feat2[2] = { 0xFB, 0xFF };
    send_hiworld_frame(ctx, HIWORLD_CMD_FEATURE_ENABLE2, feat2, 2);
}

static void handle_parsed_command(hiworld_connection_ctx_t *ctx, uint8_t cmd_id, const uint8_t *payload, uint8_t len) {
    if (!ctx) return;

    switch (cmd_id) {
        case HIWORLD_CMD_CAR_TYPE_SET: {
            if (len >= 1) {
                ctx->car_model_id = payload[0];
                ctx->car_variant  = (len >= 2) ? payload[1] : 0;
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

                /* Acknowledge connection: Send Version and Feature Enables */
                hiworld_conn_send_version(ctx);
                hiworld_conn_send_feature_enables(ctx);

                ctx->state = HIWORLD_LINK_ACTIVE;
            }
            break;
        }

        case HIWORLD_CMD_VERSION_QUERY: {
            hiworld_conn_send_version(ctx);
            break;
        }

        case HIWORLD_CMD_DATE_TIME_SET: {
            /* GPS Time received from Android: Year, Month, Day, Hour, Min, 24H */
            if (len >= 6) {
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
