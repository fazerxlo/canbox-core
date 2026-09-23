#ifndef HIWORLD_CONNECTION_H
#define HIWORLD_CONNECTION_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "hal/hal_can.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HIWORLD_SOF1  0x5A
#define HIWORLD_SOF2  0xA5

/* Vehicle Model IDs (Decompiled from PeugeotDataDefine.adaptCurrentCarCommand) */
#define HIWORLD_CAR_MODEL_CITROEN_C_QUATRE_08  1   /* 0x01 */
#define HIWORLD_CAR_MODEL_CITROEN_C4_16        2   /* 0x02 */
#define HIWORLD_CAR_MODEL_CITROEN_C4L_13       3   /* 0x03 */
#define HIWORLD_CAR_MODEL_CITROEN_C5_10        4   /* 0x04 */
#define HIWORLD_CAR_MODEL_CITROEN_C5_13        5   /* 0x05 */
#define HIWORLD_CAR_MODEL_PEUGEOT_307_04       6   /* 0x06 */
#define HIWORLD_CAR_MODEL_PEUGEOT_308_12       7   /* 0x07 */
#define HIWORLD_CAR_MODEL_PEUGEOT_408_10       8   /* 0x08 */
#define HIWORLD_CAR_MODEL_PEUGEOT_508_LOW_11   9   /* 0x09 */
#define HIWORLD_CAR_MODEL_PEUGEOT_508_HIGH_11  10  /* 0x0A */
#define HIWORLD_CAR_MODEL_PEUGEOT_3008_13      11  /* 0x0B */
#define HIWORLD_CAR_MODEL_CITROEN_DS5_12       12  /* 0x0C */
#define HIWORLD_CAR_MODEL_CITROEN_DS5LS_12     13  /* 0x0D */
#define HIWORLD_CAR_MODEL_PEUGEOT_2008_14      14  /* 0x0E */
#define HIWORLD_CAR_MODEL_CITROEN_DS4_12       15  /* 0x0F */
#define HIWORLD_CAR_MODEL_PEUGEOT_308S_408_14  16  /* 0x10 */
#define HIWORLD_CAR_MODEL_PEUGEOT_3008_KEEP_13 17  /* 0x11 */
#define HIWORLD_CAR_MODEL_PEUGEOT_301_12       18  /* 0x12 */
#define HIWORLD_CAR_MODEL_CITROEN_C3_XR_15     19  /* 0x13 */
#define HIWORLD_CAR_MODEL_PEUGEOT_4008_5008_17 20  /* 0x14 */
#define HIWORLD_CAR_MODEL_PEUGEOT_508_FL_15    21  /* 0x15 */
#define HIWORLD_CAR_MODEL_CITROEN_DS6_16       22  /* 0x16 */
#define HIWORLD_CAR_MODEL_PEUGEOT_301_19       23  /* 0x17 */
#define HIWORLD_CAR_MODEL_PEUGEOT_RIFTER_HI_19 24  /* 0x18 */
#define HIWORLD_CAR_MODEL_PEUGEOT_RIFTER_LO_19 25  /* 0x19 */
#define HIWORLD_CAR_MODEL_CITROEN_TIANYI_C5_17 32  /* 0x20 */
#define HIWORLD_CAR_MODEL_PEUGEOT_308_CC_11    33  /* 0x21 */
#define HIWORLD_CAR_MODEL_PEUGEOT_407_06       34  /* 0x22 */
#define HIWORLD_CAR_MODEL_OPEL_COMBO_CORSA_19  35  /* 0x23 */
#define HIWORLD_CAR_MODEL_CITROEN_C3_23        36  /* 0x24 */
#define HIWORLD_CAR_MODEL_CITROEN_C4_09        37  /* 0x25 */
#define HIWORLD_CAR_MODEL_PEUGEOT_3008_22      38  /* 0x26 */
#define HIWORLD_CAR_MODEL_PEUGEOT_PARTNER_09   39  /* 0x27 */
#define HIWORLD_CAR_MODEL_CITROEN_BERLINGO_17  40  /* 0x28 */

/* Master Command IDs - Inbound Uplink (CAN Box -> HU) */
#define HIWORLD_CMD_CAR_BASE_INFO         0x11 /* 17 */
#define HIWORLD_CMD_DOOR_WINDOW_STATE     0x12 /* 18 */
#define HIWORLD_CMD_ECU_INFO_PAGE1        0x13 /* 19 */
#define HIWORLD_CMD_ECU_INFO_PAGE2        0x14 /* 20 */
#define HIWORLD_CMD_ECU_INFO_PAGE3        0x15 /* 21 */
#define HIWORLD_CMD_CONTROL_PANEL_KEY     0x21 /* 33 */
#define HIWORLD_CMD_CONTROL_PANEL_KNOB    0x22 /* 34 */
#define HIWORLD_CMD_CAR_AC_STATE          0x31 /* 49 */
#define HIWORLD_CMD_CAR_RADAR_STATE       0x41 /* 65 */
#define HIWORLD_CMD_WARNING_INFO          0x42 /* 66 */
#define HIWORLD_CMD_FEATURE_ENABLE1       0x71 /* 113 */
#define HIWORLD_CMD_FEATURE_ENABLE2       0x72 /* 114 */
#define HIWORLD_CMD_CENTRAL_STATE1        0x76 /* 118 */
#define HIWORLD_CMD_CENTRAL_STATE2        0x79 /* 121 */
#define HIWORLD_CMD_HOST_INFO             0x93 /* 147 */
#define HIWORLD_CMD_CAR_MEDIA_STATE       0x97 /* 151 */
#define HIWORLD_CMD_UNIT_INFO             0xC1 /* 193 */
#define HIWORLD_CMD_DATE_TIME_INFO        0xC2 /* 194 */
#define HIWORLD_CMD_VERSION_REPORT        0xF0 /* 240 */

/* Master Command IDs - Outbound Downlink (HU -> CAN Box) */
#define HIWORLD_CMD_ECU_SETTING_SET       0x1B /* 27 */
#define HIWORLD_CMD_CAR_TYPE_SET          0x24 /* 36 */
#define HIWORLD_CMD_VERSION_QUERY         0x30 /* 48 */
#define HIWORLD_CMD_AC_SETTING_SET        0x3B /* 59 */
#define HIWORLD_CMD_CENTRAL_SETTING1      0x7B /* 123 */
#define HIWORLD_CMD_CENTRAL_SETTING2      0x7D /* 125 */
#define HIWORLD_CMD_SPEED_VALUE_SET       0x8A /* 138 */
#define HIWORLD_CMD_CRUISE_SPEED_SET      0x8B /* 139 */
#define HIWORLD_CMD_LANGUAGE_SET          0x9A /* 154 */
#define HIWORLD_CMD_HOST_STATE_SET        0xA1 /* 161 */
#define HIWORLD_CMD_TUNER_SET             0xA2 /* 162 */
#define HIWORLD_CMD_CD_SET                0xA4 /* 164 */
#define HIWORLD_CMD_DSP_STATE_SET         0xAD /* 173 */
#define HIWORLD_CMD_UNIT_SET              0xCA /* 204 */
#define HIWORLD_CMD_DATE_TIME_SET         0xCB /* 203 */
#define HIWORLD_CMD_HOST_SOURCE_INFO      0xE1 /* 225 */
#define HIWORLD_CMD_ID3_INFO              0xE4 /* 228 */

/* Connection State Machine States */
typedef enum {
    HIWORLD_LINK_DISCONNECTED = 0,
    HIWORLD_LINK_WAIT_MODEL   = 1,
    HIWORLD_LINK_INITIALIZED  = 2,
    HIWORLD_LINK_ACTIVE       = 3
} hiworld_link_state_t;

/* Serial and CAN Configuration Callbacks */
typedef void (*hiworld_uart_tx_fn)(const uint8_t *buf, size_t len);
typedef void (*hiworld_can_config_fn)(uint8_t car_model_id, uint32_t baud_rate);
typedef void (*hiworld_can_tx_fn)(const can_frame_t *frame);

typedef struct {
    hiworld_link_state_t state;
    uint8_t              car_model_id;
    uint8_t              car_variant;
    char                 fw_version[20];

    uint32_t             last_rx_millis;
    uint32_t             last_handshake_millis;

    hiworld_uart_tx_fn    uart_tx;
    hiworld_can_config_fn can_config;
    hiworld_can_tx_fn     can_tx;
} hiworld_connection_ctx_t;

/* Checksum calculation helper */
static inline uint8_t hiworld_calc_checksum(const uint8_t *frame, size_t total_len) {
    if (!frame || total_len < 5) {
        return 0;
    }
    uint8_t payload_len = frame[2];
    uint8_t sum = 0;
    for (size_t i = 2; i < (size_t)(payload_len + 4) && i < total_len; i++) {
        sum = (uint8_t)(sum + frame[i]);
    }
    return (uint8_t)((sum - 1) & 0xFF);
}

/* Public API */
void hiworld_conn_init(hiworld_connection_ctx_t *ctx,
                       const char *fw_version_str,
                       hiworld_uart_tx_fn uart_tx,
                       hiworld_can_config_fn can_config);

void hiworld_conn_process_rx_byte(hiworld_connection_ctx_t *ctx, uint8_t byte);
void hiworld_conn_task_periodic(hiworld_connection_ctx_t *ctx, uint32_t now_millis);
void hiworld_conn_send_version(hiworld_connection_ctx_t *ctx);
void hiworld_conn_send_feature_enables(hiworld_connection_ctx_t *ctx);
size_t hiworld_conn_build_frame(uint8_t cmd_id, const uint8_t *payload, uint8_t len, uint8_t *out_buf, size_t max_out);

#ifdef __cplusplus
}
#endif

#endif /* HIWORLD_CONNECTION_H */
