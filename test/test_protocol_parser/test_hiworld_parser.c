#include "unity.h"
#include "protocols/proto_hiworld.h"
#include "protocols/hiworld_connection.h"
#include "protocols/hiworld_car_mapping.h"
#include "hal/hal_can.h"
#include <string.h>

static hiworld_packet_t s_last_packet;
static int s_rx_call_count = 0;

static uint8_t s_mock_uart_tx_buf[256];
static size_t s_mock_uart_tx_len = 0;
static uint8_t s_mock_last_model_id = 0;
static uint32_t s_mock_last_baud = 0;

static can_frame_t s_mock_last_can_frame;
static bool s_mock_can_frame_sent = false;

static void mock_can_tx(const can_frame_t *frame) {
    if (frame) {
        s_mock_last_can_frame = *frame;
        s_mock_can_frame_sent = true;
    }
}

static void mock_uart_tx(const uint8_t *buf, size_t len) {
    if (len <= sizeof(s_mock_uart_tx_buf) - s_mock_uart_tx_len) {
        memcpy(&s_mock_uart_tx_buf[s_mock_uart_tx_len], buf, len);
        s_mock_uart_tx_len += len;
    }
}

static void mock_can_config(uint8_t car_model_id, uint32_t baud_rate) {
    s_mock_last_model_id = car_model_id;
    s_mock_last_baud = baud_rate;
}

static void test_hiworld_rx_callback(const hiworld_packet_t *packet) {
    s_rx_call_count++;
    s_last_packet = *packet;
}

void setUp_hiworld(void) {
    s_rx_call_count = 0;
    s_mock_uart_tx_len = 0;
    s_mock_last_model_id = 0;
    s_mock_last_baud = 0;
    s_mock_can_frame_sent = false;
    memset(&s_last_packet, 0, sizeof(s_last_packet));
    memset(s_mock_uart_tx_buf, 0, sizeof(s_mock_uart_tx_buf));
    memset(&s_mock_last_can_frame, 0, sizeof(s_mock_last_can_frame));
    proto_hiworld_init(test_hiworld_rx_callback);
}

void test_hiworld_serialize_valid_packet(void) {
    const uint8_t payload[] = { 0x01, 0x01 };
    uint8_t out[16];

    /* Payload len = 2, Cmd = 0x11, CS = (0x02 + 0x11 + 0x01 + 0x01 - 1) & 0xFF = 0x14 */
    size_t written = proto_hiworld_serialize(0x11, payload, sizeof(payload), out, sizeof(out));

    TEST_ASSERT_EQUAL_UINT32(7, written);
    TEST_ASSERT_EQUAL_HEX8(HIWORLD_SYNC_1, out[0]);
    TEST_ASSERT_EQUAL_HEX8(HIWORLD_SYNC_2, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x02, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x11, out[3]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, &out[4], 2);
    TEST_ASSERT_EQUAL_HEX8(0x14, out[6]);
}

void test_hiworld_parse_valid_stream(void) {
    /* 0x5A 0xA5, Len: 1, Cmd: 0x21, Payload: 0x05, CS: (1 + 0x21 + 0x05 - 1) = 0x26 */
    const uint8_t stream[] = { 0x5A, 0xA5, 0x01, 0x21, 0x05, 0x26 };

    for (size_t i = 0; i < sizeof(stream); i++) {
        proto_hiworld_feed_byte(stream[i]);
    }

    TEST_ASSERT_EQUAL_INT(1, s_rx_call_count);
    TEST_ASSERT_EQUAL_HEX8(0x21, s_last_packet.cmd);
    TEST_ASSERT_EQUAL_HEX8(1, s_last_packet.payload_len);
    TEST_ASSERT_EQUAL_HEX8(0x05, s_last_packet.payload[0]);
}

void test_hiworld_reject_bad_checksum(void) {
    const uint8_t stream[] = { 0x5A, 0xA5, 0x01, 0x21, 0x05, 0xFF };

    for (size_t i = 0; i < sizeof(stream); i++) {
        proto_hiworld_feed_byte(stream[i]);
    }

    TEST_ASSERT_EQUAL_INT(0, s_rx_call_count);
}

void test_hiworld_resync_after_false_sync1(void) {
    /* Extra 0x5A byte before real header: 5A 5A A5 00 FF FE (CS = (0 + 0xFF - 1) = 0xFE) */
    const uint8_t stream[] = { 0x5A, 0x5A, 0xA5, 0x00, 0xFF, 0xFE };

    for (size_t i = 0; i < sizeof(stream); i++) {
        proto_hiworld_feed_byte(stream[i]);
    }

    TEST_ASSERT_EQUAL_INT(1, s_rx_call_count);
    TEST_ASSERT_EQUAL_HEX8(0xFF, s_last_packet.cmd);
    TEST_ASSERT_EQUAL_HEX8(0, s_last_packet.payload_len);
}

/* Vector 1: Headunit Sets Vehicle Model to Peugeot 407 (Model 34 / 0x22) */
void test_hiworld_verification_vector_1_car_type_set(void) {
    hiworld_connection_ctx_t ctx;
    hiworld_conn_init(&ctx, "HW_PSA_V2.04", mock_uart_tx, mock_can_config);

    TEST_ASSERT_EQUAL_INT(HIWORLD_LINK_WAIT_MODEL, ctx.state);

    /* Downlink: 5A A5 02 24 22 00 47 */
    const uint8_t stream[] = { 0x5A, 0xA5, 0x02, 0x24, 0x22, 0x00, 0x47 };
    for (size_t i = 0; i < sizeof(stream); i++) {
        hiworld_conn_process_rx_byte(&ctx, stream[i]);
    }

    TEST_ASSERT_EQUAL_INT(HIWORLD_LINK_ACTIVE, ctx.state);
    TEST_ASSERT_EQUAL_HEX8(0x22, ctx.car_model_id);
    TEST_ASSERT_EQUAL_HEX8(0x00, ctx.car_variant);
    TEST_ASSERT_EQUAL_HEX8(34, s_mock_last_model_id);
    TEST_ASSERT_EQUAL_UINT32(125000, s_mock_last_baud);
    TEST_ASSERT_EQUAL_INT(VEHICLE_PROFILE_PSA_2004, vehicle_profile_get_active()->id);
}

/* Vector 2: CAN Box Acknowledges and Transmits Firmware Version */
void test_hiworld_verification_vector_2_version_report(void) {
    uint8_t frame[32];
    const char *ver_str = "HW_PSA_V2.0";
    size_t len = hiworld_conn_build_frame(HIWORLD_CMD_VERSION_REPORT, (const uint8_t *)ver_str, (uint8_t)strlen(ver_str), frame, sizeof(frame));

    /* CS = (11 + 0xF0 + 'H' + 'W' + '_' + 'P' + 'S' + 'A' + '_' + 'V' + '2' + '.' + '0' - 1) & 0xFF = 0x21 */
    const uint8_t expected[] = { 0x5A, 0xA5, 0x0B, 0xF0, 0x48, 0x57, 0x5F, 0x50, 0x53, 0x41, 0x5F, 0x56, 0x32, 0x2E, 0x30, 0x21 };

    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, frame, sizeof(expected));
}

/* Vector 3: CAN Box Broadcasts Feature Enable Bitmasks */
void test_hiworld_verification_vector_3_feature_enables(void) {
    uint8_t frame1[16];
    uint8_t feat1[2] = { 0xFF, 0xFF };
    size_t len1 = hiworld_conn_build_frame(HIWORLD_CMD_FEATURE_ENABLE1, feat1, sizeof(feat1), frame1, sizeof(frame1));

    /* CS1 = (0x02 + 0x71 + 0xFF + 0xFF - 1) & 0xFF = 0x70 */
    const uint8_t expected1[] = { 0x5A, 0xA5, 0x02, 0x71, 0xFF, 0xFF, 0x70 };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected1), len1);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected1, frame1, sizeof(expected1));

    uint8_t frame2[16];
    uint8_t feat2[2] = { 0xFB, 0xFF };
    size_t len2 = hiworld_conn_build_frame(HIWORLD_CMD_FEATURE_ENABLE2, feat2, sizeof(feat2), frame2, sizeof(frame2));

    /* CS2 = (0x02 + 0x72 + 0xFB + 0xFF - 1) & 0xFF = 0x6D */
    const uint8_t expected2[] = { 0x5A, 0xA5, 0x02, 0x72, 0xFB, 0xFF, 0x6D };
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected2), len2);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected2, frame2, sizeof(expected2));
}

/* Vector 4: Headunit Synchronizes GPS Clock (22 Sept 2026, 15:30, 24H) */
void test_hiworld_verification_vector_4_gps_time_sync(void) {
    hiworld_connection_ctx_t ctx;
    hiworld_conn_init(&ctx, "HW_PSA_V2.04", mock_uart_tx, mock_can_config);
    ctx.can_tx = mock_can_tx;

    /* Downlink: 5A A5 06 CB 1A 09 16 0F 1E 01 37 */
    /* CS = (0x06 + 0xCB + 0x1A + 0x09 + 0x16 + 0x0F + 0x1E + 0x01 - 1) & 0xFF = 0x37 */
    const uint8_t stream[] = { 0x5A, 0xA5, 0x06, 0xCB, 0x1A, 0x09, 0x16, 0x0F, 0x1E, 0x01, 0x37 };
    for (size_t i = 0; i < sizeof(stream); i++) {
        hiworld_conn_process_rx_byte(&ctx, stream[i]);
    }

    TEST_ASSERT_TRUE(s_mock_can_frame_sent);
    TEST_ASSERT_EQUAL_HEX32(0x228, s_mock_last_can_frame.id);
    TEST_ASSERT_EQUAL_UINT8(8, s_mock_last_can_frame.dlc);

    /* Expected PSA CAN ID 0x228 payload: [Hour=0x0F, Min=0x1E, Day=0x16, Month=0x09, Year=0x1A, 0, 0, 0] */
    const uint8_t expected_can[8] = { 0x0F, 0x1E, 0x16, 0x09, 0x1A, 0x00, 0x00, 0x00 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_can, s_mock_last_can_frame.data, 8);
}

void test_hiworld_connection_periodic_ping(void) {
    hiworld_connection_ctx_t ctx;
    hiworld_conn_init(&ctx, "HW_TEST", mock_uart_tx, mock_can_config);

    TEST_ASSERT_EQUAL_INT(HIWORLD_LINK_WAIT_MODEL, ctx.state);
    s_mock_uart_tx_len = 0;

    /* No ping before 1000ms */
    hiworld_conn_task_periodic(&ctx, 500);
    TEST_ASSERT_EQUAL_UINT32(0, s_mock_uart_tx_len);

    /* Periodic ping triggered at 1000ms */
    hiworld_conn_task_periodic(&ctx, 1000);
    TEST_ASSERT_TRUE(s_mock_uart_tx_len > 0);

    /* Check that reported packet is Cmd 0xF0 */
    TEST_ASSERT_EQUAL_HEX8(HIWORLD_SOF1, s_mock_uart_tx_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(HIWORLD_SOF2, s_mock_uart_tx_buf[1]);
    TEST_ASSERT_EQUAL_HEX8(HIWORLD_CMD_VERSION_REPORT, s_mock_uart_tx_buf[3]);
}

void test_hiworld_car_mapping_lookup(void) {
    vehicle_profile_id_t profile;

    TEST_ASSERT_TRUE(hiworld_car_mapping_get_profile(HIWORLD_CAR_MODEL_PEUGEOT_407_06, &profile));
    TEST_ASSERT_EQUAL_INT(VEHICLE_PROFILE_PSA_2004, profile);
    TEST_ASSERT_EQUAL_UINT32(125000, hiworld_car_mapping_get_baud_rate(HIWORLD_CAR_MODEL_PEUGEOT_407_06));

    TEST_ASSERT_TRUE(hiworld_car_mapping_get_profile(HIWORLD_CAR_MODEL_PEUGEOT_308S_408_14, &profile));
    TEST_ASSERT_EQUAL_INT(VEHICLE_PROFILE_PSA_2004, profile);
    TEST_ASSERT_EQUAL_UINT32(500000, hiworld_car_mapping_get_baud_rate(HIWORLD_CAR_MODEL_PEUGEOT_308S_408_14));

    TEST_ASSERT_FALSE(hiworld_car_mapping_get_profile(0xFF, &profile));

    uint8_t model_id;
    TEST_ASSERT_TRUE(hiworld_car_mapping_get_model_id(VEHICLE_PROFILE_PSA_2004, &model_id));
}
