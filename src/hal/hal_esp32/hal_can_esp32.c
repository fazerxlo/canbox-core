#include "hal/hal_can.h"
#include "driver/twai.h"
#include <string.h>

#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

static bool s_twai_installed = false;

hal_status_t hal_can_init(can_baudrate_t baudrate) {
    if (s_twai_installed) {
        twai_stop();
        twai_driver_uninstall();
        s_twai_installed = false;
    }

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, 
        CAN_RX_PIN, 
        TWAI_MODE_NORMAL
    );
    g_config.rx_queue_len = 32;
    g_config.tx_queue_len = 16;

    twai_timing_config_t t_config;
    switch (baudrate) {
        case CAN_BAUD_1M:
            t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
            break;
        case CAN_BAUD_500K:
            t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
            break;
        case CAN_BAUD_250K:
            t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
            break;
        case CAN_BAUD_125K:
            t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS();
            break;
        default:
            t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
            break;
    }

    // Accept all incoming frames
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        return HAL_ERROR;
    }

    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return HAL_ERROR;
    }

    s_twai_installed = true;
    return HAL_OK;
}

hal_status_t hal_can_set_filters(const can_filter_t *filters, uint8_t count) {
    // TWAI hardware accepts a single acceptance filter configuration window.
    // If complex software multi-filtering is required, filter inside can_router.c.
    (void)filters;
    (void)count;
    return HAL_OK;
}

hal_status_t hal_can_send(const can_frame_t *frame) {
    if (!s_twai_installed || !frame) {
        return HAL_ERROR;
    }

    twai_message_t tx_msg;
    memset(&tx_msg, 0, sizeof(tx_msg));

    tx_msg.identifier = frame->id;
    tx_msg.extd       = frame->is_extended ? 1 : 0;
    tx_msg.rtr        = frame->is_remote ? 1 : 0;
    tx_msg.data_length_code = (frame->dlc > CAN_MAX_DLC) ? CAN_MAX_DLC : frame->dlc;
    memcpy(tx_msg.data, frame->data, tx_msg.data_length_code);

    // Non-blocking transmission request (timeout = 0)
    esp_err_t err = twai_transmit(&tx_msg, 0);
    if (err == ESP_OK) {
        return HAL_OK;
    } else if (err == ESP_ERR_TIMEOUT) {
        return HAL_BUSY;
    }

    return HAL_ERROR;
}

hal_status_t hal_can_receive(can_frame_t *frame) {
    if (!s_twai_installed || !frame) {
        return HAL_ERROR;
    }

    twai_message_t rx_msg;
    // Non-blocking fetch from driver internal queue
    esp_err_t err = twai_receive(&rx_msg, 0);
    if (err == ESP_ERR_TIMEOUT) {
        return HAL_TIMEOUT;
    } else if (err != ESP_OK) {
        return HAL_ERROR;
    }

    frame->id          = rx_msg.identifier;
    frame->is_extended = rx_msg.extd;
    frame->is_remote   = rx_msg.rtr;
    frame->dlc         = rx_msg.data_length_code;
    memcpy(frame->data, rx_msg.data, rx_msg.data_length_code);
    frame->timestamp_ms = hal_get_tick_ms();

    return HAL_OK;
}
