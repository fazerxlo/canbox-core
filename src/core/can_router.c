#include "core/can_router.h"
#include "core/vehicle_profile.h"
#include "protocols/canbox_parser.h"
#include <string.h>

static vehicle_state_t s_vehicle_state;
static canbox_parser_t s_uart_parser;

static void on_hu_frame_received(const canbox_rx_frame_t *frame) {
    (void)frame;
    // Handle head unit commands (e.g., settings request, climate change, etc.)
}

void can_router_init(void) {
    memset(&s_vehicle_state, 0, sizeof(s_vehicle_state));
    vehicle_profile_set_active(VEHICLE_PROFILE_PSA_2004);
    canbox_parser_init(&s_uart_parser, CANBOX_DIALECT_RAISE, on_hu_frame_received);
}

void can_router_process_can(const can_frame_t *frame) {
    vehicle_profile_process_frame(frame, &s_vehicle_state);
}

void can_router_process_uart_byte(uint8_t byte) {
    canbox_parser_feed_byte(&s_uart_parser, byte);
}

void can_router_periodic_100ms(void) {
    // Periodic processing / heartbeat / status push
}

const vehicle_state_t *can_router_get_state(void) {
    return &s_vehicle_state;
}

