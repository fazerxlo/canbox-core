set(srcs
    "main_esp32.c"
    "core/can_router.c"
    "core/ring_buffer.c"
    "core/vehicle_profile.c"
    "profiles/profile_psa.c"
    "profiles/profile_vag.c"
    "protocols/canbox_parser.c"
    "protocols/hu_protocol.c"
    "protocols/proto_hiworld.c"
    "protocols/proto_hiworld_adapter.c"
    "protocols/proto_raise.c"
    "protocols/proto_raise_adapter.c"
    "protocols/raise_car_mapping.c"
    "hal/hal_esp32/hal_can_esp32.c"
    "hal/hal_esp32/hal_gpio_esp32.c"
    "hal/hal_esp32/hal_system_esp32.c"
    "hal/hal_esp32/hal_uart_esp32.c"
)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "." "../include")
