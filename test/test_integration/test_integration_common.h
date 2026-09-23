#ifndef TEST_INTEGRATION_COMMON_H
#define TEST_INTEGRATION_COMMON_H

#include "unity.h"
#include "core/can_router.h"
#include "protocols/hu_protocol_driver.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

size_t read_uart_output(uint8_t *buf, size_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* TEST_INTEGRATION_COMMON_H */

