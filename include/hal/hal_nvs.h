#ifndef HAL_NVS_H
#define HAL_NVS_H

#include <stdint.h>
#include <stdbool.h>
#include "hal/hal_system.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t brand_code;
    uint8_t model_code;
    uint8_t magic;
} nvs_car_config_t;

hal_status_t hal_nvs_init(void);
hal_status_t hal_nvs_load_car_config(nvs_car_config_t *cfg);
hal_status_t hal_nvs_save_car_config(const nvs_car_config_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* HAL_NVS_H */
