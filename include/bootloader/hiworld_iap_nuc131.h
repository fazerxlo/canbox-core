#ifndef HIWORLD_IAP_NUC131_H
#define HIWORLD_IAP_NUC131_H

#include <stdint.h>
#include <stdbool.h>

#define BOOT_TOKEN_ADDR         (0x20000000UL)
#define BOOT_TOKEN_MAGIC        (0x55AA55AAUL)

#define APP_BASE_ADDR           (0x00002000UL)
#define APP_MAX_ADDR            (0x00010000UL) /* 64 KB APROM */
#define FMC_PAGE_SIZE           (512U)

#define IAP_REC_SIZE            (136U)
#define IAP_DATA_SIZE           (128U)

void iap_init_hardware(void);
void iap_run(void);

#endif /* HIWORLD_IAP_NUC131_H */

