#ifndef CANBOX_VERSION_H
#define CANBOX_VERSION_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CANBOX_BUILD_VERSION
#define CANBOX_BUILD_VERSION "CANBOX-CORE-Vdev"
#endif

/**
 * @brief Get the embedded firmware version string.
 *
 * For development builds, formatted as "CANBOX-CORE-V<YYYYMMDD.hhmmss>".
 * Can be overridden at build time via CANBOX_BUILD_VERSION preprocessor define
 * or environment variable.
 *
 * @return Null-terminated ASCII version string.
 */
const char *canbox_get_version(void);

#ifdef __cplusplus
}
#endif

#endif /* CANBOX_VERSION_H */

