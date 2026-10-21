#include "profiles/peugeot_407.h"
#include "hal/hal_can.h"
#include <string.h>

static uint8_t s_cached_0x361_raw[6] = { 0x00, 0x00, 0x00, 0x00, 0x01, 0x00 };
static bool    s_cached_0x361_valid  = false;

/* Convert 4-bit delay code to seconds */
static inline uint8_t delay_code_to_sec(uint8_t code) {
    switch (code & 0x03) {
        case 1:  return 15;
        case 2:  return 30;
        case 3:  return 60;
        default: return 0;
    }
}

void psa_car_config_init(void) {
    s_cached_0x361_raw[0] = 0x00;
    s_cached_0x361_raw[1] = 0x00;
    s_cached_0x361_raw[2] = 0x00;
    s_cached_0x361_raw[3] = 0x00;
    s_cached_0x361_raw[4] = 0x01;
    s_cached_0x361_raw[5] = 0x00;
    s_cached_0x361_valid  = false;
}

void psa_decode_config_0x361(vehicle_config_t *cfg, const uint8_t *data, uint8_t dlc) {
    if (!cfg || !data || dlc < 6) return;

    memcpy(s_cached_0x361_raw, data, 6);
    s_cached_0x361_valid = true;

    cfg->follow_me_home         = (data[0] & 0x80) != 0;
    cfg->drl_enabled            = (data[0] & 0x40) != 0;
    cfg->rear_wiper_reverse     = (data[0] & 0x20) != 0;
    cfg->auto_door_lock         = (data[0] & 0x08) != 0;
    cfg->selective_boot_unlock  = (data[0] & 0x04) != 0;
    cfg->directional_headlights = (data[0] & 0x02) != 0;
    cfg->motorized_tailgate     = (data[0] & 0x01) != 0;

    cfg->follow_me_home_sec     = cfg->follow_me_home ? delay_code_to_sec(data[1] & 0x0F) : 0;
    cfg->ambient_lighting_level = data[2] & 0x0F;
    cfg->parking_radar_enabled  = (data[4] != 0x00);
}

hal_status_t psa_car_config_apply_downlink(uint8_t cmd_id, uint8_t setting_id, uint8_t val) {
    can_frame_t frame;
    memset(&frame, 0, sizeof(frame));

    if (cmd_id == 0x7B) {
        frame.id  = 0x361;
        frame.dlc = 6;
        memcpy(frame.data, s_cached_0x361_raw, 6);

        switch (setting_id) {
            case 0x01: /* Rear Wiper in Reverse */
                if (val) frame.data[0] |= 0x20;
                else     frame.data[0] &= ~0x20;
                break;
            case 0x04: /* Selective Door Unlocking */
                if (val) frame.data[0] |= 0x04;
                else     frame.data[0] &= ~0x04;
                break;
            case 0x05: /* Daytime Running Lights */
                if (val) frame.data[0] |= 0x40;
                else     frame.data[0] &= ~0x40;
                break;
            case 0x06: /* Follow-Me-Home Headlamp Delay */
                if (val == 0) {
                    frame.data[0] &= ~0x80;
                    frame.data[1] &= 0xF0;
                } else {
                    frame.data[0] |= 0x80;
                    frame.data[1]  = (frame.data[1] & 0xF0) | (val & 0x03);
                }
                break;
            case 0x0A: /* Ambient Lighting Brightness */
                if (val & 0x80) {
                    frame.data[2] = (frame.data[2] & 0xF0) | (val & 0x0F);
                } else {
                    frame.data[2] &= 0xF0;
                }
                break;
            case 0x0B: /* Parking Radar (AAS) Master Toggle */
                frame.data[4] = val ? 0x01 : 0x00;
                break;
            case 0x0C: /* Auto Door Lock while Driving */
                if (val) frame.data[0] |= 0x08;
                else     frame.data[0] &= ~0x08;
                break;
            default:
                return HAL_STATUS_ERROR;
        }

        /* Update local cache optimistically and inject to BSI */
        memcpy(s_cached_0x361_raw, frame.data, 6);
        s_cached_0x361_valid = true;
        return hal_can_send(&frame);
    }
    else if (cmd_id == 0x7D) {
        if (setting_id == 0x01) { /* Remote Boot Mode */
            frame.id = 0x361;
            frame.dlc = 6;
            memcpy(frame.data, s_cached_0x361_raw, 6);
            if (val) frame.data[0] |= 0x01;
            else     frame.data[0] &= ~0x01;
            memcpy(s_cached_0x361_raw, frame.data, 6);
            s_cached_0x361_valid = true;
            return hal_can_send(&frame);
        }
        else if (setting_id == 0x02) { /* Directional Headlights */
            frame.id = 0x361;
            frame.dlc = 6;
            memcpy(frame.data, s_cached_0x361_raw, 6);
            if (val) frame.data[0] |= 0x02;
            else     frame.data[0] &= ~0x02;
            memcpy(s_cached_0x361_raw, frame.data, 6);
            s_cached_0x361_valid = true;
            return hal_can_send(&frame);
        }
        else if (setting_id == 0x03) { /* TPMS Calibration Pulse */
            frame.id = 0x221;
            frame.dlc = 8;
            frame.data[3] = 0x10;
            return hal_can_send(&frame);
        }
        else if (setting_id == 0x0B) { /* Driver Profile Select */
            frame.id = 0x260;
            frame.dlc = 8;
            frame.data[3] = (val & 0x03) + 1;
            return hal_can_send(&frame);
        }
        else if (setting_id == 0x24) { /* Zone Temp Sync (Mono/Dual) */
            frame.id = 0x1D0;
            frame.dlc = 7;
            frame.data[0] = 0x01; /* DUAL_REQ */
            return hal_can_send(&frame);
        }
    }
    else if (cmd_id == 0x3B) {
        if (setting_id == 0x0F) { /* Climate Dual/Mono Sync (Section 3.3) */
            frame.id = 0x1D0;
            frame.dlc = 7;
            frame.data[0] = 0x01; /* DUAL_REQ */
            return hal_can_send(&frame);
        }
    }

    return HAL_STATUS_ERROR;
}

void psa_car_config_get_cached(vehicle_config_t *cfg) {
    if (cfg && s_cached_0x361_valid) {
        psa_decode_config_0x361(cfg, s_cached_0x361_raw, 6);
    }
}

