#include "profiles/peugeot_407.h"
#include "core/can_router.h"
#include "hal/hal_can.h"
#include "hal/hal_system.h"
#include <string.h>

static uint8_t  s_cached_0x361_raw[6] = { 0x01, 0x01, 0x91, 0x40, 0x30, 0x10 };
static bool     s_cached_0x361_valid  = false;

/* Live cache of active Profile 1 status frame (0x260 / 0x15B) */
static uint8_t  s_cached_profile1_raw[8] = { 0x01, 0x03, 0xB1, 0x40, 0x00, 0xD0, 0x00, 0x10 };
static bool     s_cached_profile1_valid  = false;
static uint32_t s_last_mdf_seen_ms       = 0;

static uint8_t s_mdf_trip_page_mask        = PSA_MDF_TRIP_PAGE_INSTANT; /* Default: Instant (0x09) */
static uint8_t s_mdf_trip_reset_mask       = 0x00; /* 0x80=Reset Trip1, 0x40=Reset Trip2 */
static uint8_t s_mdf_reset_ticks           = 0;
static uint8_t s_mdf_trip_inactivity_ticks = 0;    /* 7-second inactivity timeout (70 ticks of 100ms) */
static uint8_t s_periodic_1000ms_ticks     = 0;
static uint8_t s_mdf_3f6_counter           = 0;

/* Feature enables (Cmd 0x71 and Cmd 0x72) telecoded in BSI */
static uint8_t s_feature_enable_1[2] = { 0x60, 0x12 };
static uint8_t s_feature_enable_2[8] = { 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* Convert Byte 2 delay code to seconds:
 * Bit 2 (0x04) = 60s
 * Bit 1 (0x02) = 30s
 * Bit 0 (0x01) = 15s
 * Also supports sequential 0x03 = 60s. Default minimum is 15s. */
static inline uint8_t delay_code_to_sec(uint8_t byte2) {
    if (byte2 & 0x04) return 60;
    if ((byte2 & 0x03) == 3) return 60;
    if (byte2 & 0x02) return 30;
    if (byte2 & 0x01) return 15;
    return 15;
}

void psa_car_config_init(void) {
    s_cached_0x361_raw[0] = 0x01;
    s_cached_0x361_raw[1] = 0x01;
    s_cached_0x361_raw[2] = 0x91;
    s_cached_0x361_raw[3] = 0x40;
    s_cached_0x361_raw[4] = 0x30;
    s_cached_0x361_raw[5] = 0x10;
    s_cached_0x361_valid  = false;

    s_cached_profile1_raw[0] = 0x01;
    s_cached_profile1_raw[1] = 0x03;
    s_cached_profile1_raw[2] = 0xB1;
    s_cached_profile1_raw[3] = 0x40;
    s_cached_profile1_raw[4] = 0x00;
    s_cached_profile1_raw[5] = 0xD0;
    s_cached_profile1_raw[6] = 0x00;
    s_cached_profile1_raw[7] = 0x10;
    s_cached_profile1_valid  = false;

    s_last_mdf_seen_ms = 0;

    s_mdf_trip_page_mask        = PSA_MDF_TRIP_PAGE_INSTANT;
    s_mdf_trip_reset_mask       = 0x00;
    s_mdf_reset_ticks           = 0;
    s_mdf_trip_inactivity_ticks = 0;
    s_periodic_1000ms_ticks     = 0;
    s_mdf_3f6_counter           = 0;

    s_feature_enable_1[0] = 0x60;
    s_feature_enable_1[1] = 0x12;

    memset(s_feature_enable_2, 0, sizeof(s_feature_enable_2));
    s_feature_enable_2[0] = 0x00;
    s_feature_enable_2[1] = 0x01;
    s_feature_enable_2[2] = 0x80;
}

void psa_car_config_notify_mdf_seen(void) {
    s_last_mdf_seen_ms = hal_get_tick_ms();
}

bool psa_car_config_is_mdf_present(void) {
    if (s_last_mdf_seen_ms == 0) return false;
    return (hal_get_tick_ms() - s_last_mdf_seen_ms) < 3000;
}

bool psa_car_config_is_cached_valid(void) {
    return s_cached_profile1_valid;
}

void psa_car_config_update_from_bsi(const uint8_t *data, uint8_t dlc) {
    if (!data || dlc < 8) return;
    if (data[0] != 0x01) return;

    memcpy(s_cached_profile1_raw, data, 8);
    s_cached_profile1_valid = true;
}

void psa_decode_config_0x361(const uint8_t *data, uint8_t dlc) {
    if (!data || dlc < 6) return;

    memcpy(s_cached_0x361_raw, data, 6);
    s_cached_0x361_valid = true;

    /*
     * 0x361 is Menu Availability (Equipment Mask), NOT Current Settings.
     * Byte 0: profile_number
     * Byte 2: auto_lighting (bit 0), follow_me_home (bit 4), hinge_panel (bit 7)
     * Byte 3: drl_present (bit 6)
     * Byte 4: emf_present (bit 4), aas_disable (bit 5)
     * Byte 5: tpms_present (bits 4..6)
     */

    /* Cmd 0x71: CentralStateEnable1 (2 bytes) */
    uint8_t feat1_b0 = 0x80; /* Ambient mood lighting menu enabled */
    if (data[2] & 0x10) {
        feat1_b0 |= 0x03; /* Follow-Me-Home delay supported */
    }
    if (data[3] & 0x40) {
        feat1_b0 |= 0x04; /* Daytime running lights (DRL) supported */
    }
    if (data[4] & 0x20) {
        feat1_b0 |= 0x20; /* Parking Radar (AAS) toggle supported */
    }
    s_feature_enable_1[0] = feat1_b0;
    s_feature_enable_1[1] = 0x80 | 0x20 | 0x08; /* Rear Wiper, Auto Door Lock, Selective Boot */

    /* Cmd 0x72: CentralStateEnable2 (8 bytes) */
    memset(s_feature_enable_2, 0, sizeof(s_feature_enable_2));
    uint8_t feat2_b0 = 0x01; /* Drive Mode Profile supported */
    if (data[2] & 0x80) {
        feat2_b0 |= 0x80; /* Hinge panel / remote trunk */
    }
    if (data[2] & 0x01) {
        feat2_b0 |= 0x40; /* Directional / adaptive lighting */
    }
    if (data[5] & 0x70) {
        feat2_b0 |= 0x02; /* TPMS calibration reset */
    }
    s_feature_enable_2[0] = feat2_b0;
    s_feature_enable_2[1] = 0x04; /* Cluster dimmer */
    s_feature_enable_2[5] = 0x02; /* Zone temp sync */
}

void psa_car_config_get_feature_enables(uint8_t feat1[2], uint8_t feat2[8]) {
    if (feat1) {
        memcpy(feat1, s_feature_enable_1, 2);
    }
    if (feat2) {
        memcpy(feat2, s_feature_enable_2, 8);
    }
}

void psa_decode_config_0x260(vehicle_config_t *cfg, const uint8_t *data, uint8_t dlc) {
    if (!cfg || !data || dlc < 8) return;

    /* Profile 1 is the active user personalization profile on Peugeot 407 */
    if (data[0] != 0x01) {
        return;
    }

    cfg->active_profile_id      = 1;
    cfg->directional_headlights = (data[2] & 0x80) != 0;
    cfg->follow_me_home         = (data[2] & 0x20) != 0;
    cfg->follow_me_home_sec     = cfg->follow_me_home ? delay_code_to_sec(data[2]) : 0;
    cfg->auto_lighting_enabled  = (data[2] & 0x10) != 0;
    cfg->drl_enabled            = (data[3] & 0x40) != 0;
    cfg->auto_door_lock         = (data[4] & 0x08) != 0;
    cfg->selective_boot_unlock  = (data[4] & 0x04) != 0;
    cfg->motorized_tailgate     = (data[5] & 0x80) != 0;
    cfg->parking_radar_enabled  = (data[5] & 0x40) != 0;
    cfg->rear_wiper_reverse     = (data[5] & 0x10) != 0;
}

hal_status_t psa_car_config_apply_downlink(uint8_t cmd_id, uint8_t setting_id, uint8_t val) {
    can_frame_t frame;
    memset(&frame, 0, sizeof(frame));

    if (cmd_id == 0x7B) {
        frame.id  = PSA_CAN_ID_BSI_PERSO_REQ_15B; /* 0x15B */
        frame.dlc = 8;
        memcpy(frame.data, s_cached_profile1_raw, 8);
        frame.data[0] = 0x01; /* Profile 1, Bit 2 cleared = pending configuration write request */

        switch (setting_id) {
            case 0x01: /* Rear Wiper in Reverse */
                if (val) frame.data[5] |= 0x10;
                else     frame.data[5] &= ~0x10;
                break;
            case 0x04: /* Selective Door Unlocking */
                if (val) frame.data[4] |= 0x04;
                else     frame.data[4] &= ~0x04;
                break;
            case 0x05: /* Daytime Running Lights */
                if (val) frame.data[3] |= 0x40;
                else     frame.data[3] &= ~0x40;
                break;
            case 0x06: /* Follow-Me-Home Headlamp Delay */
                frame.data[2] &= ~0x27; /* Clear bit 5 (enable) and bits 2..0 (duration) */
                if (val == 1) {
                    frame.data[2] |= (0x20 | 0x01); /* 15s */
                } else if (val == 2) {
                    frame.data[2] |= (0x20 | 0x02); /* 30s */
                } else if (val == 3) {
                    frame.data[2] |= (0x20 | 0x04); /* 60s */
                } else {
                    frame.data[2] |= 0x01; /* 0 = OFF (bit 5 cleared), default 15s base */
                }
                break;
            case 0x0A: /* Ambient Lighting Brightness */
                if (val & 0x80) {
                    frame.data[2] = (uint8_t)((frame.data[2] & 0xF0) | (val & 0x0F));
                } else {
                    frame.data[2] &= 0xF0;
                }
                break;
            case 0x0B: /* Parking Radar (AAS) Master Toggle */
                if (val) frame.data[5] |= 0x40;
                else     frame.data[5] &= ~0x40;
                break;
            case 0x0C: /* Auto Door Lock while Driving */
                if (val) frame.data[4] |= 0x08;
                else     frame.data[4] &= ~0x08;
                break;
            default:
                return HAL_STATUS_ERROR;
        }

        /* Update local cache optimistically and inject to BSI */
        memcpy(s_cached_profile1_raw, frame.data, 8);
        s_cached_profile1_valid = true;
        return hal_can_send(&frame);
    }
    else if (cmd_id == 0x7D) {
        if (setting_id == 0x01) { /* Remote Boot Mode */
            frame.id  = PSA_CAN_ID_BSI_PERSO_REQ_15B;
            frame.dlc = 8;
            memcpy(frame.data, s_cached_profile1_raw, 8);
            frame.data[0] = 0x01;
            if (val) frame.data[5] |= 0x80;
            else     frame.data[5] &= ~0x80;
            memcpy(s_cached_profile1_raw, frame.data, 8);
            s_cached_profile1_valid = true;
            return hal_can_send(&frame);
        }
        else if (setting_id == 0x02) { /* Directional Headlights */
            frame.id  = PSA_CAN_ID_BSI_PERSO_REQ_15B;
            frame.dlc = 8;
            memcpy(frame.data, s_cached_profile1_raw, 8);
            frame.data[0] = 0x01;
            if (val) frame.data[2] |= 0x80;
            else     frame.data[2] &= ~0x80;
            memcpy(s_cached_profile1_raw, frame.data, 8);
            s_cached_profile1_valid = true;
            return hal_can_send(&frame);
        }
        else if (setting_id == 0x03) { /* TPMS Calibration Pulse */
            frame.id = 0x221;
            frame.dlc = 8;
            frame.data[3] = 0x10;
            return hal_can_send(&frame);
        }
        else if (setting_id == 0x0B) { /* Driver Profile Select */
            frame.id  = PSA_CAN_ID_BSI_PERSO_REQ_15B;
            frame.dlc = 8;
            memcpy(frame.data, s_cached_profile1_raw, 8);
            frame.data[0] = (uint8_t)((val & 0x03) + 1); /* Profile ID with bit 2 cleared */
            memcpy(s_cached_profile1_raw, frame.data, 8);
            s_cached_profile1_valid = true;
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
    if (cfg && s_cached_profile1_valid) {
        psa_decode_config_0x260(cfg, s_cached_profile1_raw, 8);
    }
}

static hal_status_t psa_car_config_send_0x167(void) {
    can_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.id      = PSA_CAN_ID_MDF_TRIP_CMD_167;
    frame.dlc     = 8;
    frame.data[0] = (uint8_t)(s_mdf_trip_page_mask | s_mdf_trip_reset_mask);
    frame.data[1] = 0x06;
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    return hal_can_send(&frame);
}

uint8_t psa_car_config_get_trip_page_mask(void) {
    return s_mdf_trip_page_mask;
}

void psa_car_config_set_trip_reset(uint8_t trip_index) {
    if (trip_index == 1) {
        s_mdf_trip_page_mask        = PSA_MDF_TRIP_PAGE_TRIP1;
        s_mdf_trip_reset_mask       = 0x80;
        s_mdf_reset_ticks           = 3; /* 300 ms pulse */
        s_mdf_trip_inactivity_ticks = 70;
    } else if (trip_index == 2) {
        s_mdf_trip_page_mask        = PSA_MDF_TRIP_PAGE_TRIP2;
        s_mdf_trip_reset_mask       = 0x40;
        s_mdf_reset_ticks           = 3; /* 300 ms pulse */
        s_mdf_trip_inactivity_ticks = 70;
    }
}

void psa_car_config_cycle_trip_page(void) {
    if (psa_car_config_is_mdf_present()) return;
    if (s_mdf_trip_page_mask == PSA_MDF_TRIP_PAGE_INSTANT) {
        s_mdf_trip_page_mask        = PSA_MDF_TRIP_PAGE_TRIP1;
        s_mdf_trip_inactivity_ticks = 70; /* 7-second timeout to revert to Instant */
    } else if (s_mdf_trip_page_mask == PSA_MDF_TRIP_PAGE_TRIP1) {
        s_mdf_trip_page_mask        = PSA_MDF_TRIP_PAGE_TRIP2;
        s_mdf_trip_inactivity_ticks = 70; /* 7-second timeout to revert to Instant */
    } else {
        s_mdf_trip_page_mask        = PSA_MDF_TRIP_PAGE_INSTANT;
        s_mdf_trip_inactivity_ticks = 0; /* Default page has no timeout */
    }
    psa_car_config_send_0x167();
}

void psa_car_config_periodic_100ms(void) {
    if (psa_car_config_is_mdf_present()) {
        return;
    }

    const vehicle_state_t *st = can_router_get_state();
    if (st == NULL || (!st->ignition_on && st->media.radio.power_status == 0)) {
        s_periodic_1000ms_ticks = 0;
        return;
    }

    if (s_mdf_reset_ticks > 0) {
        s_mdf_reset_ticks--;
        if (s_mdf_reset_ticks == 0) {
            s_mdf_trip_reset_mask = 0x00;
        }
        psa_car_config_send_0x167();
    }

    if (s_mdf_trip_inactivity_ticks > 0) {
        s_mdf_trip_inactivity_ticks--;
        if (s_mdf_trip_inactivity_ticks == 0) {
            s_mdf_trip_page_mask = PSA_MDF_TRIP_PAGE_INSTANT;
            psa_car_config_send_0x167();
        }
    }

    if (++s_periodic_1000ms_ticks >= 10) {
        s_periodic_1000ms_ticks = 0;

        /* 0x525 (DLC 8): Virtual MDF presence / diagnostic heartbeat: 05 00 01 00 00 00 00 00 */
        can_frame_t frame_525;
        memset(&frame_525, 0, sizeof(frame_525));
        frame_525.id = PSA_CAN_ID_MDF_DIAG_525;
        frame_525.dlc = 8;
        frame_525.data[0] = 0x05;
        frame_525.data[2] = 0x01;
        hal_can_send(&frame_525);

        /* 0x3F6 (DLC 7): 00 00 <counter> 00 00 C0 01 */
        can_frame_t frame_3f6;
        memset(&frame_3f6, 0, sizeof(frame_3f6));
        frame_3f6.id = PSA_CAN_ID_MDF_TRIP_3F6;
        frame_3f6.dlc = 7;
        frame_3f6.data[2] = s_mdf_3f6_counter++;
        frame_3f6.data[5] = 0xC0;
        frame_3f6.data[6] = 0x01;
        hal_can_send(&frame_3f6);

        /* 0x0DF (DLC 3): 10 00 50 */
        can_frame_t frame_0df;
        memset(&frame_0df, 0, sizeof(frame_0df));
        frame_0df.id = PSA_CAN_ID_MDF_STATUS_0DF;
        frame_0df.dlc = 3;
        frame_0df.data[0] = 0x10;
        frame_0df.data[1] = 0x00;
        frame_0df.data[2] = 0x50;
        hal_can_send(&frame_0df);

        /* 0x167 (DLC 8): Virtual MDF trip page supervisory frame: 09 06 FF FF 00 00 00 00 */
        psa_car_config_send_0x167();
    }
}
