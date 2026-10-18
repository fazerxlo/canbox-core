#include "profiles/peugeot_407.h"
#include "profiles/psa_virtual_cdc.h"
#include <string.h>

/* --------------------------------------------------------------------------
 * 1.7 OEM JBL Sound Amplifier (DSP)
 * -------------------------------------------------------------------------- */
void psa_decode_amplifier_0x1a0(const uint8_t *data, uint8_t dlc, jbl_amplifier_state_t *amp) {
    if (!data || !amp || dlc < 8) return;
    memset(amp, 0, sizeof(*amp));

    amp->bass           = data[1] & 0x0F;
    amp->treble         = data[2] & 0x0F;
    amp->balance        = data[3] & 0x0F;
    amp->fader          = data[4] & 0x0F;
    amp->eq_preset      = data[5] & 0x07;
    amp->loudness       = (data[6] & 0x10) != 0;
    amp->speed_vol_comp = data[6] & 0x0F;
    amp->master_volume  = data[7] & 0x1F;
}

/* --------------------------------------------------------------------------
 * 1.7b RD4 Radio Volume & Audio Settings (DSP)
 * -------------------------------------------------------------------------- */
static inline uint8_t rd4_clamp18(int val) {
    if (val < 0) return 0;
    if (val > 18) return 18;
    return (uint8_t)val;
}

bool psa_decode_rd4_volume_0x1a5(const uint8_t *data, uint8_t dlc, vehicle_audio_settings_t *audio) {
    if (!data || !audio || dlc < 1) {
        return false;
    }

    uint8_t new_vol = data[0] & 0x1F;
    if (new_vol > 30) {
        new_vol = 30;
    }

    /* Bits [7:5]:
     * 0x00 (0b000) -> Volume knob is actively rotating / changing
     * 0xE0 (0b111) -> Volume knob is idle / stationary
     * 0x80 (0b100) -> Mute active (isolated bit 7)
     */
    uint8_t status_flags = data[0] & 0xE0;
    bool is_turning = (status_flags == 0x00);
    bool is_muted   = (status_flags == 0x80);

    uint8_t prev_mode = audio->active_mode;
    uint8_t new_mode;
    if (is_turning || is_muted) {
        new_mode = 1; /* Master volume active */
    } else if (prev_mode == 1) {
        new_mode = 0; /* Stopped rotating -> dismiss volume OSD */
    } else {
        new_mode = prev_mode; /* Retain active DSP mode if user was in DSP menu */
    }

    bool changed = (!audio->valid) ||
                   (new_vol != audio->master_volume) ||
                   (is_muted != audio->is_muted) ||
                   (new_mode != prev_mode);

    audio->master_volume = new_vol;
    audio->is_muted      = is_muted;
    audio->active_mode   = new_mode;
    audio->valid         = true;

    return changed;
}

bool psa_decode_rd4_audio_settings_0x1e5(const uint8_t *data, uint8_t dlc, vehicle_audio_settings_t *audio) {
    if (!data || !audio || dlc < 7) {
        return false;
    }

    uint8_t new_bal   = rd4_clamp18(((int)(data[0] & 0x7F) - 63) + 9);
    uint8_t new_fad   = rd4_clamp18(((int)(data[1] & 0x7F) - 63) + 9);
    uint8_t new_bass  = rd4_clamp18(((int)(data[2] & 0x7F) - 63) + 9);
    uint8_t new_treb  = rd4_clamp18(((int)(data[4] & 0x7F) - 63) + 9);
    bool    new_loud  = (data[5] & 0x40) != 0;
    bool    new_spd   = ((data[5] & 0x07) == 0x07);

    uint8_t new_eq;
    switch (data[6] & 0xBF) {
        case 0x07: new_eq = 1; break; /* Classical */
        case 0x0B: new_eq = 2; break; /* Jazz-Blues */
        case 0x0F: new_eq = 3; break; /* Pop-Rock */
        case 0x13: new_eq = 4; break; /* Vocals */
        case 0x17: new_eq = 5; break; /* Techno */
        default:   new_eq = 0; break; /* None */
    }

    uint8_t dsp_mode = 0;
    if (data[2] & 0x80)      dsp_mode = 2; /* Bass */
    else if (data[4] & 0x80) dsp_mode = 3; /* Treble */
    else if (data[0] & 0x80) dsp_mode = 4; /* Balance */
    else if (data[1] & 0x80) dsp_mode = 5; /* Fader */
    else if (data[5] & 0x80) dsp_mode = 6; /* Loudness */
    else if (data[5] & 0x10) dsp_mode = 7; /* Speed Volume */
    else if (data[6] & 0x40) dsp_mode = 8; /* Equalizer */
    else                     dsp_mode = 0; /* None / Closed */

    uint8_t prev_mode = audio->active_mode;
    uint8_t new_mode;
    if (dsp_mode >= 2) {
        new_mode = dsp_mode;
    } else if (prev_mode >= 2) {
        new_mode = 0; /* DSP menu dismissed / closed */
    } else {
        new_mode = prev_mode; /* Retain volume active mode (1) or idle (0) */
    }

    bool values_changed = (!audio->valid) ||
                          (audio->balance != new_bal) ||
                          (audio->fader != new_fad) ||
                          (audio->bass != new_bass) ||
                          (audio->treble != new_treb) ||
                          (audio->loudness != new_loud) ||
                          (audio->speed_volume != new_spd) ||
                          (audio->eq_preset != new_eq);

    bool mode_changed = (new_mode != prev_mode);

    audio->balance      = new_bal;
    audio->fader        = new_fad;
    audio->bass         = new_bass;
    audio->treble       = new_treb;
    audio->loudness     = new_loud;
    audio->speed_volume = new_spd;
    audio->eq_preset    = new_eq;
    audio->active_mode  = new_mode;
    audio->valid        = true;

    return (mode_changed || values_changed);
}

/* --------------------------------------------------------------------------
 * 1.8 RD4 Radio & CD Changer Media Data
 * -------------------------------------------------------------------------- */
void psa_decode_cd_changer_0x3a6(const uint8_t *data, uint8_t dlc, cd_changer_state_t *cdc) {
    if (!data || !cdc || dlc < 7) return;
    memset(cdc, 0, sizeof(*cdc));

    cdc->disc_slot    = data[1];
    cdc->track_num    = data[2];
    cdc->total_tracks = data[3];
    cdc->elapsed_min  = data[4];
    cdc->elapsed_sec  = data[5];
    cdc->play_flags   = data[6];
}

void psa_decode_rds_name_0x396(const uint8_t *data, uint8_t dlc, char out_name[9]) {
    if (!data || !out_name) return;
    size_t len = (dlc > 8) ? 8 : (size_t)dlc;
    memcpy(out_name, data, len);
    out_name[len] = '\0';
}

static psa_isotp_rx_ctx_t s_rd4_isotp_ctx;
static psa_isotp_rx_ctx_t s_rd4_list_isotp_ctx;

void psa_rd4_media_init(vehicle_media_t *media, psa_isotp_rx_ctx_t *isotp) {
    if (media) {
        memset(media, 0, sizeof(*media));
        media->radio.source_mode = 0xFF; /* Standby / Off */
        memset(media->radio.station_name, ' ', 8);
        media->radio.station_name[8] = '\0';
    }
    if (isotp) {
        memset(isotp, 0, sizeof(*isotp));
    }
    memset(&s_rd4_isotp_ctx, 0, sizeof(s_rd4_isotp_ctx));
    memset(&s_rd4_list_isotp_ctx, 0, sizeof(s_rd4_list_isotp_ctx));
}

void psa_rd4_process_can_0x165(vehicle_radio_t *radio, const uint8_t *data, uint8_t dlc) {
    if (!radio || !data || dlc < 1) return;

    radio->present = true;
    /* Bits [7:6] == 0 indicates radio is in deep sleep / standby (e.g. 0x08) */
    radio->sleep = ((data[0] & 0xC0) == 0);

    /* Bit 7 of byte 0: radio audio power on/off */
    bool radio_powered = (data[0] & 0x80) != 0;
    if (!radio_powered) {
        radio->source_mode = 0xFF;
        radio->power_status = 0x00;
        psa_virtual_cdc_process_can_0x165(radio->source_mode, radio->power_status);
        return;
    }

    if (dlc < 3) {
        if (radio->power_status == 0) {
            radio->power_status = 0x01;
        }
        return;
    }

    uint8_t rd4_src_nibble = (data[2] >> 4) & 0x0F;
    switch (rd4_src_nibble) {
        case 0x1: /* Tuner */
            radio->source_mode = radio->band;
            if (radio->power_status != 0x02) {
                radio->power_status = 0x01;
            }
            break;
        case 0x2: /* Internal CD */
            radio->source_mode = 0x20;
            radio->power_status = 0x01;
            break;
        case 0x3: /* Virtual CDC */
            radio->source_mode = 0x31;
            radio->power_status = 0x01;
            break;
        case 0x4: /* AUX 1 (Input 1) */
            radio->source_mode = 0x40;
            radio->power_status = 0x01;
            break;
        case 0x5: /* AUX 2 (Input 2) */
            radio->source_mode = 0x41;
            radio->power_status = 0x01;
            break;
        default:
            radio->source_mode = 0xFF;
            radio->power_status = 0x00;
            break;
    }

    psa_virtual_cdc_process_can_0x165(radio->source_mode, radio->power_status);
}

void psa_rd4_process_can_0x225(vehicle_radio_t *radio, const uint8_t *data, uint8_t dlc) {
    if (!radio || !data || dlc < 5) return;

    /* Indicator flags: preserve TA (bit 7) if already set by 0x265 or set by 0x225 bit 2 */
    uint8_t ta_flag = (radio->indicators & 0x80);
    if (data[0] & 0x04) ta_flag = 0x80; /* TA from 0x225 */

    radio->indicators = ta_flag;
    if (data[0] & 0x20) radio->indicators |= 0x20; /* RDS */
    if (data[0] & 0x40) radio->indicators |= 0x10; /* SCAN */
    if (radio->radio_text_len > 0) radio->indicators |= 0x04; /* RDTEXT */

    /* Seeking / Tuning status (only when radio is already powered ON) */
    if (radio->power_status != 0) {
        if (data[0] & 0x08) {
            radio->power_status = 0x02; /* Seeking */
        } else if (radio->power_status == 0x02) {
            radio->power_status = 0x01; /* Restored to playing */
        }
    }

    /* Preset memory: 0=manual, 1..6 */
    radio->preset_slot = (data[1] >= 0x10) ? ((data[1] >> 4) & 0x0F) : (data[1] & 0x0F);

    /* Band conversion:
     * Hiworld Peugeot OriginalTuner maps:
     *   0x00 -> "FM", 0x01 -> "FM1", 0x02 -> "FM2", 0x03 -> "FM3", 0x04 -> "FM-AST"
     *   0x10 -> "AM", 0x11 -> "AM1", 0x12 -> "AM2", 0x13 -> "AM-AST"
     * PSA CAN 0x225 data[2]:
     *   0x10 / 0x90: FM1
     *   0x20 / 0xA0: FM2
     *   0x40 / 0xC0: FM-AST
     *   0x50 / 0xD0: AM
     */
    uint8_t raw_band = data[2];
    if (raw_band == 0x10 || raw_band == 0x90)      radio->band = 0x01; /* FM1 */
    else if (raw_band == 0x20 || raw_band == 0xA0) radio->band = 0x02; /* FM2 */
    else if (raw_band == 0x40 || raw_band == 0xC0) radio->band = 0x04; /* FM-AST */
    else if (raw_band == 0x50 || raw_band == 0xD0) radio->band = 0x10; /* AM */

    /* Frequency conversion: Raw to 0.1 MHz */
    uint16_t raw_freq = ((uint16_t)data[3] << 8) | data[4];
    uint16_t new_freq_0_1mhz;
    if (radio->band < 0x10) {
        /* FM: Freq * 10 = (raw / 2) + 500 */
        new_freq_0_1mhz = (raw_freq / 2) + 500;
    } else {
        /* AM: kHz direct */
        new_freq_0_1mhz = raw_freq;
    }

    /* If frequency changed or RDS sync is lost, clear previous station name and radio text */
    if (radio->freq_0_1mhz != new_freq_0_1mhz || !(radio->indicators & 0x20)) {
        if (radio->freq_0_1mhz != new_freq_0_1mhz) {
            radio->freq_0_1mhz = new_freq_0_1mhz;
            memset(radio->station_name, ' ', 8);
            radio->station_name[8] = '\0';
            radio->radio_text[0] = '\0';
            radio->radio_text_len = 0;
        } else if (!(radio->indicators & 0x20) && radio->station_name[0] != ' ') {
            memset(radio->station_name, ' ', 8);
            radio->station_name[8] = '\0';
            radio->radio_text[0] = '\0';
            radio->radio_text_len = 0;
        }
    }

    /* Cache frequency into preset slot (1..6) if valid preset active */
    if (radio->preset_slot >= 1 && radio->preset_slot <= 6) {
        uint8_t slot_idx = radio->preset_slot - 1;
        if (radio->preset_freqs[slot_idx] != radio->freq_0_1mhz) {
            radio->preset_freqs[slot_idx] = radio->freq_0_1mhz;
        }
    }

    /* Only update active source_mode if currently in Tuner mode (< 0x20).
     * Never overwrite OFF (0xFF) or external sources (AUX/CD/CDC >= 0x20). */
    if (radio->source_mode < 0x20) {
        radio->source_mode = radio->band;
    }
}

void psa_rd4_process_can_0x265(vehicle_radio_t *radio, const uint8_t *data, uint8_t dlc) {
    if (!radio || !data || dlc < 1) return;
    if (data[0] & 0x20) {
        if (!(radio->indicators & 0x80)) {
            radio->indicators |= 0x80;
        }
    } else {
        if (radio->indicators & 0x80) {
            radio->indicators &= ~0x80;
        }
    }
}

void psa_rd4_process_can_0x2a5(vehicle_radio_t *radio, const uint8_t *data, uint8_t dlc) {
    if (!radio || !data || dlc < 8) return;

    /* Check if frame is all 0x00 or all spaces (no RDS signal) */
    bool all_empty = true;
    for (size_t i = 0; i < 8; i++) {
        if (data[i] != 0x00 && data[i] != ' ') {
            all_empty = false;
            break;
        }
    }

    if (all_empty) {
        memset(radio->station_name, ' ', 8);
        radio->station_name[8] = '\0';
    } else {
        memcpy(radio->station_name, data, 8);
        radio->station_name[8] = '\0';
    }
}

bool psa_rd4_process_can_0x125(vehicle_radio_t *radio, psa_isotp_rx_ctx_t *isotp, const uint8_t *data, uint8_t dlc) {
    if (!radio || !isotp || !data || dlc < 2) return false;

    /* Check for list dismiss / close frame: 01 00 */
    if (dlc == 2 && data[0] == 0x01 && data[1] == 0x00) {
        isotp->active = false;
        return false;
    }

    uint8_t pci = data[0] & 0xF0;

    /* Single Frame (0x0N) */
    if (pci == 0x00) {
        uint8_t len = data[0] & 0x0F;
        if (len >= 13 && len <= (dlc - 1)) {
            uint8_t raw_band = data[1];
            if (raw_band == 0x10 || raw_band == 0x90)      radio->band = 0x01; /* FM1 */
            else if (raw_band == 0x20 || raw_band == 0xA0) radio->band = 0x02; /* FM2 */
            else if (raw_band == 0x40 || raw_band == 0xC0) radio->band = 0x04; /* FM-AST */
            else if (raw_band == 0x50 || raw_band == 0xD0) radio->band = 0x10; /* AM */

            uint8_t num_stations = (uint8_t)((len - 4) / 9);
            if (num_stations > 6) num_stations = 6;

            for (uint8_t i = 0; i < 6; i++) {
                if (i < num_stations) {
                    const uint8_t *src_entry = &data[1 + 4 + (i * 9)];
                    for (uint8_t j = 0; j < 8; j++) {
                        uint8_t ch = src_entry[j];
                        radio->preset_names[i][j] = (ch >= 0x20 && ch <= 0x7E) ? (char)ch : ' ';
                    }
                    radio->preset_names[i][8] = '\0';
                } else {
                    memset(radio->preset_names[i], ' ', 8);
                    radio->preset_names[i][8] = '\0';
                }
            }
            isotp->active = false;
            return true;
        }
        isotp->active = false;
        return false;
    }

    /* First Frame (0x1N) */
    if (pci == 0x10) {
        uint16_t total_len = (((uint16_t)(data[0] & 0x0F)) << 8) | data[1];
        if (total_len > sizeof(isotp->buffer)) total_len = sizeof(isotp->buffer);

        uint8_t chunk_len = (dlc > 2) ? (uint8_t)(dlc - 2) : 0;
        if (chunk_len > 0) {
            memcpy(isotp->buffer, &data[2], chunk_len);
        }
        isotp->total_length    = total_len;
        isotp->received_length = chunk_len;
        isotp->next_sn         = 1;
        isotp->active          = true;
        return false;
    }

    /* Consecutive Frame (0x2N) */
    if (pci == 0x20 && isotp->active) {
        uint8_t sn = data[0] & 0x0F;
        if (sn == (isotp->next_sn & 0x0F)) {
            uint8_t chunk_len = (dlc > 1) ? (uint8_t)(dlc - 1) : 0;
            if (isotp->received_length + chunk_len > isotp->total_length) {
                chunk_len = (uint8_t)(isotp->total_length - isotp->received_length);
            }
            if (chunk_len > 0) {
                memcpy(&isotp->buffer[isotp->received_length], &data[1], chunk_len);
                isotp->received_length += chunk_len;
            }
            isotp->next_sn = (isotp->next_sn + 1) & 0x0F;

            /* Completed reassembly */
            if (isotp->received_length >= isotp->total_length) {
                isotp->active = false;
                uint16_t len = isotp->total_length;
                if (len >= 13) {
                    uint8_t raw_band = isotp->buffer[0];
                    if (raw_band == 0x10 || raw_band == 0x90)      radio->band = 0x01; /* FM1 */
                    else if (raw_band == 0x20 || raw_band == 0xA0) radio->band = 0x02; /* FM2 */
                    else if (raw_band == 0x40 || raw_band == 0xC0) radio->band = 0x04; /* FM-AST */
                    else if (raw_band == 0x50 || raw_band == 0xD0) radio->band = 0x10; /* AM */

                    uint8_t num_stations = (uint8_t)((len - 4) / 9);
                    if (num_stations > 6) num_stations = 6;

                    for (uint8_t i = 0; i < 6; i++) {
                        if (i < num_stations) {
                            const uint8_t *src_entry = &isotp->buffer[4 + (i * 9)];
                            for (uint8_t j = 0; j < 8; j++) {
                                uint8_t ch = src_entry[j];
                                radio->preset_names[i][j] = (ch >= 0x20 && ch <= 0x7E) ? (char)ch : ' ';
                            }
                            radio->preset_names[i][8] = '\0';
                        } else {
                            memset(radio->preset_names[i], ' ', 8);
                            radio->preset_names[i][8] = '\0';
                        }
                    }
                    return true;
                }
            }
        } else {
            /* Sequence error */
            isotp->active = false;
        }
    }
    return false;
}

bool psa_rd4_process_can_0x0a4(vehicle_radio_t *radio, psa_isotp_rx_ctx_t *isotp, const uint8_t *data, uint8_t dlc) {
    if (!radio || !isotp || !data || dlc < 2) return false;

    uint8_t pci = data[0] & 0xF0;

    /* Single Frame (0x0N) */
    if (pci == 0x00) {
        uint8_t len = data[0] & 0x0F;
        if (len > 0 && len <= (dlc - 1)) {
            const uint8_t *payload = &data[1];
            /* Strip 4-byte prefix 10 00 00 00 if present */
            if (len > 4 && payload[0] == 0x10 && payload[1] == 0x00 && payload[2] == 0x00 && payload[3] == 0x00) {
                payload += 4;
                len -= 4;
            }
            if (len > PSA_RD4_MAX_RADIO_TEXT_LEN) len = PSA_RD4_MAX_RADIO_TEXT_LEN;
            memcpy(radio->radio_text, payload, len);
            radio->radio_text[len] = '\0';
            radio->radio_text_len = len;
            radio->indicators |= 0x04; /* RDTEXT */
            isotp->active = false;
            return true;
        }
        isotp->active = false;
        return false;
    }

    /* First Frame (0x1N) */
    if (pci == 0x10) {
        uint16_t total_len = (((uint16_t)(data[0] & 0x0F)) << 8) | data[1];
        if (total_len > sizeof(isotp->buffer)) total_len = sizeof(isotp->buffer);

        uint8_t chunk_len = (dlc > 2) ? (uint8_t)(dlc - 2) : 0;
        if (chunk_len > 0) {
            memcpy(isotp->buffer, &data[2], chunk_len);
        }
        isotp->total_length    = total_len;
        isotp->received_length = chunk_len;
        isotp->next_sn         = 1;
        isotp->active          = true;
        return false;
    }

    /* Consecutive Frame (0x2N) */
    if (pci == 0x20 && isotp->active) {
        uint8_t sn = data[0] & 0x0F;
        if (sn == (isotp->next_sn & 0x0F)) {
            uint8_t chunk_len = (dlc > 1) ? (uint8_t)(dlc - 1) : 0;
            if (isotp->received_length + chunk_len > isotp->total_length) {
                chunk_len = (uint8_t)(isotp->total_length - isotp->received_length);
            }
            if (chunk_len > 0) {
                memcpy(&isotp->buffer[isotp->received_length], &data[1], chunk_len);
                isotp->received_length += chunk_len;
            }
            isotp->next_sn = (isotp->next_sn + 1) & 0x0F;

            /* Completed reassembly */
            if (isotp->received_length >= isotp->total_length) {
                const uint8_t *payload = isotp->buffer;
                uint16_t len = isotp->total_length;

                /* Strip 4-byte prefix 10 00 00 00 if present */
                if (len > 4 && payload[0] == 0x10 && payload[1] == 0x00 && payload[2] == 0x00 && payload[3] == 0x00) {
                    payload += 4;
                    len -= 4;
                }
                if (len > PSA_RD4_MAX_RADIO_TEXT_LEN) len = PSA_RD4_MAX_RADIO_TEXT_LEN;

                memcpy(radio->radio_text, payload, len);
                radio->radio_text[len] = '\0';
                radio->radio_text_len = (uint8_t)len;
                radio->indicators |= 0x04; /* RDTEXT */
                isotp->active = false;
                return true;
            }
        } else {
            /* Sequence error */
            isotp->active = false;
        }
    }
    return false;
}

bool psa_rd4_process_can_0x325(vehicle_cdc_t *cdc, const uint8_t *data, uint8_t dlc) {
    if (!cdc || !data || dlc < 2) return false;

    /* CAN 0x325: Internal CD Drive Mechanism & Disc Presence (DLC: 3)
     * Byte 0: Mechanism status (0x00 = Idle/Normal, 0x80 = Tray/Disc active)
     * Byte 1: Disc presence (0x03 = Disc Loaded/Present, 0x00 = Ejected/Empty)
     * For internal in-dash CD, OEM Canbox keeps active_disc = 0x00, discs_loaded_mask = 0x00.
     */
    if (data[1] == 0x00) {
        cdc->play_status = 0x0C; /* Eject */
    }
    return true;
}

bool psa_rd4_process_can_0x365(vehicle_cdc_t *cdc, const uint8_t *data, uint8_t dlc) {
    if (!cdc || !data || dlc < 1) return false;

    /* Ignore uninitialized / empty drive sentinel frames (e.g. FF FF FF ...) */
    if (data[0] == 0xFF) return false;

    /* CAN 0x365: Internal CD Disc Info (DLC: 5)
     * Byte 0: Total tracks on disc (1..99)
     * Byte 1: CD Format / Mechanism (0x26)
     * Byte 2: Disc attributes (0x00 = Data, 0x02 = CD-DA Audio)
     */
    cdc->total_tracks = data[0];
    if (dlc >= 3 && data[2] == 0x00) {
        cdc->disc_format = 0x01; /* MP3 */
    } else {
        cdc->disc_format = 0x00; /* CD-DA */
    }
    cdc->active_disc = 0x00;
    cdc->discs_loaded_mask = 0x00;
    return true;
}

bool psa_rd4_process_can_0x3a5(vehicle_cdc_t *cdc, const uint8_t *data, uint8_t dlc) {
    if (!cdc || !data || dlc < 5) return false;

    /* Ignore uninitialized / empty drive sentinel frames (e.g. FF FF FF FF 7F 00) */
    if (data[0] == 0xFF || data[2] == 0xFF) return false;

    /* CAN 0x3A5: Internal CD Track & Playback Telemetry (DLC: 6)
     * Byte 0: Current track number (1..99)
     * Byte 1: Track index / sub-index
     * Byte 2: Drive status & modes (e.g. 0x19, 0x1E = Normal Play, 0x25 = Fast Forward/Seeking, 0x01 = Pause)
     * Byte 3: Elapsed minutes (0..59)
     * Byte 4: Elapsed seconds (0..59)
     * Byte 5: Reserved (0x00)
     */
    cdc->track_num   = data[0];
    cdc->elapsed_min = data[3];
    cdc->elapsed_sec = data[4];

    /* Drive status (Byte 2) */
    uint8_t raw_status = data[2];
    if (raw_status == 0x01) {
        cdc->play_status = 0x01; /* Pause */
    } else if (raw_status == 0x00) {
        cdc->play_status = 0x06; /* Stop */
    } else if (raw_status == 0x25) {
        cdc->play_status = 0x02; /* Fast Forward / Seeking playback */
    } else {
        cdc->play_status = 0x02; /* Play (0x19, 0x1E, etc. all map to 0x02 Play) */
    }

    /* Mode flags (Repeat, Random, Scan)
     * Bit 7: Scan, Bit 6: Disc Scan, Bit 5: Repeat Track (0x20), Bit 4: Repeat Disc (0x10), Bit 3: Random (0x08)
     * In OEM captures, standard in-dash playback sends play_modes = 0x00.
     */
    cdc->play_modes = 0x00;

    /* Single in-dash CD drive: active disc and magazine loaded mask are 0x00 */
    cdc->active_disc = 0x00;
    cdc->discs_loaded_mask = 0x00;
    return true;
}

bool psa_rd4_process_can_0x3a6(vehicle_cdc_t *cdc, const uint8_t *data, uint8_t dlc) {
    if (!cdc || !data || dlc < 7) return false;

    cdc->active_disc       = data[1];
    cdc->track_num         = data[2];
    cdc->total_tracks      = data[3];
    cdc->elapsed_min       = data[4];
    cdc->elapsed_sec       = data[5];
    cdc->play_modes        = data[6];

    /* Disc slot bitmask */
    if (cdc->active_disc >= 1 && cdc->active_disc <= 6) {
        cdc->discs_loaded_mask |= (uint8_t)(1 << (cdc->active_disc - 1));
        cdc->play_status = 0x02; /* Play (Hiworld wire code 0x02 = Play) */
    } else {
        cdc->play_status = 0x06; /* Stop (Hiworld wire code 0x06 = Stop) */
    }
    cdc->disc_format = 0x00;
    return true;
}

void psa_rd4_decode_frame(const can_frame_t *frame, vehicle_state_t *state) {
    if (!frame || !state) return;

    switch (frame->id) {
        case PSA_CAN_ID_ETAT_AUTORADIO:
            psa_rd4_process_can_0x165(&state->media.radio, frame->data, frame->dlc);
            state->dirty.radio = true;
            break;
        case PSA_CAN_ID_RD4_VOLUME:
            if (psa_decode_rd4_volume_0x1a5(frame->data, frame->dlc, &state->audio)) {
                state->dirty.audio = true;
            }
            break;
        case PSA_CAN_ID_RD4_AUDIO_SETTINGS:
            if (psa_decode_rd4_audio_settings_0x1e5(frame->data, frame->dlc, &state->audio)) {
                state->dirty.audio = true;
            }
            break;
        case PSA_CAN_ID_STATION_LIST_125:
            if (psa_rd4_process_can_0x125(&state->media.radio, &s_rd4_list_isotp_ctx, frame->data, frame->dlc)) {
                state->dirty.radio_preset = true;
                state->dirty.radio = true;
            }
            break;
        case PSA_CAN_ID_ETAT_TUNER: {
            uint16_t prev_freqs[6];
            memcpy(prev_freqs, state->media.radio.preset_freqs, sizeof(prev_freqs));
            uint8_t prev_text_len = state->media.radio.radio_text_len;

            psa_rd4_process_can_0x225(&state->media.radio, frame->data, frame->dlc);
            state->dirty.radio = true;

            if (memcmp(prev_freqs, state->media.radio.preset_freqs, sizeof(prev_freqs)) != 0) {
                state->dirty.radio_preset = true;
            }
            if (prev_text_len != state->media.radio.radio_text_len) {
                state->dirty.radio_text = true;
            }
            break;
        }
        case PSA_CAN_ID_INFO_TUNER:
            psa_rd4_process_can_0x265(&state->media.radio, frame->data, frame->dlc);
            state->dirty.radio = true;
            break;
        case PSA_CAN_ID_NOM_STATION:
            psa_rd4_process_can_0x2a5(&state->media.radio, frame->data, frame->dlc);
            state->dirty.radio = true;
            break;
        case PSA_CAN_ID_TEXTE_RADIO:
            if (psa_rd4_process_can_0x0a4(&state->media.radio, &s_rd4_isotp_ctx, frame->data, frame->dlc)) {
                state->dirty.radio_text = true;
            }
            break;
        case PSA_CAN_ID_CD_TRAY_325:
            if (psa_rd4_process_can_0x325(&state->media.cdc, frame->data, frame->dlc)) {
                state->dirty.cdc = true;
            }
            break;
        case PSA_CAN_ID_CD_DISC_INFO_365:
            if (psa_rd4_process_can_0x365(&state->media.cdc, frame->data, frame->dlc)) {
                state->dirty.cdc = true;
            }
            break;
        case PSA_CAN_ID_CD_PLAY_3A5:
            if (psa_rd4_process_can_0x3a5(&state->media.cdc, frame->data, frame->dlc)) {
                state->dirty.cdc = true;
            }
            break;
        case PSA_CAN_ID_CD_CHANGER:
            if (psa_rd4_process_can_0x3a6(&state->media.cdc, frame->data, frame->dlc)) {
                state->dirty.cdc = true;
            }
            break;
        default:
            break;
    }
}

