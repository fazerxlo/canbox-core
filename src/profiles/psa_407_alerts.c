#include "profiles/peugeot_407.h"
#include <string.h>

/* --------------------------------------------------------------------------
 * 2.4 Driver Assistance & ADAS Features
 * -------------------------------------------------------------------------- */
void psa_decode_alerts_0x168(const uint8_t *data, uint8_t dlc,
                             bool *tpms_fault, bool *tpms_underinflation,
                             bool *tpms_puncture, bool *esp_fault) {
    if (!data) return;
    if (tpms_fault && dlc >= 1) {
        *tpms_fault = (data[0] & 0x01) != 0;
    }
    if (tpms_underinflation && dlc >= 2) {
        *tpms_underinflation = (data[1] & 0x80) != 0;
    }
    if (tpms_puncture && dlc >= 2) {
        *tpms_puncture = (data[1] & 0x40) != 0;
    }
    if (esp_fault && dlc >= 4) {
        *esp_fault = (data[3] & 0x10) != 0;
    }
}

/* --------------------------------------------------------------------------
 * 2.5 Vehicle Alerts & Diagnostic Journal (Hiworld Command 0x42)
 * -------------------------------------------------------------------------- */
#define TOTAL_ALARM_ENTRIES 208

static const uint16_t Alarm_IndexToPointer_Tab[TOTAL_ALARM_ENTRIES] = {
    [0x00] = 0x000C, [0x01] = 0x000B, [0x02] = 0x019C, [0x03] = 0x0008,
    [0x04] = 0x019B, [0x05] = 0x019A, [0x06] = 0x0002, [0x07] = 0x0001,
    [0x08] = 0x000F, [0x09] = 0x000A, [0x0A] = 0x000E, [0x0B] = 0xFFFF,
    [0x0C] = 0x000D, [0x0D] = 0x0003, [0x0E] = 0x0005, [0x0F] = 0x0004,
    [0x10] = 0x007F, [0x11] = 0x0067, [0x12] = 0x0000, [0x13] = 0x006E,
    [0x14] = 0x006B, [0x15] = 0x006A, [0x16] = 0x006C, [0x17] = 0x0066,
    [0x18] = 0xFFFF, [0x19] = 0xFFFF, [0x1A] = 0x006F, [0x1B] = 0x007E,
    [0x1C] = 0xFFFF, [0x1D] = 0x0073, [0x1E] = 0x0072, [0x1F] = 0x007D,
    [0x20] = 0x0087, [0x21] = 0x0075, [0x22] = 0x0086, [0x23] = 0x0074,
    [0x24] = 0x0085, [0x25] = 0x0084, [0x26] = 0x0081, [0x27] = 0x0080,
    [0x28] = 0x00D9, [0x29] = 0x00D2, [0x2A] = 0x00D0, [0x2B] = 0x00DF,
    [0x2C] = 0x00CB, [0x2D] = 0x00CA, [0x2E] = 0x00C9, [0x2F] = 0xFFFF,
    [0x30] = 0xFFFF, [0x31] = 0xFFFF, [0x32] = 0x00E4, [0x33] = 0x00E3,
    [0x34] = 0xFFFF, [0x35] = 0x00D7, [0x36] = 0xFFFF, [0x37] = 0x00E0,
    [0x38] = 0xFFFF, [0x39] = 0xFFFF, [0x3A] = 0xFFFF, [0x3B] = 0xFFFF,
    [0x3C] = 0xFFFF, [0x3D] = 0x00E5, [0x3E] = 0xFFFF, [0x3F] = 0xFFFF,
    [0x40] = 0x0135, [0x41] = 0x013A, [0x42] = 0x0137, [0x43] = 0x0133,
    [0x44] = 0x0131, [0x45] = 0x012F, [0x46] = 0x0083, [0x47] = 0x012E,
    [0x48] = 0xFFFF, [0x49] = 0xFFFF, [0x4A] = 0xFFFF, [0x4B] = 0xFFFF,
    [0x4C] = 0x00E2, [0x4D] = 0x0082, [0x4E] = 0xFFFF, [0x4F] = 0xFFFF,
    [0x50] = 0x0198, [0x51] = 0x0195, [0x52] = 0x0194, [0x53] = 0xFFFF,
    [0x54] = 0xFFFF, [0x55] = 0x0193, [0x56] = 0x0192, [0x57] = 0x0191,
    [0x58] = 0xFFFF, [0x59] = 0xFFFF, [0x5A] = 0xFFFF, [0x5B] = 0xFFFF,
    [0x5C] = 0x0202, [0x5D] = 0x020B, [0x5E] = 0x020A, [0x5F] = 0xFFFF,
    [0x60] = 0x0209, [0x61] = 0x0206, [0x62] = 0x0205, [0x63] = 0x0204,
    [0x64] = 0x0203, [0x65] = 0xFFFF, [0x66] = 0xFFFF, [0x67] = 0xFFFF,
    [0x68] = 0x01FB, [0x69] = 0x01FA, [0x6A] = 0x01F9, [0x6B] = 0x01F8,
    [0x6C] = 0x01F7, [0x6D] = 0x01F6, [0x6E] = 0x01FE, [0x6F] = 0x01F5,
    [0x70] = 0xFFFF, [0x71] = 0xFFFF, [0x72] = 0xFFFF, [0x73] = 0xFFFF,
    [0x74] = 0x007C, [0x75] = 0x01FD, [0x76] = 0xFFFF, [0x77] = 0x01FC,
    [0x78] = 0x0012, [0x79] = 0x0068, [0x7A] = 0x0069, [0x7B] = 0x006D,
    [0x7C] = 0x0013, [0x7D] = 0x0078, [0x7E] = 0x0088, [0x7F] = 0x0089,
    [0x80] = 0x008A, [0x81] = 0x0076, [0x82] = 0x008C, [0x83] = 0x008D,
    [0x84] = 0x0222, [0x85] = 0x0221, [0x86] = 0x0220, [0x87] = 0x0091,
    [0x88] = 0x00CD, [0x89] = 0x00D1, [0x8A] = 0x00D4, [0x8B] = 0x00D8,
    [0x8C] = 0x00DE, [0x8D] = 0x00E1, [0x8E] = 0x00E6, [0x8F] = 0x00EC,
    [0x90] = 0x00ED, [0x91] = 0x00EE, [0x92] = 0x00EF, [0x93] = 0x012D,
    [0x94] = 0x0130, [0x95] = 0x0132, [0x96] = 0x0136, [0x97] = 0x0138,
    [0x98] = 0x0139, [0x99] = 0x013B, [0x9A] = 0x013C, [0x9B] = 0x00DA,
    [0x9C] = 0x0079, [0x9D] = 0x00E7, [0x9E] = 0x00E9, [0x9F] = 0x00EA,
    [0xA0] = 0x00EB, [0xA1] = 0x00D3, [0xA2] = 0x013D, [0xA3] = 0x013E,
    [0xA4] = 0x00CE, [0xA5] = 0x0134, [0xA6] = 0x0064, [0xA7] = 0x007A,
    [0xA8] = 0x0092, [0xA9] = 0x0095, [0xAA] = 0x0096, [0xAB] = 0x0097,
    [0xAC] = 0x009A, [0xAD] = 0x009B, [0xAE] = 0x009C, [0xAF] = 0x009D,
    [0xB0] = 0x009E, [0xB1] = 0x009F, [0xB2] = 0x00A0, [0xB3] = 0x00D5,
    [0xB4] = 0x00D6, [0xB5] = 0x00E8, [0xB6] = 0x013F, [0xB7] = 0x0140,
    [0xB8] = 0x0196, [0xB9] = 0x0197, [0xBA] = 0x0199, [0xBB] = 0x0011,
    [0xBC] = 0x0063, [0xBD] = 0x00A1, [0xBE] = 0x00F0, [0xBF] = 0x00F1,
    [0xC0] = 0x00F2, [0xC1] = 0x00F3, [0xC2] = 0x00F4, [0xC3] = 0x00F5,
    [0xC4] = 0x00F6, [0xC5] = 0x00F7, [0xC6] = 0x00F8, [0xC7] = 0x00F9,
    [0xC8] = 0x01FF, [0xC9] = 0x0200, [0xCA] = 0x0201, [0xCB] = 0x0061,
    [0xCC] = 0x0062, [0xCD] = 0x007B, [0xCE] = 0xFFFF, [0xCF] = 0xFFFF
};

static const uint16_t Alarm_BitToIndex_Tab[PSA_JOURNAL_TOTAL_BITS] = {
    0x0E, 0x07, 0xFFFF, 0x03, 0xFFFF, 0x7B, 0x0D, 0xFFFF, 0x0F, 0x28, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x14, 0x80, 0xFFFF, 0x1F, 0x11, 0x37, 0x7D,
    0x27, 0x10, 0x15, 0x08, 0xFFFF, 0x29, 0x26, 0x13, 0x16, 0x1A, 0x81, 0x1B,
    0x1D, 0x46, 0x2C, 0x20, 0x21, 0x22, 0x8C, 0x8C, 0x8C, 0x82, 0x8C, 0x8C,
    0x8C, 0x8C, 0x8C, 0x84, 0x10, 0x8C, 0x8C, 0x2D, 0x2E, 0x2E, 0x2E, 0x2E,
    0x32, 0x33, 0x35, 0xB2, 0xB2, 0xB2, 0xB2, 0xAC, 0xAC, 0xAD, 0xAD, 0xAE,
    0xAE, 0xAF, 0xAF, 0xAF, 0xAF, 0xB0, 0xB0, 0xB0, 0xB0, 0xB1, 0xB1, 0x47,
    0x4C, 0x4D, 0x50, 0x46, 0x51, 0x52, 0x9C, 0x7E, 0x7F, 0x55, 0x83, 0x83,
    0x83, 0x83, 0x56, 0x57, 0x89, 0x5C, 0x5D, 0xA1, 0xA1, 0xA1, 0x5E, 0x8B,
    0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x60, 0x8E, 0x9D, 0x91,
    0x99, 0x9A, 0x61, 0x79, 0x7A, 0x62, 0x63, 0x9E, 0x9F, 0xA0, 0x8F, 0x92,
    0x64, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0x6B, 0xB5, 0xA6, 0x68, 0xBB, 0x69,
    0x6A, 0xC5, 0xC6, 0xC7, 0x94, 0x44, 0x3D, 0x97, 0x78, 0x7C, 0x74, 0x75,
    0x77, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
};

uint16_t psa_can_alarm_id_to_hiworld_code(uint16_t can_alarm_id, uint8_t door_mask, uint8_t param) {
    (void)door_mask;
    switch (can_alarm_id) {
        case 0x000D: return PSA_HIWORLD_ALERT_LOW_FUEL;           /* Low fuel */
        case 0x0001: return PSA_HIWORLD_ALERT_ENGINE_TEMP;        /* Engine temp too high */
        case 0x0002: return PSA_HIWORLD_ALERT_OIL_PRESSURE;       /* Oil pressure low */
        case 0x0003: return PSA_HIWORLD_ALERT_OIL_PRESSURE;       /* Check engine oil level */
        case 0x000F: return PSA_HIWORLD_ALERT_BRAKE_FLUID;        /* Brake system faulty */
        case 0x000C: return PSA_HIWORLD_ALERT_HANDBRAKE;          /* Handbrake on */
        case 0x00DF: return PSA_HIWORLD_ALERT_KEY_BATTERY;        /* Remote key battery flat */
        case 0x0195: return PSA_HIWORLD_ALERT_DIRECTIONAL_LIGHTS; /* Directional headlamps faulty */
        case 0x006B: return PSA_HIWORLD_ALERT_BATTERY_CHARGE;     /* Battery charge fault */
        case 0x006A: return PSA_HIWORLD_ALERT_ESP;                /* ESP / ASR faulty */
        case 0x0069: return PSA_HIWORLD_ALERT_ABS;                /* ABS direct code */
        case 0x006C: return PSA_HIWORLD_ALERT_ABS;                /* ABS braking system faulty */
        case 0x006F: return PSA_HIWORLD_ALERT_DPF;                /* DPF risk of clogging */
        case 0x0064: return PSA_HIWORLD_ALERT_DPF;                /* DPF direct code */
        case 0x006E: return PSA_HIWORLD_ALERT_ANTIPOLLUTION;      /* Depollution system faulty */
        case 0x0068: return PSA_HIWORLD_ALERT_ANTIPOLLUTION;      /* Depollution direct code */
        case 0x007E: return PSA_HIWORLD_ALERT_ANTIPOLLUTION;      /* Depollution system error */
        case 0x0073:
        case 0x0202: return PSA_HIWORLD_ALERT_GEARBOX;            /* Gearbox faulty */
        case 0x0067: return PSA_HIWORLD_ALERT_GEARBOX;            /* Gearbox direct code */
        case 0x0061: return PSA_HIWORLD_ALERT_SERVICE_DUE;        /* Service due */
        case 0x009E: return PSA_HIWORLD_ALERT_SUSPENSION_90KMH;   /* Suspension max 90 km/h */
        case 0x0072:
        case 0x00D8:
        case 0x00E2: return PSA_HIWORLD_ALERT_SUSPENSION_SYSTEM;  /* Suspension system error */
        case 0x007F: return PSA_HIWORLD_ALERT_ANTIPOLLUTION;      /* Depollution system faulty */
        case 0x0081: return PSA_HIWORLD_ALERT_AUTO_LIGHTS;        /* Auto lights code */
        case 0x00CB: return PSA_HIWORLD_ALERT_AUTO_WIPERS;        /* Automatic wipers */
        case 0x0083: return PSA_HIWORLD_ALERT_IMMOBILIZER;        /* Electronic anti-theft faulty */
        case 0x0139: return PSA_HIWORLD_ALERT_AUTO_WIPERS;        /* Auto wipers deactivated */
        case 0x0138: return PSA_HIWORLD_ALERT_LOW_FUEL;          /* Fuel level too low */
        case 0x0074: return 0;                                    /* FL door open - suppressed */
        case 0x0085: return 0;                                    /* FR door open - suppressed */
        case 0x0084: return 0;                                    /* Rear door open - suppressed */
        case 0x0080: return 0;                                    /* Boot open - suppressed */
        case 0x00D9: return 0;                                    /* Rear screen open - suppressed */
        case 0x0008:                                              /* Door open / Braking system faulty */
            return PSA_HIWORLD_ALERT_HANDBRAKE;                   /* 0x0008: Braking system faulty in Hiworld HU */
        case 0x000B:                                              /* Door open general */
            return 0x000B;
        case 0x0004:                                              /* Tyre pressure low */
            if (param == 1) return PSA_HIWORLD_ALERT_TPMS_UNDER_FR;
            if (param == 2) return PSA_HIWORLD_ALERT_TPMS_UNDER_RR;
            if (param == 3) return PSA_HIWORLD_ALERT_TPMS_UNDER_RL;
            return PSA_HIWORLD_ALERT_TPMS_UNDER_FL;
        case 0x0005:                                              /* Tyre puncture */
            if (param == 1) return PSA_HIWORLD_ALERT_TPMS_PUNCTURE_FR;
            if (param == 2 || param == 3) return PSA_HIWORLD_ALERT_TPMS_PUNCTURE_REAR;
            return PSA_HIWORLD_ALERT_TPMS_PUNCTURE_FL;
        case 0x00C9: return PSA_HIWORLD_ALERT_TPMS_UNDER_FL;      /* Tyre pressure(s) not monitored */
        case 0x00E5: return PSA_HIWORLD_ALERT_TPMS_UNDER_FL;      /* Tyre pressure(s) not monitored */
        case 0x00E3: return PSA_HIWORLD_ALERT_BULB_SIDELIGHT;     /* Sidelight bulb */
        case 0x00E7: return PSA_HIWORLD_ALERT_BULB_BRAKE;         /* Brake light bulb */
        case 0x0078: return PSA_HIWORLD_ALERT_AIRBAG;             /* Airbag / pretensioner */
        case 0x00F0: return 0x00F0;                               /* Diagnosis in progress... */
        case 0x00F1: return 0x00F1;                               /* Diagnosis complete */
        case 0x012F: return PSA_HIWORLD_ALERT_SEATBELT_FL;        /* Driver seatbelt */
        case 0x013A: return PSA_HIWORLD_ALERT_SEATBELT_FR;        /* Front passenger seatbelt */
        case 0x0130: return PSA_HIWORLD_ALERT_AUTO_LIGHTS;        /* Automatic headlamp lighting activated */
        case 0x01FA: return PSA_HIWORLD_ALERT_IMMOBILIZER;        /* Electronic immobilizer */
        default:
            return can_alarm_id;
    }
}

uint8_t psa_can_alarm_id_get_severity(uint16_t can_alarm_id) {
    switch (can_alarm_id) {
        /* STOP (2) - Critical alerts requiring immediate stop */
        case 0x0001: /* Engine temperature fault: Stop vehicle */
        case 0x0002: /* Warning, engine oil pressure */
        case 0x0005: /* Tyre puncture(s) detected */
        case 0x0008: /* Braking system faulty */
        case 0x000D: /* Puncture(s) in tyres detected */
        case 0x000F: /* Brake system faulty: Stop vehicle */
        case 0x0066: /* Braking system faulty */
            return 2;

        /* SERVICE (1) - Minor/major vehicle faults requiring service */
        case 0x0003: /* Top up engine oil level */
        case 0x0004: /* Tyre pressure(s) low */
        case 0x000E: /* Power steering faulty */
        case 0x0011: /* Suspension faulty: Max speed 90 km/h */
        case 0x0064: /* Particle filter clogging risk */
        case 0x0067: /* Brake pads worn / Gearbox */
        case 0x0068: /* Depollution system faulty */
        case 0x006A: /* ABS system faulty */
        case 0x006B: /* Battery charge faulty */
        case 0x006C: /* Suspension faulty */
        case 0x006E: /* Gearbox fault: Repair needed */
        case 0x006F: /* Risk of particle filter clogging */
        case 0x0072: /* Suspension faulty: Max speed 90 km/h */
        case 0x0073: /* Automatic gearbox faulty */
        case 0x0076: /* Directional headlamps faulty */
        case 0x0078: /* Airbag(s) or pretensioner faulty */
        case 0x0079: /* Active bonnet faulty */
        case 0x007A: /* Automatic gearbox faulty */
        case 0x007E: /* ABS / ESP system faulty */
        case 0x007F: /* Depollution system faulty */
        case 0x0083: /* Electronic anti-theft faulty */
        case 0x0086: /* Right-hand sliding side door faulty */
        case 0x0087: /* Left-hand sliding side door faulty */
        case 0x0088: /* Parking assistance system faulty */
        case 0x0097: /* Anti-wander lane-crossing warning device faulty */
        case 0x009A: /* Dipped headlamp bulb faulty */
        case 0x009B: /* Main beam headlamp bulb faulty */
        case 0x009C: /* Left hand brake light bulb faulty */
        case 0x009D: /* Foglamp bulb faulty */
        case 0x009E: /* Direction indicators faulty */
        case 0x009F: /* Left hand reversing light bulb faulty */
        case 0x00A0: /* Sidelamp bulb faulty */
        case 0x00E0: /* Fuel level low / TPMS */
        case 0x00E5: /* Tyre pressure(s) not monitored */
        case 0x00E8: /* Tyre pressure too low */
        case 0x0195: /* Directional headlamps faulty */
        case 0x0198: /* Stop & Start system faulty */
        case 0x0202: /* Automatic gearbox faulty */
            return 1;

        /* INFO (0) - Confirmations, reminders, statuses, all other codes */
        default:
            return 0;
    }
}

void psa_decode_alert_message_0x1a1(const uint8_t *data, uint8_t dlc, vehicle_alert_item_t *alert) {
    if (!data || !alert || dlc < 4) {
        return;
    }
    bool toggle_bit = (data[0] & 0x80) != 0;
    (void)toggle_bit; /* Sequence toggle bit (1, 0, 1, 0...); not dismissal flag */
    uint16_t can_alarm_id = ((uint16_t)(data[0] & 0x7F) << 8) | (uint16_t)data[1];
    bool display_req = (data[2] & 0x80) != 0;
    uint8_t priority = (data[2] >> 4) & 0x07;
    uint8_t sound_id = data[2] & 0x0F;
    uint8_t door_mask = data[3];
    uint8_t param = (dlc >= 5) ? data[4] : 0;

    if (display_req && can_alarm_id != 0) {
        alert->is_active = true;
        alert->can_alarm_id = can_alarm_id;
        alert->display_req = true;
        bool is_diag_sys = (can_alarm_id == 0x00F0 || can_alarm_id == 0x00F1);
        alert->severity = is_diag_sys ? 0 : priority;
        alert->chime_id = is_diag_sys ? 0 : sound_id;
        alert->door_mask = is_diag_sys ? 0 : door_mask;
        alert->param_detail = is_diag_sys ? 0 : param;
        alert->alert_code = psa_can_alarm_id_to_hiworld_code(can_alarm_id, door_mask, param);
    } else if (can_alarm_id == 0 && (display_req || data[2] != 0)) {
        /* Cockpit CHECK sequence inter-alert blanking (Section 4.1 Step 3) */
        alert->is_active = true;
        alert->can_alarm_id = 0x0000;
        alert->display_req = true;
        alert->severity = 0;
        alert->chime_id = 0;
        alert->door_mask = 0;
        alert->param_detail = 0;
        alert->alert_code = 0;
    } else if (!display_req || can_alarm_id == 0) {
        alert->is_active = false;
        alert->display_req = false;
        alert->can_alarm_id = 0;
        alert->alert_code = 0;
    }
}

void psa_journal_iso_tp_init(psa_journal_iso_tp_t *ctx) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
}

static uint8_t psa_journal_extract_codes(const uint8_t *buffer, vehicle_alert_item_t *out_items, uint8_t max_out) {
    uint8_t count = 0;
    for (uint16_t bit = 0; bit < PSA_JOURNAL_TOTAL_BITS && count < max_out; bit++) {
        uint8_t b_idx = (uint8_t)(bit >> 3);
        uint8_t mask = (uint8_t)(1 << (7 - (bit & 7)));
        if (buffer[b_idx] & mask) {
            uint16_t a_idx = Alarm_BitToIndex_Tab[bit];
            if (a_idx < TOTAL_ALARM_ENTRIES && Alarm_IndexToPointer_Tab[a_idx] != 0xFFFF) {
                uint16_t can_id = Alarm_IndexToPointer_Tab[a_idx];
                uint16_t hw_code = psa_can_alarm_id_to_hiworld_code(can_id, 0, 0);

                bool already_present = false;
                for (uint8_t i = 0; i < count; i++) {
                    if (out_items[i].can_alarm_id == can_id) {
                        already_present = true;
                        break;
                    }
                }
                if (!already_present) {
                    memset(&out_items[count], 0, sizeof(vehicle_alert_item_t));
                    out_items[count].can_alarm_id = can_id;
                    out_items[count].alert_code = hw_code;
                    out_items[count].is_active = true;
                    out_items[count].display_req = true;
                    out_items[count].severity = psa_can_alarm_id_get_severity(can_id);
                    count++;
                }
            }
        }
    }
    return count;
}

bool psa_process_journal_0x120(psa_journal_iso_tp_t *ctx, const uint8_t *data, uint8_t dlc,
                               vehicle_alert_item_t *out_items, uint8_t *out_count) {
    if (!ctx || !data || !out_items || !out_count || dlc < 2) {
        return false;
    }

    /* Mode A: Real PSA Block Multiplexing (2-bit block header in Byte 0 [7:6])
     * 0x7C (01b = 1): Block 1 -> buffer[0..6]
     * 0xBC (10b = 2): Block 2 -> buffer[7..13]
     * 0xFC (11b = 3): Block 3 -> buffer[14..20]
     */
    uint8_t block_id = (data[0] >> 6) & 0x03;
    if (dlc >= 8 && block_id >= 1 && block_id <= 3) {
        uint8_t offset = (uint8_t)((block_id - 1) * 7);
        memcpy(&ctx->buffer[offset], &data[1], 7);
        ctx->blocks_received |= (uint8_t)(1 << (block_id - 1));

        if ((ctx->blocks_received & 0x07) == 0x07) {
            *out_count = psa_journal_extract_codes(ctx->buffer, out_items, CANBOX_MAX_ACTIVE_ALERTS);
            return true;
        }
        return false;
    }

    /* Mode B: ISO-TP Multi-frame Fallback (PCI nibbles 0x10 and 0x20) */
    uint8_t pci = data[0];

    /* First Frame: high nibble 0x1 */
    if ((pci & 0xF0) == 0x10) {
        uint16_t total_len = ((uint16_t)(pci & 0x0F) << 8) | (uint16_t)data[1];
        if (total_len == PSA_JOURNAL_PAYLOAD_BYTES) {
            ctx->in_progress = true;
            ctx->bytes_rx = 0;
            ctx->expected_seq = 1;
            uint8_t chunk = (dlc > 2) ? (dlc - 2) : 0;
            if (chunk > 6) chunk = 6;
            memcpy(&ctx->buffer[0], &data[2], chunk);
            ctx->bytes_rx = chunk;
        }
        return false;
    }

    /* Consecutive Frame: high nibble 0x2 */
    if ((pci & 0xF0) == 0x20 && ctx->in_progress) {
        uint8_t seq = pci & 0x0F;
        if (seq == ctx->expected_seq) {
            ctx->expected_seq = (ctx->expected_seq + 1) & 0x0F;
            uint8_t chunk = (uint8_t)(dlc - 1);
            if (ctx->bytes_rx + chunk > PSA_JOURNAL_PAYLOAD_BYTES) {
                chunk = (uint8_t)(PSA_JOURNAL_PAYLOAD_BYTES - ctx->bytes_rx);
            }
            memcpy(&ctx->buffer[ctx->bytes_rx], &data[1], chunk);
            ctx->bytes_rx += chunk;

            if (ctx->bytes_rx >= PSA_JOURNAL_PAYLOAD_BYTES) {
                ctx->in_progress = false;
                *out_count = psa_journal_extract_codes(ctx->buffer, out_items, CANBOX_MAX_ACTIVE_ALERTS);
                return true;
            }
        } else {
            /* Out-of-order sequence: abort */
            ctx->in_progress = false;
        }
    }

    return false;
}

