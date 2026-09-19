#include "protocol.h"
#include "mbedtls/aes.h"
#include <string.h>

// Wire contracts: ndeadly/switch2_controller_research commands.md (03/0D,
// 03/0A, 07/01, 09/01-08, 16/01, 15/01-04) and hid_reports.md (05/07/08). USB reply headers
// and status payloads match captures/usb/rumble-procon-gccon.pcapng.gz.
// This public component is not a pairing key. The host supplies the other half.
static const uint8_t device_key_component[16] = {
    0x5c, 0xf6, 0xee, 0x79, 0x2c, 0xdf, 0x05, 0xe1,
    0xba, 0x2b, 0x63, 0x25, 0xc4, 0x1a, 0x5f, 0x10,
};

// Identical 11/03 payload in five genuine Joy-Con BLE captures; also present
// in the GameCube USB capture. Full semantics remain undocumented.
static const uint8_t joycon_info11_03[] = {
    0x01, 0x20, 0x03, 0x00, 0x00, 0x0a, 0xe8, 0x1c,
    0x3b, 0x79, 0x7d, 0x8b, 0x3a, 0x0a, 0xe8, 0x9c,
    0x42, 0x58, 0xa0, 0x0b, 0x42, 0x0a, 0xe8, 0x9c,
    0x41, 0x58, 0xa0, 0x0b, 0x41,
};

static void clear_pending_pairing(probe_protocol_state* state) {
    state->pending_host_count = 0;
    memset(state->pending_host_addresses, 0, sizeof(state->pending_host_addresses));
    state->pending_key_valid = false;
    memset(state->pending_key, 0, sizeof(state->pending_key));
    state->challenge_confirmed = false;
}

static const uint8_t* pairing_key_for_context(const probe_protocol_state* state) {
    if (!state->pending_host_count) return NULL;
    if (state->pending_key_valid) return state->pending_key;
    // Host addresses are key associations: order and subset size may change,
    // but no new host may borrow the committed key without its own exchange.
    for (unsigned i = 0; i < state->pending_host_count; ++i) {
        bool stored = false;
        for (unsigned j = 0; j < state->committed_host_count; ++j) {
            if (memcmp(state->pending_host_addresses[i],
                       state->committed_host_addresses[j], 6) == 0) {
                stored = true;
                break;
            }
        }
        if (!stored) return NULL;
    }
    return state->committed_key;
}

static bool challenge_response(const uint8_t* key, const uint8_t* wire_challenge,
                               uint8_t* response) {
    uint8_t challenge[16];
    for (unsigned i = 0; i < sizeof(challenge); ++i) {
        challenge[i] = wire_challenge[sizeof(challenge) - 1u - i];
    }
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    int result = mbedtls_aes_setkey_enc(&aes, key, 128);
    if (result == 0) {
        // Genuine challenge capture: reverse key/input, but NOT ciphertext.
        result = mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, challenge, response);
    }
    mbedtls_aes_free(&aes);
    return result == 0;
}

static bool finalize_pairing(probe_protocol_state* state, const uint8_t* key) {
    uint8_t blob[PROBE_PAIRING_BLOB_SIZE] = {0};
    memcpy(blob, state->controller_address, sizeof(state->controller_address));
    blob[6] = state->pending_host_count;
    memcpy(blob + 7, state->pending_host_addresses, 6u * state->pending_host_count);
    memcpy(blob + sizeof(blob) - 16u, key, 16);
    // Preserve both the old committed key and pending retry on any save failure.
    if (!state->save_pairing(state->context, blob, sizeof(blob))) return false;
    state->committed_host_count = blob[6];
    memcpy(state->committed_host_addresses, blob + 7, sizeof(state->committed_host_addresses));
    memcpy(state->committed_key, blob + sizeof(blob) - 16u, sizeof(state->committed_key));
    state->pending_key_valid = false;
    memset(state->pending_key, 0, sizeof(state->pending_key));
    // Keep this context confirmed so a repeated finalize can safely retry its ACK.
    return true;
}

void probe_protocol_reset(probe_protocol_state* state, bool is_left) {
    memset(state, 0, sizeof(*state));
    state->is_left = is_left;
    state->report_id = is_left ? 0x07 : 0x08;
    state->stick_center[1] = 0x08;
    state->stick_center[2] = 0x80;
}

bool probe_protocol_restore_pairing(probe_protocol_state* state,
                                    const uint8_t* blob, size_t length) {
    if (!state || !blob || length != PROBE_PAIRING_BLOB_SIZE ||
        memcmp(state->controller_address, blob, sizeof(state->controller_address)) != 0 ||
        blob[6] == 0 || blob[6] > PROBE_HOST_MAX_ADDRESSES) return false;
    for (size_t i = 7u + 6u * blob[6]; i < length - 16u; ++i) {
        if (blob[i] != 0) return false;
    }
    // Validate the whole record before changing either committed or pending state.
    state->committed_host_count = blob[6];
    memcpy(state->committed_host_addresses, blob + 7, sizeof(state->committed_host_addresses));
    memcpy(state->committed_key, blob + length - 16u, sizeof(state->committed_key));
    clear_pending_pairing(state);
    return true;
}

size_t probe_protocol_command(probe_protocol_state* state, const uint8_t* command,
                              size_t length, uint8_t* reply, size_t capacity,
                              uint64_t* deferred_token) {
    if (deferred_token) *deferred_token = 0;
    if (!state || !command || !reply || length < 8 ||
        command[1] != 0x91 || command[2] != 0 ||
        command[4] != 0 || command[6] != 0 || command[7] != 0 ||
        length != 8u + command[5]) return 0;
    const bool initialize = command[0] == 0x03 && command[3] == 0x0d;
    const bool select_report = command[0] == 0x03 && command[3] == 0x0a;
    const bool status_query = (command[0] == 0x07 || command[0] == 0x16) && command[3] == 1;
    const bool exchange_addresses = command[0] == 0x15 && command[3] == 1;
    const bool confirm_key = command[0] == 0x15 && command[3] == 2;
    const bool finalize = command[0] == 0x15 && command[3] == 3;
    const bool exchange_keys = command[0] == 0x15 && command[3] == 4;
    const bool power07 = command[0] == 0x0b && command[3] == 0x07;
    const bool player_leds = command[0] == 0x09 && command[3] >= 1 && command[3] <= 8;
    const bool features = command[0] == 0x0c && command[3] >= 1 && command[3] <= 5;
    const bool memory_read = command[0] == 0x02 && (command[3] == 1 || command[3] == 4);
    const bool info11_03 = command[0] == 0x11 && command[3] == 3;
    const bool info11_01 = command[0] == 0x11 && command[3] == 1;
    const bool vibration_setup = command[0] == 0x0a && command[3] == 8;
    const bool vibration_sample = command[0] == 0x0a && command[3] == 2;
    const bool joycon_query = command[0] == 0x13 && command[3] >= 1 && command[3] <= 3;
    const bool firmware_info = command[0] == 0x10 && command[3] == 1;
    const bool nfc_info = command[0] == 0x01 && command[3] == 0x0c;
    const bool runtime_toggle = command[0] == 0x03 && command[3] == 0x0c;
    uint32_t memory_address = 0;
    uint8_t memory_length = 0;
    size_t reply_length;
    const uint8_t* pairing_key = NULL;
    if (initialize) {
        if (length != 16 || command[8] != 1 || command[9] != 0) return 0;
        reply_length = 12;
    } else if (select_report) {
        if (length != 12) return 0;
        reply_length = 8;
    } else if (status_query) {
        if (length != 8) return 0;
        reply_length = command[0] == 0x07 ? 9 : 32;
    } else if (exchange_addresses) {
        if (length != 8 &&
            (length < 10 || command[8] != 0 ||
             command[9] > PROBE_HOST_MAX_ADDRESSES ||
             length != 10u + 6u * command[9])) return 0;
        reply_length = 17;
    } else if (exchange_keys) {
        if (length != 25 || command[8] != 0 || !state->pending_host_count) return 0;
        reply_length = 25;
    } else if (confirm_key) {
        if (length != 25 || command[8] != 0) return 0;
        pairing_key = pairing_key_for_context(state);
        if (!pairing_key) return 0;
        reply_length = 25;
    } else if (finalize) {
        if (length != 9 || command[8] != 0 || !state->challenge_confirmed ||
            !state->save_pairing) return 0;
        pairing_key = pairing_key_for_context(state);
        if (!pairing_key) return 0;
        reply_length = 9;
    } else if (power07) {
        // Donor BLE capture: 0b 01 01 07 10 78 00 00. Apply the previously
        // corroborated USB header mapping. Semantics remain unknown: only the
        // console's observed four-zero argument has been queried on the donor.
        if (length != 12 || command[8] || command[9] || command[10] || command[11]) return 0;
        reply_length = 8;
    } else if (player_leds) {
        if (command[3] <= 6) {
            if (length != 8) return 0;
        } else {
            // Current console sends four payload bytes; published captures use
            // eight. Only the first byte carries the mask/flashing setting.
            if (length != 12 && length != 16) return 0;
            if (command[3] == 8 && command[8] > 1) return 0;
        }
        reply_length = 8;
    } else if (features) {
        if (length != 12) return 0;
        reply_length = command[3] == 1 ? 20 : 12;
    } else if (memory_read) {
        if (length != 16 || !state->read_memory || command[10] || command[11]) return 0;
        if (command[3] == 1) {
            if (command[8] || command[9]) return 0;
            memory_length = 64;
        } else {
            if (command[9] != 0x7e || command[8] > 80) return 0;
            memory_length = command[8];
        }
        for (unsigned i = 0; i < 4; ++i)
            memory_address |= (uint32_t)command[12 + i] << (8 * i);
        reply_length = 16u + memory_length;
    } else if (info11_03 || info11_01) {
        if (length != 8) return 0;
        reply_length = info11_03 ? 8 + sizeof(joycon_info11_03) : 12;
    } else if (vibration_setup) {
        // Five genuine Joy-Con traces acknowledge this 20-byte parameter block
        // with no response payload. Its first byte is consistently 1.
        if (length != 28 || command[8] != 1) return 0;
        reply_length = 8;
    } else if (vibration_sample) {
        if (length != 12 || command[8] > 7 ||
            command[9] || command[10] || command[11] ||
            !state->play_sample || !deferred_token) return 0;
        reply_length = 8;
    } else if (joycon_query) {
        if (length != 8) return 0;
        reply_length = command[3] == 1 ? 12 : 16;
    } else if (firmware_info) {
        if (length != 8 || !state->firmware_version) return 0;
        reply_length = 20;
    } else if (nfc_info) {
        if (length != 8) return 0;
        reply_length = 12;
    } else if (runtime_toggle) {
        if (length != 12 || command[8] > 1 || command[9] || command[10] || command[11]) return 0;
        reply_length = 8;
    } else {
        return 0;
    }
    if (capacity < reply_length) return 0;
    if (vibration_sample) {
        uint64_t token = 0;
        if (!state->play_sample(state->context, command[8], &token) || !token) return 0;
        *deferred_token = token;
    }
    uint8_t encrypted_challenge[16];
    if (confirm_key && !challenge_response(pairing_key, command + 9, encrypted_challenge)) return 0;
    if (finalize && !finalize_pairing(state, pairing_key)) return 0;
    if (memory_read &&
        !state->read_memory(state->context, memory_address, reply + 16, memory_length)) return 0;
    const uint8_t header[] = {command[0], 0x01, 0, command[3], 0, 0xf8, 0, 0};
    memcpy(reply, header, sizeof(header));
    if (info11_03) {
        memcpy(reply + 8, joycon_info11_03, sizeof(joycon_info11_03));
    } else if (firmware_info) {
        memcpy(reply + 8, state->firmware_version, 12);
    } else if (nfc_info) {
        // Identical in five Joy-Con captures; Pro's last byte differs.
        const uint8_t info[] = {0x61, 0x12, 0x50, 0x0d};
        memcpy(reply + 8, info, sizeof(info));
    } else {
        // Memory payload was supplied directly into the reply; initialize only its metadata.
        memset(reply + 8, 0, memory_read ? 8 : reply_length - 8);
    }
    if (initialize) {
        memcpy(state->host_address, command + 10, sizeof(state->host_address));
        state->initialized = true;
        // Retransmission is idempotent: do not reset an already-running counter.
        reply[8] = 1;
    } else if (select_report) {
        // The real controller acknowledges but ignores unsupported report IDs.
        if (command[8] == 0x05 || command[8] == (state->is_left ? 0x07 : 0x08))
            state->report_id = command[8];
    } else if (exchange_addresses) {
        if (length != 8) {
            clear_pending_pairing(state);
            state->pending_host_count = command[9];
            memcpy(state->pending_host_addresses, command + 10, 6u * command[9]);
        }
        reply[8] = 1;
        reply[9] = 4; // Observed address-response field; semantics unresolved.
        reply[10] = 1;
        memcpy(reply + 11, state->controller_address, sizeof(state->controller_address));
    } else if (exchange_keys) {
        for (unsigned i = 0; i < sizeof(state->pending_key); ++i) {
            const unsigned wire_index = sizeof(state->pending_key) - 1u - i;
            state->pending_key[i] = command[9 + wire_index] ^ device_key_component[wire_index];
        }
        state->pending_key_valid = true;
        state->challenge_confirmed = false;
        reply[8] = 1;
        memcpy(reply + 9, device_key_component, sizeof(device_key_component));
    } else if (confirm_key) {
        reply[8] = 1;
        memcpy(reply + 9, encrypted_challenge, sizeof(encrypted_challenge));
        state->challenge_confirmed = true;
    } else if (finalize) {
        reply[8] = 1;
    } else if (player_leds) {
        if (command[3] <= 4) {
            state->player_leds = (uint8_t)(1u << (command[3] - 1));
        } else if (command[3] == 5) {
            state->player_leds = 0x0f;
        } else if (command[3] == 6) {
            state->player_leds = 0;
        } else if (command[3] == 7) {
            state->player_leds = command[8] & 0x0f;
        } else {
            state->player_leds_flashing = command[8] != 0;
        }
    } else if (features) {
        const uint8_t flags = command[8] & 0xb7; // Bits 3 and 6 are unused.
        if (command[3] == 1) {
            // Published Joy-Con-specific feature-info encoding; first four
            // response bytes and final two feature-info bytes remain zero.
            reply[12] = flags & 0x01 ? 7 : 0;
            reply[13] = flags & 0x02 ? 7 : 0;
            reply[14] = flags & 0x04 ? 3 : 0;
            reply[15] = flags & 0x80 ? 3 : 0;
            reply[16] = flags & 0x10 ? 3 : 0;
            reply[17] = flags & 0x20 ? 3 : 0;
        } else if (command[3] == 2) {
            state->feature_mask = flags;
            state->enabled_features &= flags;
        } else if (command[3] == 3) {
            state->feature_mask = state->enabled_features = 0;
        } else if (command[3] == 4) {
            state->enabled_features |= flags & state->feature_mask;
        } else {
            state->enabled_features &= (uint8_t)~(flags & state->feature_mask);
        }
    } else if (memory_read) {
        reply[8] = memory_length;
        for (unsigned i = 0; i < 4; ++i)
            reply[12 + i] = (uint8_t)(memory_address >> (8 * i));
    } else if (vibration_setup) {
        memcpy(state->vibration_parameters, command + 8, sizeof(state->vibration_parameters));
        state->vibration_parameters_set = true;
    } else if (info11_01) {
        // Identical 01 00 00 00 payload in all five genuine Joy-Con captures.
        reply[8] = 1;
    } else if (joycon_query) {
        // Published 13/01-03 replies: leading 1, then reserved zero bytes.
        // These queries' full semantics are still undocumented.
        reply[8] = 1;
    } else if (runtime_toggle) {
        state->runtime03_0c = command[8] != 0;
    }
    return reply_length;
}

size_t probe_protocol_report(const probe_protocol_state* state, uint8_t report_id,
                             uint8_t* output, size_t capacity) {
    if (!state || !state->initialized || !output || capacity < PROBE_INPUT_SIZE ||
        (report_id != 0x05 && report_id != (state->is_left ? 0x07 : 0x08))) return 0;
    memset(output, 0, PROBE_INPUT_SIZE);
    const bool buttons_enabled = (state->enabled_features & 1) != 0;
    const uint8_t buttons0 = state->controller_active && buttons_enabled ? state->controller_buttons[0] : 0;
    const uint8_t buttons1 = state->controller_active && buttons_enabled ?
        state->controller_buttons[1] & (state->is_left ? 0xc1 : 0xd1) : 0;
    const uint8_t* stick = state->controller_active && (state->enabled_features & 2) ?
                           state->controller_stick : state->stick_center;
    if (report_id != 0x05) {
        output[0] = (uint8_t)state->report_counter;
        output[1] = 0x25;  // Virtual full battery, external USB power.
        output[2] = buttons0;
        output[3] = buttons1;
        if (state->test_rail_buttons && (state->enabled_features & 1))
            output[3] |= 0xc0; // Both models' native SL + SR.
        output[4] = 0x07;
        memcpy(output + 5, stick, 3);
        // Diagnostic snapshot only; complete live native packets bypass this generator.
    } else {
        for (unsigned i = 0; i < 4; ++i) output[i] = (uint8_t)(state->report_counter >> (8 * i));
        if (state->is_left) {
            output[5] = (uint8_t)(((buttons0 & 0x40) >> 6) | ((buttons0 & 0x80) >> 4) |
                                  ((buttons1 & 0x01) << 5));
            output[6] = (uint8_t)((buttons0 & 0x01) | ((buttons0 & 0x06) << 1) |
                                  ((buttons0 & 0x08) >> 2) | ((buttons0 & 0x30) << 2) |
                                  ((buttons1 & 0xc0) >> 2));
            if (state->test_rail_buttons && buttons_enabled)
                output[6] |= 0x30; // Common report: left SL + SR.
            memcpy(output + 10, stick, 3);
            output[14] = 0x08;
            output[15] = 0x80;
        } else {
            output[4] = (uint8_t)(((buttons0 & 0x03) << 2) | ((buttons0 & 0x0c) >> 2) |
                                  ((buttons0 & 0x30) << 2) | ((buttons1 & 0xc0) >> 2));
            output[5] = (uint8_t)(((buttons0 & 0xc0) >> 5) | ((buttons1 & 0x01) << 4) |
                                  ((buttons1 & 0x10) << 2));
            if (state->test_rail_buttons && buttons_enabled)
                output[4] |= 0x30; // Common report: right SL + SR.
            output[11] = 0x08;
            output[12] = 0x80;
            memcpy(output + 13, stick, 3);
        }
        output[31] = 0xa0;
        output[32] = 0x0f; // Virtual battery voltage 4000mV.
        output[33] = 0x20;
        output[41] = 1;
    }
    return PROBE_INPUT_SIZE;
}

void probe_protocol_gate_native_report(const probe_protocol_state* state,
                                       uint8_t input[PROBE_INPUT_SIZE]) {
    const uint8_t imu_length_offset = state->is_left ? 14u : 15u;
    if (!(state->enabled_features & 1)) memset(input + 2, 0, 2);
    if (!(state->enabled_features & 2))
        memcpy(input + 5, state->stick_center, sizeof(state->stick_center));
    if (!(state->enabled_features & 0x10)) memset(input + 9, 0, 5);
#ifdef SWITCH2_PROBE_OMIT_NATIVE_IMU
    // Deliberate A/B fault injection: leave every other field and feature bit intact.
    memset(input + imu_length_offset, 0, 41);
#elif defined(SWITCH2_PROBE_ZERO_NATIVE_IMU_PAYLOAD)
    if (!(state->enabled_features & 4)) input[imu_length_offset] = 0;
    memset(input + imu_length_offset + 1u, 0, 40); // Preserve enabled genuine length.
#else
    if (!(state->enabled_features & 4))
        memset(input + imu_length_offset, 0, 41);
#endif
}

bool probe_protocol_decode_rumble(uint8_t report_id, const uint8_t* data,
                                  size_t length, probe_rumble_frame* output) {
    if (!data || !output) return false;
    if (report_id == 0) {
        if (length < 17 || length > 64 || data[0] != 0x01) return false;
        ++data;
    } else if (report_id != 0x01 || length < 16 || length > 63) {
        return false;
    }
    if ((data[0] & 0xc0u) != 0x40u) return false;

    // Wire block: ndeadly/switch2_controller_research hid_reports.md#output-report-0x01.
    // SDL src/joystick/hidapi/SDL_hidapi_switch2.c (EncodeHDRumble / UpdateRumble)
    // packs frequency/amplitude/frequency/amplitude as four 10-bit LE fields.
    // Header: format 01 [7:6], sample count [5:4], sequence [3:0].
    // Sequence is informational; unused sample bytes and USB padding may be stale.
    probe_rumble_frame decoded = {.count = (data[0] >> 4) & 3u};
    for (unsigned i = 0; i < decoded.count; ++i) {
        const uint8_t* sample = data + 1u + 5u * i;
        const unsigned first = (sample[1] >> 2) | ((sample[2] & 0x0fu) << 6);
        const unsigned second = (sample[3] >> 6) | ((unsigned)sample[4] << 2);
        const unsigned amplitude = first > second ? first : second;
        // ERM compatibility, not HD waveform reproduction: ignore frequencies
        // and round max(amplitudes) across the full 10-bit range to 0..255.
        // SDL's conservative outbound clamp is not an inbound validity limit.
        decoded.magnitude[i] = (uint8_t)((amplitude * 255u + 511u) / 1023u);
    }
    *output = decoded;
    return true;
}
