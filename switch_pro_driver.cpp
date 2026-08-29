#include "switch_pro_driver.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <stdio.h>
#include "pico/rand.h"
#include "pico/time.h"
#include "tusb.h"

#ifdef SWITCH_PICO_LOG
#define LOG_PRINTF(...) printf(__VA_ARGS__)
#else
#define LOG_PRINTF(...) ((void)0)
#endif

// force a report to be sent every X ms
#define SWITCH_PRO_KEEPALIVE_TIMER 5
// Real Pro Controller cadence: 3 IMU frames per report, 5ms/frame => 15ms/report
// (~66.7Hz). Emitting the 3-frame 0x30 faster makes the console over-integrate
// gyro (3 frames assumed 5ms apart delivered too often) => wild camera swing.
#define SWITCH_PRO_IMU_REPORT_TIMER 15

static SwitchInputState g_input_state{
    false, false, false, false,
    false, false, false, false, false, false, false, false,
    false, false, false, false, false, false,
    SWITCH_PRO_JOYSTICK_MID, SWITCH_PRO_JOYSTICK_MID,
    SWITCH_PRO_JOYSTICK_MID, SWITCH_PRO_JOYSTICK_MID};

static uint8_t report_buffer[SWITCH_PRO_ENDPOINT_SIZE] = {};
static uint8_t last_report[SWITCH_PRO_ENDPOINT_SIZE] = {};
static SwitchProReport switch_report{};
static uint8_t last_report_counter = 0;
static uint32_t last_report_timer = 0;
static uint32_t last_host_activity_ms = 0;
static bool is_ready = false;
static bool is_initialized = false;
static bool is_report_queued = false;
static bool report_sent = false;
static uint8_t queued_report_id = 0;
static bool forced_ready = false;
static uint8_t handshake_counter = 0;

static SwitchDeviceInfo device_info{};
static uint8_t player_id = 0;
static uint8_t input_mode = 0x30;
enum class SwitchImuMode : uint8_t {
    Off = 0,
    Raw = 1,
    Quaternion = 2,
};

static SwitchImuMode imu_mode = SwitchImuMode::Off;
static bool is_vibration_enabled = false;

// Optional compile-time colour override (body/buttons/grips).
#if __has_include("controller_color_config.h")
#include "controller_color_config.h"
#endif
#ifndef SWITCH_COLOR_BODY_R
#define SWITCH_COLOR_BODY_R 0x1B
#define SWITCH_COLOR_BODY_G 0x1B
#define SWITCH_COLOR_BODY_B 0x1D
#endif
#ifndef SWITCH_COLOR_BUTTON_R
#define SWITCH_COLOR_BUTTON_R 0xFF
#define SWITCH_COLOR_BUTTON_G 0xFF
#define SWITCH_COLOR_BUTTON_B 0xFF
#endif
#ifndef SWITCH_COLOR_LEFT_GRIP_R
#define SWITCH_COLOR_LEFT_GRIP_R 0xEC
#define SWITCH_COLOR_LEFT_GRIP_G 0x00
#define SWITCH_COLOR_LEFT_GRIP_B 0x8C
#endif
#ifndef SWITCH_COLOR_RIGHT_GRIP_R
#define SWITCH_COLOR_RIGHT_GRIP_R 0xEC
#define SWITCH_COLOR_RIGHT_GRIP_G 0x00
#define SWITCH_COLOR_RIGHT_GRIP_B 0x8C
#endif

static uint16_t leftMinX, leftMinY;
static uint16_t leftCenX, leftCenY;
static uint16_t leftMaxX, leftMaxY;
static uint16_t rightMinX, rightMinY;
static uint16_t rightCenX, rightCenY;
static uint16_t rightMaxX, rightMaxY;
static SwitchRumbleCallback rumble_callback = nullptr;

static const uint8_t factory_config_data[0xEFF] = {
    // serial number
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,

    0xFF, 0xFF,

    // device type
    SWITCH_TYPE_PRO_CONTROLLER,

    // unknown
    0xA0,

    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,

    // color options
    0x02,

    0xFF, 0xFF, 0xFF, 0xFF,

    // config & calibration 1
    // IMU calibration @ 0x6020: zero offsets, 4096 LSB/g, and
    // 818.5 LSB/(rad/s), matching the bridge's raw-value conversion.
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
    0x00, 0x40, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3B, 0x34, 0x3B, 0x34, 0x3B, 0x34,

    0xFF, 0xFF, 0xFF, 0xFF, 0xFF,

    // config & calibration 2
    // left stick
    0xa4, 0x46, 0x6a, 0x00, 0x08, 0x80, 0xa4, 0x46,
    0x6a,

    // right stick
    0x00, 0x08, 0x80, 0xa4, 0x46, 0x6a, 0xa4, 0x46,
    0x6a,

    0xFF,

    // body color
    SWITCH_COLOR_BODY_R, SWITCH_COLOR_BODY_G, SWITCH_COLOR_BODY_B,

    // button color
    SWITCH_COLOR_BUTTON_R, SWITCH_COLOR_BUTTON_G, SWITCH_COLOR_BUTTON_B,

    // left grip color
    SWITCH_COLOR_LEFT_GRIP_R, SWITCH_COLOR_LEFT_GRIP_G, SWITCH_COLOR_LEFT_GRIP_B,

    // right grip color
    SWITCH_COLOR_RIGHT_GRIP_R, SWITCH_COLOR_RIGHT_GRIP_G, SWITCH_COLOR_RIGHT_GRIP_B,

    0x01,

    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF,

    0x50, 0xFD, 0x00, 0x00, 0xC6, 0x0F,
    0x0F, 0x30, 0x61, 0xAE, 0x90, 0xD9, 0xD4, 0x14,
    0x54, 0x41, 0x15, 0x54, 0xC7, 0x79, 0x9C, 0x33,
    0x36, 0x63,

    0x0F, 0x30, 0x61, 0xAE, 0x90, 0xD9, 0xD4, 0x14,
    0x54, 0x41, 0x15, 0x54,

    0xC7,

    0x79,

    0x9C,

    0x33,

    0x36,

    0x63, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF
};

static const uint8_t user_calibration_data[0x3F] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,

    // Left Stick
    0xB2, 0xA1, 0xa4, 0x46, 0x6a, 0x00, 0x08, 0x80,
    0xa4, 0x46, 0x6a,

    // Right Stick
    0xB2, 0xA1, 0x00, 0x08, 0x80, 0xa4, 0x46, 0x6a,
    0xa4, 0x46, 0x6a,

    // Motion
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

static const SwitchFactoryConfig* factory_config = reinterpret_cast<const SwitchFactoryConfig*>(factory_config_data);
static const SwitchUserCalibration* user_calibration [[maybe_unused]] = reinterpret_cast<const SwitchUserCalibration*>(user_calibration_data);

static std::map<uint32_t, const uint8_t*> spi_flash_data = {
    {0x6000, factory_config_data},
    {0x8000, user_calibration_data}
};

static inline uint16_t scale16To12(uint16_t pos) { return pos >> 4; }

struct MotionQuaternion {
    float x;
    float y;
    float z;
    float w;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
};

static MotionQuaternion motion_quaternion{0.0f, 0.0f, 0.0f, 1.0f, 0, 0, 0};

static void reset_motion_quaternion() {
    motion_quaternion = {0.0f, 0.0f, 0.0f, 1.0f, 0, 0, 0};
}

static void write_int16_le(uint8_t* dst, int16_t value) {
    dst[0] = static_cast<uint8_t>(value & 0xFF);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

static void write_bits_le(uint8_t* dst, uint16_t bit_offset, uint32_t value, uint8_t width) {
    for (uint8_t bit = 0; bit < width; ++bit) {
        if ((value & (1u << bit)) != 0) {
            uint16_t output_bit = static_cast<uint16_t>(bit_offset + bit);
            dst[output_bit >> 3] |= static_cast<uint8_t>(1u << (output_bit & 7u));
        }
    }
}

static void integrate_motion_sample(const SwitchImuSample& sample) {
    constexpr float sample_dt = 0.005f;
    constexpr float gyro_rad_per_lsb = 1.0f / 818.5f;

    // Nintendo mode 2 uses Y, X, Z sensor order for quaternion axes.
    float angle_x = static_cast<float>(sample.gyro_y) * gyro_rad_per_lsb * sample_dt;
    float angle_y = static_cast<float>(sample.gyro_x) * gyro_rad_per_lsb * sample_dt;
    float angle_z = static_cast<float>(sample.gyro_z) * gyro_rad_per_lsb * sample_dt;
    float norm = sqrtf(angle_x * angle_x + angle_y * angle_y + angle_z * angle_z);
    float half = 0.5f * norm;
    float vector_scale = norm > 1e-12f ? sinf(half) / norm : 0.5f;
    float scalar = norm > 1e-12f ? cosf(half) : 1.0f;

    float dx = angle_x * vector_scale;
    float dy = angle_y * vector_scale;
    float dz = angle_z * vector_scale;
    float dw = scalar;

    float x = motion_quaternion.w * dx + motion_quaternion.x * dw
            + motion_quaternion.y * dz - motion_quaternion.z * dy;
    float y = motion_quaternion.w * dy - motion_quaternion.x * dz
            + motion_quaternion.y * dw + motion_quaternion.z * dx;
    float z = motion_quaternion.w * dz + motion_quaternion.x * dy
            - motion_quaternion.y * dx + motion_quaternion.z * dw;
    float w = motion_quaternion.w * dw - motion_quaternion.x * dx
            - motion_quaternion.y * dy - motion_quaternion.z * dz;
    float magnitude = sqrtf(x * x + y * y + z * z + w * w);
    if (magnitude > 1e-12f) {
        float inverse = 1.0f / magnitude;
        motion_quaternion.x = x * inverse;
        motion_quaternion.y = y * inverse;
        motion_quaternion.z = z * inverse;
        motion_quaternion.w = w * inverse;
    } else {
        reset_motion_quaternion();
    }

    motion_quaternion.accel_x = sample.accel_x;
    motion_quaternion.accel_y = sample.accel_y;
    motion_quaternion.accel_z = sample.accel_z;
}

static void fill_raw_imu_report_data(const SwitchInputState& state) {
    if (state.imu_sample_count == 0) {
        memset(switch_report.imuData, 0x00, sizeof(switch_report.imuData));
        return;
    }
    uint8_t sample_count = state.imu_sample_count > 3 ? 3 : state.imu_sample_count;
    uint8_t* dst = switch_report.imuData;
    for (uint8_t i = 0; i < 3; ++i) {
        const SwitchImuSample& sample =
            (i < sample_count) ? state.imu_samples[i] : state.imu_samples[sample_count - 1];
        write_int16_le(dst + 0, sample.accel_x);
        write_int16_le(dst + 2, sample.accel_y);
        write_int16_le(dst + 4, sample.accel_z);
        write_int16_le(dst + 6, sample.gyro_x);
        write_int16_le(dst + 8, sample.gyro_y);
        write_int16_le(dst + 10, sample.gyro_z);
        dst += 12;
    }
}

static void fill_quaternion_imu_report_data(const SwitchInputState& state, uint32_t now_ms) {
    if (state.imu_sample_count > 0) {
        uint8_t sample_count = state.imu_sample_count > 3 ? 3 : state.imu_sample_count;
        for (uint8_t i = 0; i < 3; ++i) {
            const SwitchImuSample& sample =
                (i < sample_count) ? state.imu_samples[i] : state.imu_samples[sample_count - 1];
            integrate_motion_sample(sample);
        }
    }

    uint8_t* dst = switch_report.imuData;
    memset(dst, 0x00, sizeof(switch_report.imuData));

    // Mode 2 accelerometer vectors are encoded in Y, X, Z order.
    write_int16_le(dst + 0, motion_quaternion.accel_y);
    write_int16_le(dst + 2, motion_quaternion.accel_x);
    write_int16_le(dst + 4, motion_quaternion.accel_z);

    float quaternion[4] = {
        motion_quaternion.x,
        motion_quaternion.y,
        motion_quaternion.z,
        motion_quaternion.w,
    };
    uint8_t max_index = 0;
    for (uint8_t i = 1; i < 4; ++i) {
        if (fabsf(quaternion[i]) > fabsf(quaternion[max_index])) {
            max_index = i;
        }
    }

    uint32_t packed_component[3]{};
    float sign = quaternion[max_index] < 0.0f ? -1.0f : 1.0f;
    for (uint8_t i = 0; i < 3; ++i) {
        int32_t component = static_cast<int32_t>(
            quaternion[(max_index + i + 1) & 3] * 1073741824.0f * sign);
        packed_component[i] = static_cast<uint32_t>(component >> 10) & 0x1FFFFFu;
    }

    write_bits_le(dst, 48, 2, 2);
    write_bits_le(dst, 50, max_index, 2);
    write_bits_le(dst, 52, packed_component[0], 21);
    write_bits_le(dst, 73, packed_component[1], 21);
    write_bits_le(dst, 94, packed_component[2] & 0x3u, 2);
    write_bits_le(dst, 144, packed_component[2] >> 2, 19);

    uint16_t timestamp = static_cast<uint16_t>(now_ms & 0x7FFu);
    write_bits_le(dst, 271, timestamp, 11);
    write_bits_le(dst, 282, 3, 6);
}

static void fill_imu_report_data(const SwitchInputState& state, uint32_t now_ms) {
    switch (imu_mode) {
        case SwitchImuMode::Raw:
            fill_raw_imu_report_data(state);
            break;
        case SwitchImuMode::Quaternion:
            fill_quaternion_imu_report_data(state, now_ms);
            break;
        case SwitchImuMode::Off:
        default:
            memset(switch_report.imuData, 0x00, sizeof(switch_report.imuData));
            break;
    }
}

static SwitchInputState make_neutral_state() {
    SwitchInputState s{};
    s.lx = SWITCH_PRO_JOYSTICK_MID;
    s.ly = SWITCH_PRO_JOYSTICK_MID;
    s.rx = SWITCH_PRO_JOYSTICK_MID;
    s.ry = SWITCH_PRO_JOYSTICK_MID;
    s.imu_sample_count = 0;
    return s;
}

static void send_identify() {
    memset(report_buffer, 0x00, sizeof(report_buffer));
    report_buffer[0] = REPORT_USB_INPUT_81;
    report_buffer[1] = IDENTIFY;
    report_buffer[2] = 0x00;
    report_buffer[3] = device_info.controllerType;
    for (uint8_t i = 0; i < 6; i++) {
        report_buffer[4 + i] = device_info.macAddress[5 - i];
    }
}

static bool send_report(uint8_t reportID, const void* reportData, uint16_t reportLength) {
    bool result = tud_hid_report(reportID, reportData, reportLength);
    if (last_report_counter < 255) {
        last_report_counter++;
    } else {
        last_report_counter = 0;
    }
    if (!result) {
        LOG_PRINTF("[HID] send_report failed id=%u len=%u\n", reportID, reportLength);
    }
    return result;
}

static void read_spi_flash(uint8_t* dest, uint32_t address, uint8_t size) {
    uint32_t addressBank = address & 0xFFFFFF00;
    uint32_t addressOffset = address & 0x000000FF;
    auto it = spi_flash_data.find(addressBank);

    if (it != spi_flash_data.end()) {
        const uint8_t* data = it->second;
        memcpy(dest, data + addressOffset, size);
    } else {
        memset(dest, 0xFF, size);
    }
}

static void forward_rumble_to_host(const uint8_t* report, uint16_t length) {
    // Output reports 0x10/0x21 include 8 rumble bytes starting at offset 2.
    if (!rumble_callback || length < 10) {
        return;
    }
    uint8_t rumble[8];
    memcpy(rumble, report + 2, sizeof(rumble));
    rumble_callback(rumble);
}

static void handle_config_report(uint8_t switchReportID, uint8_t switchReportSubID, const uint8_t *reportData, uint16_t reportLength) {
    bool canSend = false;
    last_host_activity_ms = to_ms_since_boot(get_absolute_time());

    switch (switchReportSubID) {
        case IDENTIFY:
            send_identify();
            canSend = true;
            LOG_PRINTF("[HID] CONFIG IDENTIFY\n");
            break;
        case HANDSHAKE:
            report_buffer[0] = REPORT_USB_INPUT_81;
            report_buffer[1] = HANDSHAKE;
            canSend = true;
            LOG_PRINTF("[HID] CONFIG HANDSHAKE\n");
            break;
        case BAUD_RATE:
            report_buffer[0] = REPORT_USB_INPUT_81;
            report_buffer[1] = BAUD_RATE;
            canSend = true;
            LOG_PRINTF("[HID] CONFIG BAUD_RATE\n");
            break;
        case DISABLE_USB_TIMEOUT:
            report_buffer[0] = REPORT_OUTPUT_30;
            report_buffer[1] = switchReportSubID;
            //if (handshakeCounter < 4) {
            //    handshakeCounter++;
            //} else {
                is_ready = true;
            //}
            canSend = true;
            LOG_PRINTF("[HID] CONFIG DISABLE_USB_TIMEOUT -> ready\n");
            break;
        case ENABLE_USB_TIMEOUT:
            report_buffer[0] = REPORT_OUTPUT_30;
            report_buffer[1] = switchReportSubID;
            canSend = true;
            LOG_PRINTF("[HID] CONFIG ENABLE_USB_TIMEOUT\n");
            break;
        default:
            report_buffer[0] = REPORT_OUTPUT_30;
            report_buffer[1] = switchReportSubID;
            canSend = true;
            LOG_PRINTF("[HID] CONFIG unknown subid=0x%02x\n", switchReportSubID);
            break;
    }

    if (canSend) is_report_queued = true;
}

static void handle_feature_report(uint8_t switchReportID, uint8_t switchReportSubID, const uint8_t *reportData, uint16_t reportLength) {
    uint8_t commandID = reportData[10];
    uint32_t spiReadAddress = 0;
    uint8_t spiReadSize = 0;
    bool canSend = false;
    last_host_activity_ms = to_ms_since_boot(get_absolute_time());

    report_buffer[0] = REPORT_OUTPUT_21;
    report_buffer[1] = last_report_counter;
    memcpy(report_buffer + 2, &switch_report.inputs, sizeof(SwitchInputReport));

    switch (commandID) {
        case GET_CONTROLLER_STATE:
            report_buffer[13] = 0x80;
            report_buffer[14] = commandID;
            report_buffer[15] = 0x03;
            canSend = true;
            LOG_PRINTF("[HID] FEATURE GET_CONTROLLER_STATE\n");
            break;
        case BLUETOOTH_PAIR_REQUEST:
            report_buffer[13] = 0x81;
            report_buffer[14] = commandID;
            report_buffer[15] = 0x03;
            canSend = true;
            LOG_PRINTF("[HID] FEATURE BLUETOOTH_PAIR_REQUEST\n");
            break;
        case REQUEST_DEVICE_INFO:
            report_buffer[13] = 0x82;
            report_buffer[14] = 0x02;
            memcpy(&report_buffer[15], &device_info, sizeof(device_info));
            canSend = true;
            LOG_PRINTF("[HID] FEATURE REQUEST_DEVICE_INFO\n");
            break;
        case SET_MODE:
            input_mode = reportData[11];
            report_buffer[13] = 0x80;
            report_buffer[14] = 0x03;
            report_buffer[15] = input_mode;
            canSend = true;
            LOG_PRINTF("[HID] FEATURE SET_MODE 0x%02x\n", input_mode);
            break;
        case TRIGGER_BUTTONS:
            report_buffer[13] = 0x83;
            report_buffer[14] = 0x04;
            canSend = true;
            LOG_PRINTF("[HID] FEATURE TRIGGER_BUTTONS\n");
            break;
        case SET_SHIPMENT:
            report_buffer[13] = 0x80;
            report_buffer[14] = commandID;
            canSend = true;
            LOG_PRINTF("[HID] FEATURE SET_SHIPMENT\n");
            break;
        case SPI_READ:
            spiReadAddress = (reportData[14] << 24) | (reportData[13] << 16) | (reportData[12] << 8) | (reportData[11]);
            spiReadSize = reportData[15];
            report_buffer[13] = 0x90;
            report_buffer[14] = reportData[10];
            report_buffer[15] = reportData[11];
            report_buffer[16] = reportData[12];
            report_buffer[17] = reportData[13];
            report_buffer[18] = reportData[14];
            report_buffer[19] = reportData[15];
            read_spi_flash(&report_buffer[20], spiReadAddress, spiReadSize);
            canSend = true;
            LOG_PRINTF("[HID] FEATURE SPI_READ addr=0x%08lx size=%u\n", (unsigned long)spiReadAddress, spiReadSize);
            break;
        case SET_NFC_IR_CONFIG:
            report_buffer[13] = 0x80;
            report_buffer[14] = commandID;
            canSend = true;
            LOG_PRINTF("[HID] FEATURE SET_NFC_IR_CONFIG\n");
            break;
        case SET_NFC_IR_STATE:
            report_buffer[13] = 0x80;
            report_buffer[14] = commandID;
            canSend = true;
            LOG_PRINTF("[HID] FEATURE SET_NFC_IR_STATE\n");
            break;
        case SET_PLAYER_LIGHTS:
            player_id = reportData[11];
            report_buffer[13] = 0x80;
            report_buffer[14] = commandID;
            canSend = true;
            LOG_PRINTF("[HID] FEATURE SET_PLAYER_LIGHTS player=%u\n", player_id);
            break;
        case GET_PLAYER_LIGHTS:
            player_id = reportData[11];
            report_buffer[13] = 0xB0;
            report_buffer[14] = commandID;
            report_buffer[15] = player_id;
            canSend = true;
            LOG_PRINTF("[HID] FEATURE GET_PLAYER_LIGHTS player=%u\n", player_id);
            break;
        case COMMAND_UNKNOWN_33:
            report_buffer[13] = 0x80;
            report_buffer[14] = commandID;
            report_buffer[15] = 0x03;
            canSend = true;
            LOG_PRINTF("[HID] FEATURE COMMAND_UNKNOWN_33\n");
            break;
        case SET_HOME_LIGHT:
            report_buffer[13] = 0x80;
            report_buffer[14] = commandID;
            report_buffer[15] = 0x00;
            canSend = true;
            LOG_PRINTF("[HID] FEATURE SET_HOME_LIGHT\n");
            break;
        case TOGGLE_IMU: {
            SwitchImuMode requested_mode = SwitchImuMode::Off;
            if (reportData[11] == static_cast<uint8_t>(SwitchImuMode::Raw)) {
                requested_mode = SwitchImuMode::Raw;
            } else if (reportData[11] == static_cast<uint8_t>(SwitchImuMode::Quaternion)) {
                requested_mode = SwitchImuMode::Quaternion;
            }
            if (requested_mode == SwitchImuMode::Quaternion && imu_mode != requested_mode) {
                reset_motion_quaternion();
            }
            imu_mode = requested_mode;
            report_buffer[13] = 0x80;
            report_buffer[14] = commandID;
            report_buffer[15] = 0x00;
            canSend = true;
            LOG_PRINTF("[HID] FEATURE TOGGLE_IMU %u\n", static_cast<unsigned>(imu_mode));
            break;
        }
        case IMU_SENSITIVITY:
            report_buffer[13] = 0x80;
            report_buffer[14] = commandID;
            canSend = true;
            LOG_PRINTF("[HID] FEATURE IMU_SENSITIVITY\n");
            break;
        case ENABLE_VIBRATION:
            is_vibration_enabled = reportData[11];
            report_buffer[13] = 0x80;
            report_buffer[14] = commandID;
            report_buffer[15] = 0x00;
            canSend = true;
            LOG_PRINTF("[HID] FEATURE ENABLE_VIBRATION %u\n", is_vibration_enabled);
            break;
        case READ_IMU:
            report_buffer[13] = 0xC0;
            report_buffer[14] = commandID;
            report_buffer[15] = reportData[11];
            report_buffer[16] = reportData[12];
            canSend = true;
            LOG_PRINTF("[HID] FEATURE READ_IMU addr=%u size=%u\n", reportData[11], reportData[12]);
            break;
        case GET_VOLTAGE:
            report_buffer[13] = 0xD0;
            report_buffer[14] = 0x50;
            report_buffer[15] = 0x83;
            report_buffer[16] = 0x06;
            canSend = true;
            LOG_PRINTF("[HID] FEATURE GET_VOLTAGE\n");
            break;
        default:
            report_buffer[13] = 0x80;
            report_buffer[14] = commandID;
            report_buffer[15] = 0x03;
            canSend = true;
            LOG_PRINTF("[HID] FEATURE unknown cmd=0x%02x\n", commandID);
            break;
    }

    if (canSend) is_report_queued = true;
}

static void update_switch_report_from_state() {
    switch_report.inputs.dpadUp =    g_input_state.dpad_up;
    switch_report.inputs.dpadDown =  g_input_state.dpad_down;
    switch_report.inputs.dpadLeft =  g_input_state.dpad_left;
    switch_report.inputs.dpadRight = g_input_state.dpad_right;

    switch_report.inputs.chargingGrip = 1;

    switch_report.inputs.buttonY = g_input_state.button_y;
    switch_report.inputs.buttonX = g_input_state.button_x;
    switch_report.inputs.buttonB = g_input_state.button_b;
    switch_report.inputs.buttonA = g_input_state.button_a;
    switch_report.inputs.buttonRightSR = 0;
    switch_report.inputs.buttonRightSL = 0;
    switch_report.inputs.buttonR = g_input_state.button_r;
    switch_report.inputs.buttonZR = g_input_state.button_zr;
    switch_report.inputs.buttonMinus = g_input_state.button_minus;
    switch_report.inputs.buttonPlus = g_input_state.button_plus;
    switch_report.inputs.buttonThumbR = g_input_state.button_r3;
    switch_report.inputs.buttonThumbL = g_input_state.button_l3;
    switch_report.inputs.buttonHome = g_input_state.button_home;
    switch_report.inputs.buttonCapture = g_input_state.button_capture;
    switch_report.inputs.buttonLeftSR = 0;
    switch_report.inputs.buttonLeftSL = 0;
    switch_report.inputs.buttonL = g_input_state.button_l;
    switch_report.inputs.buttonZL = g_input_state.button_zl;

    uint16_t scaleLeftStickX = scale16To12(g_input_state.lx);
    uint16_t scaleLeftStickY = scale16To12(g_input_state.ly);
    uint16_t scaleRightStickX = scale16To12(g_input_state.rx);
    uint16_t scaleRightStickY = scale16To12(g_input_state.ry);

    switch_report.inputs.leftStick.setX(std::min(std::max(scaleLeftStickX,leftMinX), leftMaxX));
    switch_report.inputs.leftStick.setY(-std::min(std::max(scaleLeftStickY,leftMinY), leftMaxY));
    switch_report.inputs.rightStick.setX(std::min(std::max(scaleRightStickX,rightMinX), rightMaxX));
    switch_report.inputs.rightStick.setY(-std::min(std::max(scaleRightStickY,rightMinY), rightMaxY));

    switch_report.rumbleReport = 0x09;
}

void switch_pro_init() {
    imu_mode = SwitchImuMode::Off;
    reset_motion_quaternion();
    player_id = 0;
    last_report_counter = 0;
    handshake_counter = 0;
    is_ready = false;
    is_initialized = false;
    is_report_queued = false;
    report_sent = false;
    forced_ready = false;
    forced_ready = true;
    is_ready = true;
    is_initialized = true;
    last_report_timer = 0;

    device_info = {
        .majorVersion = 0x03,
        .minorVersion = 0x48,
        .controllerType = SWITCH_TYPE_PRO_CONTROLLER,
        .unknown00 = 0x02,
        .macAddress = {0x7c, 0xbb, 0x8a, static_cast<uint8_t>(get_rand_32() % 0xff), static_cast<uint8_t>(get_rand_32() % 0xff), static_cast<uint8_t>(get_rand_32() % 0xff)},
        .unknown01 = 0x01,
        .storedColors = 0x02,
    };

    switch_report = {
        .reportID = 0x30,
        .timestamp = 0,

        .inputs {
            .connectionInfo = 0x01, // Pro Controller powered by the console
            .batteryLevel = 0x08,   // full battery

            .buttonY = 0,
            .buttonX = 0,
            .buttonB = 0,
            .buttonA = 0,
            .buttonRightSR = 0,
            .buttonRightSL = 0,
            .buttonR = 0,
            .buttonZR = 0,

            .buttonMinus = 0,
            .buttonPlus = 0,
            .buttonThumbR = 0,
            .buttonThumbL = 0,
            .buttonHome = 0,
            .buttonCapture = 0,
            .dummy = 0,
            .chargingGrip = 0,

            .dpadDown = 0,
            .dpadUp = 0,
            .dpadRight = 0,
            .dpadLeft = 0,
            .buttonLeftSL = 0,
            .buttonLeftSR = 0,
            .buttonL = 0,
            .buttonZL = 0,
            .leftStick = {0xFF, 0xF7, 0x7F},
            .rightStick = {0xFF, 0xF7, 0x7F},
        },
        .rumbleReport = 0,
        .imuData = {0x00},
        .padding = {0x00}
    };

    last_report_timer = to_ms_since_boot(get_absolute_time());
    last_host_activity_ms = last_report_timer;

    factory_config->leftStickCalibration.getRealMin(leftMinX, leftMinY);
    factory_config->leftStickCalibration.getCenter(leftCenX, leftCenY);
    factory_config->leftStickCalibration.getRealMax(leftMaxX, leftMaxY);
    factory_config->rightStickCalibration.getRealMin(rightMinX, rightMinY);
    factory_config->rightStickCalibration.getCenter(rightCenX, rightCenY);
    factory_config->rightStickCalibration.getRealMax(rightMaxX, rightMaxY);
}

void switch_pro_set_input(const SwitchInputState& state) {
    g_input_state = state;
}

bool switch_pro_task() {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    report_sent = false;
    bool regular_report_sent = false;

    update_switch_report_from_state();

    if (tud_suspended()) {
        tud_remote_wakeup();
    }

    if (is_report_queued) {
        if ((now - last_report_timer) > SWITCH_PRO_KEEPALIVE_TIMER) {
            if (tud_hid_ready() && send_report(queued_report_id, report_buffer, 64) == true ) {
                is_report_queued = false;
                last_report_timer = now;
            }
        }
        report_sent = true;
    }

    if (is_ready && !report_sent) {
        if ((now - last_report_timer) >= SWITCH_PRO_IMU_REPORT_TIMER) {
            // One timer tick per 5ms IMU frame; three frames per report.
            fill_imu_report_data(g_input_state, now);
            switch_report.timestamp += 3;
            void * inputReport = &switch_report;
            uint16_t report_size = sizeof(switch_report);
            if (tud_hid_ready() && send_report(0, inputReport, report_size) == true ) {
                memcpy(last_report, inputReport, report_size);
                g_input_state.imu_sample_count = 0;
                report_sent = true;
                regular_report_sent = true;
            }

            last_report_timer = now;
        }
    } else {
        if (!is_initialized) {
            send_identify();
            if (tud_hid_ready() && tud_hid_report(0, report_buffer, 64) == true) {
                is_initialized = true;
                report_sent = true;
            }

            last_report_timer = now;
        }
    }
    return regular_report_sent;
}

bool switch_pro_apply_uart_packet(const uint8_t* packet, uint8_t length, SwitchInputState* out_state) {
    // v2 format: 0xAA + 0x02 + payload_len + payload... + checksum
    if (length < 12) {
        return false;
    }
    if (packet[0] != 0xAA) {
        return false;
    }
    if (packet[1] != 0x02) {
        return false;
    }

    uint8_t payload_len = packet[2];
    if ((uint16_t)payload_len + 4u != length) {
        return false;
    }

    uint16_t sum = 0;
    for (uint16_t i = 0; i < (uint16_t)(3u + payload_len); ++i) {
        sum += packet[i];
    }
    if ((sum & 0xFF) != packet[length - 1]) {
        return false;
    }

    // payload: buttons(2 LE), hat, lx, ly, rx, ry, imu_count, [imu_samples...]
    if (payload_len < 8) {
        return false;
    }

    SwitchProOutReport out{};
    out.buttons = static_cast<uint16_t>(packet[3]) | (static_cast<uint16_t>(packet[4]) << 8);
    out.hat = packet[5];
    out.lx = packet[6];
    out.ly = packet[7];
    out.rx = packet[8];
    out.ry = packet[9];
    uint8_t imu_count = packet[10];
    if (imu_count > 3) {
        imu_count = 3;
    }

    uint16_t required_payload_len = static_cast<uint16_t>(8u + static_cast<uint16_t>(imu_count) * 12u);
    if (payload_len < required_payload_len) {
        return false;
    }

    auto expand_axis = [](uint8_t v) -> uint16_t {
        return static_cast<uint16_t>(v) << 8 | v;
    };

    SwitchInputState state = make_neutral_state();
    state.imu_sample_count = imu_count;

    auto read_int16 = [](const uint8_t* src) -> int16_t {
        return static_cast<int16_t>(static_cast<uint16_t>(src[0]) | (static_cast<uint16_t>(src[1]) << 8));
    };
    for (uint8_t i = 0; i < imu_count; ++i) {
        const uint8_t* base = &packet[11 + i * 12];
        state.imu_samples[i].accel_x = read_int16(base + 0);
        state.imu_samples[i].accel_y = read_int16(base + 2);
        state.imu_samples[i].accel_z = read_int16(base + 4);
        state.imu_samples[i].gyro_x = read_int16(base + 6);
        state.imu_samples[i].gyro_y = read_int16(base + 8);
        state.imu_samples[i].gyro_z = read_int16(base + 10);
    }

    switch (out.hat) {
        case SWITCH_PRO_HAT_UP: state.dpad_up = true; break;
        case SWITCH_PRO_HAT_UPRIGHT: state.dpad_up = true; state.dpad_right = true; break;
        case SWITCH_PRO_HAT_RIGHT: state.dpad_right = true; break;
        case SWITCH_PRO_HAT_DOWNRIGHT: state.dpad_down = true; state.dpad_right = true; break;
        case SWITCH_PRO_HAT_DOWN: state.dpad_down = true; break;
        case SWITCH_PRO_HAT_DOWNLEFT: state.dpad_down = true; state.dpad_left = true; break;
        case SWITCH_PRO_HAT_LEFT: state.dpad_left = true; break;
        case SWITCH_PRO_HAT_UPLEFT: state.dpad_up = true; state.dpad_left = true; break;
        default: break;
    }

    state.button_y = out.buttons & SWITCH_PRO_MASK_Y;
    state.button_x = out.buttons & SWITCH_PRO_MASK_X;
    state.button_b = out.buttons & SWITCH_PRO_MASK_B;
    state.button_a = out.buttons & SWITCH_PRO_MASK_A;
    state.button_r = out.buttons & SWITCH_PRO_MASK_R;
    state.button_zr = out.buttons & SWITCH_PRO_MASK_ZR;
    state.button_plus = out.buttons & SWITCH_PRO_MASK_PLUS;
    state.button_minus = out.buttons & SWITCH_PRO_MASK_MINUS;
    state.button_r3 = out.buttons & SWITCH_PRO_MASK_R3;
    state.button_l3 = out.buttons & SWITCH_PRO_MASK_L3;
    state.button_home = out.buttons & SWITCH_PRO_MASK_HOME;
    state.button_capture = out.buttons & SWITCH_PRO_MASK_CAPTURE;
    state.button_zl = out.buttons & SWITCH_PRO_MASK_ZL;
    state.button_l = out.buttons & SWITCH_PRO_MASK_L;

    state.lx = expand_axis(out.lx);
    state.ly = expand_axis(out.ly);
    state.rx = expand_axis(out.rx);
    state.ry = expand_axis(out.ry);

    if (!out_state) {
        return false;
    }
    *out_state = state;
    return true;
}

void switch_pro_set_rumble_callback(SwitchRumbleCallback cb) {
    rumble_callback = cb;
}

bool switch_pro_is_ready() {
    return is_ready;
}

// HID callbacks
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    (void)instance;
    LOG_PRINTF("[HID] get_report id=%u type=%u len=%u\n", report_id, report_type, reqlen);
    if (!buffer) return 0;

    // Serve the current input report for any GET_REPORT request.
    uint16_t report_size = sizeof(switch_report);
    if (reqlen < report_size) report_size = reqlen;
    memcpy(buffer, &switch_report, report_size);
    return report_size;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
    (void)instance;
    if (report_type != HID_REPORT_TYPE_OUTPUT) return;

    memset(report_buffer, 0x00, bufsize);

    uint8_t switchReportID = buffer[0];
    uint8_t switchReportSubID = buffer[1];
    LOG_PRINTF("[HID] set_report type=%d id=%u switchRID=0x%02x sub=0x%02x len=%u\n",
               report_type, report_id, switchReportID, switchReportSubID, bufsize);
    if (switchReportID == REPORT_OUTPUT_10 || switchReportID == REPORT_OUTPUT_21) {
        forward_rumble_to_host(buffer, bufsize);
    }
    if (switchReportID == REPORT_OUTPUT_00) {
        // No-op, just acknowledge to clear any stalls.
        return;
    } else if (switchReportID == REPORT_FEATURE) {
        queued_report_id = report_id;
        handle_feature_report(switchReportID, switchReportSubID, buffer, bufsize);
    } else if (switchReportID == REPORT_CONFIGURATION) {
        queued_report_id = report_id;
        handle_config_report(switchReportID, switchReportSubID, buffer, bufsize);
    } else {
    }
}

void tud_hid_report_received_cb(uint8_t instance, uint8_t report_id, uint8_t const* buffer, uint16_t bufsize) {
    (void)instance;
    // Host sent data on interrupt OUT; mirror the control path handling.
    memset(report_buffer, 0x00, bufsize);
    uint8_t switchReportID = buffer[0];
    uint8_t switchReportSubID = buffer[1];
    LOG_PRINTF("[HID] report_received id=%u switchRID=0x%02x sub=0x%02x len=%u\n",
               report_id, switchReportID, switchReportSubID, bufsize);
    if (switchReportID == REPORT_OUTPUT_10 || switchReportID == REPORT_OUTPUT_21) {
        forward_rumble_to_host(buffer, bufsize);
    }
    if (switchReportID == REPORT_OUTPUT_00) {
        return;
    } else if (switchReportID == REPORT_FEATURE) {
        queued_report_id = report_id;
        handle_feature_report(switchReportID, switchReportSubID, buffer, bufsize);
    } else if (switchReportID == REPORT_CONFIGURATION) {
        queued_report_id = report_id;
        handle_config_report(switchReportID, switchReportSubID, buffer, bufsize);
    }
}

uint8_t const * tud_hid_descriptor_report_cb(uint8_t itf) {
    (void)itf;
    return switch_pro_report_descriptor;
}

uint8_t const * tud_descriptor_device_cb(void) {
    return switch_pro_device_descriptor;
}

uint8_t const * tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return switch_pro_configuration_descriptor;
}

bool tud_control_request_cb(uint8_t rhport, tusb_control_request_t const * request) {
    (void)rhport;
    LOG_PRINTF("[CTRL] bmReq=0x%02x bReq=0x%02x wValue=0x%04x wIndex=0x%04x wLen=%u\n",
               request->bmRequestType, request->bRequest, request->wValue, request->wIndex, request->wLength);
    return false; // let TinyUSB handle it normally
}

void tud_mount_cb(void) {
    LOG_PRINTF("[USB] mount_cb\n");
    last_host_activity_ms = to_ms_since_boot(get_absolute_time());
    forced_ready = false;
    is_ready = false;
    is_initialized = false;
}

void tud_umount_cb(void) {
    LOG_PRINTF("[USB] umount_cb\n");
    forced_ready = false;
    is_ready = false;
    is_initialized = false;
}

static uint16_t desc_str[32];

uint16_t const * tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;

    uint8_t chr_count;

    if ( index == 0 ) {
        memcpy(&desc_str[1], switch_pro_string_language, 2);
        chr_count = 1;
    } else {
        if ( index >= sizeof(switch_pro_string_descriptors)/sizeof(switch_pro_string_descriptors[0]) ) return nullptr;

        const uint8_t *str = switch_pro_string_descriptors[index];

        chr_count = 0;
        while ( str[chr_count] ) chr_count++;
        if ( chr_count > 31 ) chr_count = 31;

        for(uint8_t i=0; i<chr_count; i++) {
            desc_str[1+i] = str[i];
        }
    }

    desc_str[0] = (uint16_t) ((0x03 << 8 ) | (2*chr_count + 2));
    return desc_str;
}
