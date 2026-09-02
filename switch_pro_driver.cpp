#include "switch_pro_driver.h"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cstring>
#include <stdio.h>
#include "pico/rand.h"
#include "pico/time.h"
#include "tusb.h"
#ifdef SWITCH_PICO_ADAPTER_FEASIBILITY
#include "adapter_host_probe.h"
#include "xinput_feasibility_descriptors.h"
#endif

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

enum class SwitchImuMode : uint8_t {
    Off = 0,
    Raw = 1,
    Quaternion = 2,
};

struct MotionQuaternion {
    float x;
    float y;
    float z;
    float w;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
};

struct SwitchProContext {
    SwitchInputState input_state{};
    uint8_t report_buffer[SWITCH_PRO_ENDPOINT_SIZE]{};
    SwitchProReport switch_report{};
    uint8_t last_report_counter = 0;
    uint32_t last_report_timer = 0;
    bool is_ready = false;
    bool is_initialized = false;
    bool is_report_queued = false;
    SwitchDeviceInfo device_info{};
    SwitchRgbColor grip_color{};
    uint8_t player_id = 0;
    uint8_t input_mode = 0x30;
    SwitchImuMode imu_mode = SwitchImuMode::Off;
    bool is_vibration_enabled = false;
    uint16_t left_min_x = 0;
    uint16_t left_min_y = 0;
    uint16_t left_max_x = 0;
    uint16_t left_max_y = 0;
    uint16_t right_min_x = 0;
    uint16_t right_min_y = 0;
    uint16_t right_max_x = 0;
    uint16_t right_max_y = 0;
    SwitchRumbleCallback rumble_callback = nullptr;
    SwitchHapticsDecoder rumble_decoder{};
    MotionQuaternion motion_quaternion{0.0f, 0.0f, 0.0f, 1.0f, 0, 0, 0};
};

static_assert(SWITCH_PICO_HID_INSTANCE_COUNT > 0,
              "at least one HID instance is required");
static SwitchProContext contexts[SWITCH_PICO_HID_INSTANCE_COUNT]{};

static SwitchProContext* context_for(uint8_t instance) {
    if (instance >= SWITCH_PICO_HID_INSTANCE_COUNT) {
        return nullptr;
    }
    return &contexts[instance];
}

// Optional compile-time color palette shared with Bluetooth controller LEDs.
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
#ifndef SWITCH_COLOR_SLOT_1_R
#define SWITCH_COLOR_SLOT_1_R 0x00
#define SWITCH_COLOR_SLOT_1_G 0x89
#define SWITCH_COLOR_SLOT_1_B 0xEB
#define SWITCH_COLOR_SLOT_2_R 0xE6
#define SWITCH_COLOR_SLOT_2_G 0x39
#define SWITCH_COLOR_SLOT_2_B 0x46
#define SWITCH_COLOR_SLOT_3_R 0xF6
#define SWITCH_COLOR_SLOT_3_G 0xC9
#define SWITCH_COLOR_SLOT_3_B 0x45
#define SWITCH_COLOR_SLOT_4_R 0x2E
#define SWITCH_COLOR_SLOT_4_G 0xCC
#define SWITCH_COLOR_SLOT_4_B 0x71
#endif


static constexpr SwitchRgbColor slot_colors[] = {
    {SWITCH_COLOR_SLOT_1_R, SWITCH_COLOR_SLOT_1_G, SWITCH_COLOR_SLOT_1_B},
    {SWITCH_COLOR_SLOT_2_R, SWITCH_COLOR_SLOT_2_G, SWITCH_COLOR_SLOT_2_B},
    {SWITCH_COLOR_SLOT_3_R, SWITCH_COLOR_SLOT_3_G, SWITCH_COLOR_SLOT_3_B},
    {SWITCH_COLOR_SLOT_4_R, SWITCH_COLOR_SLOT_4_G, SWITCH_COLOR_SLOT_4_B},
};

static_assert(SWITCH_PICO_HID_INSTANCE_COUNT <=
              sizeof(slot_colors) / sizeof(slot_colors[0]));

SwitchRgbColor switch_pro_get_slot_color(uint8_t instance) {
    if (instance >= sizeof(slot_colors) / sizeof(slot_colors[0])) {
        return {};
    }
    return slot_colors[instance];
}
SwitchRgbColor switch_pro_get_slot_light_color(uint8_t instance) {
    if (instance >= sizeof(slot_colors) / sizeof(slot_colors[0])) {
        return {};
    }
    return switch_pro_calibrate_light_color(slot_colors[instance]);
}



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
    SWITCH_COLOR_SLOT_1_R, SWITCH_COLOR_SLOT_1_G, SWITCH_COLOR_SLOT_1_B,

    // right grip color
    SWITCH_COLOR_SLOT_1_R, SWITCH_COLOR_SLOT_1_G, SWITCH_COLOR_SLOT_1_B,

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

static const SwitchFactoryConfig* const factory_config =
    reinterpret_cast<const SwitchFactoryConfig*>(factory_config_data);

static inline uint16_t scale16To12(uint16_t pos) { return pos >> 4; }

static void reset_motion_quaternion(SwitchProContext& context) {
    context.motion_quaternion =
        {0.0f, 0.0f, 0.0f, 1.0f, 0, 0, 0};
}

static void write_int16_le(uint8_t* dst, int16_t value) {
    dst[0] = static_cast<uint8_t>(value & 0xFF);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

static void write_bits_le(uint8_t* dst, uint16_t bit_offset, uint32_t value,
                          uint8_t width) {
    for (uint8_t bit = 0; bit < width; ++bit) {
        if ((value & (1u << bit)) != 0) {
            uint16_t output_bit = static_cast<uint16_t>(bit_offset + bit);
            dst[output_bit >> 3] |=
                static_cast<uint8_t>(1u << (output_bit & 7u));
        }
    }
}

static void integrate_motion_sample(SwitchProContext& context,
                                    const SwitchImuSample& sample) {
    constexpr float sample_dt = 0.005f;
    constexpr float gyro_rad_per_lsb = 1.0f / 818.5f;
    MotionQuaternion& quaternion = context.motion_quaternion;

    // Nintendo mode 2 uses Y, X, Z sensor order for quaternion axes.
    float angle_x =
        static_cast<float>(sample.gyro_y) * gyro_rad_per_lsb * sample_dt;
    float angle_y =
        static_cast<float>(sample.gyro_x) * gyro_rad_per_lsb * sample_dt;
    float angle_z =
        static_cast<float>(sample.gyro_z) * gyro_rad_per_lsb * sample_dt;
    float norm =
        sqrtf(angle_x * angle_x + angle_y * angle_y + angle_z * angle_z);
    float half = 0.5f * norm;
    float vector_scale = norm > 1e-12f ? sinf(half) / norm : 0.5f;
    float scalar = norm > 1e-12f ? cosf(half) : 1.0f;

    float dx = angle_x * vector_scale;
    float dy = angle_y * vector_scale;
    float dz = angle_z * vector_scale;
    float dw = scalar;

    float x = quaternion.w * dx + quaternion.x * dw
            + quaternion.y * dz - quaternion.z * dy;
    float y = quaternion.w * dy - quaternion.x * dz
            + quaternion.y * dw + quaternion.z * dx;
    float z = quaternion.w * dz + quaternion.x * dy
            - quaternion.y * dx + quaternion.z * dw;
    float w = quaternion.w * dw - quaternion.x * dx
            - quaternion.y * dy - quaternion.z * dz;
    float magnitude = sqrtf(x * x + y * y + z * z + w * w);
    if (magnitude > 1e-12f) {
        float inverse = 1.0f / magnitude;
        quaternion.x = x * inverse;
        quaternion.y = y * inverse;
        quaternion.z = z * inverse;
        quaternion.w = w * inverse;
    } else {
        reset_motion_quaternion(context);
    }

    quaternion.accel_x = sample.accel_x;
    quaternion.accel_y = sample.accel_y;
    quaternion.accel_z = sample.accel_z;
}

static void fill_raw_imu_report_data(SwitchProContext& context,
                                     const SwitchInputState& state) {
    if (state.imu_sample_count == 0) {
        memset(context.switch_report.imuData, 0x00,
               sizeof(context.switch_report.imuData));
        return;
    }
    uint8_t sample_count =
        state.imu_sample_count > 3 ? 3 : state.imu_sample_count;
    uint8_t* dst = context.switch_report.imuData;
    for (uint8_t i = 0; i < 3; ++i) {
        const SwitchImuSample& sample =
            (i < sample_count) ? state.imu_samples[i]
                               : state.imu_samples[sample_count - 1];
        write_int16_le(dst + 0, sample.accel_x);
        write_int16_le(dst + 2, sample.accel_y);
        write_int16_le(dst + 4, sample.accel_z);
        write_int16_le(dst + 6, sample.gyro_x);
        write_int16_le(dst + 8, sample.gyro_y);
        write_int16_le(dst + 10, sample.gyro_z);
        dst += 12;
    }
}

static void fill_quaternion_imu_report_data(SwitchProContext& context,
                                            const SwitchInputState& state,
                                            uint32_t now_ms) {
    if (state.imu_sample_count > 0) {
        uint8_t sample_count =
            state.imu_sample_count > 3 ? 3 : state.imu_sample_count;
        for (uint8_t i = 0; i < 3; ++i) {
            const SwitchImuSample& sample =
                (i < sample_count) ? state.imu_samples[i]
                                   : state.imu_samples[sample_count - 1];
            integrate_motion_sample(context, sample);
        }
    }

    MotionQuaternion& quaternion = context.motion_quaternion;
    uint8_t* dst = context.switch_report.imuData;
    memset(dst, 0x00, sizeof(context.switch_report.imuData));

    // Mode 2 accelerometer vectors are encoded in Y, X, Z order.
    write_int16_le(dst + 0, quaternion.accel_y);
    write_int16_le(dst + 2, quaternion.accel_x);
    write_int16_le(dst + 4, quaternion.accel_z);

    float components[4] = {
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    };
    uint8_t max_index = 0;
    for (uint8_t i = 1; i < 4; ++i) {
        if (fabsf(components[i]) > fabsf(components[max_index])) {
            max_index = i;
        }
    }

    uint32_t packed_component[3]{};
    float sign = components[max_index] < 0.0f ? -1.0f : 1.0f;
    for (uint8_t i = 0; i < 3; ++i) {
        int32_t component = static_cast<int32_t>(
            components[(max_index + i + 1) & 3] * 1073741824.0f * sign);
        packed_component[i] =
            static_cast<uint32_t>(component >> 10) & 0x1FFFFFu;
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

static void fill_imu_report_data(SwitchProContext& context,
                                 const SwitchInputState& state,
                                 uint32_t now_ms) {
    switch (context.imu_mode) {
        case SwitchImuMode::Raw:
            fill_raw_imu_report_data(context, state);
            break;
        case SwitchImuMode::Quaternion:
            fill_quaternion_imu_report_data(context, state, now_ms);
            break;
        case SwitchImuMode::Off:
        default:
            memset(context.switch_report.imuData, 0x00,
                   sizeof(context.switch_report.imuData));
            break;
    }
}

static void update_switch_report_from_state(SwitchProContext& context);

static SwitchInputState make_neutral_state() {
    SwitchInputState s{};
    s.lx = SWITCH_PRO_JOYSTICK_MID;
    s.ly = SWITCH_PRO_JOYSTICK_MID;
    s.rx = SWITCH_PRO_JOYSTICK_MID;
    s.ry = SWITCH_PRO_JOYSTICK_MID;
    s.imu_sample_count = 0;
    return s;
}

static void reset_context_runtime(SwitchProContext& context, uint32_t now,
                                  bool ready_before_mount) {
    context.input_state = make_neutral_state();
    memset(context.report_buffer, 0x00, sizeof(context.report_buffer));
    context.switch_report = {};
    context.switch_report.reportID = 0x30;
    context.switch_report.inputs.connectionInfo = 0x01;
    context.switch_report.inputs.batteryLevel = 0x08;
    update_switch_report_from_state(context);
    context.last_report_counter = 0;
    context.last_report_timer = now;
    context.is_ready = ready_before_mount;
    context.is_initialized = ready_before_mount;
    context.is_report_queued = false;
    context.player_id = 0;
    context.input_mode = 0x30;
    context.imu_mode = SwitchImuMode::Off;
    context.is_vibration_enabled = false;
    context.rumble_decoder.reset();
    reset_motion_quaternion(context);
}

static void reset_all_contexts(bool ready_before_mount) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    for (uint8_t instance = 0; instance < SWITCH_PICO_HID_INSTANCE_COUNT;
         ++instance) {
        reset_context_runtime(contexts[instance], now, ready_before_mount);
    }
}

static void send_identify(SwitchProContext& context) {
    memset(context.report_buffer, 0x00, sizeof(context.report_buffer));
    context.report_buffer[0] = REPORT_USB_INPUT_81;
    context.report_buffer[1] = IDENTIFY;
    context.report_buffer[2] = 0x00;
    context.report_buffer[3] = context.device_info.controllerType;
    for (uint8_t i = 0; i < 6; ++i) {
        context.report_buffer[4 + i] =
            context.device_info.macAddress[5 - i];
    }
}

static bool send_report(uint8_t instance, SwitchProContext& context,
                        uint8_t report_id, const void* report_data,
                        uint16_t report_length) {
    bool result =
        tud_hid_n_report(instance, report_id, report_data, report_length);
    ++context.last_report_counter;
    if (!result) {
        LOG_PRINTF("[HID %u] send_report failed id=%u len=%u\n", instance,
                   report_id, report_length);
    }
    return result;
}

static void read_spi_flash(const SwitchProContext& context, uint8_t* dest,
                           uint32_t address, uint8_t size) {
    uint32_t address_bank = address & 0xFFFFFF00u;
    uint32_t address_offset = address & 0x000000FFu;
    const uint8_t* data = nullptr;
    if (address_bank == 0x6000u) {
        data = factory_config_data;
    } else if (address_bank == 0x8000u) {
        data = user_calibration_data;
    }

    if (data == nullptr) {
        memset(dest, 0xFF, size);
        return;
    }
    memcpy(dest, data + address_offset, size);
    if (address_bank != 0x6000u) {
        return;
    }

    static_assert(sizeof(SwitchRgbColor) == sizeof(SwitchColorDefinition));
    const uint8_t* color =
        reinterpret_cast<const uint8_t*>(&context.grip_color);
    constexpr size_t left_offset =
        offsetof(SwitchFactoryConfig, leftGripColor);
    constexpr size_t right_offset =
        offsetof(SwitchFactoryConfig, rightGripColor);
    for (uint8_t index = 0; index < size; ++index) {
        const size_t offset = address_offset + index;
        if (offset >= left_offset &&
            offset < left_offset + sizeof(SwitchRgbColor)) {
            dest[index] = color[offset - left_offset];
        } else if (offset >= right_offset &&
                   offset < right_offset + sizeof(SwitchRgbColor)) {
            dest[index] = color[offset - right_offset];
        }
    }
}

static void forward_decoded_rumble(uint8_t instance,
                                   SwitchProContext& context,
                                   const uint8_t* report, uint16_t length) {
    // Output reports 0x10/0x01 include 8 rumble bytes starting at offset 2.
    if (length < 10) {
        return;
    }

    SwitchRumbleOutput rumble = context.rumble_decoder.decode(report + 2);
    if (context.rumble_callback != nullptr) {
        context.rumble_callback(instance, rumble);
    }
}

static void handle_config_report(SwitchProContext& context,
                                 uint8_t switch_report_sub_id) {
    switch (switch_report_sub_id) {
        case IDENTIFY:
            send_identify(context);
            LOG_PRINTF("[HID] CONFIG IDENTIFY\n");
            break;
        case HANDSHAKE:
            context.report_buffer[0] = REPORT_USB_INPUT_81;
            context.report_buffer[1] = HANDSHAKE;
            LOG_PRINTF("[HID] CONFIG HANDSHAKE\n");
            break;
        case BAUD_RATE:
            context.report_buffer[0] = REPORT_USB_INPUT_81;
            context.report_buffer[1] = BAUD_RATE;
            LOG_PRINTF("[HID] CONFIG BAUD_RATE\n");
            break;
        case DISABLE_USB_TIMEOUT:
            context.report_buffer[0] = REPORT_OUTPUT_30;
            context.report_buffer[1] = switch_report_sub_id;
            context.is_ready = true;
            LOG_PRINTF("[HID] CONFIG DISABLE_USB_TIMEOUT -> ready\n");
            break;
        case ENABLE_USB_TIMEOUT:
            context.report_buffer[0] = REPORT_OUTPUT_30;
            context.report_buffer[1] = switch_report_sub_id;
            LOG_PRINTF("[HID] CONFIG ENABLE_USB_TIMEOUT\n");
            break;
        default:
            context.report_buffer[0] = REPORT_OUTPUT_30;
            context.report_buffer[1] = switch_report_sub_id;
            LOG_PRINTF("[HID] CONFIG unknown subid=0x%02x\n",
                       switch_report_sub_id);
            break;
    }

    context.is_report_queued = true;
}

static void handle_feature_report(SwitchProContext& context,
                                  const uint8_t* report_data) {
    uint8_t command_id = report_data[10];
    uint32_t spi_read_address = 0;
    uint8_t spi_read_size = 0;

    context.report_buffer[0] = REPORT_OUTPUT_21;
    context.report_buffer[1] = context.last_report_counter;
    memcpy(context.report_buffer + 2, &context.switch_report.inputs,
           sizeof(SwitchInputReport));

    switch (command_id) {
        case GET_CONTROLLER_STATE:
            context.report_buffer[13] = 0x80;
            context.report_buffer[14] = command_id;
            context.report_buffer[15] = 0x03;
            LOG_PRINTF("[HID] FEATURE GET_CONTROLLER_STATE\n");
            break;
        case BLUETOOTH_PAIR_REQUEST:
            context.report_buffer[13] = 0x81;
            context.report_buffer[14] = command_id;
            context.report_buffer[15] = 0x03;
            LOG_PRINTF("[HID] FEATURE BLUETOOTH_PAIR_REQUEST\n");
            break;
        case REQUEST_DEVICE_INFO:
            context.report_buffer[13] = 0x82;
            context.report_buffer[14] = 0x02;
            memcpy(&context.report_buffer[15], &context.device_info,
                   sizeof(context.device_info));
            LOG_PRINTF("[HID] FEATURE REQUEST_DEVICE_INFO\n");
            break;
        case SET_MODE:
            context.input_mode = report_data[11];
            context.report_buffer[13] = 0x80;
            context.report_buffer[14] = 0x03;
            context.report_buffer[15] = context.input_mode;
            LOG_PRINTF("[HID] FEATURE SET_MODE 0x%02x\n",
                       context.input_mode);
            break;
        case TRIGGER_BUTTONS:
            context.report_buffer[13] = 0x83;
            context.report_buffer[14] = 0x04;
            LOG_PRINTF("[HID] FEATURE TRIGGER_BUTTONS\n");
            break;
        case SET_SHIPMENT:
            context.report_buffer[13] = 0x80;
            context.report_buffer[14] = command_id;
            LOG_PRINTF("[HID] FEATURE SET_SHIPMENT\n");
            break;
        case SPI_READ:
            spi_read_address =
                (static_cast<uint32_t>(report_data[14]) << 24u) |
                (static_cast<uint32_t>(report_data[13]) << 16u) |
                (static_cast<uint32_t>(report_data[12]) << 8u) |
                static_cast<uint32_t>(report_data[11]);
            spi_read_size = report_data[15];
            context.report_buffer[13] = 0x90;
            context.report_buffer[14] = report_data[10];
            context.report_buffer[15] = report_data[11];
            context.report_buffer[16] = report_data[12];
            context.report_buffer[17] = report_data[13];
            context.report_buffer[18] = report_data[14];
            context.report_buffer[19] = report_data[15];
            read_spi_flash(context, &context.report_buffer[20],
                           spi_read_address, spi_read_size);
            LOG_PRINTF("[HID] FEATURE SPI_READ addr=0x%08lx size=%u\n",
                       static_cast<unsigned long>(spi_read_address),
                       spi_read_size);
            break;
        case SET_NFC_IR_CONFIG:
            context.report_buffer[13] = 0x80;
            context.report_buffer[14] = command_id;
            LOG_PRINTF("[HID] FEATURE SET_NFC_IR_CONFIG\n");
            break;
        case SET_NFC_IR_STATE:
            context.report_buffer[13] = 0x80;
            context.report_buffer[14] = command_id;
            LOG_PRINTF("[HID] FEATURE SET_NFC_IR_STATE\n");
            break;
        case SET_PLAYER_LIGHTS:
            context.player_id = report_data[11];
            context.report_buffer[13] = 0x80;
            context.report_buffer[14] = command_id;
            LOG_PRINTF("[HID] FEATURE SET_PLAYER_LIGHTS player=%u\n",
                       context.player_id);
            break;
        case GET_PLAYER_LIGHTS:
            context.player_id = report_data[11];
            context.report_buffer[13] = 0xB0;
            context.report_buffer[14] = command_id;
            context.report_buffer[15] = context.player_id;
            LOG_PRINTF("[HID] FEATURE GET_PLAYER_LIGHTS player=%u\n",
                       context.player_id);
            break;
        case COMMAND_UNKNOWN_33:
            context.report_buffer[13] = 0x80;
            context.report_buffer[14] = command_id;
            context.report_buffer[15] = 0x03;
            LOG_PRINTF("[HID] FEATURE COMMAND_UNKNOWN_33\n");
            break;
        case SET_HOME_LIGHT:
            context.report_buffer[13] = 0x80;
            context.report_buffer[14] = command_id;
            context.report_buffer[15] = 0x00;
            LOG_PRINTF("[HID] FEATURE SET_HOME_LIGHT\n");
            break;
        case TOGGLE_IMU: {
            SwitchImuMode requested_mode = SwitchImuMode::Off;
            if (report_data[11] ==
                static_cast<uint8_t>(SwitchImuMode::Raw)) {
                requested_mode = SwitchImuMode::Raw;
            } else if (report_data[11] ==
                       static_cast<uint8_t>(SwitchImuMode::Quaternion)) {
                requested_mode = SwitchImuMode::Quaternion;
            }
            if (requested_mode == SwitchImuMode::Quaternion &&
                context.imu_mode != requested_mode) {
                reset_motion_quaternion(context);
            }
            context.imu_mode = requested_mode;
            context.report_buffer[13] = 0x80;
            context.report_buffer[14] = command_id;
            context.report_buffer[15] = 0x00;
            LOG_PRINTF("[HID] FEATURE TOGGLE_IMU %u\n",
                       static_cast<unsigned>(context.imu_mode));
            break;
        }
        case IMU_SENSITIVITY:
            context.report_buffer[13] = 0x80;
            context.report_buffer[14] = command_id;
            LOG_PRINTF("[HID] FEATURE IMU_SENSITIVITY\n");
            break;
        case ENABLE_VIBRATION:
            context.is_vibration_enabled = report_data[11] != 0;
            context.report_buffer[13] = 0x80;
            context.report_buffer[14] = command_id;
            context.report_buffer[15] = 0x00;
            LOG_PRINTF("[HID] FEATURE ENABLE_VIBRATION %u\n",
                       context.is_vibration_enabled);
            break;
        case READ_IMU:
            context.report_buffer[13] = 0xC0;
            context.report_buffer[14] = command_id;
            context.report_buffer[15] = report_data[11];
            context.report_buffer[16] = report_data[12];
            LOG_PRINTF("[HID] FEATURE READ_IMU addr=%u size=%u\n",
                       report_data[11], report_data[12]);
            break;
        case GET_VOLTAGE:
            context.report_buffer[13] = 0xD0;
            context.report_buffer[14] = 0x50;
            context.report_buffer[15] = 0x83;
            context.report_buffer[16] = 0x06;
            LOG_PRINTF("[HID] FEATURE GET_VOLTAGE\n");
            break;
        default:
            context.report_buffer[13] = 0x80;
            context.report_buffer[14] = command_id;
            context.report_buffer[15] = 0x03;
            LOG_PRINTF("[HID] FEATURE unknown cmd=0x%02x\n", command_id);
            break;
    }

    context.is_report_queued = true;
}

static void update_switch_report_from_state(SwitchProContext& context) {
    const SwitchInputState& state = context.input_state;
    SwitchInputReport& inputs = context.switch_report.inputs;
    inputs.dpadUp = state.dpad_up;
    inputs.dpadDown = state.dpad_down;
    inputs.dpadLeft = state.dpad_left;
    inputs.dpadRight = state.dpad_right;
    inputs.chargingGrip = 1;
    inputs.buttonY = state.button_y;
    inputs.buttonX = state.button_x;
    inputs.buttonB = state.button_b;
    inputs.buttonA = state.button_a;
    inputs.buttonRightSR = 0;
    inputs.buttonRightSL = 0;
    inputs.buttonR = state.button_r;
    inputs.buttonZR = state.button_zr;
    inputs.buttonMinus = state.button_minus;
    inputs.buttonPlus = state.button_plus;
    inputs.buttonThumbR = state.button_r3;
    inputs.buttonThumbL = state.button_l3;
    inputs.buttonHome = state.button_home;
    inputs.buttonCapture = state.button_capture;
    inputs.buttonLeftSR = 0;
    inputs.buttonLeftSL = 0;
    inputs.buttonL = state.button_l;
    inputs.buttonZL = state.button_zl;

    uint16_t left_x = scale16To12(state.lx);
    uint16_t left_y = scale16To12(state.ly);
    uint16_t right_x = scale16To12(state.rx);
    uint16_t right_y = scale16To12(state.ry);

    inputs.leftStick.setX(
        std::min(std::max(left_x, context.left_min_x), context.left_max_x));
    inputs.leftStick.setY(-std::min(
        std::max(left_y, context.left_min_y), context.left_max_y));
    inputs.rightStick.setX(std::min(
        std::max(right_x, context.right_min_x), context.right_max_x));
    inputs.rightStick.setY(-std::min(
        std::max(right_y, context.right_min_y), context.right_max_y));
    context.switch_report.rumbleReport = 0x09;
}

void switch_pro_init(uint8_t instance) {
    SwitchProContext* context = context_for(instance);
    if (context == nullptr) {
        return;
    }

    context->grip_color = switch_pro_get_slot_color(instance);
    context->device_info = {
        0x03,
        0x48,
        SWITCH_TYPE_PRO_CONTROLLER,
        0x02,
        {0x7c, 0xbb, 0x8a,
         static_cast<uint8_t>(get_rand_32() % 0xff),
         static_cast<uint8_t>(get_rand_32() % 0xff),
         static_cast<uint8_t>(get_rand_32() % 0xff)},
        0x01,
        0x02,
    };

    factory_config->leftStickCalibration.getRealMin(
        context->left_min_x, context->left_min_y);
    factory_config->leftStickCalibration.getRealMax(
        context->left_max_x, context->left_max_y);
    factory_config->rightStickCalibration.getRealMin(
        context->right_min_x, context->right_min_y);
    factory_config->rightStickCalibration.getRealMax(
        context->right_max_x, context->right_max_y);

    reset_context_runtime(*context,
                          to_ms_since_boot(get_absolute_time()), true);
}

void switch_pro_set_input(uint8_t instance, const SwitchInputState& state) {
    SwitchProContext* context = context_for(instance);
    if (context != nullptr) {
        context->input_state = state;
    }
}

bool switch_pro_task(uint8_t instance) {
    SwitchProContext* context = context_for(instance);
    if (context == nullptr) {
        return false;
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());
    bool report_sent = false;
    bool regular_report_sent = false;

    update_switch_report_from_state(*context);

    if (tud_suspended()) {
        tud_remote_wakeup();
    }

    if (context->is_report_queued) {
        if ((now - context->last_report_timer) >
            SWITCH_PRO_KEEPALIVE_TIMER) {
            if (tud_hid_n_ready(instance) &&
                send_report(instance, *context, 0, context->report_buffer,
                            SWITCH_PRO_ENDPOINT_SIZE)) {
                context->is_report_queued = false;
                context->last_report_timer = now;
            }
        }
        report_sent = true;
    }

    if (context->is_ready && !report_sent) {
        if ((now - context->last_report_timer) >=
            SWITCH_PRO_IMU_REPORT_TIMER) {
            // One timer tick per 5ms IMU frame; three frames per report.
            fill_imu_report_data(*context, context->input_state, now);
            context->switch_report.timestamp += 3;
            if (tud_hid_n_ready(instance) &&
                send_report(instance, *context, 0, &context->switch_report,
                            sizeof(context->switch_report))) {
                context->input_state.imu_sample_count = 0;
                regular_report_sent = true;
            }
            context->last_report_timer = now;
        }
    } else if (!context->is_initialized) {
        send_identify(*context);
        if (tud_hid_n_ready(instance)) {
            bool result = tud_hid_n_report(
                instance, 0, context->report_buffer,
                SWITCH_PRO_ENDPOINT_SIZE);
            if (result) {
                context->is_initialized = true;
            } else {
                LOG_PRINTF("[HID %u] send_report failed id=0 len=%u\n",
                           instance, SWITCH_PRO_ENDPOINT_SIZE);
            }
        }
        context->last_report_timer = now;
    }
    return regular_report_sent;
}

bool switch_pro_apply_uart_packet(const uint8_t* packet, uint8_t length,
                                  SwitchInputState& out_state) {
    if (packet == nullptr) {
        return false;
    }
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

    out_state = state;
    return true;
}

void switch_pro_set_rumble_callback(uint8_t instance,
                                    SwitchRumbleCallback callback) {
    SwitchProContext* context = context_for(instance);
    if (context != nullptr) {
        context->rumble_callback = callback;
    }
}

bool switch_pro_is_ready(uint8_t instance) {
    const SwitchProContext* context = context_for(instance);
    return context != nullptr && context->is_ready;
}

// HID callbacks
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type, uint8_t* buffer,
                               uint16_t requested_length) {
    (void)report_id;
    (void)report_type;
    SwitchProContext* context = context_for(instance);
    LOG_PRINTF("[HID %u] get_report id=%u type=%u len=%u\n", instance,
               report_id, report_type, requested_length);
    if (context == nullptr || buffer == nullptr) {
        return 0;
    }

    // Serve the addressed instance's current input report.
    uint16_t report_size = sizeof(context->switch_report);
    if (requested_length < report_size) {
        report_size = requested_length;
    }
    memcpy(buffer, &context->switch_report, report_size);
    return report_size;
}

static void process_output_report(uint8_t instance,
                                  SwitchProContext& context,
                                  uint8_t callback_report_id,
                                  const uint8_t* payload,
                                  uint16_t payload_size) {
    uint8_t normalized[SWITCH_PRO_ENDPOINT_SIZE]{};
    size_t normalized_size = normalize_switch_output_report(
        callback_report_id, payload, payload_size, normalized);
    if (normalized_size < 2) {
        return;
    }

    memset(context.report_buffer, 0x00, sizeof(context.report_buffer));
    uint8_t switch_report_id = normalized[0];
    uint8_t switch_report_sub_id = normalized[1];
    LOG_PRINTF(
        "[HID %u] output id=%u switchRID=0x%02x sub=0x%02x len=%u\n",
        instance, callback_report_id, switch_report_id, switch_report_sub_id,
        static_cast<unsigned>(normalized_size));

    if (switch_report_id == REPORT_OUTPUT_10 ||
        switch_report_id == REPORT_FEATURE) {
        forward_decoded_rumble(
            instance, context, normalized,
            static_cast<uint16_t>(normalized_size));
    }
    if (switch_report_id == REPORT_OUTPUT_00) {
        return;
    }

    if (switch_report_id == REPORT_FEATURE) {
        if (normalized_size >= 16) {
            handle_feature_report(context, normalized);
        }
    } else if (switch_report_id == REPORT_CONFIGURATION) {
        handle_config_report(context, switch_report_sub_id);
    }
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type,
                           const uint8_t* buffer, uint16_t buffer_size) {
    SwitchProContext* context = context_for(instance);
    if (context == nullptr || report_type != HID_REPORT_TYPE_OUTPUT) {
        return;
    }
    process_output_report(instance, *context, report_id, buffer, buffer_size);
}

void tud_hid_report_received_cb(uint8_t instance, uint8_t report_id,
                                const uint8_t* buffer,
                                uint16_t buffer_size) {
    SwitchProContext* context = context_for(instance);
    if (context == nullptr) {
        return;
    }
    process_output_report(instance, *context, report_id, buffer, buffer_size);
}

uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
    if (context_for(instance) == nullptr) {
        return nullptr;
    }
    return switch_pro_report_descriptor;
}

uint8_t const* tud_descriptor_device_cb(void) {
#ifdef SWITCH_PICO_ADAPTER_FEASIBILITY
    if (adapter_host_probe_mode() == AdapterUsbMode::kXInput) {
        return XInputFeasibility::kDeviceDescriptor;
    }
    return XInputFeasibility::kSwitchProbeDeviceDescriptor;
#else
    return switch_pro_device_descriptor;
#endif
}

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
#ifdef SWITCH_PICO_ADAPTER_FEASIBILITY
    if (adapter_host_probe_mode() == AdapterUsbMode::kXInput) {
        return XInputFeasibility::kConfigurationDescriptor;
    }
#endif
    return switch_pro_configuration_descriptor;
}

bool tud_control_request_cb(uint8_t rhport,
                            tusb_control_request_t const* request) {
    (void)rhport;
    (void)request;
    LOG_PRINTF(
        "[CTRL] bmReq=0x%02x bReq=0x%02x wValue=0x%04x wIndex=0x%04x "
        "wLen=%u\n",
        request->bmRequestType, request->bRequest, request->wValue,
        request->wIndex, request->wLength);
    return false;  // let TinyUSB handle it normally
}

void tud_mount_cb(void) {
    LOG_PRINTF("[USB] mount_cb\n");
    reset_all_contexts(false);
}

void tud_umount_cb(void) {
    LOG_PRINTF("[USB] umount_cb\n");
    reset_all_contexts(false);
}

static uint16_t desc_str[32];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;

#ifdef SWITCH_PICO_ADAPTER_FEASIBILITY
    adapter_host_probe_note_string_descriptor(index);
    if (index == 0xee) {
        static constexpr char kSignature[] = "MSFT100";
        for (uint8_t i = 0; i < sizeof(kSignature) - 1; ++i) {
            desc_str[1 + i] = kSignature[i];
        }
        desc_str[8] = XInputFeasibility::kMsVendorRequest;
        desc_str[0] = static_cast<uint16_t>((0x03 << 8) | 18);
        return desc_str;
    }
#endif

    uint8_t chr_count = 0;
    if (index == 0) {
        memcpy(&desc_str[1], switch_pro_string_language, 2);
        chr_count = 1;
    } else {
        const uint8_t* str = nullptr;
#ifdef SWITCH_PICO_ADAPTER_FEASIBILITY
        static const uint8_t kManufacturer[] = "Switch Pico";
        static const uint8_t kProduct[] = "XInput Feasibility";
        static const uint8_t kSerial[] = "XINPUT-PROTOTYPE";
        static const uint8_t* const kXInputStrings[] = {
            nullptr, kManufacturer, kProduct, kSerial};
        if (adapter_host_probe_mode() == AdapterUsbMode::kXInput) {
            if (index >= sizeof(kXInputStrings) /
                             sizeof(kXInputStrings[0])) {
                return nullptr;
            }
            str = kXInputStrings[index];
        } else
#endif
        {
            if (index >= sizeof(switch_pro_string_descriptors) /
                             sizeof(switch_pro_string_descriptors[0])) {
                return nullptr;
            }
            str = switch_pro_string_descriptors[index];
        }

        while (str[chr_count] != 0) {
            ++chr_count;
        }
        if (chr_count > 31) {
            chr_count = 31;
        }
        for (uint8_t i = 0; i < chr_count; ++i) {
            desc_str[1 + i] = str[i];
        }
    }

    desc_str[0] =
        static_cast<uint16_t>((0x03 << 8) | (2 * chr_count + 2));
    return desc_str;
}
