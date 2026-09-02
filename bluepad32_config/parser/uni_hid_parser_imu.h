#pragma once

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define UNI_IMU_ACCEL_RES_PER_G 8192
#define UNI_IMU_GYRO_RES_PER_DEG_S 1024
#define UNI_PSMOVE_CALIBRATION_REPORT_SIZE 49
#define UNI_PSMOVE_ZCM1_CALIBRATION_SIZE 143
#define UNI_PSMOVE_ZCM2_CALIBRATION_SIZE 96

typedef enum {
    UNI_PSMOVE_IMU_MODEL_UNKNOWN = 0,
    UNI_PSMOVE_IMU_MODEL_ZCM1,
    UNI_PSMOVE_IMU_MODEL_ZCM2,
} uni_psmove_imu_model_t;

typedef enum {
    UNI_PSMOVE_CALIBRATION_INVALID = 0,
    UNI_PSMOVE_CALIBRATION_INCOMPLETE,
    UNI_PSMOVE_CALIBRATION_COMPLETE,
} uni_psmove_calibration_result_t;

typedef struct {
    int32_t accel[3];
    int32_t gyro[3];
} uni_imu_fixed_sample_t;

typedef struct {
    uint8_t data[UNI_PSMOVE_ZCM1_CALIBRATION_SIZE];
    uni_psmove_imu_model_t model;
    uint8_t received_blocks;
    bool complete;
} uni_psmove_imu_calibration_t;

static inline int32_t uni_imu_clamp_i64(int64_t value) {
    if (value > INT32_MAX) {
        return INT32_MAX;
    }
    if (value < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)value;
}

static inline int32_t uni_imu_scale(int32_t value, int32_t span,
                                    int32_t full_scale) {
    if (span <= 0) {
        return 0;
    }
    return uni_imu_clamp_i64((int64_t)value * full_scale / span);
}

static inline int32_t uni_psmove_scale_gyro(int32_t raw, int32_t bias,
                                            int32_t span,
                                            int32_t full_scale) {
    if (span <= 0) {
        return 0;
    }
    return uni_imu_clamp_i64(
        ((int64_t)raw - bias) * full_scale / span);
}

static inline int32_t uni_psmove_decode_value(
    uni_psmove_imu_model_t model, uint16_t value) {
    if (model == UNI_PSMOVE_IMU_MODEL_ZCM1) {
        return (int32_t)value - 0x8000;
    }
    return (int16_t)value;
}

static inline int32_t uni_psmove_read_calibration_value(
    const uint8_t* data, uni_psmove_imu_model_t model, uint8_t offset) {
    const uint16_t value =
        (uint16_t)(data[offset] | ((uint16_t)data[offset + 1] << 8));
    return uni_psmove_decode_value(model, value);
}

static inline uni_psmove_calibration_result_t
uni_psmove_add_calibration_report(uni_psmove_imu_calibration_t* calibration,
                                  uni_psmove_imu_model_t model,
                                  const uint8_t* report, uint16_t length) {
    if (calibration == NULL || report == NULL ||
        length != UNI_PSMOVE_CALIBRATION_REPORT_SIZE || report[0] != 0x10 ||
        (model != UNI_PSMOVE_IMU_MODEL_ZCM1 &&
         model != UNI_PSMOVE_IMU_MODEL_ZCM2)) {
        return UNI_PSMOVE_CALIBRATION_INVALID;
    }
    if (calibration->model != UNI_PSMOVE_IMU_MODEL_UNKNOWN &&
        calibration->model != model) {
        return UNI_PSMOVE_CALIBRATION_INVALID;
    }
    calibration->model = model;

    size_t offset;
    size_t source_offset;
    uint8_t block_mask;
    switch (report[1]) {
        case 0x00:
            offset = 0;
            source_offset = 0;
            block_mask = 0x01;
            break;
        case 0x01:
            if (model != UNI_PSMOVE_IMU_MODEL_ZCM1) {
                return UNI_PSMOVE_CALIBRATION_INVALID;
            }
            offset = UNI_PSMOVE_CALIBRATION_REPORT_SIZE;
            source_offset = 2;
            block_mask = 0x02;
            break;
        case 0x81:
            if (model != UNI_PSMOVE_IMU_MODEL_ZCM2) {
                return UNI_PSMOVE_CALIBRATION_INVALID;
            }
            offset = UNI_PSMOVE_CALIBRATION_REPORT_SIZE;
            source_offset = 2;
            block_mask = 0x02;
            break;
        case 0x82:
            if (model != UNI_PSMOVE_IMU_MODEL_ZCM1) {
                return UNI_PSMOVE_CALIBRATION_INVALID;
            }
            offset = 2 * UNI_PSMOVE_CALIBRATION_REPORT_SIZE - 2;
            source_offset = 2;
            block_mask = 0x04;
            break;
        default:
            return UNI_PSMOVE_CALIBRATION_INVALID;
    }

    const size_t copy_size = length - source_offset;
    const size_t calibration_size =
        model == UNI_PSMOVE_IMU_MODEL_ZCM1
            ? UNI_PSMOVE_ZCM1_CALIBRATION_SIZE
            : UNI_PSMOVE_ZCM2_CALIBRATION_SIZE;
    if (offset + copy_size > calibration_size) {
        return UNI_PSMOVE_CALIBRATION_INVALID;
    }
    memcpy(&calibration->data[offset], &report[source_offset], copy_size);
    calibration->received_blocks |= block_mask;

    const uint8_t required_blocks =
        model == UNI_PSMOVE_IMU_MODEL_ZCM1 ? 0x07 : 0x03;
    calibration->complete =
        (calibration->received_blocks & required_blocks) == required_blocks;
    return calibration->complete ? UNI_PSMOVE_CALIBRATION_COMPLETE
                                 : UNI_PSMOVE_CALIBRATION_INCOMPLETE;
}

static inline bool uni_psmove_normalize_imu(
    uni_psmove_imu_model_t model,
    const uni_psmove_imu_calibration_t* calibration,
    const uint16_t accel_first[3], const uint16_t accel_second[3],
    const uint16_t gyro_first[3], const uint16_t gyro_second[3],
    uni_imu_fixed_sample_t* output) {
    if (output == NULL) {
        return false;
    }
    memset(output, 0, sizeof(*output));
    if (calibration == NULL || !calibration->complete ||
        calibration->model != model || accel_first == NULL ||
        accel_second == NULL || gyro_first == NULL || gyro_second == NULL) {
        return false;
    }

    static const uint8_t zcm1_accel_low[] = {0x0a, 0x24, 0x14};
    static const uint8_t zcm1_accel_high[] = {0x16, 0x1e, 0x08};
    static const uint8_t zcm2_accel_low[] = {0x08, 0x16, 0x24};
    static const uint8_t zcm2_accel_high[] = {0x02, 0x10, 0x1e};
    static const uint8_t zcm1_gyro_bias[] = {0x2a, 0x2c, 0x2e};
    static const uint8_t zcm1_gyro_high[] = {0x46, 0x50, 0x5a};
    static const uint8_t zcm2_gyro_bias[] = {0x26, 0x28, 0x2a};
    static const uint8_t zcm2_gyro_low[] = {0x42, 0x4a, 0x52};
    static const uint8_t zcm2_gyro_high[] = {0x30, 0x38, 0x40};

    const uint8_t* accel_low =
        model == UNI_PSMOVE_IMU_MODEL_ZCM1 ? zcm1_accel_low : zcm2_accel_low;
    const uint8_t* accel_high =
        model == UNI_PSMOVE_IMU_MODEL_ZCM1 ? zcm1_accel_high : zcm2_accel_high;
    const uint8_t* gyro_bias =
        model == UNI_PSMOVE_IMU_MODEL_ZCM1 ? zcm1_gyro_bias : zcm2_gyro_bias;
    const uint8_t* gyro_high =
        model == UNI_PSMOVE_IMU_MODEL_ZCM1 ? zcm1_gyro_high : zcm2_gyro_high;
    const int32_t gyro_full_scale =
        (model == UNI_PSMOVE_IMU_MODEL_ZCM1 ? 480 : 540) *
        UNI_IMU_GYRO_RES_PER_DEG_S;

    for (uint8_t axis = 0; axis < 3; ++axis) {
        const int32_t accel_low_value = uni_psmove_read_calibration_value(
            calibration->data, model, accel_low[axis]);
        const int32_t accel_high_value = uni_psmove_read_calibration_value(
            calibration->data, model, accel_high[axis]);
        const int32_t accel_center =
            (accel_low_value + accel_high_value) / 2;
        const int32_t accel_raw =
            (uni_psmove_decode_value(model, accel_first[axis]) +
             uni_psmove_decode_value(model, accel_second[axis])) /
            2;
        const int32_t accel_delta = accel_raw - accel_center;
        const int32_t accel_span =
            accel_delta < 0 ? accel_center - accel_low_value
                            : accel_high_value - accel_center;
        output->accel[axis] =
            uni_imu_scale(accel_delta, accel_span, UNI_IMU_ACCEL_RES_PER_G);

        const int32_t gyro_bias_value = uni_psmove_read_calibration_value(
            calibration->data, model, gyro_bias[axis]);
        const int32_t gyro_raw =
            (uni_psmove_decode_value(model, gyro_first[axis]) +
             uni_psmove_decode_value(model, gyro_second[axis])) /
            2;
        int32_t gyro_span;
        if (model == UNI_PSMOVE_IMU_MODEL_ZCM1 ||
            gyro_raw >= gyro_bias_value) {
            gyro_span = uni_psmove_read_calibration_value(
                            calibration->data, model, gyro_high[axis]) -
                        gyro_bias_value;
        } else {
            gyro_span = gyro_bias_value - uni_psmove_read_calibration_value(
                                               calibration->data, model,
                                               zcm2_gyro_low[axis]);
        }
        output->gyro[axis] = uni_psmove_scale_gyro(
            gyro_raw, gyro_bias_value, gyro_span, gyro_full_scale);
    }
    return true;
}

static inline void uni_imu_normalize_wii_accel(int32_t x, int32_t y,
                                                int32_t z,
                                                int32_t output[3]) {
    if (output == NULL) {
        return;
    }
    output[0] = uni_imu_scale(-x, 100, UNI_IMU_ACCEL_RES_PER_G);
    output[1] = uni_imu_scale(z, 100, UNI_IMU_ACCEL_RES_PER_G);
    output[2] = uni_imu_scale(y, 100, UNI_IMU_ACCEL_RES_PER_G);
}
