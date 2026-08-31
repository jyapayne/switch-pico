#include "parser/uni_hid_parser_imu.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>

namespace {

int failures = 0;

void expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        ++failures;
    }
}

void write_calibration_value(
    std::array<uint8_t, UNI_PSMOVE_ZCM1_CALIBRATION_SIZE>& blob,
    uni_psmove_imu_model_t model, uint8_t offset, int32_t value) {
    const uint16_t encoded =
        model == UNI_PSMOVE_IMU_MODEL_ZCM1
            ? static_cast<uint16_t>(value + 0x8000)
            : static_cast<uint16_t>(static_cast<int16_t>(value));
    blob[offset] = static_cast<uint8_t>(encoded);
    blob[offset + 1] = static_cast<uint8_t>(encoded >> 8u);
}

uint16_t encode_input(uni_psmove_imu_model_t model, int32_t value) {
    return model == UNI_PSMOVE_IMU_MODEL_ZCM1
               ? static_cast<uint16_t>(value + 0x8000)
               : static_cast<uint16_t>(static_cast<int16_t>(value));
}

std::array<uint8_t, UNI_PSMOVE_CALIBRATION_REPORT_SIZE> first_report(
    const std::array<uint8_t, UNI_PSMOVE_ZCM1_CALIBRATION_SIZE>& blob) {
    std::array<uint8_t, UNI_PSMOVE_CALIBRATION_REPORT_SIZE> report{};
    std::memcpy(report.data(), blob.data(), report.size());
    report[0] = 0x10;
    report[1] = 0x00;
    return report;
}

std::array<uint8_t, UNI_PSMOVE_CALIBRATION_REPORT_SIZE> continuation_report(
    const std::array<uint8_t, UNI_PSMOVE_ZCM1_CALIBRATION_SIZE>& blob,
    uint8_t block, size_t blob_offset) {
    std::array<uint8_t, UNI_PSMOVE_CALIBRATION_REPORT_SIZE> report{};
    report[0] = 0x10;
    report[1] = block;
    std::memcpy(report.data() + 2, blob.data() + blob_offset,
                report.size() - 2);
    return report;
}

void set_accel_calibration(
    std::array<uint8_t, UNI_PSMOVE_ZCM1_CALIBRATION_SIZE>& blob,
    uni_psmove_imu_model_t model, int32_t low, int32_t high) {
    const uint8_t* low_offsets;
    const uint8_t* high_offsets;
    static const uint8_t zcm1_low[] = {0x0a, 0x24, 0x14};
    static const uint8_t zcm1_high[] = {0x16, 0x1e, 0x08};
    static const uint8_t zcm2_low[] = {0x08, 0x16, 0x24};
    static const uint8_t zcm2_high[] = {0x02, 0x10, 0x1e};
    if (model == UNI_PSMOVE_IMU_MODEL_ZCM1) {
        low_offsets = zcm1_low;
        high_offsets = zcm1_high;
    } else {
        low_offsets = zcm2_low;
        high_offsets = zcm2_high;
    }
    for (uint8_t axis = 0; axis < 3; ++axis) {
        write_calibration_value(blob, model, low_offsets[axis], low);
        write_calibration_value(blob, model, high_offsets[axis], high);
    }
}

void test_wii_accelerometer() {
    int32_t output[3]{};
    uni_imu_normalize_wii_accel(100, -50, 25, output);
    expect(output[0] == -8192 && output[1] == 2048 &&
               output[2] == -4096,
           "Wii accelerometer scale or SDL axis mapping is wrong");
}

void test_zcm1_calibration_and_normalization() {
    constexpr auto model = UNI_PSMOVE_IMU_MODEL_ZCM1;
    std::array<uint8_t, UNI_PSMOVE_ZCM1_CALIBRATION_SIZE> blob{};
    set_accel_calibration(blob, model, -1000, 1000);
    expect(uni_psmove_scale_gyro(32767, -32768, 1,
                                 1080 * UNI_IMU_GYRO_RES_PER_DEG_S) ==
               INT32_MAX,
           "corrupt PS Move calibration overflow was not clamped");
    const uint8_t bias_offsets[] = {0x2a, 0x2c, 0x2e};
    const uint8_t high_offsets[] = {0x46, 0x50, 0x5a};
    for (uint8_t axis = 0; axis < 3; ++axis) {
        write_calibration_value(blob, model, bias_offsets[axis], 0);
        write_calibration_value(blob, model, high_offsets[axis], 1000);
    }

    auto first = first_report(blob);
    auto second = continuation_report(blob, 0x01, 49);
    auto third = continuation_report(blob, 0x82, 96);
    uni_psmove_imu_calibration_t calibration{};
    expect(uni_psmove_add_calibration_report(
               &calibration, model, second.data(), second.size()) ==
               UNI_PSMOVE_CALIBRATION_INCOMPLETE,
           "ZCM1 second calibration block was not accepted out of order");
    expect(uni_psmove_add_calibration_report(
               &calibration, model, first.data(), first.size()) ==
               UNI_PSMOVE_CALIBRATION_INCOMPLETE,
           "ZCM1 first calibration block completed too early");
    expect(uni_psmove_add_calibration_report(
               &calibration, model, third.data(), third.size()) ==
               UNI_PSMOVE_CALIBRATION_COMPLETE,
           "ZCM1 calibration did not complete");

    const uint16_t accel_first[] = {
        encode_input(model, 1000), encode_input(model, 0),
        encode_input(model, -1000)};
    const uint16_t accel_second[] = {
        encode_input(model, 0), encode_input(model, 0),
        encode_input(model, -1000)};
    const uint16_t gyro_first[] = {
        encode_input(model, 500), encode_input(model, 0),
        encode_input(model, -500)};
    const uint16_t gyro_second[] = {
        encode_input(model, 500), encode_input(model, 0),
        encode_input(model, -500)};
    uni_imu_fixed_sample_t output{};
    expect(uni_psmove_normalize_imu(
               model, &calibration, accel_first, accel_second, gyro_first,
               gyro_second, &output),
           "ZCM1 calibrated sample was rejected");
    expect(output.accel[0] == 4096 && output.accel[1] == 0 &&
               output.accel[2] == -8192,
           "ZCM1 accelerometer normalization is wrong");
    expect(output.gyro[0] == 245760 && output.gyro[1] == 0 &&
               output.gyro[2] == -245760,
           "ZCM1 gyroscope normalization is wrong");
}

void test_zcm2_calibration_and_normalization() {
    constexpr auto model = UNI_PSMOVE_IMU_MODEL_ZCM2;
    std::array<uint8_t, UNI_PSMOVE_ZCM1_CALIBRATION_SIZE> blob{};
    set_accel_calibration(blob, model, -1000, 1000);
    const uint8_t bias_offsets[] = {0x26, 0x28, 0x2a};
    const uint8_t low_offsets[] = {0x42, 0x4a, 0x52};
    const uint8_t high_offsets[] = {0x30, 0x38, 0x40};
    for (uint8_t axis = 0; axis < 3; ++axis) {
        write_calibration_value(blob, model, bias_offsets[axis], 100);
        write_calibration_value(blob, model, low_offsets[axis], -900);
        write_calibration_value(blob, model, high_offsets[axis], 1100);
    }

    auto first = first_report(blob);
    auto second = continuation_report(blob, 0x81, 49);
    uni_psmove_imu_calibration_t calibration{};
    expect(uni_psmove_add_calibration_report(
               &calibration, model, first.data(), first.size()) ==
               UNI_PSMOVE_CALIBRATION_INCOMPLETE,
           "ZCM2 first calibration block completed too early");
    expect(uni_psmove_add_calibration_report(
               &calibration, model, second.data(), second.size()) ==
               UNI_PSMOVE_CALIBRATION_COMPLETE,
           "ZCM2 calibration did not complete");

    const uint16_t accel[] = {
        encode_input(model, -1000), encode_input(model, 0),
        encode_input(model, 1000)};
    const uint16_t gyro[] = {
        encode_input(model, -900), encode_input(model, 100),
        encode_input(model, 1100)};
    uni_imu_fixed_sample_t output{};
    expect(uni_psmove_normalize_imu(model, &calibration, accel, accel,
                                    gyro, gyro, &output),
           "ZCM2 calibrated sample was rejected");
    expect(output.accel[0] == -8192 && output.accel[1] == 0 &&
               output.accel[2] == 8192,
           "ZCM2 signed accelerometer normalization is wrong");
    expect(output.gyro[0] == -552960 && output.gyro[1] == 0 &&
               output.gyro[2] == 552960,
           "ZCM2 signed gyroscope normalization is wrong");
}

void test_uncalibrated_psmove_is_suppressed() {
    uni_psmove_imu_calibration_t calibration{};
    const uint16_t values[] = {0xffff, 0xffff, 0xffff};
    uni_imu_fixed_sample_t output{{1, 2, 3}, {4, 5, 6}};
    expect(!uni_psmove_normalize_imu(
               UNI_PSMOVE_IMU_MODEL_ZCM1, &calibration, values, values,
               values, values, &output),
           "uncalibrated PS Move sample was accepted");
    expect(output.accel[0] == 0 && output.accel[1] == 0 &&
               output.accel[2] == 0 && output.gyro[0] == 0 &&
               output.gyro[1] == 0 && output.gyro[2] == 0,
           "uncalibrated PS Move motion was not neutralized");
}

}  // namespace

int main() {
    test_wii_accelerometer();
    test_zcm1_calibration_and_normalization();
    test_zcm2_calibration_and_normalization();
    test_uncalibrated_psmove_is_suppressed();
    if (failures != 0) {
        std::cerr << failures << " IMU normalization test(s) failed\n";
        return 1;
    }
    return 0;
}
