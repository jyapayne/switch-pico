#include "switch_pro_driver.h"

#include <algorithm>
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
static bool is_imu_enabled = false;
static bool is_vibration_enabled = false;

static uint16_t leftMinX, leftMinY;
static uint16_t leftCenX, leftCenY;
static uint16_t leftMaxX, leftMaxY;
static uint16_t rightMinX, rightMinY;
static uint16_t rightCenX, rightCenY;
static uint16_t rightMaxX, rightMaxY;

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
    0xE3, 0xFF, 0x39, 0xFF, 0xED, 0x01, 0x00, 0x40,
    0x00, 0x40, 0x00, 0x40, 0x09, 0x00, 0xEA, 0xFF,
    0xA1, 0xFF, 0x3B, 0x34, 0x3B, 0x34, 0x3B, 0x34,

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
    0x1B, 0x1B, 0x1D,

    // button color
    0xFF, 0xFF, 0xFF,

    // left grip color
    0xEC, 0x00, 0x8C,

    // right grip color
    0xEC, 0x00, 0x8C,

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

static SwitchInputState make_neutral_state() {
    SwitchInputState s{};
    s.lx = SWITCH_PRO_JOYSTICK_MID;
    s.ly = SWITCH_PRO_JOYSTICK_MID;
    s.rx = SWITCH_PRO_JOYSTICK_MID;
    s.ry = SWITCH_PRO_JOYSTICK_MID;
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
        case TOGGLE_IMU:
            is_imu_enabled = reportData[11];
            report_buffer[13] = 0x80;
            report_buffer[14] = commandID;
            report_buffer[15] = 0x00;
            canSend = true;
            LOG_PRINTF("[HID] FEATURE TOGGLE_IMU %u\n", is_imu_enabled);
            break;
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
        .majorVersion = 0x04,
        .minorVersion = 0x91,
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
            .connectionInfo = 0x08, // wired connection
            .batteryLevel = 0x0F,   // full battery

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

void switch_pro_task() {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    report_sent = false;

    update_switch_report_from_state();

    if (tud_suspended()) {
        tud_remote_wakeup();
    }

    if (is_report_queued) {
        if ((now - last_report_timer) > SWITCH_PRO_KEEPALIVE_TIMER) {
            if (tud_hid_ready() && send_report(queued_report_id, report_buffer, 64) == true ) {
            }
            is_report_queued = false;
            last_report_timer = now;
        }
        report_sent = true;
    }

    if (is_ready && !report_sent) {
        if ((now - last_report_timer) > SWITCH_PRO_KEEPALIVE_TIMER) {
            switch_report.timestamp = last_report_counter;
            void * inputReport = &switch_report;
            uint16_t report_size = sizeof(switch_report);
            if (memcmp(last_report, inputReport, report_size) != 0) {
                if (tud_hid_ready() && send_report(0, inputReport, report_size) == true ) {
                    memcpy(last_report, inputReport, report_size);
                    report_sent = true;
                }

                last_report_timer = now;
            }
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
}

bool switch_pro_apply_uart_packet(const uint8_t* packet, uint8_t length, SwitchInputState* out_state) {
    // Packet format: 0xAA, buttons(2 LE), hat, lx, ly, rx, ry
    if (length < 8 || packet[0] != 0xAA) {
        return false;
    }

    SwitchProOutReport out{};
    out.buttons = static_cast<uint16_t>(packet[1]) | (static_cast<uint16_t>(packet[2]) << 8);
    out.hat = packet[3];
    out.lx = packet[4];
    out.ly = packet[5];
    out.rx = packet[6];
    out.ry = packet[7];

    auto expand_axis = [](uint8_t v) -> uint16_t {
        return static_cast<uint16_t>(v) << 8 | v;
    };

    SwitchInputState state = make_neutral_state();

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

    if (out_state) {
        *out_state = state;
    } else {
        switch_pro_set_input(state);
    }
    return true;
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
