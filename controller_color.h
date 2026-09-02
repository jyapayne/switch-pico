#pragma once

#include <stdint.h>

struct SwitchRgbColor {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

constexpr SwitchRgbColor switch_pro_calibrate_light_color(
    SwitchRgbColor grip) {
    const uint8_t minimum =
        grip.red < grip.green
            ? (grip.red < grip.blue ? grip.red : grip.blue)
            : (grip.green < grip.blue ? grip.green : grip.blue);
    const uint8_t maximum =
        grip.red > grip.green
            ? (grip.red > grip.blue ? grip.red : grip.blue)
            : (grip.green > grip.blue ? grip.green : grip.blue);
    const uint16_t chroma = static_cast<uint16_t>(maximum - minimum);
    const uint16_t peak =
        static_cast<uint16_t>((static_cast<uint16_t>(maximum) * 2u + 1u) /
                              3u);
    if (chroma == 0) {
        const uint8_t gray = static_cast<uint8_t>(peak);
        return {gray, gray, gray};
    }

    const auto calibrate = [minimum, chroma, peak](uint8_t component) {
        const uint32_t delta =
            static_cast<uint32_t>(component - minimum);
        return static_cast<uint8_t>(
            (static_cast<uint32_t>(peak) * delta * delta) /
            (static_cast<uint32_t>(chroma) * chroma));
    };
    return {calibrate(grip.red), calibrate(grip.green),
            calibrate(grip.blue)};
}

SwitchRgbColor switch_pro_get_slot_color(uint8_t instance);
SwitchRgbColor switch_pro_get_slot_light_color(uint8_t instance);
