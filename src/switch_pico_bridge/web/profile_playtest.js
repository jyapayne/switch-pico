"use strict";

(function publishProfilePlaytestMath(root) {
  function applyCurveQ16(input, curve) {
    if (input === 0 || input === 65536 || curve === 256) return input;
    const denominator = curve * (65536 - input) + 256 * input;
    return Math.min(
      65536,
      Math.floor(
        (input * 256 * 65536 + Math.floor(denominator / 2)) /
        denominator
      )
    );
  }

  function transformStickAxis(adjusted, magnitude, response, invert) {
    if (adjusted === 0 || magnitude === 0 || response === 0) return 0;
    const negative = (adjusted < 0) !== invert;
    const limit = negative ? 32768 : 32767;
    const denominator = magnitude * 65536;
    const output = Math.min(
      limit,
      Math.floor(
        (
          Math.abs(adjusted) * limit * response +
          Math.floor(denominator / 2)
        ) / denominator
      )
    );
    return negative ? -output : output;
  }

  function transformStick(input, config) {
    if (
      config.center_x === 0 && config.center_y === 0 &&
      config.inner_deadzone === 0 && config.outer_saturation === 32767 &&
      config.curve_q8_8 === 256 && !config.invert_x && !config.invert_y
    ) {
      return { x: input.x, y: input.y };
    }
    const x = Math.max(-32768, Math.min(32767, input.x - config.center_x));
    const y = Math.max(-32768, Math.min(32767, input.y - config.center_y));
    if (config.outer_saturation <= config.inner_deadzone) {
      return { x: 0, y: 0 };
    }
    const magnitude = Math.max(Math.abs(x), Math.abs(y));
    let response = 0;
    if (magnitude >= config.outer_saturation) {
      response = 65536;
    } else if (magnitude > config.inner_deadzone) {
      const range = config.outer_saturation - config.inner_deadzone;
      const normalized = Math.floor(
        (
          (magnitude - config.inner_deadzone) * 65536 +
          Math.floor(range / 2)
        ) / range
      );
      response = applyCurveQ16(normalized, config.curve_q8_8);
    }
    return {
      x: transformStickAxis(x, magnitude, response, config.invert_x),
      y: transformStickAxis(y, magnitude, response, config.invert_y),
    };
  }

  function transformTrigger(input, config) {
    if (
      config.lower_deadzone === 0 &&
      config.upper_saturation === 65535 &&
      config.curve_q8_8 === 256
    ) return input;
    if (config.upper_saturation <= config.lower_deadzone) return 0;
    if (input <= config.lower_deadzone) return 0;
    if (input >= config.upper_saturation) return 65535;
    const range = config.upper_saturation - config.lower_deadzone;
    const offset = input - config.lower_deadzone;
    if (config.curve_q8_8 === 256) {
      return Math.floor(
        (offset * 65535 + Math.floor(range / 2)) / range
      );
    }
    const normalized = Math.floor(
      (offset * 65536 + Math.floor(range / 2)) / range
    );
    return Math.floor(
      (applyCurveQ16(normalized, config.curve_q8_8) * 65535 + 32768) /
      65536
    );
  }

  function transformMappings(sample, profile, shiftActive = false) {
    const buttons = new Set();
    const triggers = { left: 0, right: 0 };
    const consumed = profile.shift.mode !== "off" ? profile.shift.modifier : null;
    const map = shiftActive ? profile.shift : profile;
    const route = (output, value = 65535) => {
      if (output === "left_trigger" || output === "right_trigger") {
        const side = output === "left_trigger" ? "left" : "right";
        triggers[side] = Math.max(triggers[side], value);
      } else if (output && Object.hasOwn(profile.button_map, output)) {
        buttons.add(output);
      }
    };
    for (const source of sample.buttons) {
      if (source !== consumed) route(map.button_map[source]);
    }
    for (const source of sample.extra_buttons || []) {
      if (source !== consumed) route(map.extra_button_map[source]);
    }
    for (const side of ["left", "right"]) {
      if (`${side}_trigger` === consumed) continue;
      const config = profile.triggers[side];
      const value = transformTrigger(sample.triggers[side], config);
      if (config.output === "left_trigger" || config.output === "right_trigger") {
        route(config.output, value);
      } else if (value >= config.digital_threshold) {
        route(config.output);
      }
    }
    return {
      buttons: Object.keys(profile.button_map).filter((button) => buttons.has(button)),
      triggers,
    };
  }

  function stickPosition(value) {
    return Math.max(3, Math.min(97, 50 + value / 32768 * 47));
  }

  function stickCoordinates(input) {
    return {
      left: stickPosition(input.x),
      top: stickPosition(input.y),
    };
  }

  function triggerPercent(value) {
    return Math.max(0, Math.min(100, value / 65535 * 100));
  }

  root.ProfilePlaytestMath = Object.freeze({
    transformStick,
    transformTrigger,
    transformMappings,
    stickCoordinates,
    triggerPercent,
  });
})(globalThis);
