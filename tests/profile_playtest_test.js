"use strict";

const assert = require("node:assert/strict");
const path = require("node:path");

require(path.join(
  __dirname,
  "..",
  "src",
  "switch_pico_bridge",
  "web",
  "profile_playtest.js"
));

const {
  transformStick,
  transformTrigger,
  stickCoordinates,
  triggerPercent,
} = globalThis.ProfilePlaytestMath;

const defaultStick = {
  center_x: 0,
  center_y: 0,
  inner_deadzone: 0,
  outer_saturation: 32767,
  curve_q8_8: 256,
  invert_x: false,
  invert_y: false,
};
const defaultTrigger = {
  lower_deadzone: 0,
  upper_saturation: 65535,
  curve_q8_8: 256,
};

assert.deepEqual(
  transformStick({ x: -1234, y: 2345 }, defaultStick),
  { x: -1234, y: 2345 }
);
assert.ok(stickCoordinates({ x: 0, y: 20000 }).top > 50,
          "positive/down input must render below center");
assert.ok(stickCoordinates({ x: 0, y: -20000 }).top < 50,
          "negative/up input must render above center");
assert.ok(stickCoordinates({ x: -20000, y: 0 }).left < 50,
          "negative/left input must render left of center");
assert.deepEqual(
  transformStick(
    { x: -1234, y: 2345 },
    { ...defaultStick, inner_deadzone: 5000 }
  ),
  { x: 0, y: 0 }
);
assert.deepEqual(
  transformStick(
    { x: -30000, y: 30000 },
    { ...defaultStick, curve_q8_8: 512 }
  ),
  { x: -27665, y: 27664 }
);
assert.equal(transformTrigger(65000, defaultTrigger), 65000);
assert.ok(triggerPercent(65000) > 99,
          "nonzero trigger input must produce a visible bar");
assert.equal(triggerPercent(0), 0);
assert.equal(triggerPercent(65535), 100);

const { transformMappings } = globalThis.ProfilePlaytestMath;
const buttons = [
  "south", "east", "west", "north", "left_shoulder", "right_shoulder",
  "select", "start", "system", "capture", "left_stick", "right_stick",
  "dpad_up", "dpad_down", "dpad_left", "dpad_right",
];
const extras = ["c", "gl", "gr", "left_sl", "left_sr", "right_sl", "right_sr"];
const profile = {
  button_map: Object.fromEntries(buttons.map((button) => [button, button])),
  extra_button_map: Object.fromEntries(extras.map((button) => [button, null])),
  shift: {
    mode: "off", modifier: null,
    button_map: Object.fromEntries(buttons.map((button) => [button, button])),
    extra_button_map: Object.fromEntries(extras.map((button) => [button, null])),
  },
  triggers: {
    left: { ...defaultTrigger, output: "left_trigger", digital_threshold: 22934 },
    right: { ...defaultTrigger, output: "right_trigger", digital_threshold: 22934 },
  },
};
const sample = { buttons: ["south"], extra_buttons: extras, triggers: { left: 123, right: 50000 } };
assert.deepEqual(transformMappings(sample, profile), {
  buttons: ["south"], triggers: { left: 123, right: 50000 },
});
for (const [index, extra] of extras.entries()) {
  profile.extra_button_map[extra] = buttons[index + 1];
}
assert.deepEqual(transformMappings(sample, profile).buttons, buttons.slice(0, 8));
profile.extra_button_map.c = "left_trigger";
profile.extra_button_map.gl = "south";
assert.deepEqual(transformMappings(sample, profile), {
  buttons: ["south", ...buttons.slice(3, 8)],
  triggers: { left: 65535, right: 50000 },
});
profile.shift.mode = "hold";
profile.shift.modifier = "c";
profile.shift.extra_button_map.gr = "system";
assert.deepEqual(transformMappings(sample, profile, true), {
  buttons: ["south", "system"], triggers: { left: 123, right: 50000 },
});
assert.equal(transformMappings(sample, profile, false).triggers.left, 123,
             "the Shift modifier is consumed even on the base layer");
assert.deepEqual(transformMappings({ ...sample, extra_buttons: [] }, profile), {
  buttons: ["south"], triggers: { left: 123, right: 50000 },
});
profile.shift.mode = "off";
profile.triggers.right.output = "north";
assert.deepEqual(transformMappings({ ...sample, buttons: [], extra_buttons: [], triggers: { left: 0, right: 22933 } }, profile).buttons, []);
assert.deepEqual(transformMappings({ ...sample, buttons: [], extra_buttons: [], triggers: { left: 0, right: 22934 } }, profile).buttons, ["north"]);
