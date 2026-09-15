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
const ordinaryMapped = transformMappings(sample, profile);
assert.deepEqual(ordinaryMapped.buttons, ["south"]);
assert.deepEqual(ordinaryMapped.triggers, { left: 123, right: 50000 });
for (const [index, extra] of extras.entries()) {
  profile.extra_button_map[extra] = buttons[index + 1];
}
assert.deepEqual(transformMappings(sample, profile).buttons, buttons.slice(0, 8));
profile.extra_button_map.c = "left_trigger";
profile.extra_button_map.gl = "south";
const extraMapped = transformMappings(sample, profile);
assert.deepEqual(extraMapped.buttons, ["south", ...buttons.slice(3, 8)]);
assert.deepEqual(extraMapped.triggers, { left: 65535, right: 50000 });
profile.shift.mode = "hold";
profile.shift.modifier = "c";
profile.shift.extra_button_map.gr = "system";
const shiftedMapped = transformMappings(sample, profile, true);
assert.deepEqual(shiftedMapped.buttons, ["south", "system"]);
assert.deepEqual(shiftedMapped.triggers, { left: 123, right: 50000 });
assert.equal(transformMappings(sample, profile, false).triggers.left, 123,
             "the Shift modifier is consumed even on the base layer");
const releasedExtraMapped = transformMappings({ ...sample, extra_buttons: [] }, profile);
assert.deepEqual(releasedExtraMapped.buttons, ["south"]);
assert.deepEqual(releasedExtraMapped.triggers, { left: 123, right: 50000 });
profile.shift.mode = "off";
profile.triggers.right.output = "north";
assert.deepEqual(transformMappings({ ...sample, buttons: [], extra_buttons: [], triggers: { left: 0, right: 22933 } }, profile).buttons, []);
assert.deepEqual(transformMappings({ ...sample, buttons: [], extra_buttons: [], triggers: { left: 0, right: 22934 } }, profile).buttons, ["north"]);

// Test output mapping to extra rails/buttons and swap_sticks
profile.triggers.right.output = "right_trigger";
profile.button_map.south = "left_sl";
profile.button_map.east = "right_sr";
assert.deepEqual(transformMappings({ ...sample, buttons: ["south", "east"], extra_buttons: [] }, profile).buttons, ["left_sl", "right_sr"]);

const swapProfile = {
  ...profile,
  swap_sticks: true,
  button_map: { ...profile.button_map, left_stick: "left_stick", right_stick: "right_stick" },
};
assert.deepEqual(
  transformMappings({ ...sample, buttons: ["left_stick"], extra_buttons: [] }, swapProfile).buttons,
  ["right_stick"],
  "swap_sticks must swap left_stick click output to right_stick"
);
assert.deepEqual(
  transformMappings({ ...sample, buttons: ["right_stick"], extra_buttons: [] }, swapProfile).buttons,
  ["left_stick"],
  "swap_sticks must swap right_stick click output to left_stick"
);
assert.deepEqual(
  transformMappings({ ...sample, buttons: ["left_stick", "right_stick"], extra_buttons: [] }, swapProfile).buttons,
  ["left_stick", "right_stick"],
  "both stick clicks remain pressed when swapped"
);

const railProfile = {
  ...profile,
  button_map: { ...profile.button_map, south: "right_sl", east: "right_sl" },
  extra_button_map: { ...profile.extra_button_map, c: "right_sr" },
  triggers: {
    ...profile.triggers,
    left: { ...defaultTrigger, lower_deadzone: 1000, output: "right_sr", digital_threshold: 0 },
  },
};
const neutralSample = { buttons: [], extra_buttons: [], triggers: { left: 0, right: 0 } };
assert.deepEqual(transformMappings(neutralSample, railProfile).buttons, [],
                 "a zero-threshold extra output cannot activate at rest");
assert.deepEqual(transformMappings({ ...neutralSample, triggers: { left: 1000, right: 0 } }, railProfile).buttons, [],
                 "extra trigger routing uses the transformed value, not raw input");
assert.deepEqual(transformMappings({ ...neutralSample, triggers: { left: 2000, right: 0 } }, railProfile).buttons, ["right_sr"]);
assert.deepEqual(transformMappings({ ...neutralSample, buttons: ["south", "east"], extra_buttons: ["c"] }, railProfile).buttons,
                 ["right_sl", "right_sr"], "mapped sources OR together without raw-extra passthrough");
railProfile.shift = {
  ...railProfile.shift, mode: "hold", modifier: "c",
  button_map: { ...railProfile.shift.button_map, south: "left_sr" },
  extra_button_map: { ...railProfile.shift.extra_button_map, gl: "left_sl" },
};
assert.deepEqual(transformMappings({ ...neutralSample, buttons: ["south"], extra_buttons: ["c", "gl"] }, railProfile, true).buttons,
                 ["left_sl", "left_sr"], "Shift routes ordinary and extra sources to rails while consuming its modifier");

const axisProfile = {
  ...swapProfile,
  sticks: {
    left: { ...defaultStick, inner_deadzone: 5000 },
    right: { ...defaultStick },
  },
  button_map: { ...swapProfile.button_map, south: "left_stick", left_stick: "south" },
};
const axisSample = {
  ...neutralSample, buttons: ["south", "left_stick"],
  left_stick: { x: 1234, y: -2345 },
  right_stick: { x: -32768, y: 17000 },
};
const physicalSettings = JSON.stringify(axisProfile.sticks);
assert.deepEqual(globalThis.ProfilePlaytestMath.transformSticks(axisSample, axisProfile),
                 { left: { x: -32768, y: 17000 }, right: { x: 0, y: 0 } },
                 "swap follows per-physical-stick calibration, including signed boundaries");
assert.deepEqual(transformMappings(axisSample, axisProfile).buttons, ["south", "right_stick"],
                 "swap applies to mapped click outputs, not physical click sources");
assert.equal(JSON.stringify(axisProfile.sticks), physicalSettings);
axisProfile.swap_sticks = false;
assert.deepEqual(globalThis.ProfilePlaytestMath.transformSticks(axisSample, axisProfile),
                 { left: { x: 0, y: 0 }, right: { x: -32768, y: 17000 } },
                 "turning swap off restores output channels without moving calibration");

// Test left stick direction outputs and analog stick priority
const directionProfile = {
  ...profile,
  button_map: {
    ...profile.button_map,
    dpad_up: "left_stick_up",
    dpad_down: "left_stick_down",
    dpad_left: "left_stick_left",
    dpad_right: "left_stick_right",
  },
  sticks: {
    left: { ...defaultStick, inner_deadzone: 5000 },
    right: { ...defaultStick },
  },
  swap_sticks: false,
};

// 1. Cardinal directions produce magnitude 32767
const upSample = { ...neutralSample, buttons: ["dpad_up"], left_stick: { x: 0, y: 0 }, right_stick: { x: 0, y: 0 } };
assert.deepEqual(
  globalThis.ProfilePlaytestMath.transformSticks(upSample, directionProfile),
  { left: { x: 0, y: -32767 }, right: { x: 0, y: 0 } },
  "left_stick_up produces y = -32767"
);
assert.deepEqual(
  transformMappings(upSample, directionProfile).buttons,
  [],
  "direction outputs do not populate digital buttons array"
);

const downSample = { ...neutralSample, buttons: ["dpad_down"], left_stick: { x: 0, y: 0 }, right_stick: { x: 0, y: 0 } };
assert.deepEqual(
  globalThis.ProfilePlaytestMath.transformSticks(downSample, directionProfile),
  { left: { x: 0, y: 32767 }, right: { x: 0, y: 0 } },
  "left_stick_down produces y = 32767"
);

const leftSample = { ...neutralSample, buttons: ["dpad_left"], left_stick: { x: 0, y: 0 }, right_stick: { x: 0, y: 0 } };
assert.deepEqual(
  globalThis.ProfilePlaytestMath.transformSticks(leftSample, directionProfile),
  { left: { x: -32767, y: 0 }, right: { x: 0, y: 0 } },
  "left_stick_left produces x = -32767"
);

const rightSample = { ...neutralSample, buttons: ["dpad_right"], left_stick: { x: 0, y: 0 }, right_stick: { x: 0, y: 0 } };
assert.deepEqual(
  globalThis.ProfilePlaytestMath.transformSticks(rightSample, directionProfile),
  { left: { x: 32767, y: 0 }, right: { x: 0, y: 0 } },
  "left_stick_right produces x = 32767"
);

// 2. Diagonal directions produce component 23169
const upRightSample = { ...neutralSample, buttons: ["dpad_up", "dpad_right"], left_stick: { x: 0, y: 0 }, right_stick: { x: 0, y: 0 } };
assert.deepEqual(
  globalThis.ProfilePlaytestMath.transformSticks(upRightSample, directionProfile),
  { left: { x: 23169, y: -23169 }, right: { x: 0, y: 0 } },
  "diagonal up-right produces x = 23169, y = -23169"
);

const downLeftSample = { ...neutralSample, buttons: ["dpad_down", "dpad_left"], left_stick: { x: 0, y: 0 }, right_stick: { x: 0, y: 0 } };
assert.deepEqual(
  globalThis.ProfilePlaytestMath.transformSticks(downLeftSample, directionProfile),
  { left: { x: -23169, y: 23169 }, right: { x: 0, y: 0 } },
  "diagonal down-left produces x = -23169, y = 23169"
);

// 3. Opposites cancel independently
const oppositesHorizontal = { ...neutralSample, buttons: ["dpad_left", "dpad_right"], left_stick: { x: 0, y: 0 }, right_stick: { x: 0, y: 0 } };
assert.deepEqual(
  globalThis.ProfilePlaytestMath.transformSticks(oppositesHorizontal, directionProfile),
  { left: { x: 0, y: 0 }, right: { x: 0, y: 0 } },
  "left + right opposites cancel to 0"
);

const oppositesAndVertical = { ...neutralSample, buttons: ["dpad_left", "dpad_right", "dpad_up"], left_stick: { x: 0, y: 0 }, right_stick: { x: 0, y: 0 } };
assert.deepEqual(
  globalThis.ProfilePlaytestMath.transformSticks(oppositesAndVertical, directionProfile),
  { left: { x: 0, y: -32767 }, right: { x: 0, y: 0 } },
  "horizontal opposites cancel while vertical up remains cardinal"
);

// 4. Analog priority: any nonzero mapped analog vector takes priority over digital directions
const movingAnalogWithDpad = {
  ...neutralSample,
  buttons: ["dpad_up"],
  left_stick: { x: 0, y: 10000 }, // outside inner_deadzone of 5000
  right_stick: { x: 0, y: 0 },
};
const analogResult = globalThis.ProfilePlaytestMath.transformSticks(movingAnalogWithDpad, directionProfile);
assert.ok(
  analogResult.left.y > 0,
  "analog vector outside deadzone takes full priority over digital up direction"
);

// 5. Analog inside inner deadzone evaluates to zero and lets digital directions take over
const deadzoneAnalogWithDpad = {
  ...neutralSample,
  buttons: ["dpad_up"],
  left_stick: { x: 0, y: 3000 }, // inside inner_deadzone of 5000 -> output is (0, 0)
  right_stick: { x: 0, y: 0 },
};
assert.deepEqual(
  globalThis.ProfilePlaytestMath.transformSticks(deadzoneAnalogWithDpad, directionProfile),
  { left: { x: 0, y: -32767 }, right: { x: 0, y: 0 } },
  "analog input within deadzone yields (0,0), allowing digital direction to activate"
);

// 6. Stick swap: directions target mapped left stick, which is unaffected by physical stick swap
const swappedDirectionProfile = {
  ...directionProfile,
  swap_sticks: true,
};
// When swapped, physical right stick maps to left stick output.
// If physical right stick has input, it overrides digital directions.
const swappedRightStickActive = {
  ...neutralSample,
  buttons: ["dpad_up"],
  left_stick: { x: 0, y: 10000 }, // physical left stick (swapped to right output)
  right_stick: { x: 0, y: 0 },    // physical right stick (swapped to left output, neutral)
};
assert.deepEqual(
  globalThis.ProfilePlaytestMath.transformSticks(swappedRightStickActive, swappedDirectionProfile),
  {
    left: { x: 0, y: -32767 },
    right: globalThis.ProfilePlaytestMath.transformStick(
      swappedRightStickActive.left_stick, swappedDirectionProfile.sticks.left
    ),
  },
  "digital directions apply to mapped left stick when swapped mapped-left input is neutral"
);

// 7. Extra buttons and trigger routing to directions
const triggerAndExtraDirectionProfile = {
  ...profile,
  button_map: { ...profile.button_map },
  extra_button_map: { ...profile.extra_button_map, c: "left_stick_right" },
  triggers: {
    left: { ...defaultTrigger, lower_deadzone: 1000, output: "left_stick_up", digital_threshold: 20000 },
    right: { ...defaultTrigger, output: "right_trigger" },
  },
  sticks: { left: { ...defaultStick }, right: { ...defaultStick } },
  swap_sticks: false,
};
const extraSample = {
  buttons: [],
  extra_buttons: ["c"],
  triggers: { left: 30000, right: 0 },
  left_stick: { x: 0, y: 0 },
  right_stick: { x: 0, y: 0 },
};
assert.deepEqual(
  globalThis.ProfilePlaytestMath.transformSticks(extraSample, triggerAndExtraDirectionProfile),
  { left: { x: 23169, y: -23169 }, right: { x: 0, y: 0 } },
  "extra button c (right) and left trigger above threshold (up) form diagonal up-right"
);

const shiftedDirectionProfile = {
  ...directionProfile,
  shift: {
    ...directionProfile.shift,
    mode: "hold",
    modifier: "c",
    button_map: { ...directionProfile.button_map, dpad_up: "left_stick_down" },
  },
};
const shiftedDirectionSample = { ...upSample, extra_buttons: ["c"] };
const shiftedDirections = transformMappings(shiftedDirectionSample, shiftedDirectionProfile, true);
assert.deepEqual(
  globalThis.ProfilePlaytestMath.transformSticks(shiftedDirectionSample, shiftedDirectionProfile, shiftedDirections),
  { left: { x: 0, y: 32767 }, right: { x: 0, y: 0 } },
  "stick preview must consume the selected Shift mapping rather than remap the base layer"
);
assert.deepEqual(shiftedDirections.buttons, [], "movement outputs and the Shift modifier cannot become click/button outputs");
