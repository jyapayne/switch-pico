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
