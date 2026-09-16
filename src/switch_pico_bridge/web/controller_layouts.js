"use strict";

// Coordinates are in the supplied SVGs' original viewBoxes, including their
// offsets and nested group translations. A single stage rotates art + targets.
const ControllerLayouts = (() => {
  const point = (x, y, label, glyph) => ({ x, y, label, glyph });
  const back = (label, glyph) => ({ label, glyph, offArt: true });
  const dpad = (x, y, distance) => ({
    dpad_up: point(x, y - distance, "D-pad Up", "▲"),
    dpad_right: point(x + distance, y, "D-pad Right", "▶"),
    dpad_down: point(x, y + distance, "D-pad Down", "▼"),
    dpad_left: point(x - distance, y, "D-pad Left", "◀"),
  });
  const triggers = {
    left_trigger: back("Left trigger · rear", "LT"),
    right_trigger: back("Right trigger · rear", "RT"),
  };
  const xbox = {
    name: "Xbox Wireless Controller", style: "xbox", asset: "xbox-controller-simple.svg",
    viewBox: [0, 0, 800, 552], minWidth: 640,
    controls: {
      ...triggers,
      left_shoulder: point(204, 39, "LB", "LB"), right_shoulder: point(596, 39, "RB", "RB"),
      left_stick: point(204, 177, "Left stick click", "L3"), right_stick: point(501, 298, "Right stick click", "R3"),
      ...dpad(295, 306, 37),
      north: point(600, 131, "Y", "Y"), east: point(652, 182, "B", "B"),
      south: point(600, 233, "A", "A"), west: point(548, 182, "X", "X"),
      select: point(340, 180, "View", "View"), start: point(460, 180, "Menu", "Menu"),
      capture: point(400, 225, "Share", "Share"), system: point(400, 93, "Xbox", "Xbox"),
    },
  };
  const leftFace = {
    left_stick: point(84, 139, "Left stick click", "L3"),
    select: point(133, 62.5, "Minus", "−"), capture: point(119, 374, "Capture", "Cap"),
    ...dpad(84, 278, 39),
  };
  // The SVG's left/right directional centers are 40 units from its center.
  leftFace.dpad_left.x = 44;
  leftFace.dpad_right.x = 124;
  const rightFace = {
    north: point(80, 100, "X", "X"), west: point(40, 139, "Y", "Y"),
    east: point(120, 139, "A", "A"), south: point(80, 178, "B", "B"),
    right_stick: point(80, 276, "Right stick click", "R3"),
    start: point(31, 63, "Plus", "+"), system: point(45, 374, "Home", "Home"),
    c: point(45, 430, "C", "C"),
  };
  const translate = (controls, x, y = 0) => Object.fromEntries(
    Object.entries(controls).map(([id, value]) => [id, { ...value, x: value.x + x, y: value.y + y }])
  );
  const railNote = "Rail SL/SR are extra inputs and also emit the left/right shoulder source in physical solo mode. The physical L or R button shares that shoulder source. Each source ID is editable once; changing an extra does not disable its shoulder alias. Rail output destinations are available for native Joy-Con and Pro emulation.";
  const wiiCommon = {
    select: point(60, 294, "Minus", "−"), system: point(100, 294, "Home", "Home"),
    start: point(140, 294, "Plus", "+"),
  };
  const wiiVertical = {
    ...wiiCommon, ...dpad(100, 128, 25),
    south: back("B · underside", "B"), east: point(100, 216, "A", "A"),
    west: point(100, 445, "1", "1"), north: point(100, 495, "2", "2"),
  };
  const layouts = {
    generic: { ...xbox, name: "Generic controller reference", style: "generic", generic: true },
    xbox,
    dualsense: {
      name: "Sony DualSense", style: "playstation", asset: "ps5-dualsense-simple.svg",
      viewBox: [0, 0, 800, 548], minWidth: 640,
      controls: {
        left_trigger: back("L2 · rear", "L2"), right_trigger: back("R2 · rear", "R2"),
        left_shoulder: point(165, 32, "L1", "L1"), right_shoulder: point(635, 32, "R1", "R1"),
        left_stick: point(282, 280.3, "Left stick click", "L3"), right_stick: point(518, 280.3, "Right stick click", "R3"),
        ...dpad(166, 174, 39),
        north: point(634, 119, "Triangle", "△"), east: point(688, 173, "Circle", "○"),
        south: point(634, 227, "Cross", "×"), west: point(580, 173, "Square", "□"),
        select: point(231, 92, "Create", "Create"), start: point(569, 92, "Options", "Opt"),
        capture: point(400, 120, "Touchpad click", "TP"), system: point(400, 277, "PS", "PS"),
      },
      note: "The microphone mute button is not exposed as a remappable source. Rear triggers are listed below the front view.",
    },
    "switch-pro": {
      name: "Nintendo Switch Pro Controller", style: "switch", asset: "switch-pro-controller-simple.svg",
      viewBox: [0, 0, 800, 570], minWidth: 640,
      controls: {
        left_trigger: back("ZL · rear", "ZL"), right_trigger: back("ZR · rear", "ZR"),
        left_shoulder: point(205, 32, "L", "L"), right_shoulder: point(595, 32, "R", "R"),
        left_stick: point(190, 179, "Left stick click", "L3"), right_stick: point(500, 304, "Right stick click", "R3"),
        ...dpad(294, 306, 40),
        north: point(610, 137, "X", "X"), east: point(662, 189, "A", "A"),
        south: point(610, 241, "B", "B"), west: point(558, 189, "Y", "Y"),
        select: point(301, 118, "Minus", "−"), start: point(499, 118, "Plus", "+"),
        capture: point(349, 180, "Capture", "Cap"), system: point(451, 180, "Home", "Home"),
      },
    },
    "switch2-pro": {
      name: "Nintendo Switch 2 Pro Controller", style: "switch", asset: "switch-2-pro-controller-simple.svg",
      viewBox: [-16, 160, 1232, 884], minWidth: 720,
      controls: {
        left_trigger: back("ZL · rear", "ZL"), right_trigger: back("ZR · rear", "ZR"),
        gl: back("GL · back button", "GL"), gr: back("GR · back button", "GR"),
        left_shoulder: point(290, 193, "L", "L"), right_shoulder: point(910, 193, "R", "R"),
        left_stick: point(272, 434, "Left stick click", "L3"), right_stick: point(746, 605, "Right stick click", "R3"),
        ...dpad(424, 602, 60),
        north: point(909, 350, "X", "X"), east: point(1005, 433, "A", "A"),
        south: point(909, 517, "B", "B"), west: point(813, 433, "Y", "Y"),
        select: point(454, 341, "Minus", "−"), start: point(743, 341, "Plus", "+"),
        capture: point(526, 434, "Capture", "Cap"), system: point(672, 435, "Home", "Home"),
        c: point(599, 693, "C", "C"),
      },
      note: "C is on the front. GL and GR are rear controls, not front-face buttons. C, GL and GR are source-only inputs.",
    },
    "joycon2-pair": {
      name: "Joy-Con 2 pair", style: "switch", asset: "switch-2-joycons-connected.svg",
      viewBox: [-8, -10, 688, 560], minWidth: 620,
      controls: {
        ...translate(leftFace, 64), ...translate(rightFace, 444),
        left_shoulder: back("L · top edge", "L"), right_shoulder: back("R · top edge", "R"),
        left_trigger: back("ZL · rear", "ZL"), right_trigger: back("ZR · rear", "ZR"),
        left_sl: back("Left SL · rail", "L SL"), left_sr: back("Left SR · rail", "L SR"),
        right_sl: back("Right SL · rail", "R SL"), right_sr: back("Right SR · rail", "R SR"),
      },
      note: "Select the L+R profile owner for paired settings; its eight profiles are independent of both solo banks. If no L+R owner is listed, update firmware and connect both members. The grip is illustrative, not detected. Rear triggers and rails are not visible in this front view. C is a source-only extra; rails can also be mapped outputs. GL/GR belong to Switch 2 Pro, not Joy-Con 2.",
    },
    "joycon2-left": {
      name: "Joy-Con 2 left · sideways solo", style: "switch", asset: "switch-2-joycon-left.svg",
      viewBox: [-17, -10, 200, 560], rotation: -90, minWidth: 560,
      controls: {
        left_stick: leftFace.left_stick, select: leftFace.select, capture: leftFace.capture,
        south: { ...leftFace.dpad_left, label: "Left arrow (solo south)", glyph: "▼" },
        east: { ...leftFace.dpad_down, label: "Down arrow (solo east)", glyph: "▶" },
        west: { ...leftFace.dpad_up, label: "Up arrow (solo west)", glyph: "◀" },
        north: { ...leftFace.dpad_right, label: "Right arrow (solo north)", glyph: "▲" },
        left_shoulder: back("L / SL · shoulder alias", "L / SL"), right_shoulder: back("SR · shoulder alias", "SR"),
        left_trigger: back("ZL · rear", "ZL"),
        left_sl: back("SL · rail extra", "SL extra"), left_sr: back("SR · rail extra", "SR extra"),
      },
      note: "Solo firmware rotates the directional buttons into face-button sources and keeps the stick on LEFT axes. Arrow names refer to the original upright art. " + railNote,
    },
    "joycon2-right": {
      name: "Joy-Con 2 right · sideways solo", style: "switch", asset: "switch-2-joycon-single.svg",
      viewBox: [-19, -10, 200, 560], rotation: 90, minWidth: 560,
      controls: {
        south: { ...rightFace.east, label: "A (solo south)" }, east: { ...rightFace.north, label: "X (solo east)" },
        west: { ...rightFace.south, label: "B (solo west)" }, north: { ...rightFace.west, label: "Y (solo north)" },
        left_stick: { ...rightFace.right_stick, label: "Right stick · solo LEFT axes/click", glyph: "L3" },
        start: rightFace.start, system: rightFace.system, c: rightFace.c,
        left_shoulder: back("SL · shoulder alias", "SL"), right_shoulder: back("R / SR · shoulder alias", "R / SR"),
        right_trigger: back("ZR · rear", "ZR"),
        right_sl: back("SL · rail extra", "SL extra"), right_sr: back("SR · rail extra", "SR extra"),
      },
      note: "Physical solo input rotates ABXY sources; the right stick becomes LEFT axes and left-stick click. The emulated output layout is configured separately in the profile. Select the separate L+R profile owner for paired settings, not this solo bank. " + railNote,
    },
    "wii-remote": {
      name: "Wii Remote · orientation unknown", style: "switch", asset: "wii-remote-simple.svg",
      viewBox: [0, 0, 200, 632], rotation: -90, minWidth: 760,
      controls: {
        ...wiiCommon,
        south: point(100, 445, "1 (horizontal default)", "1"), east: point(100, 495, "2 (horizontal default)", "2"),
        west: point(100, 216, "A (horizontal default)", "A"), north: back("B · underside", "B"),
        dpad_left: point(100, 103, "D-pad physical Up → Left (horizontal default)", "◀"),
        dpad_up: point(125, 128, "D-pad physical Right → Up (horizontal default)", "▲"),
        dpad_right: point(100, 153, "D-pad physical Down → Right (horizontal default)", "▶"),
        dpad_down: point(75, 128, "D-pad physical Left → Down (horizontal default)", "▼"),
      },
      note: "Legacy firmware reports a Wii Remote without detected orientation. Controls default to standard horizontal positions. Select an explicit preview to compare layouts. B is underneath; Power is not remappable.",
    },
    "wii-horizontal": {
      name: "Wii Remote · horizontal / accelerometer", style: "switch", asset: "wii-remote-simple.svg",
      viewBox: [0, 0, 200, 632], rotation: -90, minWidth: 760,
      controls: {
        ...wiiCommon,
        south: point(100, 445, "1", "1"), east: point(100, 495, "2", "2"),
        west: point(100, 216, "A", "A"), north: back("B · underside", "B"),
        dpad_left: point(100, 103, "D-pad physical Up → Left", "◀"),
        dpad_up: point(125, 128, "D-pad physical Right → Up", "▲"),
        dpad_right: point(100, 153, "D-pad physical Down → Right", "▶"),
        dpad_down: point(75, 128, "D-pad physical Left → Down", "▼"),
      },
      note: "1→south, 2→east, A→west, B→north. D-pad sources rotate counterclockwise for horizontal grip. B is underneath; Power is not remappable.",
    },
    "wii-vertical": {
      name: "Wii Remote · vertical", style: "switch", asset: "wii-remote-simple.svg",
      viewBox: [0, 0, 200, 632], minWidth: 240, maxWidth: 260, controls: wiiVertical,
      note: "B→south, A→east, 1→west, 2→north. D-pad directions stay upright. B is underneath; Power is not remappable.",
    },
    "wii-nunchuk": {
      name: "Wii Remote + Nunchuk", style: "switch", asset: "wii-remote-nunchuk-simple.svg",
      viewBox: [0, 0, 540, 764], minWidth: 600,
      controls: {
        ...translate({ ...wiiCommon, ...dpad(100, 128, 25), east: wiiVertical.east }, 320, 10),
        south: back("B · Remote underside", "B"),
        left_shoulder: point(420, 455, "1", "1"), right_shoulder: point(420, 505, "2", "2"),
        west: back("Nunchuk C · rear", "C"), north: back("Nunchuk Z · rear", "Z"),
      },
      annotations: [{ x: 233, y: 220.4, label: "Stick → LEFT axes (no click)" }],
      note: "Nunchuk stick uses LEFT axes with calibrated travel and has no click. B→south, A→east, 1/2→left/right shoulder, Nunchuk C→west and Z→north. Nunchuk C is a normal face source, not the Switch 2 C extra. Power is not remappable.",
    },
  };
  layouts["wii-reference"] = {
    ...layouts["wii-remote"],
    name: "Wii Remote · reference layout",
    referenceOnly: true,
    note: "Wii-family reference, shown with the default horizontal controls. The saved identity cannot confirm orientation, extensions, or distinguish a Wii U Pro Controller. Connect the controller for its detected layout, or choose a diagram preview. All source mappings remain available; physical highlighting is off.",
  };
  return layouts;
})();
