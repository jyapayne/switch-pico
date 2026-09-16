"use strict";

const PROFILE_OWNER_STORAGE_KEY = "switch-pico.profile-owner";

const MACRO_STEP_LIMIT = 8;
const MACRO_SHARED_STEP_LIMIT = 16;
const MACRO_BYTE_LIMIT = 136;
const DEFAULT_MACRO_STEP_BYTES = 5;

// Visual-only state never enters the persisted profile or USB output.
const macroPreview = {
  frame: 0,
  running: false,
  startedAt: 0,
  steps: [],
  duration: 0,
  index: -1,
  cycles: 1,
  cycle: -1,
};
let draggedMacroStep = null;
const macroCapture = {
  session: null,
  requestActive: false,
  timer: 0,
};
const joyconMode = {
  saved: null,
  selected: null,
  pending: false,
  supported: false,
  generation: 0,
  loading: true,
  applying: false,
  requestActive: false,
  revision: 0,
  readError: "",
  applyError: "",
};
const wiiOrientation = {
  current: null,
  selected: "horizontal",
  pending: false,
  applying: false,
  applyError: "",
  submittedGeneration: null,
  desiredLayout: null,
  ownerKey: null,
  applyTimer: 0,
};
function resetWiiOrientationDraft() {
  wiiOrientation.pending = false;
  wiiOrientation.applyError = "";
  if (wiiOrientation.current) {
    wiiOrientation.selected = wiiOrientation.current;
  }
}
const state = {
  schema: null,
  identities: [],
  identityIndex: 0,
  profileIndex: 0,
  profile: null,
  profileNames: Array(8).fill(""),
  original: "",
  pendingName: false,
  active: false,
  selectedButton: "south",
  selectedMacro: 0,
  token: "",
  busy: false,
  identifyAvailable: false,
  adapterConnected: false,
  previewInputConnected: false,
  playtestRequestActive: false,
  liveSample: null,
  layoutPreview: "auto",
  presentation: null,
  presentationKey: "",
  diagramKey: "",
  playtestShift: { key: "", held: false, active: false },
  playtestTimer: 0,
  libraryRequestActive: false,
  libraryTimer: 0,
};

const elements = {
  connection: document.querySelector("#connectionState"),
  connectionText: document.querySelector("#connectionText"),
  identity: document.querySelector("#identitySelect"),
  adapterSettingsButton: document.querySelector("#adapterSettingsButton"),
  adapterSettingsDialog: document.querySelector("#adapterSettingsDialog"),
  closeAdapterSettings: document.querySelector("#closeAdapterSettingsButton"),
  joyconMode: document.querySelector("#joyconModeSelect"),
  applyJoyconMode: document.querySelector("#applyJoyconModeButton"),
  joyconModeStatus: document.querySelector("#joyconModeStatus"),
  profileList: document.querySelector("#profileList"),
  profileOwner: document.querySelector("#profileOwner"),
  controllerAlias: document.querySelector("#controllerAlias"),
  controllerDetails: document.querySelector("#controllerDetails"),
  saveAlias: document.querySelector("#saveAliasButton"),
  identify: document.querySelector("#identifyButton"),
  profileTitle: document.querySelector("#profileTitle"),
  activeBadge: document.querySelector("#activeBadge"),
  dirtyBadge: document.querySelector("#dirtyBadge"),
  profileName: document.querySelector("#profileName"),
  saveProfileName: document.querySelector("#saveProfileNameButton"),
  loading: document.querySelector("#loadingCard"),
  form: document.querySelector("#profileForm"),
  controllerCanvas: document.querySelector("#controllerCanvas"),
  controllerImage: document.querySelector("#controllerImage"),
  controllerModel: document.querySelector("#controllerModel"),
  controllerLayoutStatus: document.querySelector("#controllerLayoutStatus"),
  controllerLayoutPreview: document.querySelector("#controllerLayoutPreview"),
  controllerLayoutHelp: document.querySelector("#controllerLayoutHelp"),
  controllerLayoutNote: document.querySelector("#controllerLayoutNote"),
  nativeJoyconLayout: document.querySelector("#nativeJoyconLayoutSelect"),
  mapShoulders: document.querySelector("#mapShouldersButton"),
  mapDpad: document.querySelector("#mapDpadButton"),
  swapSticks: document.querySelector("#swapSticksCheckbox"),
  wiiOrientationControl: document.querySelector("#wiiOrientationControl"),
  wiiOrientation: document.querySelector("#wiiOrientationSelect"),
  applyWiiOrientation: document.querySelector("#applyWiiOrientationButton"),
  wiiOrientationStatus: document.querySelector("#wiiOrientationStatus"),
  wiiOrientationHelp: document.querySelector("#wiiOrientationHelp"),
  controllerPhotoWrap: document.querySelector("#controllerPhotoWrap"),
  controllerStage: document.querySelector("#controllerStage"),
  selectedSource: document.querySelector("#selectedSource"),
  controllerHotspots: document.querySelector("#controllerHotspots"),
  extraControls: document.querySelector("#extraControls"),
  selectedMapping: document.querySelector("#selectedMapping"),
  selectedControlGlyph: document.querySelector("#selectedControlGlyph"),
  selectedControlName: document.querySelector("#selectedControlName"),
  selectedControlDescription: document.querySelector("#selectedControlDescription"),
  analog: document.querySelector("#analogFields"),
  rumble: document.querySelector("#rumbleFields"),
  builtinActions: document.querySelector("#builtinActions"),
  swing: document.querySelector("#swingFields"),
  turbo: document.querySelector("#turboFields"),
  turboDefaults: document.querySelector("#turboDefaultFields"),
  turboTimingNotice: document.querySelector("#turboTimingNotice"),
  shortcutModifier: document.querySelector("#shortcutModifierFields"),
  shortcuts: document.querySelector("#shortcutFields"),
  shift: document.querySelector("#shiftFields"),
  shiftMap: document.querySelector("#shiftMapFields"),
  macroControls: document.querySelector("#macroControls"),
  macroSteps: document.querySelector("#macroSteps"),
  macroStepsTitle: document.querySelector("#macroStepsTitle"),
  macroDuration: document.querySelector("#macroDuration"),
  macroNotice: document.querySelector("#macroNotice"),
  macroPreview: document.querySelector("#macroPreview"),
  macroPreviewTitle: document.querySelector("#macroPreviewTitle"),
  macroPreviewModeHelp: document.querySelector("#macroPreviewModeHelp"),
  macroPreviewPlay: document.querySelector("#macroPreviewPlay"),
  macroPreviewStop: document.querySelector("#macroPreviewStop"),
  macroPreviewRestart: document.querySelector("#macroPreviewRestart"),
  macroPreviewStatus: document.querySelector("#macroPreviewStatus"),
  macroPreviewTime: document.querySelector("#macroPreviewTime"),
  macroPreviewProgress: document.querySelector("#macroPreviewProgress"),
  macroPreviewButtons: document.querySelector("#macroPreviewButtons"),
  macroCapture: document.querySelector("#macroCapture"),
  captureTitle: document.querySelector("#macroCaptureTitle"),
  captureRecord: document.querySelector("#macroCaptureRecord"),
  captureStop: document.querySelector("#macroCaptureStop"),
  captureOptions: document.querySelector("#macroCaptureOptions"),
  captureAxis: document.querySelector("#macroCaptureAxis"),
  captureTrigger: document.querySelector("#macroCaptureTrigger"),
  captureDuration: document.querySelector("#macroCaptureDuration"),
  captureBudget: document.querySelector("#macroCaptureBudget"),
  captureState: document.querySelector("#macroCaptureState"),
  captureElapsed: document.querySelector("#macroCaptureElapsed"),
  captureProgress: document.querySelector("#macroCaptureProgress"),
  captureNotice: document.querySelector("#macroCaptureNotice"),
  captureEvents: document.querySelector("#macroCaptureEvents"),
  captureUse: document.querySelector("#macroCaptureUse"),
  captureDiscard: document.querySelector("#macroCaptureDiscard"),
  captureRecover: document.querySelector("#macroCaptureRecover"),
  playtestPanel: document.querySelector("#playtestPanel"),
  playtestTitle: document.querySelector("#playtestTitle"),
  playtestStatus: document.querySelector("#playtestStatus"),
  playtestHelp: document.querySelector("#playtestHelp"),
  playtestExtraInputs: document.querySelector("#playtestExtraInputs"),
  playtestMappedButtons: document.querySelector("#playtestMappedButtons"),
  playtestMappingHelp: document.querySelector("#playtestMappingHelp"),
  playtestLeftValues: document.querySelector("#playtestLeftValues"),
  playtestRightValues: document.querySelector("#playtestRightValues"),
  playtestLeftTriggerLabel: document.querySelector("#playtestLeftTriggerLabel"),
  playtestRightTriggerLabel: document.querySelector("#playtestRightTriggerLabel"),
  refresh: document.querySelector("#refreshButton"),
  resetDraft: document.querySelector("#resetDraftButton"),
  activate: document.querySelector("#activateButton"),
  save: document.querySelector("#saveButton"),
  addMacroStep: document.querySelector("#addMacroStepButton"),
  copyProfile: document.querySelector("#copyProfileButton"),
  exportProfile: document.querySelector("#exportProfileButton"),
  importProfile: document.querySelector("#importProfileButton"),
  importProfileFile: document.querySelector("#importProfileFile"),
  copyDialog: document.querySelector("#copyDialog"),
  copyIdentity: document.querySelector("#copyIdentity"),
  copySlot: document.querySelector("#copySlot"),
  confirmCopy: document.querySelector("#confirmCopyButton"),
  toast: document.querySelector("#toast"),
};

const sectionLinks = document.querySelectorAll(".section-nav a");
const sectionPanels = Array.from(sectionLinks, (link) =>
  document.getElementById(link.hash.slice(1))
);
let selectedSection = null;

function showSection(id) {
  const panel = sectionPanels.find((section) => section.id === id) || sectionPanels[0];
  if (selectedSection === panel) return;
  if (selectedSection) {
    stopMacroPreview("Preview stopped: section changed.");
    clearMacroDrag();
    stopCapture("Recording stopped because you navigated to another section.");
  }
  selectedSection = panel;
  sectionPanels.forEach((section) => {
    section.hidden = section !== panel;
  });
  sectionLinks.forEach((link) => {
    if (link.hash === `#${panel.id}`) {
      link.setAttribute("aria-current", "page");
      // Keep the current section visible in the horizontally scrolling mobile nav.
      link.parentElement.scrollLeft = link.offsetLeft - (link.parentElement.clientWidth - link.offsetWidth) / 2;
    } else link.removeAttribute("aria-current");
  });
}

function showSectionFromHash() {
  showSection(window.location.hash.slice(1));
}

function revealInvalidControl(control) {
  // A modal copy dialog would otherwise keep the invalid draft field inert.
  if (elements.copyDialog.open) elements.copyDialog.close();
  const panel = sectionPanels.find((section) => section.contains(control));
  if (panel) {
    showSection(panel.id);
    if (window.location.hash !== `#${panel.id}`) window.location.hash = panel.id;
  }
  for (let ancestor = control.parentElement; ancestor && ancestor !== elements.form; ancestor = ancestor.parentElement) {
    if (ancestor.tagName === "DETAILS") ancestor.open = true;
    if (ancestor.hidden) ancestor.hidden = false;
  }
}

function reportProfileValidity() {
  // Validate only this form's controls, not the separate recording settings.
  // Report one field at a time so other hidden panels never receive focus.
  for (const control of elements.form.elements) {
    if (control.willValidate && !control.validity.valid) {
      revealInvalidControl(control);
      control.reportValidity();
      return false;
    }
  }
  return true;
}

function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

function label(value) {
  return value
    .split("_")
    .map((part) => part.charAt(0).toUpperCase() + part.slice(1))
    .join(" ");
}


const directionalLabels = {
  dpad_up: "D-pad Up",
  dpad_right: "D-pad Right",
  dpad_down: "D-pad Down",
  dpad_left: "D-pad Left",
  left_stick: "Left Stick Click",
  right_stick: "Right Stick Click",
  left_stick_up: "Left Stick Up",
  left_stick_down: "Left Stick Down",
  left_stick_left: "Left Stick Left",
  left_stick_right: "Left Stick Right",
  c: "C",
  gl: "GL",
  gr: "GR",
  left_sl: "Left SL",
  left_sr: "Left SR",
  right_sl: "Right SL",
  right_sr: "Right SR",
};

function matchingLiveSample(sample = state.liveSample) {
  const owner = currentOwner();
  return Boolean(sample?.connected && owner && sample.owner_key === owner.key &&
    (owner.index === 0 || sample.identity_key === owner.key));
}

function currentControllerStyle() {
  const style = state.presentation?.style || currentOwner()?.controller?.style || "generic";
  return style === "wii" ? "switch" : style;
}

function sourceLabel(control) {
  return (!state.presentation?.layout.generic && state.presentation?.layout.controls[control]?.label) || controlLabel(control);
}

function sourceGlyph(control) {
  return (!state.presentation?.layout.generic && state.presentation?.layout.controls[control]?.glyph) || controlGlyph(control, currentControllerStyle());
}

function sourceAvailable(control) {
  return state.presentation?.sources.includes(control) ?? true;
}

function sourceChoices(choices, stored = []) {
  return choices.filter(control => sourceAvailable(control) || stored.includes(control));
}

function sourceOptions(selected, choices = state.schema.controls, excluded = []) {
  const available = sourceChoices(choices);
  if (selected && !available.includes(selected)) available.push(selected);
  return `<option value=""${selected === null ? " selected" : ""}>None</option>` +
    available.map(control => `<option value="${escapeHtml(control)}"${selected === control ? " selected" : ""}${excluded.includes(control) && control !== selected ? " disabled" : ""}>${escapeHtml(sourceLabel(control))}${sourceAvailable(control) ? "" : " (stored / unavailable)"}</option>`).join("");
}

function syncControllerPresentation() {
  const owner = currentOwner();
  const live = matchingLiveSample() ? state.liveSample : null;
  const controller = live?.controller || owner?.controller || {};
  const preview = state.layoutPreview !== "auto";
  const layoutId = preview ? state.layoutPreview : controller.layout || "generic";
  const layout = ControllerLayouts[layoutId] || ControllerLayouts.generic;
  const reported = live?.source_controls || owner?.source_controls || state.schema.controls;
  const sources = preview ? Object.keys(layout.controls) : reported.filter(control =>
    layout.generic || layout.referenceOnly || Object.hasOwn(layout.controls, control));
  const topologyKnown = !layout.referenceOnly && layoutId !== "wii-remote" &&
    (!(layoutId.startsWith("joycon2-") || layoutId.startsWith("wii-")) || live?.layout === layoutId);
  const liveDiagram = Boolean(live && !preview && topologyKnown && !layout.generic);
  const style = preview ? layout.style : controller.style || layout.style;
  const key = [owner?.key, layoutId, style, preview, liveDiagram, Boolean(live), live?.identity_key, controller.model, sources.join(",")].join("|");
  if (state.presentationKey === key) return false;
  state.presentationKey = key;
  state.presentation = { layoutId, layout, sources, preview, liveDiagram, live, style, topologyKnown };
  return true;
}

function controlLabel(control, style = currentControllerStyle()) {
  if (style === "wii") style = "switch";
  return state.schema.control_labels?.[style]?.[control] ||
    directionalLabels[control] ||
    label(control);
}

const nativeLayoutLabels = {
  paired: "Paired",
  left_solo: "Left sideways",
  right_solo: "Right sideways",
};
const soloFaceLabels = {
  left_solo: {
    south: "Bottom / Left Joy-Con Left arrow",
    east: "Right / Left Joy-Con Down arrow",
    west: "Left / Left Joy-Con Up arrow",
    north: "Top / Left Joy-Con Right arrow",
  },
  right_solo: {
    south: "Bottom / Right Joy-Con A",
    east: "Right / Right Joy-Con X",
    west: "Left / Right Joy-Con B",
    north: "Top / Right Joy-Con Y",
  },
};

function outputLabel(control, style = currentControllerStyle()) {
  const layout = state.profile?.native_joycon_layout || "paired";
  if (soloFaceLabels[layout]) {
    if (soloFaceLabels[layout][control]) return soloFaceLabels[layout][control];
    if (control.startsWith("dpad_")) return `${controlLabel(control, "switch")} · no separate solo D-pad`;
    if (control === "left_stick") return "Solo stick click · mapped left output";
    if (control === "right_stick") return "Right stick click · unused in solo";
    if (control.startsWith("left_stick_")) return `${controlLabel(control, "switch")} · mapped left movement`;
    return controlLabel(control, "switch");
  }
  return controlLabel(control, style);
}

function renderNativeLayout() {
  const layout = state.profile?.native_joycon_layout || "paired";
  const choices = state.schema?.native_joycon_layouts || [];
  elements.nativeJoyconLayout.innerHTML = choices.map(value =>
    `<option value="${value}"${layout === value ? " selected" : ""}>${nativeLayoutLabels[value]}</option>`
  ).join("");
  elements.nativeJoyconLayout.disabled = state.busy || captureBlocking() || !state.profile;
  elements.mapShoulders.hidden = layout === "paired";
  elements.mapShoulders.disabled = elements.nativeJoyconLayout.disabled;
  if (elements.mapDpad) elements.mapDpad.disabled = elements.nativeJoyconLayout.disabled;
  elements.swapSticks.checked = Boolean(state.profile?.swap_sticks);
  elements.swapSticks.disabled = elements.nativeJoyconLayout.disabled;
}

function clone(value) {
  return JSON.parse(JSON.stringify(value));
}

function canonical(value) {
  return JSON.stringify(value);
}

const {
  transformTrigger,
  stickCoordinates,
  triggerPercent,
} = ProfilePlaytestMath;

function updateStickPlaytest(side, raw, output, config) {
  const scope = document.querySelector(`[data-playtest-stick="${side}"]`);
  const rawPosition = stickCoordinates(raw);
  const outputPosition = stickCoordinates(output);
  scope.style.setProperty("--raw-x", `${rawPosition.left}%`);
  scope.style.setProperty("--raw-y", `${rawPosition.top}%`);
  scope.style.setProperty("--output-x", `${outputPosition.left}%`);
  scope.style.setProperty("--output-y", `${outputPosition.top}%`);
  scope.style.setProperty(
    "--inner-size",
    `${Math.max(0, Math.min(100, config.inner_deadzone / 32767 * 100))}%`
  );
  scope.style.setProperty(
    "--outer-size",
    `${Math.max(0, Math.min(100, config.outer_saturation / 32767 * 100))}%`
  );
  const values = `${raw.x}, ${raw.y} → ${output.x}, ${output.y}`;
  (side === "left" ? elements.playtestLeftValues : elements.playtestRightValues)
    .textContent = values;
}

function updateTriggerPlaytest(side, raw, output, config) {
  const meter = document.querySelector(`[data-playtest-trigger="${side}"]`);
  const track = meter.querySelector(".trigger-track");
  track.style.setProperty("--raw", `${triggerPercent(raw)}%`);
  track.style.setProperty("--output", `${triggerPercent(output)}%`);
  track.style.setProperty(
    "--threshold",
    `${config.digital_threshold / 65535 * 100}%`
  );
  meter.querySelector("output").textContent = `${raw} → ${output}`;
}

function updateCurveMarker(group, side, input) {
  const panel = document.querySelector(
    `[data-analog-group="${group}"][data-analog-side="${side}"]`
  );
  if (!panel) return;
  const normalized = Math.max(0, Math.min(1, input));
  const config = state.profile[group][side];
  const output = ProfilePlaytestMath.transformTrigger(
    Math.round(normalized * 65535),
    {
      lower_deadzone: 0,
      upper_saturation: 65535,
      curve_q8_8: config.curve_q8_8,
    }
  ) / 65535;
  const marker = panel.querySelector(".curve-marker");
  marker.setAttribute("cx", String(normalized * 100));
  marker.setAttribute("cy", String(60 - output * 60));
  marker.classList.add("visible");
}

function clearPlaytest(message, stateName = "waiting") {
  state.liveSample = null;
  state.playtestShift = { key: "", held: false, active: false };
  elements.playtestExtraInputs.textContent = "None";
  elements.playtestMappedButtons.textContent = "None";
  if (state.previewInputConnected) stopMacroPreview("Preview stopped: controller disconnected.");
  state.previewInputConnected = false;
  state.identifyAvailable = false;
  elements.identify.disabled = true;
  elements.playtestPanel.dataset.state = stateName;
  elements.playtestStatus.textContent =
    stateName === "error" ? "Unavailable" : "Waiting";
  elements.playtestTitle.textContent =
    stateName === "error" ? "Live input unavailable" : "Waiting for controller input";
  elements.playtestHelp.textContent = message;
  elements.controllerCanvas.querySelectorAll(".pressed")
    .forEach((button) => button.classList.remove("pressed"));
  document.querySelectorAll(".curve-marker.visible")
    .forEach((marker) => marker.classList.remove("visible"));
  if (state.schema && state.profile && syncControllerPresentation()) {
    renderButtonMap();
    refreshSourceControls();
  }
  renderCapture();
  if (wiiOrientation.applying) {
    window.clearTimeout(wiiOrientation.applyTimer);
    wiiOrientation.applying = false;
    wiiOrientation.submittedGeneration = null;
    wiiOrientation.desiredLayout = null;
    wiiOrientation.ownerKey = null;
    setBusy(false);
  }
  wiiOrientation.current = null;
  resetWiiOrientationDraft();
  renderWiiOrientation();
}

function renderPlaytest(sample) {
  state.liveSample = sample;
  if (!matchingLiveSample(sample)) {
    clearPlaytest(
      sample.connected
        ? sample.identity?.is_joycon_pair
          ? "Paired input uses the L+R controller profiles. Select L+R for paired settings; this draft and its layout are unchanged."
          : "Live input belongs to another controller; this draft and its layout are unchanged."
        : "Connect or move the selected controller to compare its raw input with this draft."
    );
    return;
  }
  if (syncControllerPresentation()) {
    renderButtonMap();
    refreshSourceControls();
  }
  state.previewInputConnected = true;
  const shift = state.profile.shift;
  const shiftKey = `${sample.identity_key}:${sample.connection_generation}:${shift.mode}:${shift.modifier}`;
  if (state.playtestShift.key !== shiftKey) {
    state.playtestShift = { key: shiftKey, held: false, active: false };
  }
  const controls = new Set([...sample.buttons, ...(sample.extra_buttons || [])]);
  for (const side of ["left", "right"]) {
    const value = transformTrigger(sample.triggers[side], state.profile.triggers[side]);
    if (value >= state.profile.triggers[side].digital_threshold) {
      controls.add(`${side}_trigger`);
    }
  }
  const held = controls.has(shift.modifier);
  if (shift.mode === "toggle" && held && !state.playtestShift.held) {
    state.playtestShift.active = !state.playtestShift.active;
  } else if (shift.mode !== "toggle") {
    state.playtestShift.active = shift.mode === "hold" && held;
  }
  state.playtestShift.held = held;
  const mapped = ProfilePlaytestMath.transformMappings(sample, state.profile, state.playtestShift.active);
  const rawLeft = sample.left_stick;
  const rawRight = sample.right_stick;
  const { left: outputLeft, right: outputRight } = ProfilePlaytestMath.transformSticks(sample, state.profile, mapped);
  const outputLeftTrigger = mapped.triggers.left;
  const outputRightTrigger = mapped.triggers.right;
  updateStickPlaytest(
    "left", rawLeft, outputLeft, state.profile.sticks.left
  );
  updateStickPlaytest(
    "right", rawRight, outputRight, state.profile.sticks.right
  );
  updateTriggerPlaytest(
    "left", sample.triggers.left, outputLeftTrigger,
    state.profile.triggers.left
  );
  updateTriggerPlaytest(
    "right", sample.triggers.right, outputRightTrigger,
    state.profile.triggers.right
  );
  updateCurveMarker(
    "sticks", "left",
    Math.max(Math.abs(rawLeft.x), Math.abs(rawLeft.y)) / 32767
  );
  updateCurveMarker(
    "sticks", "right",
    Math.max(Math.abs(rawRight.x), Math.abs(rawRight.y)) / 32767
  );
  updateCurveMarker("triggers", "left", sample.triggers.left / 65535);
  updateCurveMarker("triggers", "right", sample.triggers.right / 65535);
  const style = currentControllerStyle();
  elements.playtestLeftTriggerLabel.textContent =
    controlLabel("left_trigger", style);
  elements.playtestRightTriggerLabel.textContent =
    controlLabel("right_trigger", style);
  const pressed = new Set([...sample.buttons, ...(sample.extra_buttons || [])]);
  if (sample.triggers.left > 512) pressed.add("left_trigger");
  if (sample.triggers.right > 512) pressed.add("right_trigger");
  elements.controllerCanvas.querySelectorAll("[data-controller-button]")
    .forEach((button) => {
      button.classList.toggle(
        "pressed", state.presentation.liveDiagram && pressed.has(button.dataset.controllerButton)
      );
    });
  elements.playtestExtraInputs.textContent =
    (sample.extra_buttons || []).map(button => sourceLabel(button)).join(", ") || "None";
  elements.playtestMappedButtons.textContent =
    mapped.buttons.map((button) => outputLabel(button, style)).join(", ") || "None";
  elements.playtestMappingHelp.textContent =
    `${state.playtestShift.active ? "Shift" : "Base"} layer preview; shortcuts, macros and Turbo are not simulated.${state.profile.swap_sticks ? " Stick axes and mapped clicks are swapped; amber dots and calibration rings remain physical inputs, blue dots are output channels." : ""}${state.profile.native_joycon_layout !== "paired" ? " Native solo uses mapped left axes/click and the selected side's rails; no separate D-pad. Axes show logical output before native wire rotation." : ""}${state.presentation.liveDiagram ? "" : " Diagram is a reference/preview; physical highlighting is off."} Digital stick directions apply only when the mapped left analog stick is neutral within its deadzone (any nonzero analog vector takes priority).`;
  elements.playtestPanel.dataset.state = "live";
  elements.playtestStatus.textContent = "Live";
  const owner = currentOwner();
  state.identifyAvailable = Boolean(
    owner && owner.index !== 0 && sample.identity_key === owner.key &&
    sample.capabilities?.some((capability) =>
      ["rumble", "lightbar", "player_leds"].includes(capability)
    )
  );
  elements.identify.disabled = state.busy || !state.identifyAvailable;
  elements.playtestTitle.textContent = sample.label || "Connected controller";
  const details = [
    sample.controller?.model,
    sample.identity?.transport,
    sample.battery === null ? null : `${sample.battery}% battery`,
    ...(sample.capabilities || []).map(label),
  ].filter(Boolean);
  elements.controllerDetails.textContent = details.join(" · ");
  elements.playtestHelp.textContent =
    "Yellow is raw input; blue is the output produced by this unsaved draft.";
  renderCapture();
  if (wiiOrientation.applying) {
    const submittedGen = wiiOrientation.submittedGeneration;
    const nextGen = ((submittedGen + 1) >>> 0);
    const gen = sample.connection_generation;
    const desired = wiiOrientation.desiredLayout;
    const matchesOwner = owner && wiiOrientation.ownerKey === owner.key;
    if (matchesOwner && sample.layout === desired && (gen === submittedGen || gen === nextGen)) {
      window.clearTimeout(wiiOrientation.applyTimer);
      wiiOrientation.applying = false;
      wiiOrientation.submittedGeneration = null;
      wiiOrientation.desiredLayout = null;
      wiiOrientation.ownerKey = null;
      wiiOrientation.current = desired === "wii-horizontal" ? "horizontal" : "vertical";
      wiiOrientation.selected = wiiOrientation.current;
      wiiOrientation.pending = false;
      wiiOrientation.applyError = "";
      setBusy(false);
      toast(`${label(wiiOrientation.current)} orientation applied.`);
    } else if (!matchesOwner || (gen !== submittedGen && gen !== nextGen)) {
      window.clearTimeout(wiiOrientation.applyTimer);
      wiiOrientation.applying = false;
      wiiOrientation.submittedGeneration = null;
      wiiOrientation.desiredLayout = null;
      wiiOrientation.ownerKey = null;
      resetWiiOrientationDraft();
      setBusy(false);
    }
  }
  if (sample.layout === "wii-horizontal" || sample.layout === "wii-vertical") {
    const confirmed = sample.layout === "wii-horizontal" ? "horizontal" : "vertical";
    wiiOrientation.current = confirmed;
    if (!wiiOrientation.pending && !wiiOrientation.applying) {
      wiiOrientation.selected = confirmed;
    }
  }
  renderWiiOrientation();
}

async function pollPlaytest() {
  window.clearTimeout(state.playtestTimer);
  if (
    document.hidden || (state.busy && !wiiOrientation.applying) || state.playtestRequestActive ||
    captureBlocking() || macroCapture.requestActive || !state.schema || !state.profile
  ) {
    state.playtestTimer = window.setTimeout(pollPlaytest, 250);
    return;
  }
  const identityIndex = state.identityIndex;
  const profileIndex = state.profileIndex;
  state.playtestRequestActive = true;
  try {
    const sample = await api(
      `/api/profiles/${identityIndex}/${profileIndex + 1}/playtest`
    );
    if (
      !captureBlocking() && identityIndex === state.identityIndex &&
      profileIndex === state.profileIndex
    ) {
      renderPlaytest(sample);
    }
  } catch (error) {
    if (
      identityIndex === state.identityIndex &&
      profileIndex === state.profileIndex
    ) {
      clearPlaytest(
        `Live input unavailable: ${error.message}`,
        "error"
      );
    }
  } finally {
    state.playtestRequestActive = false;
    renderCapture();
    state.playtestTimer = window.setTimeout(pollPlaytest, 75);
  }
}

function isDirty() {
  return state.profile !== null &&
    (canonical(state.profile) !== state.original || state.pendingName);
}

function setConnection(mode, text) {
  if (mode === "ready" && document.hidden) {
    mode = "loading";
    text = "Connection check paused";
  }
  const wasConnected = state.adapterConnected;
  state.adapterConnected = mode === "ready";
  if (wasConnected && !state.adapterConnected) {
    stopMacroPreview("Preview stopped: adapter disconnected.");
    if (captureBlocking()) {
      stopCapture("Adapter disconnected; retaining the last received recording.");
    }
    clearPlaytest("Adapter connection unavailable. Your draft is retained.", "error");
  }
  elements.connection.dataset.state = mode;
  elements.connectionText.textContent = text;
  updateDeviceActions();
  renderCapture();
}

function updateDeviceActions() {
  const unavailable = state.busy || captureBlocking() || !state.adapterConnected;
  elements.save.disabled = unavailable || !isDirty();
  elements.activate.disabled = unavailable || !state.profile || state.active;
  elements.copyProfile.disabled = unavailable || !state.profile;
  elements.confirmCopy.disabled = unavailable || !state.profile;
  elements.saveProfileName.disabled = unavailable || !state.profile;
  elements.saveAlias.disabled = unavailable || currentOwner()?.index === 0;
  elements.identify.disabled = unavailable || !state.identifyAvailable;
  renderJoyconMode();
  renderWiiOrientation();
}

let toastTimer = 0;
function toast(message, error = false) {
  window.clearTimeout(toastTimer);
  elements.toast.textContent = message;
  elements.toast.classList.toggle("error", error);
  elements.toast.classList.add("show");
  toastTimer = window.setTimeout(() => elements.toast.classList.remove("show"), 3600);
}

async function api(path, options = {}) {
  const headers = { ...(options.headers || {}) };
  if (options.method && options.method !== "GET") {
    headers["X-Switch-Pico-Token"] = state.token;
  }
  let response;
  try {
    response = await fetch(path, { ...options, headers });
  } catch (error) {
    setConnection("error", "Editor server unavailable");
    throw error;
  }
  // USB failures are connection failures, not unsupported playtest features.
  // Ordinary 400 validation/feature errors must not disconnect a healthy Pico.
  if (response.status === 503) setConnection("error", "Adapter unavailable");
  let payload;
  try {
    payload = await response.json();
  } catch {
    throw new Error(`Unexpected server response (${response.status})`);
  }
  if (!response.ok) {
    throw new Error(payload.error || `Request failed (${response.status})`);
  }
  return payload;
}
function joyconModeChanged() {
  return joyconMode.selected !== null && joyconMode.selected !== joyconMode.saved;
}

function renderJoyconMode() {
  const locked = state.busy || captureBlocking() || macroCapture.requestActive ||
    joyconMode.loading || joyconMode.applying;
  elements.joyconMode.disabled = locked || !joyconMode.supported || !state.adapterConnected;
  elements.applyJoyconMode.disabled = elements.joyconMode.disabled ||
    Boolean(joyconMode.readError) || !joyconModeChanged();
  if (joyconMode.selected !== null && elements.joyconMode.value !== joyconMode.selected) {
    elements.joyconMode.value = joyconMode.selected;
  }
  let status;
  let mode;
  const saved = joyconMode.saved === null ? "" :
    `Saved default: ${label(joyconMode.saved)}.`;
  const selection = joyconModeChanged()
    ? ` Selection: ${label(joyconMode.selected)} — not applied.` : "";
  if (joyconMode.applying) {
    mode = "loading";
    status = `Saving ${label(joyconMode.selected)} default… Your profile draft is unchanged.`;
  } else if (joyconMode.loading) {
    mode = "loading";
    status = `Reading saved preference… ${saved}${selection}`;
  } else if (joyconMode.applyError || joyconMode.readError) {
    mode = "error";
    status = `${joyconMode.applyError || joyconMode.readError} ${saved ? `Last confirmed ${saved.toLowerCase()}` : "Saved preference is unknown."}${selection}`;
  } else if (joyconMode.saved === null) {
    mode = "error";
    status = "Saved preference is unknown. Refresh to read it from the adapter.";
  } else if (!joyconMode.supported) {
    mode = "unsupported";
    status = `Paired is the legacy default. This firmware cannot save player mode; update the adapter firmware (configuration schema 4 required).${selection}`;
  } else if (!state.adapterConnected) {
    mode = "error";
    status = `Adapter unavailable. Last confirmed ${saved.toLowerCase()}${selection}`;
  } else {
    mode = joyconModeChanged() ? "pending" : "saved";
    status = `${saved}${selection} Shortcuts can override these connections. Use live playtest to check the current arrangement.`;
  }
  elements.joyconModeStatus.dataset.state = mode;
  if (elements.joyconModeStatus.textContent !== status) {
    elements.joyconModeStatus.textContent = status;
  }
}

function acceptJoyconMode(payload, committed = false) {
  joyconMode.saved = payload.mode;
  joyconMode.supported = payload.supported;
  joyconMode.generation = payload.generation;
  if (committed || !joyconMode.pending) joyconMode.selected = payload.mode;
  joyconMode.pending = joyconModeChanged();
  joyconMode.readError = "";
  if (joyconMode.selected === payload.mode) joyconMode.applyError = "";
}

async function refreshJoyconMode(showLoading = false) {
  if (joyconMode.requestActive || joyconMode.applying) return;
  const revision = joyconMode.revision;
  joyconMode.requestActive = true;
  joyconMode.loading = showLoading || joyconMode.saved === null;
  renderJoyconMode();
  try {
    const payload = await api("/api/joycon-mode");
    if (revision === joyconMode.revision) acceptJoyconMode(payload);
  } catch (error) {
    if (revision === joyconMode.revision) {
      joyconMode.readError = `Could not read player mode: ${error.message}`;
    }
  } finally {
    joyconMode.requestActive = false;
    joyconMode.loading = false;
    renderJoyconMode();
  }
}

function isWiiController(owner = currentOwner(), live = matchingLiveSample() ? state.liveSample : null) {
  const vid = owner?.identity?.vendor_id;
  const pid = owner?.identity?.product_id;
  const isWiiPid = vid === 0x057E && (pid === 0x0306 || pid === 0x0330);
  const liveLayout = live?.layout;
  const isWiiLayout = liveLayout === "wii-remote" || liveLayout === "wii-nunchuk" ||
    liveLayout === "wii-horizontal" || liveLayout === "wii-vertical";
  return Boolean(isWiiPid || isWiiLayout);
}

function renderWiiOrientation() {
  const owner = currentOwner();
  const live = matchingLiveSample() ? state.liveSample : null;
  const visible = isWiiController(owner, live);
  elements.wiiOrientationControl.hidden = !visible;
  if (!visible) return;

  const isNunchuk = live?.layout === "wii-nunchuk";
  const isLegacy = live?.layout === "wii-remote";
  const isSupported = Boolean(
    live && (live.layout === "wii-horizontal" || live.layout === "wii-vertical") &&
    owner && owner.index !== 0
  );
  const locked = (state.busy && !wiiOrientation.applying) || captureBlocking() ||
    macroCapture.requestActive || wiiOrientation.applying || !state.adapterConnected || !live;
  if (isNunchuk) {
    elements.wiiOrientation.value = "vertical";
    elements.wiiOrientation.disabled = true;
    elements.applyWiiOrientation.disabled = true;
    elements.wiiOrientationStatus.dataset.state = "locked";
    elements.wiiOrientationStatus.textContent = "Nunchuk attached: orientation is locked to vertical.";
    return;
  }

  if (!live) {
    elements.wiiOrientation.disabled = true;
    elements.applyWiiOrientation.disabled = true;
    elements.wiiOrientationStatus.dataset.state = "idle";
    elements.wiiOrientationStatus.textContent = "";
    return;
  }

  if (isLegacy) {
    elements.wiiOrientation.disabled = true;
    elements.applyWiiOrientation.disabled = true;
    elements.wiiOrientationStatus.dataset.state = "unsupported";
    elements.wiiOrientationStatus.textContent = "Legacy firmware detected. Update firmware to configure Wii orientation.";
    return;
  }

  if (owner?.index === 0) {
    elements.wiiOrientation.disabled = true;
    elements.applyWiiOrientation.disabled = true;
    elements.wiiOrientationStatus.dataset.state = "unsupported";
    elements.wiiOrientationStatus.textContent = "Select the connected Wii Remote instead of the default profiles to set orientation.";
    return;
  }

  elements.wiiOrientation.disabled = locked || !isSupported;
  const hasChanged = wiiOrientation.selected !== wiiOrientation.current;
  elements.applyWiiOrientation.disabled = elements.wiiOrientation.disabled ||
    wiiOrientation.applying || !hasChanged;
  if (elements.wiiOrientation.value !== wiiOrientation.selected) {
    elements.wiiOrientation.value = wiiOrientation.selected;
  }

  let statusText = "";
  let statusState = "idle";
  if (wiiOrientation.applying) {
    statusState = "pending";
    statusText = `Applying ${wiiOrientation.selected} orientation…`;
  } else if (wiiOrientation.applyError) {
    statusState = "error";
    statusText = wiiOrientation.applyError;
  } else if (hasChanged) {
    statusState = "pending";
    statusText = `Selected: ${label(wiiOrientation.selected)} — not applied.`;
  } else if (wiiOrientation.current) {
    statusState = "idle";
    statusText = `Current: ${label(wiiOrientation.current)}.`;
  }

  elements.wiiOrientationStatus.dataset.state = statusState;
  elements.wiiOrientationStatus.textContent = statusText;
}


function setBusy(busy) {
  state.busy = busy;
  busy ||= captureBlocking();
  elements.resetDraft.disabled = busy;
  elements.refresh.disabled = busy;
  elements.identity.disabled = busy;
  elements.profileName.disabled = busy;
  elements.controllerAlias.disabled =
    busy || currentOwner()?.index === 0;
  elements.importProfile.disabled = busy;
  elements.exportProfile.disabled = busy;
  if (state.schema && state.profile) updateMacroBudgets();
  document.querySelectorAll(".profile-button").forEach((button) => {
    button.disabled = busy;
  });
  updateDeviceActions();
  renderCapture();
  renderNativeLayout();
}

function updateDirtyState() {
  const dirty = isDirty();
  elements.dirtyBadge.hidden = !dirty;
  elements.save.disabled =
    !state.adapterConnected || state.busy || captureBlocking() || !dirty;
}

function confirmDiscard() {
  return !isDirty() || window.confirm("Discard the unsaved changes to this profile?");
}

function currentOwner() {
  return state.identities.find(
    (entry) => entry.index === state.identityIndex
  ) || null;
}

function storedOwnerKey() {
  try {
    return window.localStorage.getItem(PROFILE_OWNER_STORAGE_KEY);
  } catch {
    return null;
  }
}

function persistOwnerKey(key) {
  try {
    window.localStorage.setItem(PROFILE_OWNER_STORAGE_KEY, key);
  } catch {
    // Storage can be unavailable in private or hardened browser contexts.
  }
}

function identitySignature(identities) {
  return identities.map((entry) => (
    `${entry.index}:${entry.key}:${entry.label}:` +
    `${entry.controller.model}:${entry.controller.style}:${entry.controller.layout}:${entry.source_controls?.join(",")}`
  )).join("|");
}

function syncLibraryMetadata(identities, restoreStoredOwner = false) {
  const oldOwner = currentOwner();
  const oldSignature = identitySignature(state.identities);
  const preferredKey = (
    restoreStoredOwner ? storedOwnerKey() : oldOwner?.key
  ) || storedOwnerKey();
  if (oldOwner && state.profile && !identities.some((entry) => entry.key === oldOwner.key)) {
    throw new Error("The selected controller is unavailable. Your draft is retained; reconnect the controller or export the draft before choosing another.");
  }
  state.identities = identities;
  const nextOwner = (
    identities.find((entry) => entry.key === preferredKey) ||
    identities[0] ||
    null
  );
  state.identityIndex = nextOwner?.index ?? 0;
  if (nextOwner) persistOwnerKey(nextOwner.key);

  const ownerChanged = oldOwner?.key !== nextOwner?.key;
  const identitiesChanged =
    oldSignature !== identitySignature(state.identities);
  const activeChanged =
    oldOwner?.active_profile !== nextOwner?.active_profile;
  if (state.schema && (identitiesChanged || ownerChanged)) {
    renderIdentities();
  }
  if (state.schema && (activeChanged || ownerChanged)) {
    renderProfileList();
  }
  if (state.profile) {
    state.active =
      nextOwner?.active_profile === state.profileIndex + 1;
    elements.activeBadge.hidden = !state.active;
    elements.activate.disabled =
      !state.adapterConnected || state.busy || state.active;
    if (state.schema && syncControllerPresentation()) {
      renderButtonMap();
      refreshSourceControls();
    }
  }
  return ownerChanged;
}

async function pollLibraryMetadata() {
  window.clearTimeout(state.libraryTimer);
  if (
    document.hidden || state.busy || captureBlocking() || macroCapture.requestActive ||
    state.libraryRequestActive || !state.schema
  ) {
    state.libraryTimer =
      window.setTimeout(pollLibraryMetadata, 500);
    return;
  }
  state.libraryRequestActive = true;
  try {
    await refreshJoyconMode();
    const payload = await api("/api/profiles");
    syncLibraryMetadata(payload.identities);
    setConnection("ready", "Adapter connected");
  } catch {
    setConnection("error", "Adapter disconnected");
  } finally {
    state.libraryRequestActive = false;
    renderCapture();
    state.libraryTimer =
      window.setTimeout(pollLibraryMetadata, 750);
  }
}

function renderIdentities() {
  elements.identity.innerHTML = state.identities
    .map((entry) => `<option value="${entry.index}">${escapeHtml(entry.label)}</option>`)
    .join("");
  elements.identity.value = String(state.identityIndex);
  const owner = currentOwner();
  elements.profileOwner.textContent = owner ? owner.label : "No controller";
}

function renderProfileList() {
  const owner = currentOwner();
  elements.profileList.innerHTML = Array.from({ length: state.schema.profile_capacity }, (_, index) => {
    const selected = index === state.profileIndex;
    const active = owner && owner.active_profile === index + 1;
    return `
      <button class="profile-button${selected ? " selected" : ""}" type="button" data-profile-index="${index}" aria-pressed="${selected}" aria-label="Profile ${index + 1}${state.profileNames[index] ? `: ${escapeHtml(state.profileNames[index])}` : ""}${active ? ", active on Pico" : ""}">
        <span class="profile-number"><span aria-hidden="true">${index + 1}</span>${escapeHtml(state.profileNames[index] || `Profile ${index + 1}`)}</span>
        ${active ? '<span class="mini-active">Active</span>' : ""}
      </button>`;
  }).join("");

  elements.profileList.querySelectorAll(".profile-button").forEach((button) => {
    button.addEventListener("click", async () => {
      const next = Number(button.dataset.profileIndex);
      if (captureBlocking() || next === state.profileIndex || !confirmDiscard()) return;
      state.profileIndex = next;
      await loadProfile();
    });
  });
}

function buttonOptions(
  selected,
  includeNone = true,
  choices = state.schema.output_controls,
  style = currentControllerStyle(),
  noneLabel = "None"
) {
  const none = includeNone
    ? `<option value=""${selected === null ? " selected" : ""}>${escapeHtml(noneLabel)}</option>`
    : "";
  return none + choices.map((button) => (
    `<option value="${button}"${selected === button ? " selected" : ""}>${escapeHtml(outputLabel(button, style))}</option>`
  )).join("");
}

function macroSummary(macro) {
  if (!macro) return "empty";
  const steps = macro.steps || [];
  if (!steps.length) return "empty";
  const duration = steps.reduce((sum, step) => sum + (step.duration_ms || 0), 0);
  return `${steps.length} step${steps.length === 1 ? "" : "s"} · ${duration} ms`;
}

function gestureActionOptions(
  gesture,
  choices = state.schema.buttons,
  style = currentControllerStyle(),
  noneLabel = "Disabled"
) {
  const selectedButton = gesture?.button ?? null;
  const selectedMacro = gesture?.macro ?? null;
  const noneSelected = selectedButton === null && selectedMacro === null;
  const noneOption = `<option value=""${noneSelected ? " selected" : ""}>${escapeHtml(noneLabel)}</option>`;

  const buttonGroup = choices.length ? `
    <optgroup label="Output buttons">
      ${choices.map((button) => (
        `<option value="${button}"${selectedButton === button ? " selected" : ""}>${escapeHtml(outputLabel(button, style))}</option>`
      )).join("")}
    </optgroup>` : "";

  const macroList = state.profile?.macros || [];
  const macroGroup = `
    <optgroup label="Custom macros">
      ${[1, 2, 3, 4].map((macroNum) => {
        const macro = macroList[macroNum - 1];
        const summary = macroSummary(macro);
        const isSelected = selectedMacro === macroNum;
        return `<option value="macro:${macroNum}"${isSelected ? " selected" : ""}>Macro ${macroNum} (${escapeHtml(summary)})</option>`;
      }).join("")}
    </optgroup>`;

  return noneOption + buttonGroup + macroGroup;
}

function parseGestureActionValue(value) {
  if (!value) return { button: null, macro: null };
  if (value.startsWith("macro:")) {
    const macroNum = parseInt(value.slice(6), 10);
    return { button: null, macro: Number.isInteger(macroNum) ? macroNum : null };
  }
  return { button: value, macro: null };
}

function validateGestureMacros() {
  if (!elements.swing || !state.profile) return;
  const macros = state.profile.macros || [];
  const gestures = [
    { name: "Wii Remote swing", selectId: "#swing-button", gesture: state.profile.swing },
    { name: "Nunchuk swing", selectId: "#nunchuk-swing-button", gesture: state.profile.nunchuk_swing },
    { name: "Both together", selectId: "#combined-swing-button", gesture: state.profile.combined_swing },
  ];
  for (const { name, selectId, gesture } of gestures) {
    const select = elements.swing.querySelector(selectId);
    if (!select) continue;
    if (gesture && gesture.macro != null) {
      const target = macros[gesture.macro - 1];
      const steps = target?.steps || [];
      const hasPositiveDuration = steps.some((step) => step.duration_ms > 0);
      if (!steps.length || !hasPositiveDuration) {
        select.setCustomValidity(`${name} is bound to Macro ${gesture.macro}, which must contain at least one step and a positive total duration.`);
      } else {
        select.setCustomValidity("");
      }
    } else {
      select.setCustomValidity("");
    }
  }
}

function updateGestureActionSelects() {
  if (!elements.swing || !state.profile) return;
  const controllerStyle = currentControllerStyle();
  const buttons = state.schema.buttons;
  const remoteSelect = elements.swing.querySelector("#swing-button");
  if (remoteSelect && state.profile.swing) {
    remoteSelect.innerHTML = gestureActionOptions(state.profile.swing, buttons, controllerStyle, "Disabled");
  }
  const nunchukSelect = elements.swing.querySelector("#nunchuk-swing-button");
  if (nunchukSelect && state.profile.nunchuk_swing) {
    nunchukSelect.innerHTML = gestureActionOptions(state.profile.nunchuk_swing, buttons, controllerStyle, "Disabled");
  }
  const combinedSelect = elements.swing.querySelector("#combined-swing-button");
  if (combinedSelect && state.profile.combined_swing) {
    combinedSelect.innerHTML = gestureActionOptions(state.profile.combined_swing, buttons, controllerStyle, "Disabled");
  }
  validateGestureMacros();
}

function modeOptions(modes, selected) {
  return modes.map((mode) =>
    `<option value="${mode}"${mode === selected ? " selected" : ""}>${label(mode)}</option>`
  ).join("");
}

function modifierOptions(selected, excluded = []) {
  return sourceOptions(selected, state.schema.controls, excluded);
}

function renderShortcuts() {
  const shortcuts = state.profile.shortcuts;
  elements.shortcutModifier.innerHTML = `
    <div class="control-card">
      <label for="shortcut-modifier">Shortcut modifier</label>
      <select class="select" id="shortcut-modifier" data-kind="shortcut-modifier" aria-describedby="shortcut-modifier-help">${modifierOptions(shortcuts.modifier, shortcuts.profiles)}</select>
      <p class="field-help" id="shortcut-modifier-help">None clears every shortcut. Switch 2 extra inputs can act as modifiers. Trigger modifiers are available for recognized Xbox, PlayStation and Switch 2 controllers.</p>
    </div>`;
  elements.shortcuts.innerHTML = shortcuts.profiles.map((selector, index) => `
    <div class="control-card">
      <label for="shortcut-${index}">Profile ${index + 1}${state.profileNames[index] ? ` · ${escapeHtml(state.profileNames[index])}` : ""}</label>
      <select class="select" id="shortcut-${index}" data-kind="shortcut-selector" data-index="${index}"${shortcuts.modifier === null ? " disabled" : ""}>
        ${sourceOptions(selector, state.schema.shortcut_selectors,
          [shortcuts.modifier, ...shortcuts.profiles.filter((_, other) => other !== index)])}
      </select>
    </div>`).join("");
}

function renderShift() {
  const shift = state.profile.shift;
  elements.shift.innerHTML = `
    <div class="control-card">
      <label for="shift-mode">Shift mode</label>
      <select class="select" id="shift-mode" data-kind="shift-mode">${modeOptions(state.schema.shift_modes, shift.mode)}</select>
    </div>
    <div class="control-card">
      <label for="shift-modifier">Shift modifier</label>
      <select class="select" id="shift-modifier" data-kind="shift-modifier"${shift.mode === "off" ? " disabled" : ""} aria-describedby="shift-modifier-help">${modifierOptions(shift.modifier)}</select>
      <p class="field-help" id="shift-modifier-help">Hold uses the alternate map while pressed. Toggle switches layers on each fresh press and resets when the profile or mode changes. Analog modifiers follow the same availability as shortcuts.</p>
    </div>`;
  elements.shiftMap.innerHTML = [...state.schema.buttons, ...state.schema.extra_buttons].map((button) => {
    const map = state.schema.extra_buttons.includes(button) ? shift.extra_button_map : shift.button_map;
    return `
    <div class="control-card" data-source-card="${button}"${sourceAvailable(button) ? "" : " hidden"}>
      <label for="shift-map-${button}"><span data-source-label="${button}">${escapeHtml(sourceLabel(button))}</span> → alternate output</label>
      <select class="select" id="shift-map-${button}" data-kind="shift-map" data-name="${button}"${shift.mode === "off" || shift.modifier === null ? " disabled" : ""}>${buttonOptions(map[button])}</select>
    </div>`;
  }).join("");
  updateShiftValidity();
}

function updateShiftValidity() {
  const modifier = elements.shift.querySelector("#shift-modifier");
  modifier.setCustomValidity(state.profile.shift.mode !== "off" && state.profile.shift.modifier === null
    ? "Choose a modifier for Hold or Toggle, or turn Shift off." : "");
}


const controllerGlyphs = {
  generic: {
    north: "Y", east: "B", south: "A", west: "X",
    left_shoulder: "LB", right_shoulder: "RB",
    left_trigger: "LT", right_trigger: "RT",
    select: "⧉", start: "☰", capture: "↥", system: "X",
  },
  xbox: {
    north: "Y", east: "B", south: "A", west: "X",
    left_shoulder: "LB", right_shoulder: "RB",
    left_trigger: "LT", right_trigger: "RT",
    select: "⧉", start: "☰", capture: "↥", system: "X",
  },
  switch: {
    north: "X", east: "A", south: "B", west: "Y",
    left_shoulder: "L", right_shoulder: "R",
    left_trigger: "ZL", right_trigger: "ZR",
    select: "−", start: "+", capture: "▣", system: "⌂",
  },
  playstation: {
    north: "△", east: "○", south: "×", west: "□",
    left_shoulder: "L1", right_shoulder: "R1",
    left_trigger: "L2", right_trigger: "R2",
    select: "⧉", start: "☰", capture: "TP", system: "PS",
  },
};

const fixedControlGlyphs = {
  dpad_up: "▲",
  dpad_right: "▶",
  dpad_down: "▼",
  dpad_left: "◀",
  left_stick: "L3",
  right_stick: "R3",
  left_stick_up: "L▲",
  left_stick_down: "L▼",
  left_stick_left: "L◀",
  left_stick_right: "L▶",
  select: "−",
  start: "+",
  capture: "▣",
  system: "⌂",
  c: "C", gl: "GL", gr: "GR",
  left_sl: "L SL", left_sr: "L SR", right_sl: "R SL", right_sr: "R SR",
};

function controlGlyph(button, style) {
  return controllerGlyphs[style]?.[button] || fixedControlGlyphs[button] || "?";
}

function getControlMapping(button) {
  if (button === "left_trigger") return state.profile.triggers.left.output;
  if (button === "right_trigger") return state.profile.triggers.right.output;
  if (state.schema.extra_buttons.includes(button)) return state.profile.extra_button_map[button];
  return state.profile.button_map[button];
}

function setControlMapping(button, output) {
  const triggerSides = {
    left_trigger: "left",
    right_trigger: "right",
  };
  const side = triggerSides[button];
  if (side) {
    const otherSide = side === "left" ? "right" : "left";
    if (
      (output === "left_trigger" || output === "right_trigger") &&
      state.profile.triggers[otherSide].output === output
    ) {
      state.profile.triggers[otherSide].output = button;
    }
    state.profile.triggers[side].output = output;
  } else if (state.schema.extra_buttons.includes(button)) {
    state.profile.extra_button_map[button] = output;
  } else {
    state.profile.button_map[button] = output;
  }
}


function renderButtonMap() {
  syncControllerPresentation();
  const { layout, layoutId, sources, preview, liveDiagram, live, topologyKnown } = state.presentation;
  const style = currentControllerStyle();
  const selected = state.selectedButton;
  const mappedOutput = getControlMapping(selected);
  elements.controllerCanvas.dataset.style = style;
  elements.controllerCanvas.dataset.layout = layoutId;
  elements.controllerLayoutStatus.dataset.state = liveDiagram ? "live" : "preview";
  elements.controllerLayoutStatus.textContent = preview ? "Manual layout preview" :
    liveDiagram ? "Live · selected controller" : live ? "Live input · reference diagram" : "Offline · controller reference";
  elements.controllerModel.textContent = layout.name;
  elements.controllerLayoutHelp.textContent = preview
    ? `Layout preview only${live ? `; connected input is ${live.controller.model}` : ""}. The controller setup and saved mappings are unchanged. Physical highlighting is off.`
    : layout.referenceOnly
      ? "Wii-family reference only: orientation and extensions are unknown. Connect the selected controller for its detected layout. No mappings are changed or hidden."
    : live && !topologyKnown
      ? layoutId === "wii-remote"
        ? "Legacy firmware does not report Wii orientation. This reference shows the default horizontal mapping; physical highlighting is off. Choose a preview or update firmware."
        : "Legacy firmware does not report paired or solo Joy-Con mode. This diagram is a controller reference, not a detected solo layout. Choose a preview or update firmware."
      : "Auto uses the selected controller's details. Preview changes source labels and the diagram only, never the controller setup or saved mappings.";
  elements.controllerLayoutNote.textContent =
    !preview && live?.layout === "joycon2-pair" && !live.identity?.is_joycon_pair
      ? "This older firmware uses an individual controller's profiles for paired input. There are no separate L+R profiles; edits still affect the selected controller. Update firmware for separate pair profiles. The grip is illustrative, not detected."
      : layout.note ||
        (layout.generic ? "This is a generic reference, not an identified controller model. Additional reported inputs are listed separately from the diagram." :
          "Rear triggers are listed below the front view. Saved mappings for unavailable controls are retained.");
  const diagramKey = `${layoutId}:${sources.join(",")}`;
  if (diagramKey !== state.diagramKey) {
    const focusedSource = elements.controllerCanvas.contains(document.activeElement)
      ? document.activeElement.dataset.controllerButton : null;
    state.diagramKey = diagramKey;
    const [x, y, width, height] = layout.viewBox;
    const angle = layout.rotation || 0;
    const rotated = Math.abs(angle) === 90;
    elements.controllerPhotoWrap.style.setProperty("--diagram-min", `${layout.minWidth}px`);
    elements.controllerPhotoWrap.style.setProperty("--diagram-max", `${layout.maxWidth || Math.max(layout.minWidth, 900)}px`);
    elements.controllerPhotoWrap.style.setProperty("--diagram-ratio", rotated ? `${height} / ${width}` : `${width} / ${height}`);
    elements.controllerStage.style.setProperty("--stage-width", `${rotated ? width / height * 100 : 100}%`);
    elements.controllerStage.style.setProperty("--stage-height", `${rotated ? height / width * 100 : 100}%`);
    elements.controllerStage.style.setProperty("--stage-angle", `${angle}deg`);
    elements.controllerStage.style.setProperty("--label-angle", `${-angle}deg`);
    elements.controllerImage.src = `/assets/${layout.asset}`;
    elements.controllerImage.alt = `${layout.name} · supplied front-view illustration`;
    elements.controllerHotspots.replaceChildren();
    elements.extraControls.replaceChildren();
    for (const id of sources) {
      const control = layout.controls[id];
      const hotspot = document.createElement("button");
      hotspot.type = "button";
      hotspot.dataset.controllerButton = id;
      if (control && !control.offArt) {
        hotspot.style.setProperty("--x", `${(control.x - x) / width * 100}%`);
        hotspot.style.setProperty("--y", `${(control.y - y) / height * 100}%`);
        elements.controllerHotspots.append(hotspot);
      } else {
        elements.extraControls.append(hotspot);
      }
    }
    for (const note of layout.annotations || []) {
      const annotation = document.createElement("span");
      annotation.className = "controller-annotation";
      annotation.textContent = note.label;
      annotation.style.setProperty("--x", `${(note.x - x) / width * 100}%`);
      annotation.style.setProperty("--y", `${(note.y - y) / height * 100}%`);
      elements.controllerHotspots.append(annotation);
    }
    if (focusedSource) {
      const replacement = [...elements.controllerCanvas.querySelectorAll("[data-controller-button]")]
        .find(button => button.dataset.controllerButton === focusedSource);
      (replacement || elements.selectedSource).focus({ preventScroll: true });
    }
  }
  elements.controllerCanvas.querySelectorAll("[data-controller-button]").forEach(hotspot => {
    const button = hotspot.dataset.controllerButton;
    const output = getControlMapping(button);
    hotspot.textContent = hotspot.parentElement === elements.extraControls ? sourceLabel(button) : sourceGlyph(button);
    hotspot.classList.toggle("selected", button === selected);
    hotspot.classList.toggle("disabled-map", output == null);
    if (!liveDiagram) hotspot.classList.remove("pressed");
    hotspot.setAttribute("aria-pressed", String(button === selected));
    hotspot.title = `${sourceLabel(button)} → ${output == null ? "Disabled" : outputLabel(output)}`;
    hotspot.setAttribute("aria-label", hotspot.title);
  });
  elements.selectedSource.innerHTML = sourceOptions(selected);
  elements.selectedSource.querySelector('option[value=""]')?.remove();
  elements.selectedControlGlyph.textContent = sourceGlyph(selected);
  elements.selectedControlName.textContent = sourceLabel(selected);
  elements.selectedControlDescription.textContent =
    `${sourceLabel(selected)} produces ${mappedOutput == null ? "no output" : outputLabel(mappedOutput)}.${sourceAvailable(selected) ? "" : " This control is unavailable in the current layout; its saved mapping is retained."}`;
  elements.selectedMapping.innerHTML = buttonOptions(mappedOutput, true, state.schema.output_controls, style);
  renderWiiOrientation();
}

elements.controllerCanvas.addEventListener("click", event => {
  const hotspot = event.target.closest("[data-controller-button]");
  if (!hotspot || state.busy || captureBlocking()) return;
  state.selectedButton = hotspot.dataset.controllerButton;
  stopMacroPreview("Preview stopped: control selection changed.");
  renderButtonMap();
});
elements.selectedSource.addEventListener("change", () => {
  if (state.busy || captureBlocking()) return;
  state.selectedButton = elements.selectedSource.value;
  renderButtonMap();
});
elements.selectedMapping.addEventListener("change", () => {
  if (state.busy || captureBlocking()) return;
  setControlMapping(state.selectedButton, elements.selectedMapping.value || null);
  renderButtonMap();
  updateDirtyState();
});
elements.controllerLayoutPreview.addEventListener("change", () => {
  if (state.busy || captureBlocking()) return;
  state.layoutPreview = elements.controllerLayoutPreview.value;
  renderButtonMap();
  refreshSourceControls();
});

elements.nativeJoyconLayout.addEventListener("change", () => {
  if (state.busy || captureBlocking() || !state.profile) return;
  state.profile.native_joycon_layout = elements.nativeJoyconLayout.value;
  stopMacroPreview("Preview stopped: draft edited.");
  renderNativeLayout();
  renderButtonMap();
  refreshSourceControls();
  updateDirtyState();
  if (state.liveSample) renderPlaytest(state.liveSample);
});
elements.mapShoulders.addEventListener("click", () => {
  if (state.busy || captureBlocking() || !state.profile) return;
  const layout = state.profile.native_joycon_layout;
  if (layout !== "left_solo" && layout !== "right_solo") return;
  const side = layout === "left_solo" ? "left" : "right";
  state.profile.button_map.left_shoulder = `${side}_sl`;
  state.profile.button_map.right_shoulder = `${side}_sr`;
  stopMacroPreview("Preview stopped: draft edited.");
  renderButtonMap();
  updateDirtyState();
  if (state.liveSample) renderPlaytest(state.liveSample);
  toast(`Base shoulders mapped to ${label(side)} SL/SR in the unsaved draft. Other mappings are unchanged.`);
});
if (elements.mapDpad) {
  elements.mapDpad.addEventListener("click", () => {
    if (state.busy || captureBlocking() || !state.profile) return;
    state.profile.button_map.dpad_up = "left_stick_up";
    state.profile.button_map.dpad_down = "left_stick_down";
    state.profile.button_map.dpad_left = "left_stick_left";
    state.profile.button_map.dpad_right = "left_stick_right";
    stopMacroPreview("Preview stopped: draft edited.");
    renderButtonMap();
    updateDirtyState();
    if (state.liveSample) renderPlaytest(state.liveSample);
    toast("Base D-pad mapped to left stick movement (Up, Down, Left, Right) in the unsaved draft. Other mappings are unchanged.");
  });
}

// Patch source controls in place on topology changes. Never rerender the editor,
// stop macro playback, replace focused inputs, or mutate hidden draft values.
function refreshSourceControls() {
  if (!state.profile) return;
  elements.builtinActions.dataset.controllerStyle = currentControllerStyle();
  elements.macroControls.dataset.controllerStyle = currentControllerStyle();
  const shortcuts = state.profile.shortcuts;
  const modifiers = [
    [elements.shortcutModifier.querySelector("select"), shortcuts.modifier, shortcuts.profiles],
    [elements.shift.querySelector("#shift-modifier"), state.profile.shift.modifier, []],
    [elements.swing?.querySelector("#swing-modifier"), state.profile.swing?.modifier, []],
    [elements.swing?.querySelector("#nunchuk-swing-modifier"), state.profile.nunchuk_swing?.modifier, []],
    [elements.swing?.querySelector("#combined-swing-modifier"), state.profile.combined_swing?.modifier, []],
    [elements.macroControls.querySelector("#macro-cancel"), state.profile.macros[state.selectedMacro].cancel, []],
  ];
  for (const [select, selected, excluded] of modifiers) {
    if (select) select.innerHTML = sourceOptions(selected, state.schema.controls, excluded);
  }
  elements.shortcuts.querySelectorAll("select").forEach((select, index) => {
    select.innerHTML = sourceOptions(shortcuts.profiles[index], state.schema.shortcut_selectors,
      [shortcuts.modifier, ...shortcuts.profiles.filter((_, other) => other !== index)]);
  });
  document.querySelectorAll("[data-source-card]").forEach(card => {
    const available = sourceAvailable(card.dataset.sourceCard);
    card.hidden = !available && !card.contains(document.activeElement);
    card.classList.toggle("source-unavailable", !available);
  });
  document.querySelectorAll("[data-source-label]").forEach(node => {
    const id = node.dataset.sourceLabel;
    node.textContent = sourceLabel(id) + (sourceAvailable(id) ? "" : " (stored / unavailable)");
  });
  document.querySelectorAll('[data-kind="action-chord"]').forEach(input => {
    const id = input.dataset.name;
    const card = input.closest("label");
    card.hidden = !sourceAvailable(id) && !input.checked && input !== document.activeElement;
    card.classList.toggle("source-unavailable", !sourceAvailable(id));
    card.querySelector("b").textContent = sourceGlyph(id);
    card.querySelector("small").textContent = sourceLabel(id) + (sourceAvailable(id) ? "" : " (stored / unavailable)");
  });
  elements.shiftMap.querySelectorAll("select").forEach(select => {
    const map = state.schema.extra_buttons.includes(select.dataset.name)
      ? state.profile.shift.extra_button_map : state.profile.shift.button_map;
    select.innerHTML = buttonOptions(map[select.dataset.name]);
  });
  document.querySelectorAll("[data-output-label]").forEach(node => {
    node.textContent = outputLabel(node.dataset.outputLabel);
  });
  updateGestureActionSelects();
}

elements.form.addEventListener("focusout", event => {
  const card = event.target.closest("[data-source-card]");
  if (card && !sourceAvailable(card.dataset.sourceCard) && !card.contains(event.relatedTarget)) {
    card.hidden = true;
  }
});

const analogDefinitions = [
  ["sticks", "left", "Left stick", [
    ["center_x", "Center X", -32768, 32767],
    ["center_y", "Center Y", -32768, 32767],
    ["inner_deadzone", "Inner deadzone", 0, 32767],
    ["outer_saturation", "Outer saturation", 1, 32767],
  ]],
  ["sticks", "right", "Right stick", [
    ["center_x", "Center X", -32768, 32767],
    ["center_y", "Center Y", -32768, 32767],
    ["inner_deadzone", "Inner deadzone", 0, 32767],
    ["outer_saturation", "Outer saturation", 1, 32767],
  ]],
  ["triggers", "left", "Left trigger", [
    ["lower_deadzone", "Lower deadzone", 0, 65535],
    ["upper_saturation", "Upper saturation", 1, 65535],
    ["digital_threshold", "Digital threshold", 0, 65535],
  ]],
  ["triggers", "right", "Right trigger", [
    ["lower_deadzone", "Lower deadzone", 0, 65535],
    ["upper_saturation", "Upper saturation", 1, 65535],
    ["digital_threshold", "Digital threshold", 0, 65535],
  ]],
];

const curvePresets = {
  quick: 128,
  linear: 256,
  precise: 512,
  deliberate: 768,
};

function curvePreset(value) {
  return Object.entries(curvePresets)
    .find(([, preset]) => preset === value)?.[0] || "custom";
}

function curvePath(curve) {
  return Array.from({ length: 21 }, (_, index) => {
    const input = Math.round(index / 20 * 65535);
    const output = ProfilePlaytestMath.transformTrigger(input, {
      lower_deadzone: 0,
      upper_saturation: 65535,
      curve_q8_8: curve,
    });
    return `${index === 0 ? "M" : "L"} ${index * 5} ${60 - output / 65535 * 60}`;
  }).join(" ");
}

function renderAnalog() {
  elements.swapSticks.checked = Boolean(state.profile.swap_sticks);
  elements.analog.innerHTML = analogDefinitions.map(([group, side, title, fields]) => {
    const config = state.profile[group][side];
    const toggles = group === "sticks" ? `
      <div class="toggle-row">
        ${["invert_x", "invert_y"].map((field) => `
          <label class="checkbox-pill">
            <input type="checkbox" data-kind="analog-bool" data-group="${group}" data-side="${side}" data-field="${field}"${config[field] ? " checked" : ""}>
            <span>${label(field)}</span>
          </label>`).join("")}
      </div>` : "";
    return `
      <div class="subpanel" data-analog-group="${group}" data-analog-side="${side}">
        <div class="subpanel-heading"><h4>${title}</h4><span>${group === "sticks" ? "Signed axes · 32767 full scale" : "Unsigned · 65535 full scale"}</span></div>
        <div class="curve-editor">
          <svg viewBox="0 0 100 60" role="img" aria-label="${title} response curve">
            <path class="curve-guide" d="M 0 60 L 100 0"></path>
            <path class="curve-line" d="${curvePath(config.curve_q8_8)}"></path>
            <circle class="curve-marker" cx="0" cy="60" r="3"></circle>
          </svg>
          <div class="curve-controls">
            <label>Response preset
              <select class="select" data-kind="curve-preset" data-group="${group}" data-side="${side}">
                ${Object.keys(curvePresets).map((preset) => `<option value="${preset}"${curvePreset(config.curve_q8_8) === preset ? " selected" : ""}>${label(preset)}</option>`).join("")}
                <option value="custom"${curvePreset(config.curve_q8_8) === "custom" ? " selected" : ""}>Custom</option>
              </select>
            </label>
            <label>Fine adjustment <output>${config.curve_q8_8}</output>
              <input type="range" min="1" max="2048" step="1" value="${config.curve_q8_8}" data-kind="curve-range" data-group="${group}" data-side="${side}">
            </label>
            <button class="button button-ghost copy-side" type="button" data-copy-analog="${group}" data-source-side="${side}">Apply to ${side === "left" ? "right" : "left"}</button>
          </div>
        </div>
        <div class="number-grid">
          ${fields.map(([field, fieldLabel, min, max]) => `
            <div class="number-field${field === "digital_threshold" ? " wide" : ""}">
              <label for="${group}-${side}-${field}">${fieldLabel}</label>
              <input class="number-input" id="${group}-${side}-${field}" type="number" required min="${min}" max="${max}" step="1" value="${config[field]}" data-kind="analog-number" data-group="${group}" data-side="${side}" data-field="${field}">
            </div>`).join("")}
          ${toggles}
        </div>
      </div>`;
  }).join("");
}

function renderFeedback() {
  const rumble = state.profile.rumble;
  elements.rumble.innerHTML = `
    <div class="subpanel-heading"><h4>Rumble &amp; confirmation</h4><span>0 is silent · 255 is full strength</span></div>
    ${["weak_scale", "strong_scale"].map((field) => `
      <div class="range-row">
        <div class="range-meta"><label for="rumble-${field}">${label(field)}</label><output id="rumble-${field}-value">${rumble[field]}</output></div>
        <input id="rumble-${field}" type="range" min="0" max="255" step="1" value="${rumble[field]}" data-kind="rumble-range" data-field="${field}">
      </div>`).join("")}
    <div class="number-field">
      <label for="confirmation-policy">Profile-change confirmation</label>
      <select class="select" id="confirmation-policy" data-kind="rumble-policy">
        ${state.schema.rumble_policies.map((policy) => `<option value="${policy}"${rumble.confirmation_policy === policy ? " selected" : ""}>${label(policy)}</option>`).join("")}
      </select>
    </div>`;
}

function turboSettingFields(settings, button = null, disabled = false) {
  const titles = { rate_hz: "Rate (Hz)", duty_percent: "Duty (%)", burst_count: "Burst pulses" };
  return Object.entries(state.schema.turbo_settings_bounds).map(([field, bounds]) => {
    const id = `turbo-setting-${button || "defaults"}-${field}`;
    const inactive = disabled || (button !== null && field === "burst_count" && state.profile.turbo[button] !== "burst");
    return `<div class="number-field">
      <label for="${id}">${titles[field]} · ${bounds.min}–${bounds.max}</label>
      <input class="number-input" id="${id}" type="number" required min="${bounds.min}" max="${bounds.max}" step="1" value="${settings[field]}" data-kind="turbo-setting" data-name="${button || ""}" data-field="${field}"${inactive ? " disabled" : ""}>
    </div>`;
  }).join("");
}

function renderTurbo() {
  const settings = state.profile.turbo_settings;
  elements.turboDefaults.innerHTML = `
    <div class="subpanel-heading"><h4>Shared defaults</h4><span>Settings apply to physical buttons, before either button map.</span></div>
    <div class="turbo-setting-grid">${turboSettingFields(settings.defaults)}</div>
    <p class="field-help">Duty is the ON portion of each cycle. Burst count applies only to Burst mode. Enable a per-button override to replace all three defaults for that button.</p>`;
  elements.turbo.innerHTML = state.schema.buttons.map((button) => {
    const override = settings.overrides[button];
    const off = state.profile.turbo[button] === "off";
    return `<div class="control-card turbo-card" data-turbo-button="${button}" data-source-card="${button}"${sourceAvailable(button) ? "" : " hidden"}>
      <label for="turbo-${button}"><span data-source-label="${button}">${escapeHtml(sourceLabel(button))}</span> · controller input</label>
      <select class="select" id="turbo-${button}" data-kind="turbo" data-name="${button}">
        ${modeOptions(state.schema.turbo_modes, state.profile.turbo[button])}
      </select>
      <label class="checkbox-pill turbo-override">
        <input type="checkbox" data-kind="turbo-override" data-name="${button}"${override ? " checked" : ""}${off ? " disabled" : ""}>
        <span>Override shared settings for <span data-source-label="${button}">${escapeHtml(sourceLabel(button))}</span></span>
      </label>
      <div class="turbo-setting-grid">${turboSettingFields(override || settings.defaults, button, off || !override)}</div>
      <p class="field-help" data-turbo-timing></p>
    </div>`;
  }).join("");
  updateTurboTiming();
}

function updateTurboTiming() {
  const narrow = [];
  elements.turbo.querySelectorAll("[data-turbo-button]").forEach((card) => {
    const button = card.dataset.turboButton;
    const mode = state.profile.turbo[button];
    const settings = state.profile.turbo_settings.overrides[button] || state.profile.turbo_settings.defaults;
    const on = 1000 * settings.duty_percent / (100 * settings.rate_hz);
    const off = 1000 / settings.rate_hz - on;
    const tooNarrow = mode !== "off" && Math.min(on, off) < 15;
    card.classList.toggle("narrow-pulse", tooNarrow);
    card.querySelector("[data-turbo-timing]").textContent =
      `${on.toFixed(2)} ms ON / ${off.toFixed(2)} ms OFF${mode === "off" ? " · inactive" : ""}`;
    if (tooNarrow) narrow.push(sourceLabel(button));
  });
  elements.turboTimingNotice.classList.toggle("warning", narrow.length > 0);
  elements.turboTimingNotice.textContent = narrow.length
    ? `Narrow phases: ${narrow.join(", ")}. The adapter does not report live USB cadence. Switch normally sends at 15 ms with an 8 ms endpoint interval; these shorter ON/OFF windows may be missed or quantized. Other USB modes differ. Your values are unchanged.`
    : "USB timing: live cadence is not reported. Switch normally sends at 15 ms with an 8 ms endpoint interval; other USB modes differ. Phases shorter than a report interval may be missed or quantized.";
}

function macroNumber(index, field, value, min, max, title, disabled = false) {
  const id = `macro-step-${index}-${field.replace(".", "-")}`;
  return `
    <div class="number-field">
      <label for="${id}">${title}</label>
      <input id="${id}" class="number-input" type="number" required min="${min}" max="${max}" step="1" value="${value}" data-kind="macro-number" data-index="${index}" data-field="${field}"${disabled ? " disabled" : ""}>
    </div>`;
}

function actionChordCard(action, title, description, selectedButtons, defaults) {
  const inherited = selectedButtons.length === 0 && defaults?.length;
  const effectiveButtons = inherited ? defaults : selectedButtons;
  const selected = new Set(effectiveButtons);
  const defaultText = inherited
    ? `Using default: ${defaults.map(button => sourceLabel(button)).join(" + ")}.`
    : defaults?.length
      ? `Clear every selection to restore ${defaults.map(button => sourceLabel(button)).join(" + ")}.`
      : "Empty disables this action.";
  return `
    <div class="action-card">
      <div class="action-card-heading">
        <div><span class="action-kind">${action === "custom_macro" ? "Custom macro" : "Built-in action"}</span><h4>${title}</h4></div>
        <span>${description}</span>
      </div>
      <div class="check-grid controller-choices">
        ${state.schema.controls.map((button) => `
          <label class="checkbox-pill controller-choice"${sourceAvailable(button) || selected.has(button) ? "" : " hidden"}>
            <input type="checkbox" data-kind="action-chord" data-action="${action}" data-name="${button}"${selected.has(button) ? " checked" : ""}>
            <span data-button="${button}">
              <b>${escapeHtml(sourceGlyph(button))}</b>
              <small>${escapeHtml(sourceLabel(button))}${sourceAvailable(button) ? "" : " (stored / unavailable)"}</small>
            </span>
          </label>`).join("")}
      </div>
      <p class="field-help">${defaultText}</p>
    </div>`;
}

function macroStepWireSize(step) {
  const overrides = new Set(step.overrides);
  return 3 +
    (overrides.has("buttons") ? 2 : 0) +
    (overrides.has("left_stick") ? 4 : 0) +
    (overrides.has("right_stick") ? 4 : 0) +
    (overrides.has("left_trigger") ? 2 : 0) +
    (overrides.has("right_trigger") ? 2 : 0);
}

function defaultMacroStep() {
  return {
    type: "state",
    overrides: ["buttons"],
    duration_ms: 100,
    output_buttons: [],
    left_stick: { x: 0, y: 0 },
    right_stick: { x: 0, y: 0 },
    triggers: { left: 0, right: 0 },
  };
}

function macroBudget(macros = state.profile.macros) {
  return macros.reduce((total, macro) => {
    total.steps += macro.steps.length;
    for (const step of macro.steps) {
      total.bytes += macroStepWireSize(step);
      total.duration += step.duration_ms;
    }
    return total;
  }, { steps: 0, bytes: 0, duration: 0 });
}

function macroBudgetError(macros) {
  if (!Array.isArray(macros) || macros.length !== 4) return "A profile must contain four independent macros.";
  if (macros.some((macro) => !Array.isArray(macro.steps))) return "Every macro must contain a step list.";
  if (macros.some((macro) => macro.steps.length > MACRO_STEP_LIMIT)) {
    return "Each macro can contain at most 8 steps.";
  }
  for (const macro of macros) {
    for (const step of macro.steps) {
      if (step.type !== "state" || !Number.isInteger(step.duration_ms) ||
          step.duration_ms < 0 || step.duration_ms > 10000) {
        return "State steps require a whole-number duration from 0 to 10000 ms.";
      }
    }
    if (!state.schema.macro_playback_modes.includes(macro.playback) ||
        !Number.isInteger(macro.repeat_count) ||
        macro.repeat_count < state.schema.macro_repeat_bounds.min ||
        macro.repeat_count > state.schema.macro_repeat_bounds.max) {
      return "Choose a valid playback mode and whole-number repeat count within the displayed limits.";
    }
    if (macro.trigger.length && macro.playback !== "once" && macro.steps.length &&
        macro.steps.every((step) => step.duration_ms === 0)) {
      return "A looping macro must have a nonzero cycle duration.";
    }
  }
  const budget = macroBudget(macros);
  if (budget.steps > MACRO_SHARED_STEP_LIMIT) return "The four macros share a limit of 16 steps.";
  if (budget.bytes > MACRO_BYTE_LIMIT) return "The four macros share 136 bytes of storage.";
  return "";
}

function captureBlocking() {
  const session = macroCapture.session;
  return Boolean(session && (!session.terminal || session.applying));
}

function captureOriginMatches(session) {
  return session.profile === state.profile &&
    session.ownerKey === currentOwner()?.key &&
    session.profileIndex === state.profileIndex &&
    session.macroIndex === state.selectedMacro;
}

function captureOptions() {
  const channels = Array.from(
    elements.captureOptions.querySelectorAll("[data-capture-channel]:checked")
  ).reduce((mask, input) => mask | Number(input.dataset.captureChannel), 0);
  const stepBytes = 3 + [2, 4, 4, 2, 2].reduce(
    (sum, size, index) => sum + (channels & (1 << index) ? size : 0), 0
  );
  const otherBudget = state.profile ? macroBudget(
    state.profile.macros.filter((_, index) => index !== state.selectedMacro)
  ) : { steps: 0, bytes: 0 };
  return {
    channels,
    axis_quantum: Number(elements.captureAxis.value),
    trigger_quantum: Number(elements.captureTrigger.value),
    max_duration_ms: Number(elements.captureDuration.value),
    max_events: Math.max(0, Math.min(
      MACRO_STEP_LIMIT, MACRO_SHARED_STEP_LIMIT - otherBudget.steps,
      Math.floor((MACRO_BYTE_LIMIT - otherBudget.bytes) / stepBytes)
    )),
    stepBytes,
    otherBudget,
  };
}

function renderCapture() {
  const session = macroCapture.session;
  const page = session?.page;
  const options = session?.options || captureOptions();
  const locked = captureBlocking();
  const pending = macroCapture.requestActive;
  const live = state.liveSample;
  const owner = currentOwner();
  const sourceReady = live?.connected && live.owner_key === owner?.key &&
    (owner?.index === 0 || live.identity_key === owner?.key);
  const invalid = elements.captureOptions.querySelector("input:invalid");
  elements.captureTitle.textContent = session
    ? `${session.ownerLabel} · profile ${session.profileIndex + 1} · macro ${session.macroIndex + 1}`
    : `Record into macro ${state.selectedMacro + 1}`;
  elements.captureOptions.disabled = Boolean(session) || state.busy;
  elements.captureRecord.disabled = Boolean(session) || state.busy ||
    !state.adapterConnected || !sourceReady || document.hidden ||
    state.playtestRequestActive || state.libraryRequestActive ||
    !options.channels || !options.max_events || Boolean(invalid);
  elements.captureStop.disabled = !session || session.terminal ||
    pending || !session.runId;
  elements.captureUse.disabled = !session?.terminal || pending || state.busy ||
    !page?.steps?.length || session.used || !captureOriginMatches(session);
  elements.captureDiscard.disabled = !session || pending || state.busy;
  elements.captureRecover.hidden = !session?.error;
  elements.captureRecover.disabled = pending || state.busy;
  const status = session?.error ? "error" : page?.state_name || (session ? "starting" : "idle");
  elements.macroCapture.dataset.state = status;
  elements.captureState.textContent = session?.applying ? "Validating recorded steps" : {
    idle: "Ready", starting: "Starting recording", recording: "Recording raw input",
    stopped: "Stopped", full: "Full · recording stopped",
    timed_out: "Time limit reached", disconnected: "Controller disconnected",
    error: "Recording needs attention",
  }[status] || status;
  elements.captureElapsed.textContent =
    `${Math.round((page?.elapsed_us || 0) / 1000)} / ${options.max_duration_ms} ms`;
  const count = page?.total_events || 0;
  elements.captureProgress.max = Math.max(1, options.max_events);
  elements.captureProgress.value = count;
  elements.captureBudget.textContent =
    `${count}/${options.max_events} recorded states · ${count * options.stepBytes}/${options.max_events * options.stepBytes} reserved bytes · ` +
    `other macros: ${options.otherBudget.steps}/16 shared steps, ${options.otherBudget.bytes}/136 bytes. ` +
    "Initial input counts as a state; held states split at 10 seconds. Capture stops before exceeding capacity.";
  const terminalNotice = {
    stopped: "Recording retained. Review it, then Use or Discard.",
    full: "The recording filled its requested state/byte capacity and stopped. This is a bounded partial sequence, not a complete longer performance.",
    timed_out: "The configured time limit ended this recording. Review the retained sequence before using it.",
    disconnected: "The controller disconnected. Partial recorded input is retained; reconnecting will not resume this run.",
  }[page?.state_name];
  const originWarning = session && !captureOriginMatches(session)
    ? "This capture belongs to another draft or macro. Use is disabled; it cannot overwrite this selection."
    : "";
  elements.captureNotice.textContent = [
    session?.error, terminalNotice, page?.conversion_error,
    session?.message, originWarning,
    session?.used ? "Recorded steps are in the unsaved draft. Review the preview, then use Save to Pico to store the profile." : "",
    !session && !sourceReady ? "Connect the selected controller and wait for its live input before recording." : "",
    !session && !options.channels ? "Select at least one channel." : "",
    !session && !options.max_events ? "Other macros leave no room for a recorded state. Free shared steps or bytes first." : "",
  ].filter(Boolean).join(" ");
  // Each response contains the complete firmware prefix (at most eight states),
  // not samples taken by this poll. Terminal pages are collected by the host.
  const events = page?.events || [];
  elements.captureEvents.innerHTML = events.map((event, index) => {
    const values = [];
    if (options.channels & 1) {
      const buttons = state.schema.buttons.filter((_, bit) => event.buttons & (1 << bit));
      values.push(`Buttons: ${buttons.map((button) => controlLabel(button)).join(" + ") || "released"}`);
    }
    if (options.channels & 2) values.push(`Left stick: ${event.left_x}, ${event.left_y}`);
    if (options.channels & 4) values.push(`Right stick: ${event.right_x}, ${event.right_y}`);
    if (options.channels & 8) values.push(`Left trigger: ${event.left_trigger}`);
    if (options.channels & 16) values.push(`Right trigger: ${event.right_trigger}`);
    const duration = page.steps?.[index]?.duration_ms;
    return `<li><strong>${Math.round(event.at_us / 1000)} ms${duration === undefined ? "" : ` · hold ${duration} ms`}</strong> · ${escapeHtml(values.join(" · "))}</li>`;
  }).join("");
  elements.form.querySelectorAll(":scope > .panel:not(#macro), #macro > :not(#macroCapture)")
    .forEach((node) => { node.inert = locked; });
  if (locked) {
    [
      elements.identity, elements.refresh, elements.resetDraft, elements.save,
      elements.activate, elements.identify, elements.saveAlias,
      elements.controllerAlias, elements.profileName, elements.saveProfileName,
      elements.copyProfile, elements.importProfile, elements.exportProfile,
      ...elements.profileList.querySelectorAll("button"),
    ].forEach((control) => { control.disabled = true; });
  }
  renderJoyconMode();
}

function capturePath(session, action) {
  return `/api/profiles/${session.identityIndex}/${session.profileIndex + 1}/capture/${action}`;
}

function captureQuery(session) {
  return `?owner_key=${encodeURIComponent(session.ownerKey)}&connection_generation=${session.generation}&capture_id=${encodeURIComponent(session.captureId)}`;
}

function acceptCapturePage(session, page) {
  if (macroCapture.session !== session) return;
  if (
    page.owner_key !== session.ownerKey ||
    page.capture_id !== session.captureId ||
    page.profile_number !== session.profileIndex + 1 ||
    page.macro_index !== session.macroIndex ||
    page.connection_generation !== session.generation ||
    page.slot !== session.slot || (session.runId && page.run_id !== session.runId)
  ) throw new Error("Stale recording response refused. The draft and retained input are unchanged.");
  session.runId = page.run_id;
  session.page = page;
  session.error = "";
  session.terminal = !["idle", "recording"].includes(page.state_name);
}

function finishCaptureRequest(session) {
  macroCapture.requestActive = false;
  setBusy(state.busy);
  if (macroCapture.session !== session) return;
  if (session.terminal) {
    state.playtestTimer = window.setTimeout(pollPlaytest, 75);
    state.libraryTimer = window.setTimeout(pollLibraryMetadata, 750);
  }
  if (!session.terminal && session.stopRequested && !session.stopAttempted) {
    stopCapture(session.message);
  } else if (!session.terminal && !session.error && !document.hidden) {
    macroCapture.timer = window.setTimeout(refreshCapture, 75);
  }
}

async function beginCapture() {
  if (elements.captureRecord.disabled || state.busy || macroCapture.session ||
      state.playtestRequestActive || state.libraryRequestActive) return;
  if (!reportProfileValidity() || !macroInputsValid(true)) return;
  const options = captureOptions();
  if (!options.channels || !options.max_events) return;
  const owner = currentOwner();
  const sample = state.liveSample;
  const session = {
    captureId: crypto.randomUUID(),
    profile: state.profile, ownerKey: owner.key, ownerLabel: owner.label,
    identityIndex: state.identityIndex, profileIndex: state.profileIndex,
    macroIndex: state.selectedMacro, slot: sample.slot,
    generation: sample.connection_generation, options, runId: null,
    page: null, terminal: false, error: "", message: "", used: false,
    stopRequested: false, stopAttempted: false, applying: false,
  };
  macroCapture.session = session;
  macroCapture.requestActive = true;
  window.clearTimeout(state.playtestTimer);
  window.clearTimeout(state.libraryTimer);
  stopMacroPreview("Preview stopped: recording live input.");
  setBusy(state.busy);
  try {
    const page = await api(capturePath(session, "start"), {
      method: "POST", keepalive: true,
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        owner_key: session.ownerKey, capture_id: session.captureId, slot: session.slot,
        connection_generation: session.generation, macro_index: session.macroIndex,
        profile: state.profile, channels: options.channels,
        max_events: options.max_events, axis_quantum: options.axis_quantum,
        trigger_quantum: options.trigger_quantum, max_duration_ms: options.max_duration_ms,
      }),
    });
    acceptCapturePage(session, page);
  } catch (error) {
    session.error = `${error.message} Draft unchanged. Read retained recording to recover an acknowledged start whose response was lost, or Discard.`;
  } finally {
    finishCaptureRequest(session);
  }
}

async function refreshCapture() {
  window.clearTimeout(macroCapture.timer);
  const session = macroCapture.session;
  if (!session || macroCapture.requestActive || session.terminal) return;
  macroCapture.requestActive = true;
  renderCapture();
  try {
    const page = await api(
      capturePath(session, session.runId || "current") + captureQuery(session),
      { keepalive: true }
    );
    acceptCapturePage(session, page);
    if (document.hidden) session.stopRequested = true;
  } catch (error) {
    session.error = `${error.message} The last received input and draft are retained. The firmware time/event limits still apply.`;
    session.stopRequested = true;
  } finally {
    finishCaptureRequest(session);
  }
}

async function stopCapture(message = "") {
  window.clearTimeout(macroCapture.timer);
  const session = macroCapture.session;
  if (!session || session.terminal) return;
  session.stopRequested = true;
  if (message) session.message = message;
  if (macroCapture.requestActive) return;
  session.stopAttempted = true;
  if (!session.runId) {
    session.error ||= "Start has not been acknowledged. Read retained recording to recover its run before stopping.";
    renderCapture();
    return;
  }
  macroCapture.requestActive = true;
  renderCapture();
  try {
    const page = await api(capturePath(session, "stop"), {
      method: "POST", keepalive: true,
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        run_id: session.runId, owner_key: session.ownerKey, capture_id: session.captureId,
        connection_generation: session.generation,
      }),
    });
    acceptCapturePage(session, page);
  } catch (error) {
    session.error = `${error.message} Stop was not acknowledged. Last received input and draft are retained; the bounded firmware run cannot resume automatically.`;
  } finally {
    finishCaptureRequest(session);
  }
}

async function useCapture() {
  const session = macroCapture.session;
  if (!session?.terminal || macroCapture.requestActive || !captureOriginMatches(session)) return;
  if (!session.page?.steps?.length || !reportProfileValidity()) return;
  const candidate = clone(state.profile);
  candidate.macros[session.macroIndex].steps = clone(session.page.steps);
  const error = macroBudgetError(candidate.macros);
  if (error) {
    session.message = `${error} Draft unchanged; recording retained.`;
    renderCapture();
    return;
  }
  session.applying = true;
  macroCapture.requestActive = true;
  setBusy(state.busy);
  try {
    const validated = await api("/api/profiles/validate", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(candidate),
    });
    if (!captureOriginMatches(session) || macroCapture.session !== session) {
      throw new Error("Capture destination changed; refusing to overwrite another draft");
    }
    const steps = validated.profile.macros[session.macroIndex].steps;
    const macros = state.profile.macros.map((macro, index) =>
      index === session.macroIndex ? { ...macro, steps } : macro
    );
    const budgetError = macroBudgetError(macros);
    if (budgetError) throw new Error(budgetError);
    stopMacroPreview("Ready to preview recorded steps.");
    state.profile.macros[session.macroIndex].steps = clone(steps);
    session.used = true;
    session.message = "";
    renderMacro();
    updateDirtyState();
  } catch (error) {
    session.message = `${error.message} Draft unchanged; recording retained.`;
  } finally {
    session.applying = false;
    finishCaptureRequest(session);
  }
}

async function discardCapture() {
  const session = macroCapture.session;
  if (!session || macroCapture.requestActive) return;
  if (!session.terminal) {
    await stopCapture("Recording stopped before discard.");
    if (!session.terminal && !window.confirm(
      "The recorder could not acknowledge Stop. Discard the visible capture anyway? The firmware still stops at its event/time limit. Your draft will not change."
    )) return;
  }
  window.clearTimeout(macroCapture.timer);
  macroCapture.session = null;
  setBusy(state.busy);
  pollPlaytest();
  pollLibraryMetadata();
}

function canAddMacroStep(bytes, budget) {
  return state.profile.macros[state.selectedMacro].steps.length < MACRO_STEP_LIMIT &&
    budget.steps < MACRO_SHARED_STEP_LIMIT && budget.bytes + bytes <= MACRO_BYTE_LIMIT;
}

function macroNotice(message, error = false) {
  elements.macroNotice.textContent = message;
  elements.macroNotice.classList.toggle("error", error);
}

function updateMacroBudgets() {
  const macro = state.profile.macros[state.selectedMacro];
  const budget = macroBudget();
  elements.macroControls.querySelector("[data-budget-steps]")?.replaceChildren(
    `${budget.steps}/${MACRO_SHARED_STEP_LIMIT} shared steps`
  );
  elements.macroControls.querySelector("[data-budget-bytes]")?.replaceChildren(
    `${budget.bytes}/${MACRO_BYTE_LIMIT} bytes`
  );
  const progress = elements.macroControls.querySelector("[data-budget-progress]");
  if (progress) progress.value = budget.bytes;
  elements.macroControls.querySelectorAll("[data-macro-index]").forEach((button) => {
    const item = state.profile.macros[Number(button.dataset.macroIndex)];
    const duration = item.steps.reduce((sum, step) => sum + step.duration_ms, 0);
    button.querySelector("small").textContent = `${item.steps.length}/8 steps · ${duration} ms`;
  });
  const duration = macro.steps.reduce((sum, step) => sum + step.duration_ms, 0);
  const bytes = macro.steps.reduce((sum, step) => sum + macroStepWireSize(step), 0);
  elements.macroDuration.textContent =
    `${macro.steps.length}/8 steps · ${bytes} bytes · ${duration} ms total · ${budget.duration} ms across all macros`;
  elements.addMacroStep.disabled = state.busy || !canAddMacroStep(DEFAULT_MACRO_STEP_BYTES, budget);
  elements.addMacroStep.title = "Add a 100 ms button step (5 bytes)";
  let startsAt = 0;
  elements.macroSteps.querySelectorAll("[data-step-index]").forEach((card) => {
    const index = Number(card.dataset.stepIndex);
    const step = macro.steps[index];
    card.querySelector(".step-number").textContent =
      `Step ${index + 1} · ${macroStepWireSize(step)} bytes`;
    card.querySelector(".step-timing").textContent =
      `${startsAt}–${startsAt + step.duration_ms} ms`;
    startsAt += step.duration_ms;
    card.querySelectorAll("[data-step-action]").forEach((button) => {
      const action = button.dataset.stepAction;
      button.disabled = state.busy ||
        (action === "up" && index === 0) ||
        (action === "down" && index === macro.steps.length - 1) ||
        (action === "insert" && !canAddMacroStep(DEFAULT_MACRO_STEP_BYTES, budget)) ||
        (action === "duplicate" && !canAddMacroStep(macroStepWireSize(step), budget));
    });
  });
  if (!macroPreview.running) {
    const total = duration * (macro.playback === "repeat" ? macro.repeat_count : 1);
    elements.macroPreviewTime.textContent = `0 / ${total} ms${["while_held", "toggle"].includes(macro.playback) ? " per cycle" : ""}`;
    elements.macroPreviewProgress.max = Math.max(1, total);
    elements.macroPreviewProgress.value = 0;
  }
  updateMacroPreviewControls();
  renderCapture();
}

function macroInputsValid(report = false) {
  const invalid = elements.macroSteps.querySelector("input:invalid") ||
    elements.macroControls.querySelector("input:invalid");
  if (report && invalid) {
    revealInvalidControl(invalid);
    invalid.reportValidity();
  }
  return !invalid;
}

function updateMacroPlayback() {
  const macro = state.profile.macros[state.selectedMacro];
  const count = elements.macroControls.querySelector("#macro-repeat-count");
  if (count) count.disabled = macro.playback !== "repeat";
  elements.macroPreviewModeHelp.textContent = {
    once: "Once: the preview stops at the end of one sequence.",
    repeat: `Repeat: the preview stops after ${macro.repeat_count} complete cycles.`,
    while_held: "While held: hardware repeats while the entire trigger chord is held. This visual preview repeats until you choose Stop; it does not monitor or inject trigger input.",
    toggle: "Toggle: hardware starts and stops on fresh trigger presses. This visual preview repeats until you choose Stop; it never sends USB input.",
  }[macro.playback];
}

// Clone before changing anything: rejected additions and override expansions
// leave all four drafts, including nested output values, untouched.
function mutateMacroSteps(change, focusIndex, focusSelector = "[data-step-handle]") {
  if (state.busy || captureBlocking() || !macroInputsValid(true)) return false;
  stopMacroPreview("Preview stopped: draft edited.");
  const macro = state.profile.macros[state.selectedMacro];
  const steps = clone(macro.steps);
  change(steps);
  const macros = state.profile.macros.map((item, index) =>
    index === state.selectedMacro ? { ...item, steps } : item
  );
  const error = macroBudgetError(macros);
  if (error) {
    macroNotice(`${error} Draft unchanged.`, true);
    return false;
  }
  macro.steps = steps;
  renderMacroSteps();
  updateMacroBudgets();
  updateGestureActionSelects();
  updateDirtyState();
  const index = Math.min(focusIndex, steps.length - 1);
  const focus = elements.macroSteps.querySelector(`[data-step-index="${index}"] ${focusSelector}`);
  (focus || elements.addMacroStep).focus();
  return true;
}

function updateMacroPreviewControls() {
  const macro = state.profile?.macros[state.selectedMacro];
  const zeroLoop = macro && macro.playback !== "once" &&
    macro.steps.every((step) => step.duration_ms === 0);
  const unavailable = !macro?.steps.length || zeroLoop ||
    state.busy || captureBlocking() || document.hidden || !macroInputsValid();
  elements.macroPreviewPlay.disabled = unavailable || macroPreview.running;
  elements.macroPreviewRestart.disabled = unavailable;
  elements.macroPreviewStop.disabled = !macroPreview.running;
}

function showMacroPreviewStep(step) {
  const overrides = new Set(step?.overrides || []);
  elements.macroPreview.querySelectorAll("[data-preview-field]").forEach((card) => {
    const field = card.dataset.previewField;
    const overridden = overrides.has(field);
    card.dataset.mode = overridden ? "override" : "passthrough";
    card.querySelector("[data-preview-mode]").textContent = overridden ? "Overridden" : "Passthrough";
    if (field.endsWith("_stick")) {
      const position = stickCoordinates(overridden ? step[field] : { x: 0, y: 0 });
      const scope = card.querySelector(".stick-scope");
      scope.style.setProperty("--output-x", `${position.left}%`);
      scope.style.setProperty("--output-y", `${position.top}%`);
      card.querySelector("output").textContent = overridden
        ? `${step[field].x}, ${step[field].y}` : "Controller input";
    } else if (field.endsWith("_trigger")) {
      const value = overridden ? step.triggers[field === "left_trigger" ? "left" : "right"] : 0;
      const progress = card.querySelector("progress");
      progress.value = value;
      progress.setAttribute("aria-valuetext", overridden ? String(value) : "Passthrough: controller input");
      card.querySelector("output").textContent = overridden ? String(value) : "Controller input";
    }
  });
  elements.macroPreviewButtons.querySelectorAll("[data-preview-button]").forEach((button) => {
    const overridden = overrides.has("buttons");
    const pressed = overridden && step.output_buttons.includes(button.dataset.previewButton);
    button.classList.toggle("pressed", pressed);
    button.querySelector("small").textContent = overridden ? (pressed ? "Pressed" : "Released") : "Passthrough";
  });
}

function stopMacroPreview(message = "Ready", reset = false) {
  if (!macroPreview.running && !reset) return;
  window.cancelAnimationFrame(macroPreview.frame);
  macroPreview.frame = 0;
  macroPreview.running = false;
  macroPreview.steps = [];
  macroPreview.index = -1;
  macroPreview.cycle = -1;
  elements.macroPreview.dataset.state = "stopped";
  elements.macroPreviewStatus.textContent = message;
  elements.macroSteps.querySelectorAll(".preview-active")
    .forEach((card) => card.classList.remove("preview-active"));
  showMacroPreviewStep(null);
  updateMacroPreviewControls();
}

function tickMacroPreview(now) {
  if (!macroPreview.running) return;
  const elapsed = Math.max(0, now - macroPreview.startedAt);
  const total = macroPreview.duration * macroPreview.cycles;
  const finite = macroPreview.cycles !== Infinity;
  if (macroPreview.duration === 0 || (finite && elapsed >= total)) {
    elements.macroPreviewTime.textContent = `${total} / ${total} ms`;
    elements.macroPreviewProgress.value = total;
    stopMacroPreview("Preview complete. All fields return to passthrough.");
    return;
  }
  const cycle = Math.floor(elapsed / macroPreview.duration);
  const phase = elapsed % macroPreview.duration;
  let endsAt = 0;
  let index = 0;
  // Skip elapsed cycles arithmetically; inspect at most eight states per frame.
  for (; index < macroPreview.steps.length; index += 1) {
    endsAt += macroPreview.steps[index].duration_ms;
    if (phase < endsAt) break;
  }
  elements.macroPreviewTime.textContent = finite
    ? `${Math.floor(elapsed)} / ${total} ms`
    : `${Math.floor(phase)} / ${macroPreview.duration} ms · cycle ${cycle + 1}`;
  elements.macroPreviewProgress.value = finite ? elapsed : phase;
  if (index !== macroPreview.index || cycle !== macroPreview.cycle) {
    macroPreview.index = index;
    macroPreview.cycle = cycle;
    elements.macroPreviewStatus.textContent =
      `Cycle ${cycle + 1}${finite ? ` of ${macroPreview.cycles}` : " · Stop to end"} · step ${index + 1} of ${macroPreview.steps.length}`;
    showMacroPreviewStep(macroPreview.steps[index]);
    elements.macroSteps.querySelectorAll("[data-step-index]").forEach((card) => {
      card.classList.toggle("preview-active", Number(card.dataset.stepIndex) === index);
    });
  }
  macroPreview.frame = window.requestAnimationFrame(tickMacroPreview);
}

function playMacroPreview() {
  if (state.busy || document.hidden || !state.profile || !macroInputsValid(true)) return;
  const macro = state.profile.macros[state.selectedMacro];
  const steps = macro.steps;
  if (!steps.length) return;
  const duration = steps.reduce((sum, step) => sum + step.duration_ms, 0);
  if (macro.playback !== "once" && duration === 0) {
    macroNotice("A looping preview requires a nonzero cycle duration.", true);
    return;
  }
  stopMacroPreview("Ready", true);
  macroPreview.steps = clone(steps);
  macroPreview.duration = duration;
  macroPreview.cycles = macro.playback === "once" ? 1 :
    macro.playback === "repeat" ? macro.repeat_count : Infinity;
  macroPreview.startedAt = performance.now();
  macroPreview.running = true;
  elements.macroPreview.dataset.state = "playing";
  elements.macroPreviewProgress.max = Math.max(1, macroPreview.duration *
    (macroPreview.cycles === Infinity ? 1 : macroPreview.cycles));
  updateMacroPreviewControls();
  tickMacroPreview(macroPreview.startedAt);
}

function renderMacro() {
  stopMacroPreview("Ready", true);
  draggedMacroStep = null;
  macroNotice("");
  const controllerStyle = currentControllerStyle();
  elements.builtinActions.dataset.controllerStyle = controllerStyle;
  elements.macroControls.dataset.controllerStyle = controllerStyle;
  const macro = state.profile.macros[state.selectedMacro];
  const budget = macroBudget();
  elements.builtinActions.innerHTML = [
    actionChordCard(
      "profile_switch",
      "Cycle active profile",
      `Cycle through profiles 1–${state.schema.profile_capacity}.`,
      state.profile.switching_chord,
      state.schema.default_switching_chord
    ),
    actionChordCard(
      "motion_toggle",
      "Toggle motion",
      "Enable or disable motion for this controller.",
      state.profile.motion_toggle_chord,
      state.schema.default_motion_toggle_chord
    ),
  ].join("");
  const swing = state.profile.swing;
  const nunchukSwing = state.profile.nunchuk_swing;
  const combinedSwing = state.profile.combined_swing;
  const swingSensitivities = state.schema.swing_sensitivities;
  const windowBounds = state.schema.combination_window_bounds;
  const combinationWindowMs = state.profile.combination_window_ms;

  elements.swing.innerHTML = `
    <div class="subpanel-heading">
      <div>
        <span class="action-kind">Motion gestures</span>
        <h4>Motion gestures &amp; swings</h4>
      </div>
      <span>Accelerometer only · no sensor bar or MotionPlus required</span>
    </div>
    <div class="swing-cards">
      <div class="action-card swing-card">
        <div class="action-card-heading">
          <div>
            <span class="action-kind">Individual gesture</span>
            <h4>Wii Remote swing</h4>
          </div>
          <span>Triggers when the Wii Remote is swung deliberately.</span>
        </div>
        <div class="swing-card-grid">
          <div class="control-card">
            <label for="swing-button">Remote action</label>
            <select class="select" id="swing-button" data-kind="swing-action" data-gesture="swing" data-output-select="swing">
              ${gestureActionOptions(swing, state.schema.buttons, controllerStyle, "Disabled")}
            </select>
          </div>
          <div class="control-card">
            <label for="swing-sensitivity">Sensitivity</label>
            <select class="select" id="swing-sensitivity" data-kind="swing-sensitivity" data-gesture="swing">
              ${modeOptions(swingSensitivities, swing.sensitivity)}
            </select>
          </div>
          <div class="control-card">
            <label for="swing-modifier">Held modifier (optional)</label>
            <select class="select" id="swing-modifier" data-kind="swing-modifier" data-gesture="swing">
              ${modifierOptions(swing.modifier)}
            </select>
          </div>
        </div>
      </div>
      <div class="action-card swing-card">
        <div class="action-card-heading">
          <div>
            <span class="action-kind">Individual gesture</span>
            <h4>Nunchuk swing</h4>
          </div>
          <span>Triggers when the connected Nunchuk is swung deliberately.</span>
        </div>
        <div class="swing-card-grid">
          <div class="control-card">
            <label for="nunchuk-swing-button">Nunchuk action</label>
            <select class="select" id="nunchuk-swing-button" data-kind="swing-action" data-gesture="nunchuk_swing" data-output-select="nunchuk_swing">
              ${gestureActionOptions(nunchukSwing, state.schema.buttons, controllerStyle, "Disabled")}
            </select>
          </div>
          <div class="control-card">
            <label for="nunchuk-swing-sensitivity">Sensitivity</label>
            <select class="select" id="nunchuk-swing-sensitivity" data-kind="swing-sensitivity" data-gesture="nunchuk_swing">
              ${modeOptions(swingSensitivities, nunchukSwing.sensitivity)}
            </select>
          </div>
          <div class="control-card">
            <label for="nunchuk-swing-modifier">Held modifier (optional)</label>
            <select class="select" id="nunchuk-swing-modifier" data-kind="swing-modifier" data-gesture="nunchuk_swing">
              ${modifierOptions(nunchukSwing.modifier)}
            </select>
          </div>
        </div>
      </div>
      <div class="action-card swing-card">
        <div class="action-card-heading">
          <div>
            <span class="action-kind">Combined gesture</span>
            <h4>Both together</h4>
          </div>
          <span>One full swing plus qualifying movement from the other device within the timing window.</span>
        </div>
        <div class="swing-card-grid">
          <div class="control-card">
            <label for="combined-swing-button">Combined action</label>
            <select class="select" id="combined-swing-button" data-kind="swing-action" data-gesture="combined_swing" data-output-select="combined_swing">
              ${gestureActionOptions(combinedSwing, state.schema.buttons, controllerStyle, "Disabled")}
            </select>
          </div>
          <div class="control-card">
            <label for="combination-window-ms">Combination window · ${windowBounds.min}–${windowBounds.max} ms</label>
            <input class="number-input" id="combination-window-ms" type="number" required min="${windowBounds.min}" max="${windowBounds.max}" step="1" value="${combinationWindowMs}" data-kind="combination-window">
          </div>
          <div class="control-card">
            <label for="combined-swing-modifier">Held modifier (optional)</label>
            <select class="select" id="combined-swing-modifier" data-kind="swing-modifier" data-gesture="combined_swing">
              ${modifierOptions(combinedSwing.modifier)}
            </select>
          </div>
        </div>
      </div>
    </div>
    <p class="field-help">Triggers one ~80 ms press or one-shot macro cycle per deliberate stroke. When Both together is enabled and both sensors are available, individual swings wait for the combination window. One full swing can combine with smaller sustained movement from the other sensor; two small movements alone do not trigger. Individual thresholds are unchanged, and each confirmation is used only once. Gestures play bound macros exactly once regardless of physical-trigger playback modes. Empty or zero-duration macro targets must be configured before saving.</p>`;
  validateGestureMacros();
  elements.macroControls.innerHTML = `
    <div class="macro-picker">
      <div class="macro-tabs" role="group" aria-label="Choose a macro draft">
        ${state.profile.macros.map((item, index) => `
          <button type="button" class="macro-tab${index === state.selectedMacro ? " selected" : ""}" data-macro-index="${index}" aria-pressed="${index === state.selectedMacro}">
            Macro ${index + 1}<small>${item.steps.length} step${item.steps.length === 1 ? "" : "s"}</small>
          </button>`).join("")}
      </div>
      <div class="macro-budget">
        <strong data-budget-steps>${budget.steps}/16 shared steps</strong>
        <span data-budget-bytes>${budget.bytes}/136 bytes</span>
        <progress data-budget-progress max="136" value="${budget.bytes}" aria-label="Shared macro storage"></progress>
      </div>
    </div>
    ${actionChordCard(
      "custom_macro",
      `Run macro ${state.selectedMacro + 1}`,
      "Start this editable sequence.",
      macro.trigger,
      null
    )}
    <div class="control-card">
      <label for="macro-cancel">Macro ${state.selectedMacro + 1} cancel control</label>
      <select class="select" id="macro-cancel" data-kind="macro-selector" data-field="cancel">
        ${sourceOptions(macro.cancel)}
      </select>
      <div class="macro-playback-fields">
        <div class="number-field">
          <label for="macro-playback">Playback mode</label>
          <select class="select" id="macro-playback" data-kind="macro-playback">${modeOptions(state.schema.macro_playback_modes, macro.playback)}</select>
        </div>
        <div class="number-field">
          <label for="macro-repeat-count">Repeat cycles · ${state.schema.macro_repeat_bounds.min}–${state.schema.macro_repeat_bounds.max}</label>
          <input class="number-input" id="macro-repeat-count" type="number" required min="${state.schema.macro_repeat_bounds.min}" max="${state.schema.macro_repeat_bounds.max}" step="1" value="${macro.repeat_count}" data-kind="macro-repeat"${macro.playback === "repeat" ? "" : " disabled"}>
        </div>
      </div>
      <p class="field-help">Once plays one cycle. While held repeats while the entire trigger is held. Toggle repeats until pressed again. Repeat plays the selected cycle count. Cancel ends any active mode.</p>
    </div>`;

  elements.macroControls.querySelectorAll("[data-macro-index]").forEach((button) => {
    button.onclick = () => {
      if (state.busy || captureBlocking() || !macroInputsValid(true)) return;
      state.selectedMacro = Number(button.dataset.macroIndex);
      renderCapture();
      renderMacro();
      elements.macroControls.querySelector(`[data-macro-index="${state.selectedMacro}"]`).focus();
    };
  });
  elements.macroPreviewTitle.textContent = `Macro ${state.selectedMacro + 1} draft preview`;
  elements.macroPreviewButtons.innerHTML = state.schema.buttons.map((button) => `
    <span class="preview-button" data-preview-button="${button}">
      <b data-output-label="${button}">${escapeHtml(outputLabel(button, controllerStyle))}</b><small>Passthrough</small>
    </span>`).join("");
  renderMacroSteps();
  updateMacroPlayback();
  updateMacroBudgets();
}

function renderMacroSteps() {
  const macro = state.profile.macros[state.selectedMacro];
  const controllerStyle = currentControllerStyle();

  elements.macroSteps.innerHTML = macro.steps.length === 0
    ? '<div class="macro-step end"><p class="field-help">No steps yet. Add a step or record controller input to build this macro.</p></div>'
    : macro.steps.map((step, index) => {
      const overrides = new Set(step.overrides);
      const outputButtons = new Set(step.output_buttons);
      return `
        <div class="macro-step" data-step-index="${index}" role="listitem" aria-label="Step ${index + 1}">
          <div class="step-header">
            <div class="step-identity">
              <button class="step-tool step-handle" type="button" draggable="true" data-step-handle="${index}" aria-label="Reorder step ${index + 1}" aria-describedby="macroReorderHelp" aria-keyshortcuts="Alt+ArrowUp Alt+ArrowDown">Drag</button>
              <span class="step-number">Step ${index + 1} · ${macroStepWireSize(step)} bytes</span>
              <span class="step-timing"></span>
            </div>
            <div class="step-tools" role="group" aria-label="Step ${index + 1} actions">
              <button class="step-tool" type="button" data-step-action="up" aria-label="Move step ${index + 1} up">Move up</button>
              <button class="step-tool" type="button" data-step-action="down" aria-label="Move step ${index + 1} down">Move down</button>
              <button class="step-tool" type="button" data-step-action="insert" aria-label="Insert a new step before step ${index + 1}">Insert before</button>
              <button class="step-tool" type="button" data-step-action="duplicate" aria-label="Duplicate step ${index + 1}">Duplicate</button>
              <button class="remove-step" type="button" data-step-action="remove" aria-label="Remove step ${index + 1}">Remove</button>
            </div>
          </div>
          <div class="step-grid">
            ${macroNumber(index, "duration_ms", step.duration_ms, 0, 10000, "Duration (ms)")}
            ${macroNumber(index, "left_stick.x", step.left_stick.x, -32768, 32767, "Left stick X", !overrides.has("left_stick"))}
            ${macroNumber(index, "left_stick.y", step.left_stick.y, -32768, 32767, "Left stick Y", !overrides.has("left_stick"))}
            ${macroNumber(index, "right_stick.x", step.right_stick.x, -32768, 32767, "Right stick X", !overrides.has("right_stick"))}
            ${macroNumber(index, "right_stick.y", step.right_stick.y, -32768, 32767, "Right stick Y", !overrides.has("right_stick"))}
            ${macroNumber(index, "triggers.left", step.triggers.left, 0, 65535, "Left trigger", !overrides.has("left_trigger"))}
            ${macroNumber(index, "triggers.right", step.triggers.right, 0, 65535, "Right trigger", !overrides.has("right_trigger"))}
          </div>
          <div class="step-group">
            <span class="step-group-title">Outputs controlled by this step</span>
            <div class="check-grid">
              ${state.schema.macro_overrides.map((name) => `
                <label class="checkbox-pill">
                  <input type="checkbox" data-kind="macro-override" data-index="${index}" data-name="${name}"${overrides.has(name) ? " checked" : ""}>
                  <span>${label(name)}</span>
                </label>`).join("")}
            </div>
          </div>
          <div class="step-group">
            <span class="step-group-title">Output buttons</span>
            <div class="check-grid">
              ${state.schema.buttons.map((button) => `
                <label class="checkbox-pill">
                  <input type="checkbox" data-kind="macro-output" data-index="${index}" data-name="${button}"${outputButtons.has(button) ? " checked" : ""}${overrides.has("buttons") ? "" : " disabled"}>
                  <span data-output-label="${button}">${escapeHtml(outputLabel(button, controllerStyle))}</span>
                </label>`).join("")}
            </div>
          </div>
        </div>`;
    }).join("");
  elements.macroStepsTitle.textContent = `Macro ${state.selectedMacro + 1} steps`;
  elements.addMacroStep.textContent = `Add step to macro ${state.selectedMacro + 1}`;
}

function renderEditor() {
  state.playtestShift = { key: "", held: false, active: false };
  elements.profileTitle.textContent =
    state.profileNames[state.profileIndex] || `Profile ${state.profileIndex + 1}`;
  elements.activeBadge.hidden = !state.active;
  elements.activate.disabled =
    !state.adapterConnected || state.busy || state.active;
  elements.profileName.value = state.profileNames[state.profileIndex] || "";
  const owner = currentOwner();
  elements.controllerAlias.value = owner?.alias || "";
  elements.identify.disabled =
    state.busy || !state.identifyAvailable;
  elements.controllerAlias.disabled = !owner || owner.index === 0;
  elements.saveAlias.disabled =
    !state.adapterConnected || !owner || owner.index === 0;
  elements.controllerDetails.textContent = owner && owner.index !== 0
    ? owner.controller.model
    : "Used when no dedicated controller profile exists.";
  renderIdentities();
  renderProfileList();
  renderNativeLayout();
  renderButtonMap();
  renderShortcuts();
  renderShift();
  renderAnalog();
  renderFeedback();
  renderTurbo();
  renderMacro();
  refreshSourceControls();
  elements.loading.hidden = true;
  elements.form.hidden = false;
  updateDirtyState();
}

async function loadProfile() {
  if (captureBlocking()) return;
  stopMacroPreview("Preview stopped: profile selection changed.");
  clearPlaytest("Loading the selected controller profile.");
  setBusy(true);
  elements.form.hidden = true;
  elements.loading.hidden = false;
  renderIdentities();
  renderProfileList();
  try {
    const payload = await api(`/api/profiles/${state.identityIndex}/${state.profileIndex + 1}`);
    state.profile = payload.profile;
    state.original = canonical(payload.profile);
    state.pendingName = false;
    state.profileNames = payload.profile_names;
    const owner = currentOwner();
    if (owner) owner.alias = payload.alias;
    state.active = payload.active;
    setConnection("ready", "Adapter connected");
    syncControllerPresentation();
    if (state.presentation.sources.length &&
        !sourceAvailable(state.selectedButton) &&
        getControlMapping(state.selectedButton) == null) {
      state.selectedButton = state.presentation.sources[0];
    }
    renderEditor();
  } catch (error) {
    state.profile = null;
    setConnection("error", "Adapter unavailable");
    elements.loading.querySelector("p").textContent = error.message;
    toast(error.message, true);
  } finally {
    setBusy(false);
  }
}

async function loadLibrary(preserveDraft = false) {
  setBusy(true);
  try {
    await refreshJoyconMode(true);
    const payload = await api("/api/profiles");
    syncLibraryMetadata(payload.identities, !preserveDraft);
    if (!preserveDraft || !state.profile) {
      await loadProfile();
    } else {
      setConnection("ready", "Adapter connected");
      toast("Library refreshed. Your unsaved draft is unchanged.");
    }
  } catch (error) {
    setConnection("error", "Adapter unavailable");
    elements.loading.querySelector("p").textContent = error.message;
    toast(error.message, true);
  } finally {
    setBusy(false);
  }
}

function updateNestedStep(step, path, value) {
  const parts = path.split(".");
  let target = step;
  for (const part of parts.slice(0, -1)) target = target[part];
  target[parts.at(-1)] = value;
}

function handleFormChange(event) {
  const target = event.target;
  const kind = target.dataset.kind;
  if (!kind || !state.profile || captureBlocking()) return;
  stopMacroPreview("Preview stopped: draft edited.");
  if (event.type === "change" && kind.startsWith("macro-") && kind !== "macro-selector") return;
  if (target.disabled) return;
  if (["turbo-setting", "macro-repeat"].includes(kind) && !target.validity.valid) {
    updateMacroPreviewControls();
    return;
  }
  if (kind === "analog-number") {
    state.profile[target.dataset.group][target.dataset.side][target.dataset.field] = Number(target.value);
  } else if (kind === "analog-bool") {
    state.profile[target.dataset.group][target.dataset.side][target.dataset.field] = target.checked;
  } else if (kind === "swap-sticks") {
    state.profile.swap_sticks = target.checked;
  } else if (kind === "curve-preset") {
    if (target.value !== "custom") {
      state.profile[target.dataset.group][target.dataset.side].curve_q8_8 =
        curvePresets[target.value];
      renderAnalog();
    }
  } else if (kind === "curve-range") {
    const config = state.profile[target.dataset.group][target.dataset.side];
    config.curve_q8_8 = Number(target.value);
    const editor = target.closest(".curve-editor");
    editor.querySelector("output").textContent = target.value;
    editor.querySelector(".curve-line").setAttribute(
      "d", curvePath(config.curve_q8_8)
    );
    editor.querySelector("[data-kind='curve-preset']").value =
      curvePreset(config.curve_q8_8);
  } else if (kind === "rumble-range") {
    state.profile.rumble[target.dataset.field] = Number(target.value);
    document.querySelector(`#rumble-${target.dataset.field}-value`).value = target.value;
  } else if (kind === "rumble-policy") {
    state.profile.rumble.confirmation_policy = target.value;
  } else if (kind === "action-chord") {
    const fields = {
      profile_switch: [state.profile, "switching_chord"],
      motion_toggle: [state.profile, "motion_toggle_chord"],
      custom_macro: [state.profile.macros[state.selectedMacro], "trigger"],
    };
    const [owner, field] = fields[target.dataset.action];
    const defaults = {
      profile_switch: state.schema.default_switching_chord,
      motion_toggle: state.schema.default_motion_toggle_chord,
    }[target.dataset.action];
    const current = owner[field].length === 0 && defaults ? defaults : owner[field];
    const selected = new Set(current);
    target.checked ? selected.add(target.dataset.name) : selected.delete(target.dataset.name);
    owner[field] = state.schema.controls.filter((name) => selected.has(name));
  } else if (kind === "shortcut-modifier") {
    state.profile.shortcuts.modifier = target.value || null;
    if (!target.value) state.profile.shortcuts.profiles.fill(null);
    renderShortcuts();
    elements.shortcutModifier.querySelector("select").focus();
  } else if (kind === "shortcut-selector") {
    state.profile.shortcuts.profiles[Number(target.dataset.index)] = target.value || null;
    renderShortcuts();
    elements.shortcuts.querySelector(`#shortcut-${target.dataset.index}`).focus();
  } else if (kind === "shift-mode" || kind === "shift-modifier") {
    state.profile.shift[kind === "shift-mode" ? "mode" : "modifier"] = target.value || null;
    renderShift();
    elements.shift.querySelector(`#${kind}`).focus();
  } else if (kind === "shift-map") {
    const map = state.schema.extra_buttons.includes(target.dataset.name)
      ? state.profile.shift.extra_button_map : state.profile.shift.button_map;
    map[target.dataset.name] = target.value || null;
  } else if (kind === "turbo") {
    state.profile.turbo[target.dataset.name] = target.value;
    renderTurbo();
    elements.turbo.querySelector(`#turbo-${target.dataset.name}`).focus();
  } else if (kind === "turbo-override") {
    const overrides = state.profile.turbo_settings.overrides;
    if (target.checked) {
      overrides[target.dataset.name] = clone(state.profile.turbo_settings.defaults);
    } else {
      delete overrides[target.dataset.name];
    }
    renderTurbo();
    elements.turbo.querySelector(`[data-kind="turbo-override"][data-name="${target.dataset.name}"]`).focus();
  } else if (kind === "turbo-setting") {
    const settings = state.profile.turbo_settings;
    const config = target.dataset.name ? settings.overrides[target.dataset.name] : settings.defaults;
    config[target.dataset.field] = Number(target.value);
    if (!target.dataset.name) {
      elements.turbo.querySelectorAll(`[data-kind="turbo-setting"][data-field="${target.dataset.field}"]`).forEach((input) => {
        if (!settings.overrides[input.dataset.name]) input.value = target.value;
      });
    }
    updateTurboTiming();
  } else if (kind === "swing-action") {
    const gestureName = target.dataset.gesture;
    const { button, macro } = parseGestureActionValue(target.value);
    state.profile[gestureName].button = button;
    state.profile[gestureName].macro = macro;
    validateGestureMacros();
  } else if (kind === "swing-sensitivity") {
    const gestureName = target.dataset.gesture;
    state.profile[gestureName].sensitivity = target.value;
  } else if (kind === "swing-modifier") {
    const gestureName = target.dataset.gesture;
    state.profile[gestureName].modifier = target.value || null;
  } else if (kind === "combination-window") {
    if (target.validity.valid) {
      state.profile.combination_window_ms = Number(target.value);
    }
  } else if (kind === "macro-selector") {
    state.profile.macros[state.selectedMacro][target.dataset.field] = target.value || null;
  } else if (kind === "macro-playback" || kind === "macro-repeat") {
    state.profile.macros[state.selectedMacro][kind === "macro-playback" ? "playback" : "repeat_count"] =
      kind === "macro-playback" ? target.value : Number(target.value);
    updateMacroPlayback();
  } else if (kind === "macro-number") {
    if (!target.validity.valid) {
      macroNotice("Enter a whole number within the displayed limits. The last valid draft value is retained.", true);
      updateMacroPreviewControls();
      return;
    }
    updateNestedStep(state.profile.macros[state.selectedMacro].steps[Number(target.dataset.index)], target.dataset.field, Number(target.value));
  } else if (kind === "macro-output") {
    const step = state.profile.macros[state.selectedMacro].steps[Number(target.dataset.index)];
    const selected = new Set(step.output_buttons);
    target.checked ? selected.add(target.dataset.name) : selected.delete(target.dataset.name);
    step.output_buttons = state.schema.buttons.filter((name) => selected.has(name));
  } else if (kind === "macro-override") {
    const index = Number(target.dataset.index);
    const macro = state.profile.macros[state.selectedMacro];
    const step = clone(macro.steps[index]);
    const selected = new Set(step.overrides);
    target.checked ? selected.add(target.dataset.name) : selected.delete(target.dataset.name);
    step.overrides = state.schema.macro_overrides.filter((name) => selected.has(name));
    if (!target.checked) {
      if (target.dataset.name === "buttons") step.output_buttons = [];
      if (target.dataset.name === "left_stick") step.left_stick = { x: 0, y: 0 };
      if (target.dataset.name === "right_stick") step.right_stick = { x: 0, y: 0 };
      if (target.dataset.name === "left_trigger") step.triggers.left = 0;
      if (target.dataset.name === "right_trigger") step.triggers.right = 0;
    }
    const macros = state.profile.macros.map((item, macroIndex) =>
      macroIndex === state.selectedMacro
        ? { ...item, steps: item.steps.map((itemStep, stepIndex) => stepIndex === index ? step : itemStep) }
        : item
    );
    const error = macroBudgetError(macros);
    if (error) {
      target.checked = macro.steps[index].overrides.includes(target.dataset.name);
      macroNotice(`${error} Draft unchanged.`, true);
      return;
    }
    macro.steps[index] = step;
    const card = target.closest("[data-step-index]");
    card.querySelectorAll("[data-kind='macro-number']").forEach((input) => {
      const [group, side] = input.dataset.field.split(".");
      if (!side) return;
      const override = group === "triggers" ? `${side}_trigger` : group;
      if (override !== target.dataset.name) return;
      input.disabled = !selected.has(override);
      input.value = step[group][side];
    });
    card.querySelectorAll("[data-kind='macro-output']").forEach((input) => {
      if (target.dataset.name !== "buttons") return;
      input.disabled = !selected.has("buttons");
      input.checked = step.output_buttons.includes(input.dataset.name);
    });
  }
  if (kind.startsWith("macro-")) {
    macroNotice("");
    updateMacroBudgets();
    updateGestureActionSelects();
  }
  updateDirtyState();
  if (state.liveSample) renderPlaytest(state.liveSample);
}

elements.analog.addEventListener("click", (event) => {
  const button = event.target.closest("[data-copy-analog]");
  if (!button) return;
  stopMacroPreview("Preview stopped: draft edited.");
  const group = button.dataset.copyAnalog;
  const source = button.dataset.sourceSide;
  const destination = source === "left" ? "right" : "left";
  state.profile[group][destination] = clone(state.profile[group][source]);
  renderAnalog();
  updateDirtyState();
  toast(`${label(source)} ${label(group)} settings applied to ${destination}.`);
});

elements.form.addEventListener("input", handleFormChange);
elements.form.addEventListener("change", handleFormChange);

elements.identity.addEventListener("change", async () => {
  const previous = state.identityIndex;
  if (captureBlocking() || !confirmDiscard()) {
    elements.identity.value = String(previous);
    return;
  }
  state.identityIndex = Number(elements.identity.value);
  const owner = currentOwner();
  if (owner) persistOwnerKey(owner.key);
  state.profileIndex = 0;
  state.profileNames = Array(state.schema.profile_capacity).fill("");
  wiiOrientation.current = null;
  resetWiiOrientationDraft();
  await loadProfile();
});
elements.refresh.addEventListener("click", async () => {
  await loadLibrary(true);
});

elements.adapterSettingsButton.addEventListener("click", () => {
  if (!elements.adapterSettingsDialog.open) {
    renderJoyconMode();
    elements.adapterSettingsDialog.showModal();
  }
});
elements.closeAdapterSettings.addEventListener("click", () => {
  elements.adapterSettingsDialog.close();
});
elements.adapterSettingsDialog.addEventListener("close", () => {
  elements.adapterSettingsButton.focus();
});

elements.joyconMode.addEventListener("change", () => {
  if (elements.joyconMode.disabled) return;
  joyconMode.selected = elements.joyconMode.value;
  joyconMode.pending = joyconModeChanged();
  joyconMode.applyError = "";
  renderJoyconMode();
});

elements.applyJoyconMode.addEventListener("click", async () => {
  if (elements.applyJoyconMode.disabled || state.busy || captureBlocking() ||
      macroCapture.requestActive) return;
  joyconMode.applying = true;
  joyconMode.applyError = "";
  joyconMode.revision += 1;
  setBusy(true);
  try {
    const payload = await api("/api/joycon-mode", {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ mode: joyconMode.selected }),
    });
    acceptJoyconMode(payload, true);
    toast(`${label(payload.mode)} default saved. Your profile draft is unchanged; live playtest reports the actual arrangement.`);
  } catch (error) {
    joyconMode.applyError = `Could not confirm player mode: ${error.message}`;
    toast(joyconMode.applyError, true);
  } finally {
    joyconMode.applying = false;
    setBusy(false);
    window.clearTimeout(state.libraryTimer);
    state.libraryTimer = window.setTimeout(pollLibraryMetadata, 0);
  }
});

elements.wiiOrientation.addEventListener("change", () => {
  if (elements.wiiOrientation.disabled) return;
  wiiOrientation.selected = elements.wiiOrientation.value;
  wiiOrientation.pending = wiiOrientation.selected !== wiiOrientation.current;
  wiiOrientation.applyError = "";
  renderWiiOrientation();
});

elements.applyWiiOrientation.addEventListener("click", async () => {
  const owner = currentOwner();
  const live = matchingLiveSample() ? state.liveSample : null;
  if (elements.applyWiiOrientation.disabled || (state.busy && !wiiOrientation.applying) ||
      captureBlocking() || macroCapture.requestActive || !owner || owner.index === 0 || !live) return;

  const targetOrientation = wiiOrientation.selected;
  const targetLayout = targetOrientation === "horizontal" ? "wii-horizontal" : "wii-vertical";
  const gen = live.connection_generation;
  wiiOrientation.applying = true;
  wiiOrientation.applyError = "";
  wiiOrientation.submittedGeneration = gen;
  wiiOrientation.desiredLayout = targetLayout;
  wiiOrientation.ownerKey = owner.key;
  window.clearTimeout(wiiOrientation.applyTimer);
  wiiOrientation.applyTimer = window.setTimeout(() => {
    if (wiiOrientation.applying) {
      wiiOrientation.applying = false;
      wiiOrientation.submittedGeneration = null;
      wiiOrientation.desiredLayout = null;
      wiiOrientation.ownerKey = null;
      wiiOrientation.applyError = "Orientation confirmation timed out. Check controller connection.";
      setBusy(false);
      renderWiiOrientation();
    }
  }, 5000);
  setBusy(true);
  try {
    await api(`/api/identities/${owner.index}/wii-orientation`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        orientation: targetOrientation,
        connection_generation: gen,
      }),
    });
  } catch (error) {
    window.clearTimeout(wiiOrientation.applyTimer);
    wiiOrientation.applying = false;
    wiiOrientation.submittedGeneration = null;
    wiiOrientation.desiredLayout = null;
    wiiOrientation.ownerKey = null;
    wiiOrientation.applyError = `Could not set orientation: ${error.message}`;
    toast(wiiOrientation.applyError, true);
    setBusy(false);
    renderWiiOrientation();
  }
  pollPlaytest();
});
elements.resetDraft.addEventListener("click", () => {
  if (!state.schema || !window.confirm("Replace this draft with the default profile? Nothing is saved until you choose Save to Pico.")) return;
  state.profile = clone(state.schema.default_profile);
  renderEditor();
  toast("Default profile loaded into the draft.");
});

elements.addMacroStep.addEventListener("click", () => {
  const index = state.profile.macros[state.selectedMacro].steps.length;
  if (mutateMacroSteps((steps) => steps.push(defaultMacroStep()), index)) {
    macroNotice(`Added step ${index + 1}.`);
  }
});

function moveMacroStep(from, to, focusSelector = "[data-step-handle]") {
  const count = state.profile.macros[state.selectedMacro].steps.length;
  if (from === to || from < 0 || to < 0 || from >= count || to >= count) return;
  if (mutateMacroSteps((steps) => {
    const [step] = steps.splice(from, 1);
    steps.splice(to, 0, step);
  }, to, focusSelector)) {
    macroNotice(`Moved step ${from + 1} to position ${to + 1}.`);
  }
}

elements.macroSteps.addEventListener("click", (event) => {
  const button = event.target.closest("[data-step-action]");
  if (!button || button.disabled) return;
  const index = Number(button.closest("[data-step-index]").dataset.stepIndex);
  const action = button.dataset.stepAction;
  if (action === "up" || action === "down") {
    moveMacroStep(index, index + (action === "up" ? -1 : 1));
    return;
  }
  const focusIndex = action === "duplicate" ? index + 1 : index;
  if (mutateMacroSteps((steps) => {
    if (action === "insert") steps.splice(index, 0, defaultMacroStep());
    else if (action === "duplicate") steps.splice(index + 1, 0, clone(steps[index]));
    else if (action === "remove") steps.splice(index, 1);
  }, focusIndex)) {
    macroNotice(action === "remove" ? `Removed step ${index + 1}.` :
      action === "insert" ? `Inserted step ${index + 1}.` : `Duplicated step ${index + 1} into position ${index + 2}.`);
  }
});

elements.macroSteps.addEventListener("keydown", (event) => {
  const handle = event.target.closest("[data-step-handle]");
  if (!handle || !event.altKey || !["ArrowUp", "ArrowDown"].includes(event.key)) return;
  event.preventDefault();
  const index = Number(handle.dataset.stepHandle);
  moveMacroStep(index, index + (event.key === "ArrowUp" ? -1 : 1));
});

function clearMacroDrag() {
  draggedMacroStep = null;
  elements.macroSteps.querySelectorAll(".dragging, .drop-before, .drop-after").forEach((card) => {
    card.classList.remove("dragging", "drop-before", "drop-after");
  });
}

elements.macroSteps.addEventListener("dragstart", (event) => {
  const handle = event.target.closest("[data-step-handle]");
  if (!handle || state.busy || !macroInputsValid(true)) {
    event.preventDefault();
    return;
  }
  stopMacroPreview("Preview stopped: reordering steps.");
  draggedMacroStep = Number(handle.dataset.stepHandle);
  event.dataTransfer.effectAllowed = "move";
  event.dataTransfer.setData("text/plain", String(draggedMacroStep));
  handle.closest("[data-step-index]").classList.add("dragging");
});

elements.macroSteps.addEventListener("dragover", (event) => {
  const card = event.target.closest("[data-step-index]");
  if (draggedMacroStep === null || !card) return;
  event.preventDefault();
  event.dataTransfer.dropEffect = "move";
  elements.macroSteps.querySelectorAll(".drop-before, .drop-after")
    .forEach((item) => item.classList.remove("drop-before", "drop-after"));
  const bounds = card.getBoundingClientRect();
  card.classList.add(event.clientY < bounds.top + bounds.height / 2 ? "drop-before" : "drop-after");
});

elements.macroSteps.addEventListener("drop", (event) => {
  const card = event.target.closest("[data-step-index]");
  if (draggedMacroStep === null || !card) return;
  event.preventDefault();
  const from = draggedMacroStep;
  const bounds = card.getBoundingClientRect();
  const gap = Number(card.dataset.stepIndex) + (event.clientY >= bounds.top + bounds.height / 2 ? 1 : 0);
  clearMacroDrag();
  moveMacroStep(from, gap > from ? gap - 1 : gap);
});

elements.macroSteps.addEventListener("dragend", clearMacroDrag);
elements.macroPreviewPlay.addEventListener("click", playMacroPreview);
elements.macroPreviewRestart.addEventListener("click", playMacroPreview);
elements.macroPreviewStop.addEventListener("click", () => stopMacroPreview("Preview stopped."));
elements.captureRecord.addEventListener("click", beginCapture);
elements.captureStop.addEventListener("click", () => stopCapture());
elements.captureUse.addEventListener("click", useCapture);
elements.captureDiscard.addEventListener("click", discardCapture);
elements.captureRecover.addEventListener("click", () => {
  const session = macroCapture.session;
  if (!session || macroCapture.requestActive) return;
  session.stopRequested = true;
  session.stopAttempted = false;
  refreshCapture();
});
elements.captureOptions.addEventListener("input", renderCapture);
elements.captureOptions.addEventListener("change", renderCapture);
window.addEventListener("pagehide", () => stopCapture("Recording stopped because you left the editor."));
window.addEventListener("hashchange", showSectionFromHash);
sectionLinks.forEach((link) => {
  link.addEventListener("click", (event) => {
    if (event.defaultPrevented || event.button !== 0 || event.metaKey || event.ctrlKey || event.shiftKey || event.altKey) return;
    // Reveal the destination before the anchor's native scrolling and focus.
    // Leave the default action intact for keyboard navigation and history.
    showSection(link.hash.slice(1));
  });
});
sectionPanels.forEach((panel) => { panel.tabIndex = -1; });
document.querySelector("#macroCaptureSettings").addEventListener("submit", (event) => {
  event.preventDefault();
});
document.addEventListener("visibilitychange", () => {
  if (document.hidden) {
    stopMacroPreview("Preview stopped: tab hidden.");
    clearMacroDrag();
    stopCapture("Recording stopped because the tab was hidden.");
    setConnection("loading", "Connection check paused");
  } else {
    setConnection("loading", "Checking adapter…");
    pollPlaytest();
    pollLibraryMetadata();
  }
  updateMacroPreviewControls();
  renderCapture();
});
elements.form.addEventListener("input", () => {
  stopMacroPreview("Preview stopped: draft edited.");
});
elements.form.addEventListener("change", () => {
  stopMacroPreview("Preview stopped: draft edited.");
});

elements.saveProfileName.addEventListener("click", async () => {
  setBusy(true);
  try {
    const value = elements.profileName.value.trim();
    await api(
      `/api/profiles/${state.identityIndex}/${state.profileIndex + 1}/name`,
      {
        method: "PUT",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ value }),
      }
    );
    state.profileNames[state.profileIndex] = value;
    state.pendingName = false;
    elements.profileTitle.textContent = value || `Profile ${state.profileIndex + 1}`;
    renderProfileList();
    updateDirtyState();
    toast("Profile name saved to Pico.");
  } catch (error) {
    toast(error.message, true);
  } finally {
    setBusy(false);
  }
});

elements.saveAlias.addEventListener("click", async () => {
  const owner = currentOwner();
  if (!owner) return;
  setBusy(true);
  try {
    const value = elements.controllerAlias.value.trim();
    const result = await api(`/api/identities/${owner.index}/alias`, {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ value }),
    });
    owner.alias = value;
    owner.label = result.label;
    renderIdentities();
    toast("Controller alias saved to Pico.");
  } catch (error) {
    toast(error.message, true);
  } finally {
    setBusy(false);
  }
});

elements.identify.addEventListener("click", async () => {
  const owner = currentOwner();
  if (!owner || owner.index === 0) return;
  setBusy(true);
  try {
    await api(`/api/identities/${owner.index}/identify`, { method: "POST" });
    toast("Identification pulse sent.");
  } catch (error) {
    toast(error.message, true);
  } finally {
    setBusy(false);
  }
});

elements.exportProfile.addEventListener("click", () => {
  const name = state.profileNames[state.profileIndex] || "";
  const filename = name || `profile-${state.profileIndex + 1}`;
  const blob = new Blob([
    JSON.stringify({ name, profile: state.profile }, null, 2),
  ], { type: "application/json" });
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob);
  link.download = `${filename.replace(/[^a-z0-9]+/gi, "-").toLowerCase()}.json`;
  link.click();
  URL.revokeObjectURL(link.href);
});

elements.importProfile.addEventListener("click", () => {
  elements.importProfileFile.click();
});

elements.importProfileFile.addEventListener("change", async () => {
  const [file] = elements.importProfileFile.files;
  if (!file) return;
  setBusy(true);
  try {
    const imported = JSON.parse(await file.text());
    if (
      typeof imported.name === "string" &&
      new TextEncoder().encode(imported.name).length > 31
    ) {
      throw new Error("Profile name exceeds 31 UTF-8 bytes");
    }
    const result = await api("/api/profiles/validate", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(imported.profile || imported),
    });
    state.profile = result.profile;
    if (typeof imported.name === "string") {
      state.profileNames[state.profileIndex] = imported.name;
      state.pendingName = true;
    }
    renderEditor();
    toast("Profile imported into the unsaved draft.");
  } catch (error) {
    toast(`Import failed: ${error.message}`, true);
  } finally {
    elements.importProfileFile.value = "";
    setBusy(false);
  }
});

elements.copyProfile.addEventListener("click", () => {
  if (!reportProfileValidity()) return;
  elements.copyIdentity.innerHTML = state.identities.map((identity) => (
    `<option value="${identity.index}">${escapeHtml(identity.label)}</option>`
  )).join("");
  elements.copyIdentity.value = String(state.identityIndex);
  elements.copySlot.innerHTML = Array.from(
    { length: state.schema.profile_capacity },
    (_, index) => `<option value="${index + 1}">Profile ${index + 1}</option>`
  ).join("");
  elements.copySlot.value = String(
    Math.min(state.schema.profile_capacity, state.profileIndex + 2)
  );
  elements.copyDialog.showModal();
});

elements.confirmCopy.addEventListener("click", async (event) => {
  event.preventDefault();
  if (!reportProfileValidity()) return;
  setBusy(true);
  try {
    await api(
      `/api/profiles/${state.identityIndex}/${state.profileIndex + 1}/copy`,
      {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          identity_index: Number(elements.copyIdentity.value),
          profile_number: Number(elements.copySlot.value),
          profile: state.profile,
          name: state.profileNames[state.profileIndex],
        }),
      }
    );
    if (Number(elements.copyIdentity.value) === state.identityIndex) {
      state.profileNames[Number(elements.copySlot.value) - 1] =
        state.profileNames[state.profileIndex];
      renderProfileList();
    }
    elements.copyDialog.close();
    toast(`Draft copied to profile ${elements.copySlot.value} on Pico.`);
  } catch (error) {
    toast(error.message, true);
  } finally {
    setBusy(false);
  }
});

document.querySelectorAll("[data-reset-section]").forEach((button) => {
  button.addEventListener("click", () => {
    const defaults = state.schema.default_profile;
    const section = button.dataset.resetSection;
    if (section === "mapping") {
      state.profile.native_joycon_layout = defaults.native_joycon_layout;
      state.profile.button_map = clone(defaults.button_map);
      state.profile.extra_button_map = clone(defaults.extra_button_map);
      state.profile.triggers.left.output = defaults.triggers.left.output;
      state.profile.triggers.right.output = defaults.triggers.right.output;
    } else if (section === "analog") {
      state.profile.swap_sticks = defaults.swap_sticks;
      state.profile.sticks = clone(defaults.sticks);
      const leftOutput = state.profile.triggers.left.output;
      const rightOutput = state.profile.triggers.right.output;
      state.profile.triggers = clone(defaults.triggers);
      state.profile.triggers.left.output = leftOutput;
      state.profile.triggers.right.output = rightOutput;
    } else if (section === "feedback") {
      state.profile.rumble = clone(defaults.rumble);
    } else if (section === "turbo") {
      state.profile.turbo = clone(defaults.turbo);
      state.profile.turbo_settings = clone(defaults.turbo_settings);
    } else if (section === "shortcuts" || section === "shift") {
      state.profile[section] = clone(defaults[section]);
    } else if (section === "macro") {
      state.profile.switching_chord = clone(defaults.switching_chord);
      state.profile.motion_toggle_chord = clone(defaults.motion_toggle_chord);
      state.profile.swing = clone(defaults.swing);
      state.profile.nunchuk_swing = clone(defaults.nunchuk_swing);
      state.profile.combined_swing = clone(defaults.combined_swing);
      state.profile.combination_window_ms = defaults.combination_window_ms;
      state.profile.macros = clone(defaults.macros);
    }
    renderEditor();
    toast(`${label(section)} reset in the unsaved draft.`);
  });
});

elements.save.addEventListener("click", async () => {
  if (!reportProfileValidity()) return;
  const error = macroBudgetError(state.profile.macros);
  if (error) {
    showSection("macro");
    window.location.hash = "macro";
    macroNotice(error, true);
    toast(error, true);
    return;
  }
  setBusy(true);
  try {
    await api(`/api/profiles/${state.identityIndex}/${state.profileIndex + 1}`, {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(state.profile),
    });
    if (state.pendingName) {
      await api(
        `/api/profiles/${state.identityIndex}/${state.profileIndex + 1}/name`,
        {
          method: "PUT",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            value: state.profileNames[state.profileIndex],
          }),
        }
      );
      state.pendingName = false;
    }
    state.original = canonical(state.profile);
    updateDirtyState();
    toast("Profile saved to Pico.");
  } catch (error) {
    toast(error.message, true);
  } finally {
    setBusy(false);
  }
});

elements.activate.addEventListener("click", async () => {
  if (!reportProfileValidity()) return;
  if (isDirty()) {
    toast("Use Save to Pico or discard the draft before activating this profile.", true);
    return;
  }
  setBusy(true);
  try {
    await api(`/api/profiles/${state.identityIndex}/${state.profileIndex + 1}/activate`, { method: "POST" });
    state.identities[state.identityIndex].active_profile = state.profileIndex + 1;
    state.active = true;
    renderEditor();
    toast(`Profile ${state.profileIndex + 1} is now active.`);
  } catch (error) {
    toast(error.message, true);
  } finally {
    setBusy(false);
  }
});

window.addEventListener("beforeunload", (event) => {
  stopMacroPreview("Preview stopped: leaving the editor.");
  stopCapture("Recording stopped because you left the editor.");
  if (!isDirty() && !macroCapture.session && !joyconModeChanged()) return;
  event.preventDefault();
  event.returnValue = "";
});

async function start() {
  try {
    const schema = await api("/api/schema");
    state.schema = schema;
    state.token = schema.mutation_token;
    await loadLibrary();
    pollPlaytest();
    pollLibraryMetadata();
  } catch (error) {
    setConnection("error", "Editor failed to start");
    elements.loading.querySelector("p").textContent = error.message;
    toast(error.message, true);
  }
}

showSectionFromHash();
start();
