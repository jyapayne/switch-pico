"use strict";

const PROFILE_OWNER_STORAGE_KEY = "switch-pico.profile-owner";

const state = {
  schema: null,
  identities: [],
  identityIndex: 0,
  profileIndex: 0,
  profile: null,
  original: "",
  active: false,
  selectedButton: "south",
  selectedMacro: 0,
  token: "",
  busy: false,
  adapterConnected: false,
  playtestRequestActive: false,
  playtestTimer: 0,
  libraryRequestActive: false,
  libraryTimer: 0,
};

const elements = {
  connection: document.querySelector("#connectionState"),
  connectionText: document.querySelector("#connectionText"),
  identity: document.querySelector("#identitySelect"),
  profileList: document.querySelector("#profileList"),
  profileOwner: document.querySelector("#profileOwner"),
  profileTitle: document.querySelector("#profileTitle"),
  activeBadge: document.querySelector("#activeBadge"),
  dirtyBadge: document.querySelector("#dirtyBadge"),
  loading: document.querySelector("#loadingCard"),
  form: document.querySelector("#profileForm"),
  controllerCanvas: document.querySelector("#controllerCanvas"),
  controllerImage: document.querySelector("#controllerImage"),
  controllerModel: document.querySelector("#controllerModel"),
  controllerCredit: document.querySelector("#controllerCredit"),
  controllerHotspots: document.querySelector("#controllerHotspots"),
  selectedMapping: document.querySelector("#selectedMapping"),
  selectedControlGlyph: document.querySelector("#selectedControlGlyph"),
  selectedControlName: document.querySelector("#selectedControlName"),
  selectedControlDescription: document.querySelector("#selectedControlDescription"),
  analog: document.querySelector("#analogFields"),
  rumble: document.querySelector("#rumbleFields"),
  builtinActions: document.querySelector("#builtinActions"),
  turbo: document.querySelector("#turboFields"),
  macroControls: document.querySelector("#macroControls"),
  macroSteps: document.querySelector("#macroSteps"),
  macroStepsTitle: document.querySelector("#macroStepsTitle"),
  playtestPanel: document.querySelector("#playtestPanel"),
  playtestTitle: document.querySelector("#playtestTitle"),
  playtestStatus: document.querySelector("#playtestStatus"),
  playtestHelp: document.querySelector("#playtestHelp"),
  playtestLeftValues: document.querySelector("#playtestLeftValues"),
  playtestRightValues: document.querySelector("#playtestRightValues"),
  playtestLeftTriggerLabel: document.querySelector("#playtestLeftTriggerLabel"),
  playtestRightTriggerLabel: document.querySelector("#playtestRightTriggerLabel"),
  refresh: document.querySelector("#refreshButton"),
  resetDraft: document.querySelector("#resetDraftButton"),
  activate: document.querySelector("#activateButton"),
  save: document.querySelector("#saveButton"),
  addMacroStep: document.querySelector("#addMacroStepButton"),
  toast: document.querySelector("#toast"),
};

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
};

function currentControllerStyle() {
  return state.identities[state.identityIndex]?.controller?.style || "generic";
}

function controlLabel(control, style = currentControllerStyle()) {
  return state.schema.control_labels?.[style]?.[control] ||
    directionalLabels[control] ||
    label(control);
}

function clone(value) {
  return JSON.parse(JSON.stringify(value));
}

function canonical(value) {
  return JSON.stringify(value);
}

const {
  transformStick,
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

function clearPlaytest(message, stateName = "waiting") {
  elements.playtestPanel.dataset.state = stateName;
  elements.playtestStatus.textContent =
    stateName === "error" ? "Unavailable" : "Waiting";
  elements.playtestTitle.textContent =
    stateName === "error" ? "Live input unavailable" : "Waiting for controller input";
  elements.playtestHelp.textContent = message;
  elements.controllerHotspots.querySelectorAll(".pressed")
    .forEach((button) => button.classList.remove("pressed"));
}

function renderPlaytest(sample) {
  if (!sample.connected) {
    clearPlaytest(
      "Connect or move a controller to compare its raw input with this draft."
    );
    return;
  }
  const rawLeft = sample.left_stick;
  const rawRight = sample.right_stick;
  const outputLeft = transformStick(rawLeft, state.profile.sticks.left);
  const outputRight = transformStick(rawRight, state.profile.sticks.right);
  const outputLeftTrigger = transformTrigger(
    sample.triggers.left, state.profile.triggers.left
  );
  const outputRightTrigger = transformTrigger(
    sample.triggers.right, state.profile.triggers.right
  );
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
  const style = sample.controller?.style || currentControllerStyle();
  elements.playtestLeftTriggerLabel.textContent =
    controlLabel("left_trigger", style);
  elements.playtestRightTriggerLabel.textContent =
    controlLabel("right_trigger", style);
  const pressed = new Set(sample.buttons);
  if (sample.triggers.left > 512) pressed.add("left_trigger");
  if (sample.triggers.right > 512) pressed.add("right_trigger");
  elements.controllerHotspots.querySelectorAll("[data-controller-button]")
    .forEach((button) => {
      button.classList.toggle(
        "pressed", pressed.has(button.dataset.controllerButton)
      );
    });
  elements.playtestPanel.dataset.state = "live";
  elements.playtestStatus.textContent = "Live";
  elements.playtestTitle.textContent = sample.label || "Connected controller";
  elements.playtestHelp.textContent =
    "Yellow is raw input; blue is the output produced by this unsaved draft.";
}

async function pollPlaytest() {
  window.clearTimeout(state.playtestTimer);
  if (
    document.hidden || state.playtestRequestActive ||
    !state.schema || !state.profile
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
      identityIndex === state.identityIndex &&
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
        `${error.message}. Flash current firmware to enable live playtest.`,
        "error"
      );
    }
  } finally {
    state.playtestRequestActive = false;
    state.playtestTimer = window.setTimeout(pollPlaytest, 75);
  }
}

function isDirty() {
  return state.profile !== null && canonical(state.profile) !== state.original;
}

function setConnection(mode, text) {
  state.adapterConnected = mode === "ready";
  elements.connection.dataset.state = mode;
  elements.connectionText.textContent = text;
  if (state.profile) {
    elements.save.disabled =
      !state.adapterConnected || state.busy || !isDirty();
    elements.activate.disabled =
      !state.adapterConnected || state.busy || state.active;
  }
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
  const response = await fetch(path, { ...options, headers });
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

function setBusy(busy) {
  state.busy = busy;
  elements.save.disabled =
    busy || !state.adapterConnected || !isDirty();
  elements.activate.disabled =
    busy || !state.adapterConnected || state.active;
  elements.resetDraft.disabled = busy;
  elements.refresh.disabled = busy;
  elements.identity.disabled = busy;
  if (state.schema && state.profile) {
    const macro = state.profile.macros[state.selectedMacro];
    const totalSteps = state.profile.macros.reduce(
      (sum, item) => sum + item.steps.length, 0
    );
    const totalBytes = state.profile.macros.reduce(
      (sum, item) => sum + item.steps.reduce(
        (stepSum, step) => stepSum + macroStepWireSize(step), 0
      ), 0
    );
    elements.addMacroStep.disabled = (
      busy || macro.steps.length >= 8 || totalSteps >= 16 ||
      totalBytes + 3 > 136
    );
  }
  document.querySelectorAll(".profile-button").forEach((button) => {
    button.disabled = busy;
  });
}

function updateDirtyState() {
  const dirty = isDirty();
  elements.dirtyBadge.hidden = !dirty;
  elements.save.disabled =
    !state.adapterConnected || state.busy || !dirty;
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
    `${entry.controller.model}:${entry.controller.style}`
  )).join("|");
}

function syncLibraryMetadata(identities, restoreStoredOwner = false) {
  const oldOwner = currentOwner();
  const oldSignature = identitySignature(state.identities);
  const preferredKey = (
    restoreStoredOwner ? storedOwnerKey() : oldOwner?.key
  ) || storedOwnerKey();
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
  }
  return ownerChanged;
}

async function pollLibraryMetadata() {
  window.clearTimeout(state.libraryTimer);
  if (
    document.hidden || state.busy ||
    state.libraryRequestActive || !state.schema
  ) {
    state.libraryTimer =
      window.setTimeout(pollLibraryMetadata, 500);
    return;
  }
  state.libraryRequestActive = true;
  try {
    const payload = await api("/api/profiles");
    syncLibraryMetadata(payload.identities);
    setConnection("ready", "Adapter connected");
  } catch {
    setConnection("error", "Adapter disconnected");
  } finally {
    state.libraryRequestActive = false;
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
      <button class="profile-button${selected ? " selected" : ""}" type="button" data-profile-index="${index}">
        <span class="profile-number"><span>${index + 1}</span>Profile ${index + 1}</span>
        ${active ? '<span class="mini-active">Active</span>' : ""}
      </button>`;
  }).join("");

  elements.profileList.querySelectorAll(".profile-button").forEach((button) => {
    button.addEventListener("click", async () => {
      const next = Number(button.dataset.profileIndex);
      if (next === state.profileIndex || !confirmDiscard()) return;
      state.profileIndex = next;
      await loadProfile();
    });
  });
}

function buttonOptions(
  selected,
  includeNone = true,
  choices = state.schema.controls,
  style = currentControllerStyle()
) {
  const none = includeNone
    ? `<option value=""${selected === null ? " selected" : ""}>None</option>`
    : "";
  return none + choices.map((button) => (
    `<option value="${button}"${selected === button ? " selected" : ""}>${escapeHtml(controlLabel(button, style))}</option>`
  )).join("");
}

const controllerArtwork = {
  generic: {
    source: "/assets/controller-xbox.svg",
    alt: "Generic Xbox-layout controller artwork",
  },
  xbox: {
    source: "/assets/controller-xbox.svg",
    alt: "Xbox controller artwork",
  },
  switch: {
    source: "/assets/controller-switch-pro.svg",
    alt: "Nintendo Switch Pro Controller artwork",
  },
  playstation: {
    source: "/assets/controller-dualsense.svg",
    alt: "Sony DualSense controller artwork",
  },
};

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
  select: "−",
  start: "+",
  capture: "▣",
  system: "⌂",
};

function controlGlyph(button, style) {
  return controllerGlyphs[style]?.[button] || fixedControlGlyphs[button] || "?";
}

function getControlMapping(button) {
  if (button === "left_trigger") return state.profile.triggers.left.output;
  if (button === "right_trigger") return state.profile.triggers.right.output;
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
  } else {
    state.profile.button_map[button] = output;
  }
}


function renderButtonMap() {
  const owner = state.identities[state.identityIndex];
  const style = owner?.controller?.style || "generic";
  const artwork = controllerArtwork[style] || controllerArtwork.generic;
  const selected = state.selectedButton;
  const mappedOutput = getControlMapping(selected);

  elements.controllerCanvas.dataset.style = style;
  elements.controllerImage.src = artwork.source;
  elements.controllerImage.alt = artwork.alt;
  elements.controllerModel.textContent = owner?.controller?.model || "Generic controller";
  elements.controllerCredit.href = "https://github.com/AL2009man/Gamepad-Asset-Pack";
  elements.controllerCredit.textContent = "Controller artwork by Al. Lopez · MIT";

  elements.controllerHotspots.querySelectorAll("[data-controller-button]").forEach((hotspot) => {
    const button = hotspot.dataset.controllerButton;
    const output = getControlMapping(button);
    hotspot.textContent = controlGlyph(button, style);
    hotspot.classList.toggle("selected", button === selected);
    hotspot.classList.toggle("disabled-map", output === null);
    hotspot.title = `${controlLabel(button, style)} → ${output === null ? "Disabled" : controlLabel(output, style)}`;
    hotspot.setAttribute("aria-label", hotspot.title);
    hotspot.onclick = () => {
      state.selectedButton = button;
      renderButtonMap();
    };
  });

  elements.selectedControlGlyph.textContent = controlGlyph(selected, style);
  elements.selectedControlName.textContent = controlLabel(selected, style);
  elements.selectedControlDescription.textContent = (
    `Physical ${controlLabel(selected, style)} currently produces ${mappedOutput === null ? "no output" : controlLabel(mappedOutput, style)}.`
  );
  elements.selectedMapping.innerHTML = buttonOptions(mappedOutput, true, state.schema.controls, style);
  elements.selectedMapping.onchange = () => {
    setControlMapping(selected, elements.selectedMapping.value || null);
    renderButtonMap();
    updateDirtyState();
  };
}

const analogDefinitions = [
  ["sticks", "left", "Left stick", [
    ["center_x", "Center X", -32768, 32767],
    ["center_y", "Center Y", -32768, 32767],
    ["inner_deadzone", "Inner deadzone", 0, 32767],
    ["outer_saturation", "Outer saturation", 1, 32767],
    ["curve_q8_8", "Curve (Q8.8)", 1, 65535],
  ]],
  ["sticks", "right", "Right stick", [
    ["center_x", "Center X", -32768, 32767],
    ["center_y", "Center Y", -32768, 32767],
    ["inner_deadzone", "Inner deadzone", 0, 32767],
    ["outer_saturation", "Outer saturation", 1, 32767],
    ["curve_q8_8", "Curve (Q8.8)", 1, 65535],
  ]],
  ["triggers", "left", "Left trigger", [
    ["lower_deadzone", "Lower deadzone", 0, 65535],
    ["upper_saturation", "Upper saturation", 1, 65535],
    ["curve_q8_8", "Curve (Q8.8)", 1, 65535],
    ["digital_threshold", "Digital threshold", 0, 65535],
  ]],
  ["triggers", "right", "Right trigger", [
    ["lower_deadzone", "Lower deadzone", 0, 65535],
    ["upper_saturation", "Upper saturation", 1, 65535],
    ["curve_q8_8", "Curve (Q8.8)", 1, 65535],
    ["digital_threshold", "Digital threshold", 0, 65535],
  ]],
];

function renderAnalog() {
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
      <div class="subpanel">
        <div class="subpanel-heading"><h4>${title}</h4><span>${group === "sticks" ? "Signed axes · 32767 full scale" : "Unsigned · 65535 full scale"}</span></div>
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

function renderTurbo() {
  elements.turbo.innerHTML = state.schema.buttons.map((button) => `
    <div class="control-card">
      <label for="turbo-${button}">${controlLabel(button)}</label>
      <select class="select" id="turbo-${button}" data-kind="turbo" data-name="${button}">
        ${state.schema.turbo_modes.map((mode) => `<option value="${mode}"${state.profile.turbo[button] === mode ? " selected" : ""}>${label(mode)}</option>`).join("")}
      </select>
    </div>`).join("");
}

function macroNumber(index, field, value, min, max, title, disabled = false) {
  return `
    <div class="number-field">
      <label>${title}</label>
      <input class="number-input" type="number" required min="${min}" max="${max}" step="1" value="${value}" data-kind="macro-number" data-index="${index}" data-field="${field}"${disabled ? " disabled" : ""}>
    </div>`;
}

function actionChordCard(action, title, description, selectedButtons, defaults) {
  const inherited = selectedButtons.length === 0 && defaults?.length;
  const effectiveButtons = inherited ? defaults : selectedButtons;
  const selected = new Set(effectiveButtons);
  const defaultText = inherited
    ? `Using default: ${defaults.map((button) => controlLabel(button)).join(" + ")}.`
    : defaults?.length
      ? `Clear every selection to restore ${defaults.map((button) => controlLabel(button)).join(" + ")}.`
      : "Empty disables this action.";
  return `
    <div class="action-card">
      <div class="action-card-heading">
        <div><span class="action-kind">${action === "custom_macro" ? "Custom macro" : "Built-in action"}</span><h4>${title}</h4></div>
        <span>${description}</span>
      </div>
      <div class="check-grid controller-choices">
        ${state.schema.controls.map((button) => `
          <label class="checkbox-pill controller-choice">
            <input type="checkbox" data-kind="action-chord" data-action="${action}" data-name="${button}"${selected.has(button) ? " checked" : ""}>
            <span data-button="${button}">
              <b>${controlGlyph(button, state.identities[state.identityIndex]?.controller?.style || "generic")}</b>
              <small>${controlLabel(button)}</small>
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

function renderMacro() {
  const controllerStyle =
    state.identities[state.identityIndex]?.controller?.style || "generic";
  elements.builtinActions.dataset.controllerStyle = controllerStyle;
  elements.macroControls.dataset.controllerStyle = controllerStyle;
  const macro = state.profile.macros[state.selectedMacro];
  const totalSteps = state.profile.macros.reduce(
    (sum, item) => sum + item.steps.length, 0
  );
  const totalBytes = state.profile.macros.reduce(
    (sum, item) => sum + item.steps.reduce(
      (stepSum, step) => stepSum + macroStepWireSize(step), 0
    ), 0
  );
  elements.builtinActions.innerHTML = [
    actionChordCard(
      "profile_switch",
      "Cycle active profile",
      `Advance through profile slots 1–${state.schema.profile_capacity}.`,
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
  elements.macroControls.innerHTML = `
    <div class="macro-picker">
      <div class="macro-tabs">
        ${state.profile.macros.map((item, index) => `
          <button type="button" class="macro-tab${index === state.selectedMacro ? " selected" : ""}" data-macro-index="${index}">
            Macro ${index + 1}<small>${item.steps.length} step${item.steps.length === 1 ? "" : "s"}</small>
          </button>`).join("")}
      </div>
      <div class="macro-budget">
        <strong>${totalSteps}/16 steps</strong>
        <span>${totalBytes}/136 sparse bytes</span>
        <progress max="136" value="${totalBytes}"></progress>
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
        ${buttonOptions(macro.cancel, true, state.schema.controls, controllerStyle)}
      </select>
    </div>`;

  elements.macroControls.querySelectorAll("[data-macro-index]").forEach((button) => {
    button.onclick = () => {
      state.selectedMacro = Number(button.dataset.macroIndex);
      renderMacro();
    };
  });

  elements.macroSteps.innerHTML = macro.steps.length === 0
    ? '<div class="macro-step end"><p class="field-help">No steps yet. Add a state step to build this macro.</p></div>'
    : macro.steps.map((step, index) => {
      const overrides = new Set(step.overrides);
      const outputButtons = new Set(step.output_buttons);
      return `
        <div class="macro-step">
          <div class="step-header">
            <span class="step-number">Macro ${state.selectedMacro + 1} · step ${index + 1} · ${macroStepWireSize(step)} bytes</span>
            <button class="remove-step" type="button" data-remove-step="${index}">Remove</button>
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
            <span class="step-group-title">Fields this step overrides</span>
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
                  <span>${controlLabel(button, controllerStyle)}</span>
                </label>`).join("")}
            </div>
          </div>
        </div>`;
    }).join("");
  elements.macroStepsTitle.textContent = `Macro ${state.selectedMacro + 1} steps`;
  elements.addMacroStep.textContent = `Add step to macro ${state.selectedMacro + 1}`;
  elements.addMacroStep.disabled = (
    state.busy || macro.steps.length >= 8 || totalSteps >= 16 ||
    totalBytes + 3 > 136
  );
}

function renderEditor() {
  elements.profileTitle.textContent = `Profile ${state.profileIndex + 1}`;
  elements.activeBadge.hidden = !state.active;
  elements.activate.disabled =
    !state.adapterConnected || state.busy || state.active;
  renderIdentities();
  renderProfileList();
  renderButtonMap();
  renderAnalog();
  renderFeedback();
  renderTurbo();
  renderMacro();
  elements.loading.hidden = true;
  elements.form.hidden = false;
  updateDirtyState();
}

async function loadProfile() {
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
    state.active = payload.active;
    setConnection("ready", "Adapter connected");
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

async function loadLibrary() {
  setBusy(true);
  try {
    const payload = await api("/api/profiles");
    syncLibraryMetadata(payload.identities, true);
    await loadProfile();
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
  if (!kind || !state.profile) return;
  if (kind === "analog-number") {
    state.profile[target.dataset.group][target.dataset.side][target.dataset.field] = Number(target.value);
  } else if (kind === "analog-bool") {
    state.profile[target.dataset.group][target.dataset.side][target.dataset.field] = target.checked;
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
    const current = owner[field].length === 0 && defaults
      ? defaults
      : owner[field];
    const selected = new Set(current);
    target.checked ? selected.add(target.dataset.name) : selected.delete(target.dataset.name);
    owner[field] = state.schema.controls.filter((name) => selected.has(name));
  } else if (kind === "turbo") {
    state.profile.turbo[target.dataset.name] = target.value;
  } else if (kind === "macro-selector") {
    state.profile.macros[state.selectedMacro][target.dataset.field] = target.value || null;
  } else if (kind === "macro-number") {
    updateNestedStep(state.profile.macros[state.selectedMacro].steps[Number(target.dataset.index)], target.dataset.field, Number(target.value));
  } else if (kind === "macro-output") {
    const step = state.profile.macros[state.selectedMacro].steps[Number(target.dataset.index)];
    const selected = new Set(step.output_buttons);
    target.checked ? selected.add(target.dataset.name) : selected.delete(target.dataset.name);
    step.output_buttons = state.schema.buttons.filter((name) => selected.has(name));
  } else if (kind === "macro-override") {
    const step = state.profile.macros[state.selectedMacro].steps[Number(target.dataset.index)];
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
    renderMacro();
  }
  updateDirtyState();
}

elements.form.addEventListener("input", handleFormChange);
elements.form.addEventListener("change", handleFormChange);

elements.identity.addEventListener("change", async () => {
  const previous = state.identityIndex;
  if (!confirmDiscard()) {
    elements.identity.value = String(previous);
    return;
  }
  state.identityIndex = Number(elements.identity.value);
  const owner = currentOwner();
  if (owner) persistOwnerKey(owner.key);
  state.profileIndex = 0;
  await loadProfile();
});

elements.refresh.addEventListener("click", async () => {
  if (confirmDiscard()) await loadLibrary();
});

elements.resetDraft.addEventListener("click", () => {
  if (!state.schema || !window.confirm("Replace this draft with the default profile? Nothing is saved until you choose Save to Pico.")) return;
  state.profile = clone(state.schema.default_profile);
  renderEditor();
  toast("Default profile loaded into the draft.");
});

elements.addMacroStep.addEventListener("click", () => {
  const steps = state.profile.macros[state.selectedMacro].steps;
  const totalSteps = state.profile.macros.reduce(
    (sum, macro) => sum + macro.steps.length, 0
  );
  const totalBytes = state.profile.macros.reduce(
    (sum, macro) => sum + macro.steps.reduce(
      (stepSum, step) => stepSum + macroStepWireSize(step), 0
    ), 0
  );
  if (steps.length >= 8 || totalSteps >= 16 || totalBytes + 3 > 136) return;
  steps.push({
    type: "state",
    overrides: ["buttons"],
    duration_ms: 100,
    output_buttons: [],
    left_stick: { x: 0, y: 0 },
    right_stick: { x: 0, y: 0 },
    triggers: { left: 0, right: 0 },
  });
  renderMacro();
  updateDirtyState();
});

elements.macroSteps.addEventListener("click", (event) => {
  const button = event.target.closest("[data-remove-step]");
  if (!button) return;
  state.profile.macros[state.selectedMacro].steps.splice(Number(button.dataset.removeStep), 1);
  renderMacro();
  updateDirtyState();
});

elements.save.addEventListener("click", async () => {
  if (!elements.form.reportValidity()) return;
  setBusy(true);
  try {
    const result = await api(`/api/profiles/${state.identityIndex}/${state.profileIndex + 1}`, {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(state.profile),
    });
    state.original = canonical(state.profile);
    updateDirtyState();
    toast(`Saved atomically · generation ${result.stored_generation} · CRC ${result.stored_crc}`);
  } catch (error) {
    toast(error.message, true);
  } finally {
    setBusy(false);
  }
});

elements.activate.addEventListener("click", async () => {
  if (isDirty()) {
    toast("Save or discard the draft before activating this profile.", true);
    return;
  }
  setBusy(true);
  try {
    const result = await api(`/api/profiles/${state.identityIndex}/${state.profileIndex + 1}/activate`, { method: "POST" });
    state.identities[state.identityIndex].active_profile = state.profileIndex + 1;
    state.active = true;
    renderEditor();
    toast(`Profile ${state.profileIndex + 1} is active · generation ${result.stored_generation}`);
  } catch (error) {
    toast(error.message, true);
  } finally {
    setBusy(false);
  }
});

window.addEventListener("beforeunload", (event) => {
  if (!isDirty()) return;
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

start();
