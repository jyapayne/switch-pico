# USB Wireless Adapter Feature-Parity Plan

## Goal

Turn the Pico 2 W all-in-one firmware into an open, four-controller superset of the functional features advertised for the 8BitDo USB Wireless Adapter 2.

The target is feature parity, not binary compatibility with 8BitDo Ultimate Software or its private USB configuration protocol. Configuration will use this project's own documented USB management protocol and host tools.

Official feature references:

- [USB Wireless Adapter 2 product page](https://www.8bitdo.com/usb-wireless-adapter-2/)
- [USB Wireless Adapter 2 FAQ](https://support.8bitdo.com/faq/wireless-usb-adapter-2.html)
- [USB Adapter 2 Ultimate Software](https://support.8bitdo.com/ultimate/usb-adapter-2.html)
- [DualSense Adapter 2 manual](https://support.8bitdo.com/Manual/USB-Adapter-2/ps5-switch.html)
- [Switch Pro Adapter 2 manual](https://support.8bitdo.com/Manual/USB-Adapter-2/switchpro-switch.html)

## Scope decisions

- Primary target: Pico 2 W AIO firmware with direct Bluetooth input.
- Preserve the existing UART/Pico build and behavior.
- Preserve four concurrent Bluetooth slots where the output protocol and host support them.
- Implement equivalent mapping, tuning, profiles, macros, Turbo, output modes, firmware updating, and compatibility.
- Use fixed-capacity state and no allocation in real-time input, macro, rumble, or USB-report paths.
- Keep settings and Bluetooth bond storage separate.
- DualShock 3 support is explicitly out of scope.
- Proprietary 2.4 GHz controllers are out of scope; Bluetooth Classic and BLE controllers are in scope.
- Do not impersonate vendor USB identities; compatibility modes use project-owned or clearly development-only identities.
- PlayStation Classic and Mega Drive output modes are explicitly out of scope.

The official adapter also does not support the following, so they are not parity requirements:

- waking the Switch
- controller audio/headphone jacks
- NFC/amiibo
- IR camera
- DualSense adaptive triggers
- native DualSense haptics
- exact Nintendo HD-rumble frequency and spatial fidelity on conventional-motor controllers
- analog triggers while exposed as a Switch Pro Controller

## Current architecture

The AIO firmware currently has:

- Bluepad32, BTstack, and the CYW43439 radio on Core 1
- TinyUSB and USB controller state machines on Core 0
- four fixed Bluetooth slots mapped to four USB interfaces
- generation-tagged state snapshots and rumble mailboxes across cores
- persistent Classic and BLE bonding
- a physical BOOTSEL pairing window and pairing reset
- versioned endpoint-zero configuration and pairing management
- Switch Pro input, motion, colors, and rumble per slot
- per-controller ABXY and motion hotkeys

The Bluetooth, UART, Switch, and XInput paths now share `ControllerState`:

- face buttons use positional names instead of protocol labels
- sticks use signed full-range axes with zero at rest
- triggers retain their full 16-bit analog values
- motion samples and generation-based consumption remain per slot
- normalized conventional rumble uses `ControllerRumbleOutput`
- Switch and XInput serializers own their protocol-specific mappings

## Progress and gap matrix

| Capability | Status | Evidence or remaining work |
|---|---|---|
| Bluetooth Classic and BLE | Complete | Bluepad32 supports both transports; bonds persist across reboot. |
| Pairing gate, reconnect, list, and clear | Complete | Physical BOOTSEL flow and `switch-pico-config pairings` use the versioned management protocol. |
| Four concurrent controllers | Complete for current output modes | Switch, XInput, DInput, and Mac expose four isolated interfaces; current host-specific limits remain qualification items. |
| Switch input | Complete | Buttons, sticks, lifecycle, colors, and per-slot isolation are hardware-tested. |
| Switch motion | Complete for supported parsers | DualSense, Switch-family, Wii accelerometer, PS Move, and compatible 8BitDo modes are normalized. |
| Switch rumble | Complete for tested controllers | DualSense and 8BitDo Ultimate Bluetooth are hardware-tested; Ultimate requires enable, fixed LRA frequencies, and refresh. |
| 8BitDo Ultimate reconnect | Complete | Bond preservation, scan restart, and four-second supervision timeout are implemented. |
| XInput descriptors and reports | Complete with development identity | Production-neutral XInput driver and reconnect-safe XUSB binding pass four-slot Windows hardware tests; replace `CAFE:4010` before release. |
| Automatic Windows/Switch selection | Complete | `auto` preserves the verified Switch probe → XInput reboot and manual modes bypass probing. |
| Windows output tests | Complete for XInput and DInput | Real Windows passed four XInput slots with controls/rumble isolation and four DInput interfaces with complete input. |
| Protocol-neutral controller state | Complete | `ControllerState` is shared by Bluetooth, UART, Switch, and XInput paths; analog trigger precision is retained. |
| Persistent configuration protocol | Complete | Adapter settings and identity-keyed profiles use separate two-copy CRC/generation stores with bounded transactions and recovery. |
| Production USB VID/PID | Missing | Prototype uses `CAFE:4010`; obtain an appropriate project VID/PID and repeat Windows binding tests. |
| DInput output | Complete, input-only | Four generic HID interfaces pass Linux and Windows controls; USB HID does not provide generic rumble. |
| Mac output mode | Complete, input-only | Four generic HID interfaces pass real macOS sticks, buttons, D-pad, and independent analog DualSense triggers. |
| Manual output-mode selection | Complete | Persistent PC command and three-second controller chord select auto/Switch/XInput/DInput/Mac; ten-second BOOTSEL reset restores auto. |
| General button remapping | Complete | Sixteen positional logical inputs map directly to supported logical outputs per profile. |
| Stick sensitivity | Complete | Per-stick center calibration, inner deadzone, outer saturation, fixed-point curve, and inversion run before every output serializer. |
| Trigger ranges | Complete for current outputs | Per-trigger deadzone, saturation, curve, and digital threshold preserve analog XInput values and configured Switch thresholds. |
| Vibration intensity | Complete | Independent weak/strong profile scales apply to host rumble; local confirmation policy remains separate. |
| Macros | Complete | Four sequences share sixteen steps and 136 sparse bytes; Once, While held, Toggle and bounded Repeat are supported. |
| Turbo, Auto Burst and finite Burst | Complete | Configurable 1–30 Hz and 1–99% duty, shared defaults/sparse overrides, and 1–255 finite pulses use bounded phase arithmetic. |
| Persistent profiles | Complete | Eight schema-6/384-byte profiles per global/stable identity use a two-arena indexed catalog with migration and atomic publication. |
| Profile switching | Complete | PC activation, retained cycle chord and eight direct modifier shortcuts publish feedback only after commit. |
| Firmware updater | Partial | UF2 updating works; version query and guided reboot/install tool are missing. |
| Switch 2 | Unverified | Requires real-hardware qualification. |
| Windows/SteamOS/Linux/Android compatibility | Partial | Windows XInput feasibility passed; other host/output combinations need qualification. |
| Full advertised controller matrix | Partial | Parsers exist for most families, but model-level hardware coverage is incomplete. |
| Latency claim | Unmeasured | Establish p50/p95/p99 measurements and compare against the current build and Adapter 2. |

## Completed feasibility work

Branch: `feasibility/adapter-parity`

Important commits:

- `17b3d73` — initial four-interface XInput prototype
- `7e3be86` — hardware-verified automatic mode switching
- `8351cb1` — strong 8BitDo Switch-mode rumble on master
- `db4a860` — Windows enumeration and rumble fixes

Feasibility behavior:

1. The Pico cold-boots with a Switch-compatible probe identity.
2. Windows requests the Microsoft OS `MSFT100` descriptor and `XUSB10` compatible IDs.
3. A Pico hardware alarm allows the control response to finish, then performs a one-shot watchdog reboot.
4. The reboot selects four XInput interfaces using watchdog scratch state.
5. The marker is consumed after boot so reset loops cannot persist.
6. A cold power cycle returns to the initial probe state.

The final Windows fix uses a distinct probe device revision because Windows caches Microsoft OS descriptor support by VID, PID, and `bcdDevice`. This prevents a cached result for a genuine Switch Pro Controller from suppressing the feasibility probe.

Current test artifact:

- `firmware/switch-pico-adapter-feasibility.uf2`

Windows test command:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\tools\Test-AdapterFeasibility.ps1
```

The script should be started with the Pico disconnected. It records the Switch-to-XInput transition, checks all four XInput API slots, requires D-pad/face/shoulder/trigger/stick activity for each requested physical controller, tests per-slot rumble, and writes a JSON report to `%TEMP%`.

### Bluetooth discovery latency regression

Hardware bisect established `edf6eca` (`Add dual-controller AIO USB
transport`) as the first commit with delayed IMU and rumble. The preceding
`7941294` build was responsive. The extra USB interface was not the cause.
`edf6eca` changed the Bluetooth policy so Classic inquiry and BLE scanning
continued whenever any controller slot was free. With one controller active
and another slot empty, discovery consumed CYW43439 radio time and delayed HID
input and output traffic.

Commit `7449d6d` fixes the regression with three connection-policy states:

- **Open:** active discovery and incoming connections; used with zero active
  controllers or while the explicit BOOTSEL pairing window is open.
- **Passive:** active discovery stopped, incoming connections allowed; used
  whenever at least one controller is active and a slot remains free.
- **Paused:** discovery and incoming connections stopped because all slots are
  occupied.

The diagnostic proof kept the dual-controller USB build unchanged and only
stopped discovery after the first controller became ready; IMU responsiveness
immediately returned. The production policy was then verified on the current
four-controller master build, with both IMU and rumble reported good.

Important tradeoff: controllers that initiate their own reconnect can join
while the firmware is passive. Controllers that require host-side discovery,
including an additional 8BitDo Ultimate, require opening the BOOTSEL pairing
window while another controller is active. Do not restore continuous inquiry
as a convenience feature; it causes gameplay latency.

## Implementation principles

### Protocol-neutral state

Create one controller state independent of every USB output protocol. It should contain:

- positional face buttons
- shoulders, stick clicks, Select/Start, Home, and Capture
- D-pad
- signed full-resolution left and right sticks
- full-resolution analog triggers
- motion samples
- battery and capability metadata when provided
- stable controller identity
- connection generation

The processing order is fixed:

```text
Bluetooth/raw input
  -> parser normalization and calibration
  -> stick and trigger transforms
  -> button mapping
  -> Turbo / Auto Burst / macros
  -> selected USB output driver
```

Rumble travels in the opposite direction:

```text
host output report
  -> protocol decoder
  -> profile intensity scaling
  -> slot-generation check
  -> controller-specific Bluetooth rumble
```

Required invariants:

- A disconnect publishes neutral immediately.
- A profile or output-mode change cancels synthetic macro/Turbo state before switching.
- No command from an old connection generation reaches a replacement controller.
- One slot cannot mutate another slot's state, macro engine, profile, motion, or rumble.
- The normal UART build retains its existing wire protocol and behavior.

### USB output-driver boundary

Each output driver should provide a small static interface for:

- selecting device/configuration/string/report descriptors
- initializing and resetting per-interface state
- serializing protocol-neutral controller state
- consuming host output reports
- exposing readiness and capabilities

Do not use heap allocation or virtual dispatch. Select one descriptor family before TinyUSB initialization. A mode change persists the selection and reboots; descriptors must not mutate while mounted.

## Delivery phases

### Phase 1 — Protocol-neutral controller state — Complete

Changes:

- Introduce a controller-neutral state type.
- Preserve analog trigger values from Bluepad32.
- Refactor the Switch driver into a consumer of that state.
- Adapt the UART path without changing its external protocol.
- Keep existing per-slot generation and motion-consumption behavior.

Acceptance:

- Existing Switch hardware behavior is unchanged.
- Analog trigger endpoints and midpoint stick values survive the backend boundary.
- Four slots remain isolated during connect, disconnect, and replacement.
- Current pairing, motion, rumble, and descriptor tests pass.
- UART and AIO firmware both build.

Completion evidence:

- 49 native tests passed, including analog trigger and Switch threshold boundaries
- UART, AIO, and feasibility firmware built successfully
- XInput hardware exposed full-range sticks and 8-bit analog triggers
- XInput rumble passed on the 8BitDo Ultimate
- Switch buttons, sticks, ZL/ZR, motion, and rumble matched the fixed master baseline
- the scan-latency regression was bisected, fixed, documented, and retested before acceptance

### Phase 2 — Persistent configuration protocol — Core complete

Generalize endpoint-zero management beyond pairing while keeping Switch USB enumeration unchanged.

The core delivery implements firmware/board/active-mode status, one
versioned adapter configuration object, transactional writes, and pairing
management. Connected-controller metadata and profile operations are added
with the controller identity and profile schemas in Phases 3 and 5. Reboot
operations remain with the guided updater in Phase 6.

Operations:

- firmware and board version
- active output mode
- connected controller identity and capabilities
- profile read/write/list/reset
- active profile selection
- configuration export/import
- normal reboot and reboot to BOOTSEL

Large values use a transaction:

1. begin with schema version, size, and CRC
2. upload bounded chunks
3. validate completeness and CRC
4. atomically commit
5. return stored generation and CRC

Storage requirements:

- versioned schema
- fixed maximum sizes
- CRC validation
- two-copy or journaled commit
- recovery to the last valid generation after interrupted writes
- separate flash region from Bluepad32 bond storage
- bounded write frequency

Current host tooling:

```text
switch-pico-config status
switch-pico-config config show
switch-pico-config config set --pairing-window-seconds 90
switch-pico-config config reset --yes
switch-pico-config pairings list
switch-pico-config pairings clear --yes
```

Acceptance:

- Malformed, truncated, out-of-order, oversized, and bad-CRC requests are rejected.
- Interrupted writes retain the previous valid configuration.
- Pairing management migrates to the versioned protocol in the same cutover.
- Configuration survives power cycling on hardware.

Core completion evidence:

- the configuration record has version, size, generation, payload CRC, and
  header CRC fields with a fixed 512-byte payload ceiling
- two dedicated Pico flash sectors sit immediately before, and cannot overlap,
  BTstack's two-sector bond store
- writes target the inactive copy, verify after programming, preserve the old
  copy until validation, skip identical values, and allow at most one changed
  commit per second
- native tests reject malformed, truncated, out-of-order, oversized,
  unsupported-schema, and bad-CRC data and recover from corrupt or interrupted
  writes
- pairing list/refresh/clear moved from the old pairing-only requests into the
  versioned envelope and passed on hardware with two stored Classic bonds
- 47 tests passed; UART, AIO, and feasibility firmware built
- generation 1 with a 90-second pairing window survived a physical power cycle
  and feasibility firmware reflash, after which reset stored the 60-second
  default as generation 2

### Phase 3 — Mapping, tuning, profiles, and macros — Complete

Use eight profile slots per stable controller identity. Resolve identity from Bluetooth transport, identity address, VID, and PID; use a global default when stable identity is unavailable.

Button mapping:

- map any exposed logical button to a supported logical output
- reject recursive/invalid mappings at configuration time
- preserve controller-specific inputs such as touchpad click where the parser exposes them

Sticks:

- independent inner deadzone
- independent outer saturation
- fixed-point response curve
- optional axis inversion
- optional center calibration

Triggers:

- lower deadzone
- upper saturation
- fixed-point response curve
- digital threshold for protocols such as Switch
- analog output for XInput and DInput

Vibration:

- independent weak/strong scale
- saturation after scaling
- separate local confirmation policy

Macros:

- fixed maximum step count
- press/release logical buttons
- set/clear D-pad
- optional stick/trigger values
- bounded wait
- explicit end
- deterministic monotonic scheduler

Cancellation must publish neutral synthetic state on:

- controller disconnect
- profile change
- output-mode change
- configured trigger cancellation
- configuration reset

Turbo behavior:

- Turbo: 15 activations per second while held
- Auto Burst: 15 activations per second after one press until explicitly cancelled
- phase accumulator prevents scheduling jitter from changing the long-term rate

Profile switching:

- configurable controller chord
- one to eight rumble pulses
- matching onboard LED count
- controller RGB/player LED feedback when supported

Acceptance:

- Simulated-clock tests cover exact transitions and cancellation.
- Boundary tests cover deadzones, saturation, curves, and thresholds.
- Physical input plus macro plus Turbo precedence is deterministic.
- No synthetic input remains stuck after any cancellation path.
- Four controllers can use different profiles simultaneously.

Completion evidence:

- strict 256-byte profile records and a compact indexed catalog support eight
  profiles for the global fallback and each of sixteen stable identities
- profile and adapter stores remain separate from each other and BTstack bonds;
  profile commits append one flash-safe record and compact atomically between
  two 128 KiB arenas
- legacy fixed profile databases migrate without losing custom thresholds,
  identities, active profiles, or other data
- direct mapping, stick/trigger fixed-point transforms, Switch thresholds,
  XInput analog values, rumble scaling, macros, Turbo, Auto Burst, and all
  cancellation paths have deterministic native coverage
- the AIO backend uses BTstack GAP connection type for stable Classic/BLE
  identity, not the stale cached Bluepad protocol field
- profile switching is serialized against host writes, applies only after
  commit, and uses generation-safe per-slot feedback
- profile colors/player counts are transient on connect/switch and restore the
  persistent USB slot indication after the final 75 ms gap
- 70 tests passed; UART, AIO, and feasibility variants linked and published
- hardware verified separate 8BitDo `057e:2009` and DualSense `054c:0ce6`
  identities, isolated active profiles and feedback, persistent 2.2-second
  profile commits, button remapping, stick tuning, 15 Hz Turbo, a releasing
  macro, and transient profile-to-slot LED restoration

### Phase 4 — Production USB output modes — Complete

This is shorter than Phase 3 because Switch and XInput already work, but it is
not a small change: USB descriptors are fixed before `tusb_init()`, mode changes
require a persisted reboot, and DInput/Mac need real-host qualification.

Release blocker: replace development identity `CAFE:4010` with an appropriate
project VID/PID before calling XInput production-ready. Development can continue
with the current identity.

#### 4A — Output-driver boundary and production XInput

- replace `SWITCH_PICO_ADAPTER_FEASIBILITY` branches in `src/firmware/main.cpp` with
  one fixed static driver interface: descriptors, init/reset, input, task,
  readiness/capabilities, and host output
- keep the Phase 3 profile/runtime transform exactly once before serialization
- move the verified four-slot XInput implementation out of feasibility naming
- preserve current Switch descriptors, handshake, motion, and rumble bit-for-bit

Gate: descriptor/report golden tests plus current Switch hardware and Windows
four-slot XInput regression. No mode-selection work lands before this boundary
is behavior-equivalent.

#### 4B — Persistent mode selection and recovery

- extend `AdapterConfiguration` with `auto`, `switch`, `xinput`, `dinput`, and
  `mac`; migrate the existing schema without losing profiles or bonds
- add `switch-pico-config mode MODE`
- select one descriptor family before `tusb_init()`; a changed mode commits,
  cancels synthetic state, acknowledges, and watchdog-reboots
- keep the verified Windows/Switch probe only for `auto`
- add a distinct three-second controller chord for mode cycling
- make the existing destructive ten-second BOOTSEL reset also restore `auto`,
  providing physical recovery from a bad manual mode

Gate: persistence, power-cycle, interrupted-write, reboot-loop, recovery, and
no-phantom-controller tests on Switch and Windows.

#### 4C — Generic HID outputs

- implement four-interface DInput generic HID first
- capture DInput rumble requirements before promising force feedback; Windows
  HID PID force feedback is not a free consequence of a gamepad descriptor
- Mac mode remains generic HID under the project development identity
- use X/Y/Z/Rx for left/right sticks so generic macOS axis indices 0..3 are
  stable
- expose independent analog triggers through Simulation Controls Brake and
  Accelerator usages; verify raw IOHID values and GameController behavior
- do not duplicate profile transforms inside either output driver

Gate: descriptor/report/output tests, then real Windows DInput and macOS
enumeration, controls, reconnect, and four-controller isolation. Any unavailable
Mac hardware remains an explicit qualification blocker, not an inferred pass.

Final acceptance:

- cold Switch boot never enters the Windows path
- Windows `auto` reaches four XInput slots without phantom devices
- manual modes survive power cycles and recover through BOOTSEL reset
- profiles, macros, Turbo, motion, and per-slot rumble remain isolated in every
  supported mode
- each mode passes real-target enumeration and complete input checks

Completion evidence:

- all TinyUSB callbacks are centralized in one static output-driver boundary;
  Switch, XInput, DInput, and Mac serializers consume the same Phase 3 output
- adapter configuration schema v2 migrates v1 to `auto`; selection is loaded
  before `tusb_init()`, persists across power cycles, and reboots only after a
  correlated atomic commit
- physical recovery reserves configuration ownership, clears pairings with an
  exact monotonic completion token, restores `auto`, and avoids stale reboots
- XInput was promoted from feasibility naming and its Windows XUSB binding was
  fixed for reconnect; real Windows passed four slots, controls, analog
  triggers, and rumble isolation
- real Windows DInput passed four interfaces, buttons, D-pad, both sticks,
  independent triggers, and reconnect; generic HID intentionally has no rumble
- real macOS passed four interfaces, both sticks, buttons, D-pad, and
  independent analog DualSense triggers under project identity `CAFE:4021`
- DualSense/DS4 analog brake/throttle now outrank simultaneous digital trigger
  bits; digital-only controllers retain full-scale fallback
- XInput carries Home as Guide `0x0400` and Capture as de-facto Share `0x0800`;
  standard `XInputGetState` does not expose either portably
- the CYW43 HCI drain is bounded to 16 packets per poll so continuous
  multi-controller traffic cannot starve application or controller-parser
  timers; live diagnostics expose timer, report, queue, and dispatch counters
- hardware verified that DualSense and 8BitDo rumble both start and stop under
  two-controller traffic, with timer counters continuing to advance
- the final cold Switch boot passed controls, motion, profiles, and rumble in
  `auto` / Switch-probe mode
- 107 tests passed; UART, AIO, and feasibility artifacts linked and published
- release remains blocked on an appropriate production project VID/PID

### Phase 5 — Controller compatibility

Target hardware families:

| Family | Required checks |
|---|---|
| 8BitDo Bluetooth controller | pair, reconnect, complete input, profile, rumble |
| 8BitDo Bluetooth arcade stick | buttons, stick/D-pad mode, reconnect |
| Xbox One Bluetooth | BLE pairing, analog triggers, rumble |
| Xbox Series | BLE pairing, analog triggers, rumble |
| DualSense | input, Switch motion, rumble, lightbar |
| DualShock 4 | input, Switch motion, rumble, lightbar |
| Switch Pro | input, motion, rumble, player LED |
| Joy-Con L/R | each half as the standalone controller exposed by Bluepad32 |
| Wii Remote | buttons, accelerometer, rumble |
| Wii Remote + Classic Controller | extension controls |
| Wii U Pro | buttons, sticks, rumble |

Maintain a model-level compatibility table. Parser presence alone is not proof of support. Capture parser/report fixtures when hardware is available.

Evaluate a newer tagged Bluepad32 only when it closes a specific coverage gap. Local motion, reconnect, and 8BitDo rumble changes must be upstreamed or cleanly rebased before changing the pinned dependency revision.

### Phase 6 — Firmware updater

The ROM UF2 path remains the trusted update mechanism.

The management foundation is now available:

```text
switch-pico-config reboot bootsel
```

The endpoint-zero request is acknowledged before a guarded 50 ms delayed call
to the Pico ROM `reset_usb_boot()` entry point. Physical BOOTSEL remains the
recovery path when management USB is unavailable.

Add:

```text
switch-pico-update firmware.uf2
```

Flow:

1. query firmware version, board model, and compatibility
2. verify release-manifest hash
3. command ROM BOOTSEL reboot
4. wait for the mass-storage device
5. install the UF2
6. wait for normal enumeration
7. verify the new version

Do not add a second in-application flash writer unless ROM UF2 cannot meet a concrete requirement.

### Phase 7 — Indexed profile catalog — Complete

Replace the fully decoded fixed database before increasing profile count.
Initial capacity is eight profiles for the global fallback and each of sixteen
stable controller identities; the format must support a later increase without
another storage rewrite.

Storage design:

- reserve two 128 KiB flash arenas for append-only profile records and atomic
  compaction
- store identity, profile index, generation, schema, payload length, and CRC
  in every record header
- keep two independently checksummed superblocks; publish a compacted arena
  only after every live record verifies
- retain the current four-profile bank reader for one-time migration
- do not erase an admitted legacy bank until the new catalog and superblock
  have been read back successfully
- maintain a compact RAM index, not a decoded copy of every profile
- decode only the fallback and active profile for each observed identity
- keep report-path profile access allocation-free with bounded snapshots

The completed AIO image uses 695,592 bytes of 4 MiB flash and reserves 256
KiB for the two profile arenas. Total flash use plus configuration, bonds,
and the RP2350 terminal sector is 978,216 bytes (23.32%). Linked SRAM is
99,040 of 532,480 bytes; the profile catalog index is 1,828 bytes.

Acceptance:

- all existing profiles 1–4 survive migration byte-for-byte at the semantic
  level
- profiles 5–8 default independently and persist across reboot
- interrupted append and compaction recover the last published generation
- corrupt newest records fall back to the previous valid record
- identity capacity remains aligned with the sixteen-entry bond store
- only active/fallback profiles are decoded in SRAM
- profile switching, management USB, and the graphical editor expose all
  eight slots

Completion evidence:

- the native catalog suite covers interrupted header publication, corrupt
  payload fallback, compaction, sixteen-identity capacity, and semantic
  migration of global and stable profiles 1–4
- service tests cover on-demand selection, cached active profiles, reset-all,
  controller-originated activation, and persistence of profiles 7 and 8
- all 109 tests pass; UART, AIO, and feasibility firmware build
- the browser editor renders and selects all eight slots
- Pico 2 W hardware read and activated profile 8, then restored profile 1

### Profile and Profile Studio enhancements — Sets A and B complete

The live playtest, eight-slot catalog, controller-native labels, automatic
active-profile synchronization, and Sets A/B are implemented. Set C remains
candidate work; the native Switch-family output plan below is separate.

| Priority | Candidate | Intended scope | Dependency or principal risk |
|---|---|---|---|
| A1 — Complete | Named and copyable profiles | Profile names and controller aliases use catalog metadata rather than input-profile fields. Copy/export/import preserve the current draft and per-section resets leave other settings unchanged. | Atomic metadata records; no report-path cost. |
| A2 — Complete | Visual response-curve editor | Replace raw `curve_q8_8` as the primary control with a graph, named presets, fine adjustment, live raw/output markers, and “apply to other side” for sticks or triggers. Retain the exact fixed-point value as the wire representation. | Implemented entirely in Profile Studio; existing profiles round-trip unchanged. |
| A3 — Complete | Controller aliases and Identify action | Allow names such as “Living-room DualSense”; show battery, transport, and capabilities as secondary details; provide an Identify button that briefly rumbles or lights only the selected live controller. | Implemented with catalog alias records, capability-gated live telemetry, and a bounded non-persistent Identify command. |
| B1 — Complete | Direct profile shortcuts | Modifier plus unique face/D-pad selectors address profiles 1–8, with the cycle chord retained and feedback after commit. | Deterministic arbitration, selector rollover and held/generation transitions are covered. |
| B2 — Complete | One Shift layer per profile | Hold/Toggle selects one alternate button map; base analog tuning, physical Turbo settings and macro definitions stay shared. | Modifier consumption and reset precedence are explicit. |
| B3 — Complete | Macro authoring tools | Timestamped firmware recording, insert/duplicate/remove/drag/keyboard reorder, visual-only preview and live duration/byte budgets; Once/While held/Toggle/Repeat playback. | Eight steps per macro, sixteen shared steps and 136 sparse bytes remain fixed. |
| B4 — Complete | Configurable Turbo and finite Burst | Shared rate/duty/count defaults with per-button overrides retain hold Turbo and Auto Burst. | Fixed-point timing skips elapsed cycles; UI warns about pulses narrower than host sampling. |
| C1 | Motion calibration and tuning | Expose live gyro/accelerometer values, bias calibration, axis orientation/inversion, sensitivity, drift threshold, smoothing, and hold/toggle activation. Preserve native Switch motion units rather than introducing gyro-to-stick emulation first. | Controller-specific validation and physical motion testing; filters must not add report latency. |
| C2 | Feedback preview and profile lighting | Add non-persistent weak/strong rumble tests, profile-switch preview, and player LED/lightbar preview. Optionally persist an RGB profile color where the controller supports it. | New bounded management command; unsupported output capabilities must be visibly disabled. |

Set A completion evidence:

- profile names and controller aliases survive catalog reload, interrupted
  writes, and arena compaction without changing the 256-byte profile schema
- profiles copy across identities and slots through existing validated atomic
  transactions; browser JSON import/export and per-section resets operate on
  unsaved drafts
- four response-curve graphs expose named presets, exact Q8.8 fine adjustment,
  live curve markers, and one-click linked-side application
- live controller details expose transport, normalized battery percentage, and
  rumble/lightbar/player-LED/motion capabilities
- Identify queues one bounded feedback pulse only for the matching live stable
  identity

Recommended delivery order:

1. A1 profile metadata, duplication, and browser backup controls.
2. A2 curve visualization and linked-side editing.
3. A3 controller aliases, battery/capability details, and Identify.
4. B1 direct profile shortcuts.
5. B3 macro recording and editing before adding new playback semantics.
6. B2 one bounded Shift layer.
7. B4 configurable Turbo and finite Burst.
8. C1 motion calibration.
9. C2 feedback preview and custom lighting.

Design constraints:

- Do not add automatic per-game switching on console paths; the Pico cannot
  reliably observe the active game.
- Do not add arbitrary scripts or an unbounded stack of action layers.
- Keep every report-path operation allocation-free and bounded.
- Keep live telemetry read-only, non-overlapping, and paused while the editor
  is hidden.
- Preserve unsaved drafts across metadata refreshes and transient USB
  disconnects.
- Gate every controller-specific input or output by reported capabilities.
- Add a new profile schema only when persistent runtime behavior changes;
  UI-only presets and catalog metadata must not churn the profile wire format.

Research basis:

- 8BitDo Ultimate Software: mapping, stick/trigger tuning, vibration, macros,
  profiles, and Turbo/Burst modes — https://support.8bitdo.com/ultimate/pro2.html
- DualSense Edge profiles: names, copies, direct shortcuts, curve presets,
  linked trigger settings, and live playtest —
  https://www.playstation.com/en-us/support/hardware/set-up-edge-controller/
- Steam Input action-set layers: temporary mapping overlays and precedence
  risks —
  https://partner.steamgames.com/doc/features/steam_controller/action_set_layers
- Xbox Elite Shift: modifier-driven alternate mappings —
  https://support.xbox.com/en-US/help/hardware-network/controller/shift-elite-series-2

Set B delivery evidence:

- Schema 6 retains the complete 136-byte macro stream and moves the profile
  payload to 384 bytes. Catalog 2 retains 512-byte records and existing arena
  capacity; names/aliases do not grow with the profile.
- Native regressions exercise interrupted page/arena publication, full
  identity capacity, old formats and high generation values, competing
  shortcuts, source-coordinate Shift/Turbo, finite timing and playback resets.
- Hardware migration preserved all sixteen stored profiles, their names and
  active indices. A temporary inactive profile-8 save/readback covered the new
  fields and was restored afterward.
- Real Profile Studio checks covered recording start/timeout/Use, a saved
  recorded step, repeat and continuous visual previews, and desktop/mobile
  layouts. The live recording run contained idle input; button-edge and
  quantization cases are covered by native regressions.
- Recording uses management operation `0x42`, schema 1: bounded pages of
  firmware-timestamped input, explicit start/stop/run identity and visible
  capacity/time/disconnect termination. It does not write profiles until Save.

### Native Switch-family HD rumble — Planned

Goal: preserve Nintendo's left/right, low/high-band commands on controllers
that can execute them natively. This is a separate output backend from the
DualSense PCM synthesizer, not a promise that every controller in “Switch
mode” supports the same rumble protocol. No Switch-family native forwarding
implementation is included in the current DualSense work.

Current constraints:

- `ControllerRumbleOutput.hd` retains decoded substeps but not the original
  eight wire bytes. Unity-gain forwarding therefore needs an explicit raw
  representation alongside the decoded, profile-scaled timeline.
- The patched Bluepad32 Switch parser enables vibration with subcommand
  `0x48`, then implements conventional magnitudes through fixed frequencies
  and a 40 ms refresh. Preserve that hardware-tested third-party fallback.
- `send_subcmd()` currently has a process-global four-bit packet counter;
  native ownership needs a counter per physical controller shared by every
  `0x01`/`0x10` sender. Player-LED requests currently construct zeroed rumble
  fields, so they must participate in rumble arbitration rather than silently
  overwrite the current command.
- Joy-Cons are currently separate, horizontally mapped controllers in
  Bluepad32. A paired two-Joy-Con logical controller is not implemented.

Delivery order:

1. **Qualify protocol and models.** Start with an original genuine Switch Pro
   Controller (`057E:2009`), then original Joy-Con L/R (`057E:2006/2007`).
   Capture console USB commands and actual Bluetooth output for neutral,
   repeated, relative and multi-substep words. Verify the accepted formats,
   amplitude normalization and repeated-word semantics against real hardware;
   the older public four-byte tables alone do not establish every compressed
   command's behavior. Do not infer native support from a Pro-like parser
   type, name or VID/PID alone: ambiguous clones stay on compatibility output
   until their model is qualified. Switch 2 controllers and NSO retro models
   require separate capability/protocol qualification.
2. **Add a generation-tagged native command path.** Extend the existing rumble
   envelope with original bytes and an explicit validity/unmodified flag;
   retain the decoded timeline for scaling, recovery and fallback. Add a
   fixed-capacity Core-0-to-Core-1 command queue and one output owner per
   physical Switch device. Use native Bluetooth report `0x10`, not PCM.
   The existing format is 11 bytes including the Bluetooth HID transaction
   byte: about 1,375 payload bytes/s at 125 reports/s, before L2CAP/HCI/radio
   overhead. Direct forwarding need not inherit the DualSense 10.667 ms PCM
   lookback, but its actual latency must be measured.
3. **Implement fidelity, scaling and resynchronization together.** Raw
   forwarding is a fast path only when profile gain is unity and physical
   controller state is synchronized. Other gains require a bounded encoder
   from the scaled per-band timeline, preserving all representable substeps.
   Validate the inverse amplitude mapping against independent golden data;
   decoder-normalized Q15 values are not raw wire amplitude codes. Preserve
   silence and Nintendo's safe amplitude bounds. Do not apply the DualSense
   2x/0.8-power response curve to Nintendo actuators. If a scaled multi-step
   command cannot be represented in one word, qualify a bounded legal packet
   schedule/quantization policy explicitly; do not quietly collapse it to a
   peak or latest magnitude. After queue loss, feedback or reconnect, send a
   valid absolute current-state/neutral resynchronization before dependent
   relative commands; never replay an obsolete vibration backlog.
4. **Unify LEDs, feedback and lifetime handling.** Keep the current effective
   rumble bytes in all applicable subcommand reports. Local confirmation
   temporarily overrides host output while host state continues advancing,
   then resumes the current state, not the old effect. Preserve the existing
   50 ms Switch-command expiry policy and prioritize explicit stop. Cancel
   duration, delayed-start and refresh timers when ownership changes or a
   device disconnects; old-generation callbacks must never touch a replacement.
   XInput input to this backend remains a two-magnitude, stateful effect with
   fixed carriers, not invented Nintendo frequency detail.
5. **Add single-actuator and third-party policies.** A standalone Joy-Con
   needs an explicit mono downmix so effects addressed to either host side
   are not simply lost. Preserve each band's dominant contribution with
   deterministic frequency/tie handling and safe amplitude limits; document
   the unavoidable spatial loss. Stereo routing to a Joy-Con pair belongs
   with a separate logical-pairing feature. Keep the current 8BitDo Ultimate
   enable/fixed-frequency/refresh behavior unless that exact model passes
   native qualification; never replace it with a blanket Switch-family rule.
6. **Qualify and promote per model.** Add independent codec/golden-packet
   regressions, scale-zero/unity/intermediate checks, channel separation,
   packet-counter wrap, backpressure/resync, LED/feedback coexistence,
   timeout/stop and disconnect/reuse cases. Run safe physical frequency and
   amplitude sweeps and captured game effects; then measure one and four
   controllers, mixed DualSense/Switch output, simultaneous input/motion and
   persistent-profile writes. Report p50/p95/p99/worst host-receipt-to-HCI
   submission, skipped/resynchronized commands and physical actuator onset
   where instrumentation is available. Enable the path only for qualified
   models; keep the UART protocol, bonds, calibration and wake configuration
   unchanged.

Implementation locations: `usb/switch/switch_haptics.*` and the profile
rumble transform for representation/scaling; `input/bluepad32_input_backend.*`
for routing/lifetimes; a bounded Switch output scheduler under `input/`;
and `patches/bluepad32-sdl3-imu.patch` for parser integration. Patch the
build-local Bluepad32 copy, not the upstream checkout.

Protocol references:

- Bluetooth reports, neutral values and actuator safety:
  https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/bluetooth_hid_notes.md
- Frequency/amplitude encoding tables:
  https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/rumble_data_table.md

### Phase 8 — Performance and release qualification

Measure:

- Bluetooth report arrival to neutral-state publication
- transform and macro processing
- neutral state to queued USB report
- physical actuation to host-visible report
- one versus four controllers
- idle versus simultaneous rumble
- every USB output mode

Report p50, p95, p99, and worst observed latency. Use a logic analyzer or instrumented actuator plus USB capture.

Acceptance:

- transformation work stays well below one report interval
- no missed or reordered transitions
- no regression against the current Switch build
- four-controller operation does not starve USB, Bluetooth, motion, or rumble queues
- compare against a real Adapter 2 under the same controller and host when available

Final host matrix:

- Switch
- Switch 2
- Windows 10 and 11
- Steam Deck/SteamOS
- Linux/Raspberry Pi
- Android TV

## Verification strategy

Every phase must preserve these existing checks:

- pairing gate and bond persistence
- reconnect after reboot and runtime disconnect
- four-interface descriptor integrity
- per-slot input and rumble isolation
- generation invalidation on disconnect
- motion normalization and calibration gating
- UART build compatibility

Evidence priority:

1. native deterministic tests for state machines, encoding, boundaries, and persistence
2. firmware build for every affected variant
3. host enumeration and API-level checks
4. physical input, motion, and rumble on the target hardware

Do not mark a host/controller combination complete from descriptor inspection or parser presence alone.

## Next action

Complete the final cold Switch regression with requested mode `auto`, then
begin Phase 5 model-level controller qualification. Before release, replace the
three `CAFE` development identities with an appropriate project VID/PID and
repeat Windows/macOS binding tests.
