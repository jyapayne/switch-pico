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
- Do not impersonate 8BitDo or Microsoft USB identities in a release build.
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
| Four concurrent controllers | Complete for Switch mode | Four independent USB interfaces and Bluetooth slots are implemented. |
| Switch input | Complete | Buttons, sticks, lifecycle, colors, and per-slot isolation are hardware-tested. |
| Switch motion | Complete for supported parsers | DualSense, Switch-family, Wii accelerometer, PS Move, and compatible 8BitDo modes are normalized. |
| Switch rumble | Complete for tested controllers | DualSense and 8BitDo Ultimate Bluetooth are hardware-tested; Ultimate requires enable, fixed LRA frequencies, and refresh. |
| 8BitDo Ultimate reconnect | Complete | Bond preservation, scan restart, and four-second supervision timeout are implemented. |
| XInput descriptors and reports | Feasibility complete | Four-interface prototype works on Windows and is covered by native descriptor/report tests. |
| Automatic Windows/Switch selection | Feasibility complete | Windows enumeration fix is in `db4a860`; real Windows transition and rumble were reported working. |
| Windows feasibility test | Complete | `tools/Test-AdapterFeasibility.ps1` checks transition, PnP health, four XInput slots, controls, and rumble isolation. |
| Protocol-neutral controller state | Complete | `ControllerState` is shared by Bluetooth, UART, Switch, and XInput paths; analog trigger precision is retained. |
| Persistent configuration protocol | Complete | Adapter settings and identity-keyed profiles use separate two-copy CRC/generation stores with bounded transactions and recovery. |
| Production USB VID/PID | Missing | Prototype uses `CAFE:4010`; obtain an appropriate project VID/PID and repeat Windows binding tests. |
| DInput output | Missing | Add generic HID descriptor and report driver. |
| Mac output mode | Missing | Capture/define compatible descriptor and report semantics. |
| Manual output-mode selection | Missing | Add persistent PC command and controller chord. |
| General button remapping | Complete | Sixteen positional logical inputs map directly to supported logical outputs per profile. |
| Stick sensitivity | Complete | Per-stick center calibration, inner deadzone, outer saturation, fixed-point curve, and inversion run before every output serializer. |
| Trigger ranges | Complete for current outputs | Per-trigger deadzone, saturation, curve, and digital threshold preserve analog XInput values and configured Switch thresholds. |
| Vibration intensity | Complete | Independent weak/strong profile scales apply to host rumble; local confirmation policy remains separate. |
| Macros | Complete | One bounded eight-step deterministic macro per profile supports buttons, D-pad, sticks, triggers, waits, and explicit end. |
| Turbo and Auto Burst | Complete | Fixed-point phase accumulation produces 15 activations per second with deterministic cancellation. |
| Persistent profiles | Complete | Global fallback plus sixteen stable identities each store four fixed 256-byte profiles in a two-bank atomic database. |
| Profile switching | Complete | PC commands and a configurable controller chord persist selection with isolated rumble/onboard/transient controller LED confirmation. |
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

Use four profile slots per stable controller identity. Resolve identity from Bluetooth transport, identity address, VID, and PID; use a global default when stable identity is unavailable.

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
- one to four rumble pulses
- matching onboard LED count
- controller RGB/player LED feedback when supported

Acceptance:

- Simulated-clock tests cover exact transitions and cancellation.
- Boundary tests cover deadzones, saturation, curves, and thresholds.
- Physical input plus macro plus Turbo precedence is deterministic.
- No synthetic input remains stuck after any cancellation path.
- Four controllers can use different profiles simultaneously.

Completion evidence:

- strict 256-byte profile schema and 17,696-byte fixed database support four
  profiles for the global fallback and each of sixteen stable identities
- profile and adapter stores remain separate from each other and BTstack bonds;
  profile commits use one flash-safe batched inactive-bank replacement
- schema-v1 profile databases migrate inherited trigger defaults to schema v2
  without losing custom thresholds, identities, active profiles, or other data
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

### Phase 4 — Production USB output modes

This is shorter than Phase 3 because Switch and XInput already work, but it is
not a small change: USB descriptors are fixed before `tusb_init()`, mode changes
require a persisted reboot, and DInput/Mac need real-host qualification.

Release blocker: replace development identity `CAFE:4010` with an appropriate
project VID/PID before calling XInput production-ready. Development can continue
with the current identity.

#### 4A — Output-driver boundary and production XInput

- replace `SWITCH_PICO_ADAPTER_FEASIBILITY` branches in `switch-pico.cpp` with
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
- research whether the same standards-compliant HID report is sufficient on
  macOS; share the driver when hardware proves it, otherwise add only the
  descriptor/report differences required by macOS
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

### Phase 7 — Performance and release qualification

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

Begin Phase 4 by placing the existing Switch and verified XInput
implementations behind one production output-driver boundary. Preserve the
Phase 3 transform/runtime seam before serialization, then add DInput and
Mac-compatible HID in that order.
