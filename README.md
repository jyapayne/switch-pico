# Switch Pico Controller Bridge

Raspberry Pi Pico firmware that emulates one or more Switch Pro controllers over USB. Input can come from the SDL3-to-UART computer bridge or, on Pico 2 W, directly from Bluetooth controllers through Bluepad32.

## What you get
- **Firmware** (`src/firmware/`): acts as a Switch Pro controller (one on standard Pico, four on Pico 2 W AIO), accepting either UART bridge reports or the optional Pico 2 W Bluepad32 backend.
- **Python bridge** (`switch_pico_bridge.controller_uart_bridge` / CLI `controller-uart-bridge`): reads SDL3 controllers on the host, sends reports over UART, and applies rumble locally. Hot‑plug friendly and cross‑platform (macOS/Windows/Linux).
- **Color configuration** (`src/firmware/platform/pico/controller_color_config.h`): compile-time RGB colors for emulated controller grips and supported Bluetooth controller LEDs.
- **Pico 2 W AIO firmware** (`firmware/switch-pico-aio.uf2`): hosts four concurrent Bluetooth controllers and sends their controls, calibrated motion, rumble, and slot identity through four separate Switch Pro USB interfaces without a computer.

## Source layout

Firmware code has one include root, `src/firmware`, with responsibility-based
modules:

| Path | Responsibility |
|---|---|
| `src/firmware/main.cpp` | Firmware entry point and backend orchestration |
| `src/firmware/adapter/` | USB mode selection, host probing, and managed reboot |
| `src/firmware/configuration/` | Persistent adapter configuration and transactions |
| `src/firmware/core/` | Shared controller identity, color, and input-state types |
| `src/firmware/input/` | Bluepad32 controller input backend and hotkeys |
| `src/firmware/platform/pico/` | Pico flash/BOOTSEL integrations and compile-time board configuration |
| `src/firmware/profile/` | Controller profiles, transforms, storage, and runtime |
| `src/firmware/usb/` | USB output boundary, management protocol, and per-protocol drivers |

Internal includes are rooted at `src/firmware`, for example
`#include "profile/controller_profile.h"`. Host-side Python remains in
`src/switch_pico_bridge/`; native firmware tests remain in `tests/`.

## Quick start
1. Flash the Pico with `firmware/switch-pico.uf2` (or build your own) using BOOTSEL drag-and-drop (see “Manual UF2 flashing” below).
2. Wire Pico UART1 to a USB↔UART adapter (GPIO4 TX, GPIO5 RX, GND) and plug that adapter into your host PC.
3. Enable `System Settings → Controllers and Sensors → Pro Controller Wired Communication` on the Switch.
4. Install the Python bridge (see “Python bridge”) and run `controller-uart-bridge --interactive`.
5. Connect the Pico to the Switch (dock USB-A or USB-C OTG); the Switch should see it as a wired Pro Controller.

## Pico 2 W all-in-one Bluetooth option

### Architecture

The AIO build accepts up to four concurrent Bluetooth controllers on a single Pico 2 W. TinyUSB and the four Switch report generators run on Core 0; Bluepad32, BTstack, and the CYW43439 radio run on Core 1. Each Bluetooth device index maps directly to one always-present USB Pro HID interface. Per-slot state snapshots and generation-tagged latest-value rumble mailboxes are the only cross-core data paths.

All four USB interfaces are always present to the Switch as separate Pro Controllers on one physical USB device. Input, motion, rumble, lifecycle, and displayed grip color remain isolated per slot.

### Build and flash

Initialize the pinned Bluepad32 dependency once:

```sh
git submodule update --init external/bluepad32
```

Build and flash a Pico 2 W in BOOTSEL mode:

```sh
python3 build.py --aio
```

This uses an isolated `build-aio/` CMake cache and publishes:

- `firmware/switch-pico-aio.elf`
- `firmware/switch-pico-aio.uf2`

The default `python3 build.py` command and `firmware/switch-pico.*` artifacts remain the UART/Pico build. The AIO build requires `PICO_BOARD=pico2_w`; it is not interchangeable with the original non-wireless Pico firmware.

Both `build.py --aio` and direct AIO CMake configuration copy the pinned Bluepad32 source into the active build directory and apply `patches/bluepad32-sdl3-imu.patch` there before compiling. The patch makes supported motion controllers use SDL3-equivalent axes and fixed-point units before conversion to Nintendo samples. The `external/bluepad32` submodule remains pristine; patch or source-revision drift fails configuration.

### Switch 2 wake from L + R + Home, PS, or Xbox

The AIO firmware can wake a sleeping Switch 2 when a connected controller's
physical **L + R + System** chord becomes held: L + R + Home on
Nintendo-style controllers, L1 + R1 + PS on PlayStation controllers, or
LB + RB + Xbox on Xbox controllers. Plain Home, PS, or Xbox remains a normal
button and does not start wake advertising. Setup needs one wake advertisement
captured from a Joy-Con 2 already paired with that Switch 2. The generated
configuration is console-specific and is intentionally ignored by Git.

The implementation replays the captured, unencrypted Switch 2 BLE wake
advertisement for two seconds at a 20 ms base interval. The configured AIO
firmware adopts the captured Joy-Con's public Bluetooth address once during
startup, before Bluepad32 admits controller connections. Wake bursts then
require no identity change and do not disconnect the input controller. This
follows the packet format documented by
[`ndeadly/switch2_controller_research`](https://github.com/ndeadly/switch2_controller_research/blob/master/bluetooth_interface.md)
and the capture/replay approach demonstrated by
[`alexvnesta/switch2controller`](https://github.com/alexvnesta/switch2controller)
and the MIT-licensed
[`Switch2-Wake-Beacon-ESPHome`](https://github.com/sickyj/Switch2-Wake-Beacon-ESPHome).

Back up the complete Pico flash before replacing the AIO firmware with the
temporary capture image:

```sh
picotool save -a -v switch-pico-before-wake-capture.uf2
```

Then:

1. Connect the Pico 2 W to the computer and build/flash the one-shot capture
   firmware:

   ```sh
   python3 build.py --wake-capture
   ```

   This also publishes `firmware/switch-pico-wake-capture.elf` and
   `firmware/switch-pico-wake-capture.uf2`.

2. Start the configuration tool. It auto-detects a single Pico USB serial
   port; use `--port /dev/ttyACM0` when more than one Pico is attached:

   ```sh
   python3 tools/configure_switch2_wake.py
   ```

3. Detach a Joy-Con 2 that is already paired with the target Switch 2, put the
   console to sleep, and press that Joy-Con's Home button. Do not press its
   sync button. The capture firmware accepts the first public `ADV_IND` packet
   with Nintendo's Switch 2 wake flag and nonzero console address, stops
   scanning automatically, lights the onboard LED solid, and repeats the
   captured record until the tool receives it.

4. The tool validates the packet and atomically writes
   `src/firmware/platform/pico/switch2_wake_config.h`.

5. Restore the AIO firmware with the generated wake configuration:

   ```sh
   python3 build.py --aio
   ```

6. The Pico now has a new stable Bluetooth host address. Clear its old
   Bluepad32 bonds by holding BOOTSEL for ten seconds, then open a pairing
   window and pair each input controller again. This is a one-time re-pair.


The capture tool also accepts a saved serial log:

```sh
python3 tools/configure_switch2_wake.py --input switch2-joycon-capture.log
```

With the configured AIO firmware powered while the console sleeps, first turn
on the paired input controller with Home, PS, or Xbox and let it reconnect to
the Pico. Then hold L + R and press its system button to send one wake burst.
Holding the chord does not retrigger it; release at least one chord button
before another attempt. Plain Home, PS, or Xbox is forwarded normally and does
not disturb the radio. Because the wake identity is stable from startup,
the wake code itself does not disconnect the input controller.

Hardware testing found that the Switch 2 can briefly remove USB power while
entering or leaving sleep. A Pico powered only by that port necessarily
reboots, resetting the CYW43439 and dropping every controller link regardless
of the wake implementation. Continuous controller connectivity therefore
requires a properly isolated powered USB hub or another power arrangement
that keeps the Pico powered without backfeeding the console. Do not use an
unisolated USB Y-cable.

The configured Pico continuously owns the captured Joy-Con's public Bluetooth
address, so keep that Joy-Con inactive while the AIO firmware is running to
avoid two radios using one address.

To target another Switch 2, repeat the capture and configuration steps. To
disable wake, delete the generated `switch2_wake_config.h`, rebuild the AIO
firmware, clear the Pico's bonds, and pair the input controllers to its restored
factory address. Restore the full-flash backup only if you need to recover the
exact pre-capture firmware and persistent state.

### Pairing up to four controllers

1. Flash and connect the Pico 2 W to the Switch.
2. Enable `System Settings → Controllers and Sensors → Pro Controller Wired Communication`.
3. Hold BOOTSEL for about two seconds until the onboard LED starts double-blinking. This enables new Bluetooth authentication for 60 seconds.
4. Put a controller into Bluetooth pairing mode:
   - DualSense: hold Create + PS.
   - DualShock 4: hold Share + PS.
   - Switch Pro: press its sync button.
   - Xbox Bluetooth controller: hold its pair button.
   - 8BitDo: use a Bluetooth mode supported by Bluepad32; use Switch/S mode when motion is required.
5. Wait for the controller's player light to settle. Repeat step 4 for additional controllers while the window remains open. Holding BOOTSEL again extends the deadline by 60 seconds from that point.

Pairing order determines the initial USB slot assignment. Up to four controllers map 1:1 to the four emulated Switch Pro Controller interfaces.

With no active controller, the Pico runs Bluepad32 discovery and autoconnect. After any controller becomes active, active discovery pauses to protect input, motion, and rumble latency; bonded controllers may still initiate incoming reconnects. Pairing keys persist across Pico power cycles, so reconnect a previously paired controller by pressing its normal Home, PS, or Xbox power button. Hold BOOTSEL for the bounded pairing window before pairing a new controller or a controller that requires host-side discovery. Outside that window, BTstack remains non-bondable and rejects new Classic and BLE authentication.

To clear every stored Classic and BLE pairing without a PC, hold BOOTSEL continuously for 10 seconds. The normal pairing window opens after two seconds; continuing to hold until the LED changes to a rapid blink clears all bonds, disconnects active controllers, publishes neutral state to every slot, and closes new authentication. Release BOOTSEL, open a new pairing window, and pair controllers again.


### LED meanings and device state

The Pico 2 W onboard LED reports the overall Bluetooth state:
- **Double blink**: new controller authentication is enabled for the bounded pairing window.
- **Rapid blink for two seconds**: all stored pairings were cleared.
- **Fast blink**: a controller connection is still completing its handshake.
- **Solid**: at least one controller is active.
- **Slow blink**: no controller is active; Bluetooth discovery and autoconnect are running.
- **Solid immediately after boot that never transitions**: Bluepad32 initialization did not complete; check firmware flashing and UART logs.

### Managing controller disconnect and reconnect

- **Disconnect a controller**: its slot immediately publishes neutral buttons, sticks, and motion. Other connected controllers are unaffected.
- **Reconnect a paired controller**: power it on normally with its Home, PS, or Xbox button.
- **8BitDo Ultimate Bluetooth reconnect**: leave its selector in Bluetooth mode, press Home once, then shake it. After an abrupt controller power-off, the Pico can remain solid for up to four seconds while Bluetooth link supervision confirms the disconnect; scanning restarts immediately afterward.
- **Pair a new controller**: hold BOOTSEL until the LED double-blinks, then put the controller into its explicit Bluetooth pairing mode.
- **Pairing window expires**: new authentication and active discovery stop while a controller is active; remembered controllers may still initiate reconnects.
- **Clear all pairings**: hold BOOTSEL continuously for 10 seconds, through the initial double blink, until the rapid confirmation blink starts. All controllers are disconnected and must be paired again.

### Managing configuration, profiles, and pairings from a PC

Connect the Pico 2 W to the PC while the AIO firmware is running normally; do not enter the ROM BOOTSEL drive. `switch-pico-config` uses versioned private vendor requests on USB endpoint 0, so it does not add an interface or depend on Linux `hidraw` nodes.

```sh
uv run switch-pico-config status
uv run switch-pico-config diagnostics
uv run switch-pico-config reboot bootsel
uv run switch-pico-config config show
uv run switch-pico-config config set --pairing-window-seconds 90
uv run switch-pico-config config reset --yes
uv run switch-pico-config mode auto
uv run switch-pico-config mode switch
uv run switch-pico-config mode xinput
uv run switch-pico-config mode dinput
uv run switch-pico-config mode mac
uv run switch-pico-config profiles list
uv run switch-pico-config profiles edit
uv run switch-pico-config profiles export 1 profile.json --identity 0
uv run switch-pico-config profiles import 2 profile.json --identity 0
uv run switch-pico-config profiles activate 2 --identity 0
uv run switch-pico-config profiles reset all --identity 0 --yes
uv run switch-pico-config pairings list
uv run switch-pico-config pairings clear --yes
```

Adapter configuration records use version, size, generation, and CRC fields in two dedicated flash sectors. Profiles use separate append-only arenas before the adapter and Bluepad32 bond regions. Profile writes are individually checksummed, recover the previous record after interruption or corruption, compact atomically between arenas, skip unchanged data, and are rate-limited.

Output mode is selected before TinyUSB starts and never changes while mounted. A mode command atomically stores the selection, resets synthetic input, reboots, follows the same physical USB port through re-enumeration, and verifies requested versus active mode. `auto` uses the verified Switch probe → Windows XInput transition; manual modes bypass probing. The controller chord **L + R + Select + Start + System** held for three seconds cycles `auto → switch → xinput → dinput → mac → auto`. The destructive ten-second BOOTSEL pairing reset also restores `auto` before reboot, providing physical recovery.

Development USB identities are `CAFE:4010` (XInput), `CAFE:4020` (DInput), and `CAFE:4021` (Mac). DInput and Mac expose four input-only generic HID interfaces and no rumble. Mac uses X/Y/Z/Rx sticks plus Simulation Brake/Accelerator triggers. Switch reports input, rumble, and motion capability; XInput reports input and rumble.

`profiles edit` starts a local-only browser editor at `http://127.0.0.1:8765/`. It exposes every profile field: all 16 buttons plus the L2/R2 analog triggers can be remapped to any button or trigger output; both sticks and triggers retain independent deadzone/saturation/curve settings; and rumble, confirmation, Turbo/Auto Burst, built-in action chords, and four custom macro sequences are editable. Its live playtest compares current raw stick and trigger input with the unsaved draft, shows deadzone/saturation boundaries and digital thresholds, and highlights pressed physical controls. Select a controller identity and one of its eight profile slots, use **Start from defaults** for a new draft, then **Save to Pico**. The backend validates the complete profile before using the existing chunked atomic transaction; invalid drafts never reach flash. Use `profiles edit --no-browser` for a printed URL or `profiles edit --port PORT` to choose another local port.

The editor selects Switch Pro, DualSense, or Xbox artwork from the connected controller's USB VID/PID and places each remappable control directly over the matching physical button. Controller artwork is from [AL2009man/Gamepad-Asset-Pack](https://github.com/AL2009man/Gamepad-Asset-Pack) under its MIT license; the bundled license and source revision are recorded beside the assets.

Profile names and controller aliases are stored as independently checksummed
catalog metadata. Runtime profiles use schema 6 and 384-byte records; names remain separate. The
editor can rename and copy profiles across controllers and slots, import or
export JSON backups, and reset one section without discarding the rest of the
draft. Its response-curve cards provide named presets, exact Q8.8 fine
adjustment, live curve markers, and one-click application to the opposite
stick or trigger. Connected-controller details include transport, battery,
and supported feedback/motion capabilities; **Identify** sends one bounded
rumble/light pulse only to the selected live controller.

`profiles list` prints identity index `0` for the global fallback plus each stable Bluetooth identity observed by the firmware. Each identity owns eight persistent profiles and one active index. The JSON export/import commands remain available for version-controlled or scripted profiles. Profile numbers shown to users are `1` through `8`; `--identity` uses the zero-based index from `profiles list`.

`pairings list` refreshes and prints stored Bluetooth Classic and BLE addresses. `pairings clear --yes` deletes all bonds, disconnects active controllers, closes new authentication, and resumes discovery because no controllers remain. Destructive commands require `--yes`. If multiple compatible Picos are attached, select one with `--bus N --address N`; the error lists their locations. USB access errors require permission to the matching `/dev/bus/usb` device.

`diagnostics` reports Bluetooth initialization stage, real BTstack timer
callbacks, controller report traffic, host/local rumble requests and
dispatches, active/rumble-capable slot counts, and pending feedback. The AIO
build services one CYW43 packet per poll and explicitly reschedules remaining
input. Packet-level ring reads and bounded incoming-credit batching reduce
bus work without disabling flow control. `haptics-experiment profile --json`
adds transport timings, clock/voltage settings and packet-size diagnostics.

### Per-controller profiles

The profile editor lists **Cycle active profile**, **Toggle motion**, and **Run custom macro** as separate editable actions. Every action chord can contain any combination of the 16 buttons and the L2/R2 analog triggers. The default profile-switching chord is **L + R + Select + Start**; on DualSense, use **L1 + R1 + Create + Options**. A stored empty chord selects that default.

- The chord cycles persistent profiles `1 → 2 → 3 → 4 → 5 → 6 → 7 → 8 → 1`.
- Chord buttons are consumed locally and are not forwarded to the host.
- The new profile applies only after its atomic flash commit completes.
- Confirmation uses one to eight 75 ms pulses matching the active profile number.
- The profile policy independently enables rumble and LED feedback.
- On connection and profile changes, RGB/player LEDs briefly show the active profile color/count, then return to the persistent USB slot color/player number.
- Each controller identity and each of the four active USB slots remain isolated.

Profile hotkeys use physical controls, with trigger thresholds from the base profile. Reserved output-mode handling runs first, followed by direct profile shortcuts, profile cycling, and motion toggle. Shift consumes its modifier, macro bindings use unshifted controls, Turbo gates physical button sources, the selected button map and base analog transforms produce output, and explicit macro overrides apply last.

- **Direct shortcuts:** assign one modifier and unique face/D-pad selectors to profiles 1–8. Chord controls are consumed; holding or changing a profile cannot retrigger the same press. Activation and feedback wait for the atomic commit. The existing one-second minimum commit interval still applies to rapid successive changes.
- **Shift:** one alternate button-only map, enabled while held or toggled on a fresh modifier press. It does not layer analog tuning, Turbo settings, or macro definitions. Toggle state resets with connection, profile, mode, and configuration changes.
- **Turbo/Burst:** shared settings with optional per-button overrides; 1–30 Hz, 1–99% duty, and 1–255 finite Burst pulses. Hold Turbo follows the button, Auto Burst toggles continuous repetition, and Burst runs its configured count after a press. Defaults remain 15 Hz/50%. Counts describe scheduled ON windows; narrow phases can be missed by host report sampling, so the editor warns instead of silently changing settings.
- **Macros:** four independently triggered sequences, up to eight steps each, sharing sixteen decoded steps and a **136-byte sparse stream**. Wait/button/all-field steps cost 3/5/17 bytes. Playback supports Once, While held, Toggle, and bounded Repeat (1–255 cycles). Repeating zero-duration sequences are rejected; clock gaps skip elapsed cycles rather than replay a backlog.
- **Authoring:** insert, duplicate, remove, drag-reorder, or move steps with keyboard controls. Duration and byte budgets update on every edit. The visual preview shows overridden versus passthrough fields and never injects controller output.
- **Recording:** Record input captures timestamped firmware-side changes before profile mapping, not the editor's 75 ms snapshots. Choose channels and explicit analog quantization; the initial state consumes one entry and long holds split at ten seconds. The UI limits capture to the remaining 8/16/136 budget, visibly reports capacity/time/disconnect endings, and retains data for review. **Use recorded steps** changes only the unsaved macro; **Save to Pico** is still separate. Input-report and millisecond playback precision remain real limits.

### Per-controller motion toggle

The default motion action is **D-pad Up + R + Start**; on DualSense, use **D-pad Up + R1 + Options**. Each profile can replace it with any button/trigger chord from the graphical editor; a stored empty chord selects the default.

- A longer rumble confirms motion disabled.
- A shorter rumble confirms motion enabled.
- The chord is consumed locally and is not forwarded to the Switch.
- Other controller slots are unaffected.
- Motion returns to enabled after disconnect or reboot.

Edit `src/firmware/input/controller_hotkey_config.h` only to change the default motion-enabled state or its feedback patterns.

### Per-slot controller colors

Each AIO slot has one color shared by its emulated Switch Pro grips and its physical Bluetooth controller:

1. Blue `#0089EB`
2. Red `#E63946`
3. Yellow `#F6C945`
4. Green `#2ECC71`

When a controller becomes ready, RGB-capable devices such as DualSense and DualShock 4 receive a darker, more saturated RGB value derived automatically from the slot's Switch grip color. Controllers without an RGB light use player indicator 1, 2, 3, or 4 when Bluepad32 exposes player-LED control. Devices without either capability are left unchanged. Edit only the four grip colors in `src/firmware/platform/pico/controller_color_config.h`; rebuilding automatically recalibrates their lightbar colors.

### Controller capabilities

| Controller | Buttons/sticks | Rumble | Motion |
|---|---:|---:|---:|
| DualSense / DualShock 4 | Yes | Yes | Yes |
| Switch Pro / Joy-Con | Yes | Yes | Yes |
| PS Move ZCM1/ZCM2 | Buttons/trigger | Yes | Yes, after calibration |
| Wii Remote | Mode-dependent | Yes | Accelerometer |
| 8BitDo in Switch-compatible Bluetooth mode | Yes | Model-dependent | Yes when the mode exposes IMU |
| Xbox Bluetooth controller | Yes | Yes | No hardware IMU |

Motion-producing Bluepad32 parsers normalize to 1024 units per degree/second and 8192 units per g in SDL-oriented axes before conversion to Nintendo samples. PS Move motion remains neutral until all model-specific calibration blocks have been received and validated; buttons and rumble remain available while calibration is pending or unavailable. The latest normalized sample is duplicated across the report's three nominal 5 ms slots and remains pending until a regular `0x30` USB report successfully consumes it.

### Rumble per controller

Commands remain bound to a USB slot and Bluetooth connection generation. Compatibility output uses a latest-value mailbox; native output keeps a bounded timestamped command history instead of collapsing substeps.

The standard AIO and XInput builds use **300 MHz at 1.3 V**, packet-level CYW43 reads, bounded HCI credit returns, and native DualSense haptics by default. The first eligible DualSense/DualSense Edge that becomes ready can occupy the one native PCM stream, in any slot; later controllers do not steal it. Nintendo native output is a separate, explicit per-controller opt-in described below. Unapproved and unsupported controllers retain their existing parser-specific output. To change the selected DualSense manually, stop the current run and use `haptics-experiment gameplay --slot N` (API slots are zero-based).

In Switch mode, that stream preserves decoded left/right, low/high-band HD commands. In XInput mode, strong/low magnitude drives the left 160 Hz carrier and weak/high drives the right 320 Hz carrier; these commands stay active until changed or stopped. XInput does not supply Nintendo frequency/substep detail. USB reset, unmount, and suspend stop held host rumble. Auto-mode XInput additionally reboots to Switch probe after unmount, by the existing one-attachment policy; manual XInput is exempt.

Standard native gameplay uses **64 stereo frames at 3 kHz** per Bluetooth report (46.875 reports/s), with 21.333 ms causal lookback. The 32-frame mode passed single-controller tests but skipped audio slots under mixed Pro/DualSense load, so it is an explicit experiment: `SWITCH_PICO_HD_PACKET_FRAMES=32` requires the optimized transport and at least 300 MHz. It uses 93.75 reports/s and 10.667 ms lookback but is not the mixed-controller default. Native streaming continues silence while idle; no physical actuator-onset bound is claimed. See [HAPTICS_EXPERIMENT.md](HAPTICS_EXPERIMENT.md).

400 MHz is an explicit experiment: use `SWITCH_PICO_SYS_CLOCK_MHZ=400` and `SWITCH_PICO_OVERCLOCK_MV=1400`. This board did not boot at 400 MHz/1.3 V; 1.4 V booted and passed a short run but did not outperform 300 MHz in the comparison. USB stays at 48 MHz and flash/radio bus dividers remain bounded. UART builds are unchanged; a stock-clock AIO build is an explicit recovery/compatibility option, not the normal default.

### Native Nintendo rumble — opt-in, qualification in progress

The AIO backend can send Nintendo report `0x10` directly to an explicitly
approved original Pro Controller or standalone Joy-Con. Approval is keyed to
the physical Bluetooth identity and applies across all eight profiles; matching
a Nintendo name or VID/PID does **not** enable it automatically.

```sh
uv run switch-pico-config profiles list
uv run switch-pico-config config native-rumble approve --identity N --yes
uv run switch-pico-config config native-rumble status --json
uv run switch-pico-config config native-rumble revoke --identity N
```

Use the physical controller's row from `profiles list`, not the global fallback.
`config native-rumble list` also supplies approval indices; `revoke --approval N`
can remove an approval after its profile-catalog entry has been forgotten.
Approvals persist in adapter configuration schema 3 (232 bytes). Schema 1/2
migration preserves existing settings and starts with no approvals; profile
schema 6 and the profile catalog are unchanged.

The native encoder preserves safe unity bytes when synchronized, otherwise
encodes independent actuator/band/substep state with documented quantization.
It has no DualSense PCM lookback or response curve. Native output shares a
per-device counter and effective rumble state with LED subcommands. Identical
held states are coalesced without changing the 50 ms Switch watchdog; active
states refresh at 40 ms. XInput uses held low/left-160-Hz and high/right-320-Hz
effects until explicitly stopped. Standalone Joy-Cons downmix each band by
dominant amplitude, choosing left on ties; logical Joy-Con pairing is not added.

Nintendo and DualSense native sends share a fixed-capacity **deadline-aware
host scheduler**. Real stop transitions take priority, pending packets use
earliest-deadline order with rotating ties, and earlier periodic deadlines
protect the last available controller ACL credit. Grants and releases are
connection-generation-bound; LED handoffs and reconnects do not retain stale
permissions. This schedules HCI submission, not the radio's on-air slots.
The standard firmware does **not** disable Bluetooth sniff/power-saving mode.

**Qualification is incomplete.** The optimized Pro-only build passed 1,025
distinct commands at 125 Hz with no loss or congestion. Separate held-state
testing coalesced 505 of 513 commands into eight state changes plus refreshes,
also without loss; the user confirmed both actuators, both bands and clean
stops. On the final deadline-scheduler build, a 16.6-second mixed held-effect
test received 2,049 commands per controller with no Pro command drops and one
DualSense audio-slot skip. This result was accepted for the current setup.
Continuously changing both streams at 125 Hz still loses Pro commands and
DualSense audio slots; lossless high-rate transport is not claimed.
Joy-Con hardware, four-controller operation, captured game effects and physical
actuator timing are not qualified by the native regression suite. See
[SWITCH_FAMILY_HD_RUMBLE_PLAN.md](SWITCH_FAMILY_HD_RUMBLE_PLAN.md) for the exact
implementation, quantization policy, evidence and remaining checks.

### Hardware validation

The four-interface AIO build has been verified on a real Switch with two DualSense controllers: the Switch assigned independent controller slots, and buttons, sticks, calibrated motion, rumble, and disconnect isolation worked per controller. Fresh DualSense pairing through the BOOTSEL-open window has also been verified on hardware.

To reproduce the validation:

1. **Verify USB enumeration**: Connect the Pico 2 W to a USB host or analyzer. Confirm that four HID interfaces are present, using IN/OUT endpoint pairs `0x81/0x01` through `0x84/0x04`.
2. **Verify Bluetooth pairing**: Hold BOOTSEL until the LED double-blinks, put a controller into explicit pairing mode, and confirm its player light settles.
3. **Verify input on one controller**: Move sticks and press buttons; confirm only its assigned Switch slot changes.
4. **Verify input on two controllers**: Move the second controller independently and confirm the first controller's slot is unaffected.
5. **Verify the pairing gate**: Power-cycle the Pico and confirm a paired controller reconnects with its normal Home/PS/Xbox button without BOOTSEL. Put an unpaired controller into explicit pairing mode and confirm it remains blocked until the BOOTSEL window opens.
6. **Verify rumble per slot**: Send rumble to interface 0 and confirm only the slot 0 controller vibrates. Send rumble to interface 1 and confirm only the slot 1 controller vibrates.
7. **Verify motion**: Enable gyro/accel on both controllers. Rotate each controller independently and confirm that motion is per-slot (rotating controller 0 does not affect controller 1's IMU output).

On macOS, inspect the firmware's raw Game Pad values before GameController or browser remapping with:

```sh
swift tools/Test-SwitchPicoMac.swift
```

The diagnostic matches only `CAFE:4021`, identifies each of the four interfaces by interface and location, and prints changed axes, hats, and buttons with their HID usage and logical range. The four signed stick axes remain `X`/`Y`/`Z`/`Rx`. Move each analog trigger slowly and confirm output such as `LeftBrake page=0x02 usage=0xC5 logical=0...65535 value=32768` and `RightAccelerator page=0x02 usage=0xC4 logical=0...65535 value=32768`; each trigger should traverse intermediate values across `0...65535`, not only the endpoints. The diagnostic continues through hot-plug events until Ctrl-C. If opening a device fails, allow the terminal (or the app launching Swift) under **System Settings → Privacy & Security → Input Monitoring**, then rerun it.

On the tested Linux host, all four HID interfaces enumerate immediately at the USB layer, but `auto` initially presents them as a composite Nintendo Pro Controller while probing the host. Linux binds `hid-nintendo` to each interface and performs synchronous handshake and calibration requests with retries; incomplete composite interoperability causes `-110` timeouts and can accumulate into a 15–30 second user-visible delay before the transient hidraw nodes are removed. The timeout is not observed on the Switch. For a Linux laptop, persist `dinput` for immediate generic-HID enumeration (`uv run switch-pico-config mode dinput`) or `xinput` when rumble is required, then restore `auto` or `switch` before console use. Profile management uses endpoint-zero vendor transfers and does not depend on `hid-nintendo`.

Bluepad32 is Apache-2.0. BTstack use on Pico W/Pico 2 W is covered by Raspberry Pi's BTstack license.

## Planned features

## Limitations
- No NFC/amiibo/IR support.
- Rumble is controller-specific: UART uses SDL3 haptics; AIO uses the selected DualSense PCM stream, explicitly approved Nintendo native output, or the controller's existing Bluepad32 implementation. Native Nintendo hardware qualification remains incomplete; do not assume universal Switch-mode compatibility.
- The UART firmware requires a host computer running the bridge. The Pico 2 W AIO firmware does not; it hosts controllers over Bluetooth, not USB.
- In XInput output mode, Home/System is carried in the raw XUSB Guide bit `0x0400`, and Capture is carried in the de-facto Share/reserved bit `0x0800` used by modern open XUSB stacks. The standard Microsoft XInput headers define neither Guide nor Share for `XINPUT_GAMEPAD.wButtons`, so `XInputGetState` does not expose either button portably. Guide may be reserved or intercepted by the OS, while Share/Capture support depends on the installed driver or consumers such as GameInput and Steam; qualify the intended controller, driver, and application on real Windows hardware.

## Uses
- **Remote couch co-op**: friends connect via Parsec while the host streams the Switch via a low-latency capture device (e.g., Magewell Pro Capture) and runs the bridge (see setup below).
- **Switch automation (Python)**: write scripts/bots that drive the Pico directly using `switch_pico_bridge.switch_pico_uart` (see `examples/example_switch_macro.py`).
- **Twitch chat plays**: translate chat messages into controller actions on the host, then forward them over UART to the Pico.

### Remote couch co-op setup (example)
1. Connect the Switch to a low-latency capture device on the host PC; view it in OBS (or your preferred viewer).
2. Run `controller-uart-bridge` on the host PC and connect the Pico to the Switch for input.
3. Have friends connect to the host PC using Parsec; they use their controllers on their end, which Parsec forwards to the host (SDL3 sees them).
4. Optional audio routing: Voicemeeter Potato + a virtual audio cable can help manage capture/voice/game audio mixing:
   - Voicemeeter Potato: https://vb-audio.com/Voicemeeter/potato.htm
   - VB-CABLE: https://vb-audio.com/Cable/index.htm

## End-to-end data flow (input + rumble)
```
INPUT (buttons/sticks)
[Any controller] -> [Host OS HID] -> [SDL3 Gamepad] -> [controller-uart-bridge]
                 -> [USB↔UART adapter + UART serial] -> [Pico firmware] -> [USB (Switch Pro)]
                 -> [Nintendo Switch]

RUMBLE (force feedback)
[Nintendo Switch] -> [USB rumble output report] -> [Pico firmware]
                 -> [UART serial + USB↔UART adapter] -> [controller-uart-bridge]
                 -> [SDL3 haptics] -> [Any controller motors]
```

### HD rumble translation

Nintendo sends two stateful four-byte HD-rumble actuator words with full/relative low/high-band commands and up to three substeps. `SwitchHapticsDecoder` retains this timeline as well as conventional strong/weak magnitudes. The selected DualSense's native PCM backend uses the timeline; ordinary controller-parser and UART/SDL paths use the magnitudes. Preserving frequency intent is not a claim of identical force response across actuators. Native forwarding for genuine Switch-family controllers is [planned separately](ADAPTER_PARITY_PLAN.md#native-switch-family-hd-rumble--planned), not enabled by the DualSense implementation.

The UART return frame carries the decoded result rather than raw HD-rumble bytes:

```text
0xBB, 0x02, low-frequency magnitude, high-frequency magnitude, checksum
```

The checksum is the sum of the first four bytes modulo 256. Firmware and Python bridge versions from before this change are not rumble-protocol compatible; controller input framing remains unchanged.

## Hardware wiring (Pico)
- UART1 pins (fixed in firmware):
  - **TX**: GPIO4 (Pico pin 6) → RX of your USB-serial adapter.
  - **RX**: GPIO5 (Pico pin 7) → TX of your USB-serial adapter.
  - **GND**: common ground between Pico and adapter.
- Baud rate: **921600** (default). Some adapters only handle 500,000; both bridges accept a `--baud` flag.
- Keep logic at 3.3V; do not feed 5V UART into the Pico.

### Full hookup checklist
1. **Gather the hardware**
   - Raspberry Pi Pico flashed with the provided firmware.
   - USB-A-to-micro USB cable (or USB-C if you use a Pico W) to connect the Pico to the Switch or a PC for testing.
   - USB-to-UART adapter capable of 3.3 V logic at 921600 baud (FT232, CP2102, CH340, etc.).
   - Three dupont wires (TX, RX, GND). Optionally add heat-shrink or a small proto board if you want something more permanent.

2. **Wire the Pico to the USB-to-UART adapter**
   - Pico GPIO4 → adapter RX (sometimes labelled RXD, DI, or R).
   - Pico GPIO5 → adapter TX (TXD, DO, or T).
   - Pico GND → adapter GND. Tie grounds even if the adapter is already USB-powered.
   - Leave VBUS/VCC unconnected unless your adapter explicitly supports 3.3 V power output and you intend to power the Pico from it (the bridge expects the Pico to be powered from USB instead).

3. **Connect everything to the host and Switch**
   - Plug the USB-to-UART adapter into the computer that will run the Python bridge. Note the COM port (`Device Manager > Ports`) on Windows or `/dev/cu.*`/`/dev/ttyUSB*` path on macOS/Linux; pass it via `--map`/`--ports`.
   - Connect the Pico's micro USB port to the Nintendo Switch (via the dock's USB-A port, a USB-C OTG adapter, or a PC if you are only testing). The Pico enumerates as a Switch Pro Controller over USB.
   - On the Switch, enable `System Settings → Controllers and Sensors → Pro Controller Wired Communication`.
   - Any SDL-compatible gamepads you want to use should also be plugged into (or paired with) the same host computer that runs the Python bridge; the bridge is the one reading them.

### Finding your USB↔UART adapter “description” (port filtering)
If you have multiple serial/COM devices, you can filter which ports the bridge will consider using the port **description** (or vendor/product text) shown by the OS.

- **macOS/Linux (terminal)**:
  - Quick list with descriptions: `python -m serial.tools.list_ports -v`
  - Then run the bridge with a filter, for example: `controller-uart-bridge --interactive --include-port-desc CP210`
- **Windows**:
  - Device Manager → **Ports (COM & LPT)** → open your adapter → copy the device name/vendor text.
  - Then run: `controller-uart-bridge --interactive --include-port-desc "USB-SERIAL CH340"`

Filters you can use:
- `--include-port-desc SUBSTR` (repeatable): only consider ports whose description contains the substring.
- `--ignore-port-desc SUBSTR` (repeatable): exclude ports whose description contains the substring.
- `--all-ports`: include non-USB serial devices in discovery (useful if your adapter isn’t tagged as USB by the OS).

4. **Power-on order and sanity checks**
   - Power the Switch/dock so the Pico gets 5 V over USB; its USB stack must stay alive while the bridge streams data.
   - On the host computer, run `controller-uart-bridge --list-controllers` to make sure SDL sees your pads, then start the bridge with `--map`/`--ports` (or `--interactive`) referencing the adapter path you found earlier.
   - Watch the Rich console output: you should see each controller paired with a UART port and the rumble loop logging reconnects if cables are unplugged.

5. **Common pitfalls**
   - A flipped TX/RX pair results in silence (no button presses); swap them if the Pico never shows input.
   - Some adapters default to 5 V logic—move the jumper to 3.3 V before touching the Pico.
   - If you use multiple adapters, label each cable; COM port numbers can change between boots.
- When testing on a PC before plugging into a Switch, you can verify activity with the lightweight `switch_pico_bridge.switch_pico_uart` helper or the Windows "Game Controllers" panel.

## Building and flashing firmware
Prereqs: Pico SDK, Arm GNU toolchain, CMake, and `picotool`.

### Using `build.py`

`build.py` configures CMake, builds the firmware, checks that both output formats
were created, copies the release artifacts into `firmware/`, and flashes the ELF
with `picotool`.

`build.py` automatically locates the Pico SDK and Arm GNU toolchain from valid
existing `build/`, `build-aio/`, or `build-feasibility/` CMake caches, then from
project-local `build/_deps/pico_sdk-src` and `build/toolchain` installs, and
finally from conventional user and system locations. A compiler already on
`PATH` is used without setting a toolchain override. Explicit `PICO_SDK_PATH`
and `PICO_TOOLCHAIN_PATH` values always take precedence; an invalid explicit
path is reported instead of silently falling back.

Before running it:

1. Install the Pico SDK, CMake toolchain, and `picotool`.
2. Connect the Pico in BOOTSEL mode.
3. From the repository root, run:

```sh
python3 build.py
```

The generated files are:

- `build/switch-pico.elf`, which `build.py` passes to `picotool`.
- `build/switch-pico.uf2`, which can also be copied to the Pico manually.
- `firmware/switch-pico.elf` and `firmware/switch-pico.uf2`, refreshed from the
  corresponding `build/` artifacts after every successful build.

To assign one color to every emulated controller slot while building, pass one
of these mutually exclusive options:

```sh
# Use one random color for all slots
python3 build.py --random-grip-color

# Use one specific six-digit RGB color for all slots
python3 build.py --grip-color FF00AA
```

Both options update all four slot definitions in
`src/firmware/platform/pico/controller_color_config.h` before building. With no color option, the
per-slot blue/red/yellow/green palette is left unchanged. Run
`python3 build.py --help` to see the available command-line options.

If the tools or artifacts are in non-default locations, use these environment
variables:

```sh
PICO_SDK_PATH=/path/to/pico-sdk \
PICO_TOOLCHAIN_PATH=/path/to/arm-none-eabi-toolchain \
PICOTOOL_PATH=/path/to/picotool \
ELF_PATH=/path/to/switch-pico.elf \
UF2_PATH=/path/to/switch-pico.uf2 \
python3 build.py
```

`PICO_SDK_PATH` and `PICO_TOOLCHAIN_PATH` explicitly select the SDK and
cross-compiler installations. `PICOTOOL_PATH` selects the flashing tool,
`ELF_PATH` selects the ELF that is checked and flashed, and `UF2_PATH` selects
the UF2 that is checked after the build. Their defaults are `picotool` from
`PATH`, `build/switch-pico.elf`, and `build/switch-pico.uf2`, respectively.

### Manual build
```sh
cmake -S . -B build -DSWITCH_PICO_LOG=OFF
cmake --build build -j
```
This produces both `build/switch-pico.elf` and a flashable `build/switch-pico.uf2`.

### Manual UF2 flashing (BOOTSEL, no tools)
If you already have a built (or use the pre-built one in `firmware/`) `.uf2`, you can flash it without rebuilding:
1. Unplug the Pico.
2. Hold the **BOOTSEL** button.
3. While holding BOOTSEL, plug the Pico into your computer over USB (not the Switch), then release BOOTSEL.
4. A USB mass-storage drive (usually `RPI-RP2`) will appear. Copy the `.uf2` onto it (drag-and-drop).
5. The Pico will reboot automatically and the `RPI-RP2` drive will disappear when flashing completes.

Tip: if you don’t see `RPI-RP2`, try a different USB cable (some are charge-only) or a different USB port/hub.

When the AIO firmware is already running on a PC, enter ROM BOOTSEL without
touching the board:

```sh
uv run switch-pico-config reboot bootsel
```

The firmware acknowledges the endpoint-zero request, waits 50 ms, and then
calls the Pico ROM `reset_usb_boot()` entry point. Physical BOOTSEL remains the
fallback if the firmware or USB management path is unavailable.

Flash alternatives: bootsel + drag-drop or `picotool load`.
Flags:
- `SWITCH_PICO_LOG`: enable/disable UART logging on the Pico.

## Python bridge (recommended)
Works on macOS, Windows, Linux. Uses SDL3 + pyserial.

### Install dependencies (pyproject-enabled)
The repository now includes a `pyproject.toml`, so you can install the bridge and helper scripts as an editable package:

```sh
# from repo root
uv venv .venv
source .venv/bin/activate  # or .venv\Scripts\activate on Windows
uv pip install -e .
```

Prefer stock pip?

```sh
python -m venv .venv
source .venv/bin/activate  # or .venv\Scripts\activate on Windows
pip install -e .
```

- SDL3 runtime: install via your OS package manager (macOS: `brew install sdl3`; Windows: place `SDL3.dll` on PATH or next to the script; Linux: install `libsdl3-0` or your distribution's equivalent).

### Run
```sh
source .venv/bin/activate  # or .venv\Scripts\activate on Windows
controller-uart-bridge --interactive
# or, equivalently
python -m switch_pico_bridge.controller_uart_bridge --interactive
```
Options:
- `--map index:PORT` (repeatable) to pin controller index to serial (e.g., `--map 0:/dev/cu.usbserial-0001` or `--map 0:COM5`).
- `--ports PORTS...` or `--interactive` for auto/interactive pairing.
- `--all-ports` to include non-USB serial devices in discovery.
- `--ignore-port-desc SUBSTR` / `--include-port-desc SUBSTR` to filter serial ports by description (repeatable).
- `--include-controller-name SUBSTR` to only open controllers whose name matches (repeatable).
- `--list-controllers` to print detected controllers and their GUIDs, then exit (useful for GUID-based options).
- `--baud 921600` (default 921600; use `500000` if your adapter can’t do 900K).
- `--frequency 1000` to send at 1 kHz.
- `--deadzone 0.08` to change stick deadzone (0.0-1.0).
- `--zero-sticks` to sample the current stick positions on connect and treat them as neutral (cancel drift).
- `--zero-hotkey z` to choose the terminal hotkey that re-zeroes all connected controllers on demand (press `z` by default; pass an empty string to disable).
- `--update-controller-db` to download the latest SDL GameController database before launching (defaults to the bundled copy in `switch_pico_bridge/controller_db/`).
- `--controller-db-url URL` to override the source URL when updating the controller database (defaults to the official mdqinc repo).
- `--trigger-threshold 0.35` to change analog trigger press threshold (0.0-1.0).
- `--swap-abxy` to flip AB/XY globally.
- `--swap-abxy-index N` (repeatable) to flip AB/XY for controllers first seen at index N (auto-converts to a stable GUID).
- `--swap-abxy-guid GUID` (repeatable) to flip AB/XY for a specific physical controller (GUID is stable across runs).
- `--swap-hotkey x` to pick the runtime hotkey that prompts you to toggle ABXY layout for a specific connected controller (default `x`; empty string disables).
- `--sdl-mapping path/to/gamecontrollerdb.txt` to load extra SDL mappings (defaults to `switch_pico_bridge/controller_db/gamecontrollerdb.txt`).
- `--debug-imu` to print raw gyroscope and accelerometer readings every ~200ms (useful for verifying sensor data and troubleshooting).
- `--no-imu` to disable sensor reading entirely (useful for controllers without gyro, or if motion causes issues).
- `--gyro-scale FLOAT` to adjust gyroscope sensitivity (default 1.0; reduce below 1.0 if camera rotates too fast; increase above 1.0 for more sensitivity).

### Runtime hotkeys
- By default, pressing `z` in the terminal re-samples every connected controller's sticks and re-applies neutral offsets. Change/disable with `--zero-hotkey`.
- Press `x` (configurable via `--swap-hotkey`) to open an in-CLI prompt and toggle the ABXY layout for a specific connected controller. This updates the controller's stable GUID list immediately; press again to revert.
- Hotkeys work only when the bridge is started from a TTY/console that currently has focus. Pass an empty string to either flag to disable that shortcut (useful when running unattended).
- If you launch the bridge with `--swap-abxy` (global swap), the per-controller toggle hotkey will show that the layout is enforced globally and will not override it.

### Updating SDL controller mappings
- The bridge ships with a pinned `switch_pico_bridge/controller_db/gamecontrollerdb.txt`. Run `controller-uart-bridge --update-controller-db ...` to download the latest database from the official upstream (`mdqinc/SDL_GameControllerDB`).
- The download only touches `switch_pico_bridge/controller_db/gamecontrollerdb.txt`; add `--controller-db-url https://.../custom.txt` if you maintain your own fork.
- If the file is missing, the bridge will automatically attempt a download on startup.

Hot-plugging: controllers and UARTs can be plugged/unplugged while running; the bridge will auto reconnect when possible.

### Using the lightweight UART helper (no SDL needed)
For simple scripts or tests you can skip SDL and drive the Pico directly with `switch_pico_bridge.switch_pico_uart`:
```python
from switch_pico_bridge import SwitchUARTClient, SwitchButton, SwitchDpad

with SwitchUARTClient("/dev/cu.usbserial-0001") as client:
    client.press(SwitchButton.A)
    client.release(SwitchButton.A)
    client.move_left_stick(0.0, -1.0)  # push up
    client.set_hat(SwitchDpad.UP_RIGHT)
    print(client.poll_rumble())  # returns (left, right) amplitudes 0.0-1.0 or None
```
- `SwitchButton` is an `IntFlag` (bitwise friendly) and `SwitchDpad` is an `IntEnum` for the DPAD/hat values (alias `SwitchHat` remains for older scripts).
- The helper only depends on `pyserial`; SDL is not required.

### macOS tips
- Ensure the USB‑serial adapter shows up (use `/dev/cu.usb*` for TX).
- Some controllers’ Guide/Home buttons are intercepted by macOS; using XInput/DInput mode or disabling Steam’s controller handling helps.

### Windows tips
- Use `COMx` for ports (e.g., `COM5`). Auto‑detect lists COM ports.
- Ensure SDL3.dll is on PATH or alongside the script.

### Linux tips
- You may need udev permissions for `/dev/ttyUSB*`/`/dev/ttyACM*` (add user to `dialout`/`uucp` or use `udev` rules).
- For the development XInput/DInput/Mac identities, install `udev/99-switch-pico.rules` into `/etc/udev/rules.d/`, reload udev, and reconnect the Pico so `switch-pico-config` can access endpoint zero without root.

## IMU / Motion Controls

The bridge supports gyroscope and accelerometer passthrough from controllers that have motion sensors (e.g. the Nintendo Switch Pro Controller and DualSense). Motion data is forwarded to the Pico as a rolling three-sample window; the Pico emits standard 0x30 reports at 15 ms intervals and supports both raw IMU mode 1 and packed quaternion mode 2.

### Requirements
- A controller with gyro/accelerometer support that SDL3 can enable.
- The Switch will automatically use motion data once the controller is recognised as a Pro Controller.

### Gyro bias calibration
On startup, the bridge collects the first 200 gyro readings while the controller is stationary and averages them to compute a per-axis bias (zero-rate offset). The bias is subtracted from subsequent readings. Keep the controller still during startup for best results.

### CLI flags
- `--debug-imu`: Print raw sensor values (m/s² and rad/s) and converted Switch integer counts every ~200ms. Useful for verifying the sensor is detected and producing sensible data.
- `--no-imu`: Disable IMU entirely. The bridge sends zero motion data to the Pico, which sends zero-filled IMU bytes to the Switch. Buttons and sticks are unaffected.
- `--gyro-scale FLOAT` (default 1.0): Multiply all gyro values by this factor before sending. Reduce below 1.0 if the camera moves too fast; increase above 1.0 for more sensitivity.

### Troubleshooting
- **Gyro not detected**: Run with `--debug-imu`. If no IMU readings appear, SDL3 cannot see sensors on the controller. On Linux, the `hid-nintendo` kernel driver may expose Nintendo controller motion differently; DualSense motion is supported by SDL3's PlayStation HID driver.
- **Wild camera swinging**: Rebuild and flash the current Pico firmware. Older builds acknowledged quaternion IMU mode 2 but emitted raw mode-1 bytes, which Zelda interpreted as random quaternion data. Keep the controller still during startup, then use `--gyro-scale` only for deliberate sensitivity adjustment.
- **Verifying Pico output**: Use `uv run python tools/read_pro_imu.py --vid 0x057E --pid 0x2009` to read raw IMU bytes directly from the Pico's USB HID output. A stationary controller should show gyro values near zero and three non-empty, non-duplicated samples per report.

### Implementation notes for maintainers

#### The failure

Nintendo subcommand `0x40` is a mode selector, not a Boolean enable:

| Value | Meaning | Required bytes 13-48 in report `0x30` |
|---|---|---|
| `0` | IMU off | Zero-filled |
| `1` | Raw IMU | Three 12-byte accelerometer/gyro samples |
| `2` | Quaternion | Nintendo's packed 36-byte mode-2 structure |

The previous firmware stored the argument in `bool is_imu_enabled`. A mode-2 request therefore enabled the raw mode-1 packer. Zelda then decoded raw sensor bytes as mode bits, compressed quaternion components, deltas, and timestamps, producing apparently random camera rotation. The fake also advertised firmware `4.91`, while the genuine wired Pro Controller used during diagnosis reported `3.48`.

Keep `SwitchImuMode` as a three-state value. Never acknowledge mode 2 and then emit mode-1 bytes.

#### Mode-1 implementation

- Emit one `0x30` report every 15 ms.
- Advance the report timer by 3: one timer tick for each nominal 5 ms IMU sample.
- Pack three chronological samples as signed little-endian `accel X/Y/Z`, then `gyro X/Y/Z`.
- The host bridge must retain and republish its latest three-sample window. Do not drain it at the faster UART rate; that previously produced empty and duplicated USB reports.
- With the advertised factory calibration, 1g is approximately 4096 counts and 1 rad/s is approximately 818.5 gyro counts.

#### Mode-2 implementation

`src/firmware/usb/switch/switch_pro_driver.cpp` implements this in `integrate_motion_sample()` and `fill_quaternion_imu_report_data()`:

1. Reset quaternion state to `(0, 0, 0, 1)` when transitioning into mode 2.
2. Integrate each report's three gyro samples at 5 ms per sample. The Nintendo quaternion axes use sensor `Y, X, Z`, not `X, Y, Z`.
3. Build a delta quaternion from the angular rotation vector, multiply it into the current orientation, and normalize after every sample.
4. Select the largest absolute quaternion component. Its index and sign represent the omitted component; encode the other three signed components at 21-bit precision.
5. Pack accelerometer data in `Y, X, Z` order, set the mode field to `2`, write the 11-bit millisecond timestamp, and set the timestamp/sample count to `3`.
6. Integrate and repack only when transmitting the next 15 ms USB report. Calling the integrator from the unrestricted main loop over-integrates the same UART samples.

The mode-2 wire format is bit-packed and fields cross byte boundaries. Use `write_bits_le()` rather than C/C++ bitfields so layout does not depend on compiler bitfield rules.

#### Regression and hardware verification

After changing any IMU conversion, calibration, timing, or report packing:

1. Run `uv run --with pytest pytest -q`.
2. Build with `cmake --build build -j`.
3. Capture at least 200 raw `0x30` reports. Stationary gyro should remain near zero; there should be no empty windows, duplicated three-sample windows, or timer-step errors.
4. Send subcommand `0x40` with value `2`. Every resulting report must have mode bits `2` and timestamp count `3`.
5. Inject a known single-axis gyro rate and decode the packed quaternion. The corresponding component must change smoothly with the expected sign.
6. Perform the decisive end-to-end check: genuine Pro Controller → SDL3 bridge → UART → emulated Pico → Zelda. This path was confirmed correct after the mode-2 fix.

## Firmware resource usage

The Pico 2 W AIO build is measured from `build-aio/switch-pico.elf` and its
linked binary, not from the larger debug-bearing ELF or UF2 transport file:

| Resource | Used or reserved | Device capacity |
|---|---:|---:|
| Executable flash image | 786,864 bytes | 4 MiB |
| Indexed profile arenas | 256 KiB | 4 MiB flash |
| Adapter configuration | 8 KiB | 4 MiB flash |
| BTstack bonds | 8 KiB | 4 MiB flash |
| RP2350 terminal sector | 4 KiB | 4 MiB flash |
| Allocated/reserved SRAM, including heap and stacks | 139,616 bytes | 520 KiB |

The executable plus persistent reservations consume 1,069,488 bytes of flash,
leaving 3,124,816 bytes. Allocated SRAM sections leave 392,864 bytes of link-time
headroom; this is not a runtime heap high-water measurement. Core 0 has a
4 KiB stack, and Core 1 uses a dedicated 16 KiB stack in main SRAM for nested
catalog migration/compaction rather than overflowing its 4 KiB scratch bank.

Profiles use two 128 KiB append-only arenas and retain 248 physical record
slots. Catalog 2 uses a 128-byte header plus a 384-byte profile in the same
512-byte stride. The second page is programmed before the header-containing
first page, and records are read back before publication. Profile names
remain 256-byte metadata payloads and aliases remain 32 bytes. The compact
index stores locations and generations; active/fallback profiles for observed
identities and the selected profile are decoded, not the entire database.

The catalog supports eight profiles for the global fallback and each of 16
stable identities. Missing records resolve to defaults, so profiles 5–8 do
not consume flash until changed. When an arena fills, the latest indexed
records are compacted into its peer and the new superblock is published last.
Interrupted or corrupt appends leave the previous valid record available.
Catalog 1 and retired four-profile banks migrate through the alternate arena;
the old published data is retained until all copies and the new superblock
verify. Schema 1–5 profiles retain their meaning when decoded as schema 6.
Keep a profile export before downgrading: older firmware cannot read the new
catalog/profile format.

## References
- GP2040-CE (controller firmware ecosystem): https://github.com/OpenStickCommunity/GP2040-CE
- nxbt (Switch controller research/tools): https://github.com/Brikwerk/nxbt
- Nintendo Switch Reverse Engineering notes: https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering
- `hid-nintendo` driver reference: https://github.com/DanielOgorchock/linux/blob/ogorchock/drivers/hid/hid-nintendo.c

## Troubleshooting
- **No input on Switch**: verify UART wiring (Pico GPIO4/5), baud matches both sides, Pico flashed with current firmware, and `Pro Controller Wired Communication` is enabled on the Switch.
- **Constant buzzing rumble**: the bridge filters small rumble payloads; ensure baud isn’t dropping bytes. Try lowering rumble scale in `switch_pico_bridge.controller_uart_bridge` if needed.
- **Guide/Home triggers system menu (macOS)**: try different controller mode (XInput/DInput), disable Steam overlay/controller support, or connect wired.
- **SDL can’t see controller**: load `switch_pico_bridge/controller_db/gamecontrollerdb.txt` (default), add your own mapping, or try a different mode on the pad (e.g., XInput).
