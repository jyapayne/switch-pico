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

This uses an isolated `build-aio/` CMake cache. The main firmware variants are:

| Variant | UF2 image |
| --- | --- |
| Regular Pico, UART input | [switch-pico.uf2](firmware/switch-pico.uf2) |
| Pico 2 W AIO, mixed Bluetooth | [switch-pico-aio.uf2](firmware/switch-pico-aio.uf2) |
| Pico 2 W AIO, BLE only | [switch-pico-aio-ble.uf2](firmware/switch-pico-aio-ble.uf2) |
| Pico 2 W AIO, Classic only | [switch-pico-aio-classic.uf2](firmware/switch-pico-aio-classic.uf2) |

Each image has a matching `.elf` in `firmware/`. All AIO variants include the same
automatic Switch/XInput and manual USB output modes; no separate feasibility
firmware is needed. The wake-capture image remains a separate setup utility.

The default `python3 build.py` command and `firmware/switch-pico.*` artifacts remain the UART/Pico build. The AIO build requires `PICO_BOARD=pico2_w`; it is not interchangeable with the original non-wireless Pico firmware.

Both `build.py --aio` and direct AIO CMake configuration copy the pinned Bluepad32 source into the active build directory and apply `patches/bluepad32-sdl3-imu.patch` there before compiling. The patch makes supported motion controllers use SDL3-equivalent axes and fixed-point units before conversion to Nintendo samples. The `external/bluepad32` submodule remains pristine; patch or source-revision drift fails configuration.

### Bluetooth transport selection

`SWITCH_PICO_BLUETOOTH_MODE` selects the active radio transports at build time:

| CMake value | Active radio behavior |
| --- | --- |
| `MIXED` (default) | Bluetooth Classic and BLE |
| `BLE` | BLE only; no Classic inquiry, page scanning, or controller admission |
| `CLASSIC` | Classic only; no BLE scanning, controller admission, or wake advertising |

To **build without flashing**, use a separate directory for each mode:

```sh
cmake -S . -B build-aio-ble \
  -DPICO_BOARD=pico2_w \
  -DSWITCH_PICO_INPUT_BACKEND=BLUEPAD32 \
  -DSWITCH_PICO_BLUETOOTH_MODE=BLE
cmake --build build-aio-ble
```

Use `CLASSIC` or `MIXED` and a matching build directory for the other modes.
Every AIO build includes automatic Switch/XInput and manual USB output modes.
Invalid modes and single-transport selections with the UART backend are rejected.

The build helper also supports the selector; these commands **build and flash**:

```sh
python3 build.py --aio --bluetooth-mode ble
python3 build.py --aio --bluetooth-mode classic
python3 build.py --aio --bluetooth-mode mixed
```

The helper defaults explicitly to `mixed`. Single-transport build directories and
published `.elf`/`.uf2` names get `-ble` or `-classic` suffixes, so they do not
overwrite mixed artifacts—for example, `build-aio-ble/` and
`firmware/switch-pico-aio-ble.uf2`.

These flags select radio activity, not complete removal of the unused host stack:
shared BTstack code and both pairing databases remain available. Switching modes
does not erase bonds or profiles; pairing lists still include inactive-transport
bonds and explicit clear-all still clears both stores. A configured stable public
Bluetooth address is retained in every mode, including Classic-only, so changing
modes does not silently change the host identity used by existing bonds.
Classic-only disables Switch 2 wake advertising regardless of the saved wake
configuration. Each build remains a standalone USB adapter; no inter-Pico link
is introduced.

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
   - Switch 2 Pro / Joy-Con 2: hold SYNC while the Pico pairing window is open; do not pair through the PC's Bluetooth settings.
   - Wii Remote / Remote Plus: press the red SYNC button with any desired extension already attached. SYNC creates a remembered pairing; 1 + 2 is temporary discovery.
   - Xbox Bluetooth controller: hold its pair button.
   - 8BitDo: use a Bluetooth mode supported by Bluepad32; use Switch/S mode when motion is required.
5. Wait for the controller's player light to settle. Repeat step 4 for additional controllers while the window remains open. Holding BOOTSEL again extends the deadline by 60 seconds from that point.

Pairing order determines the initial USB slot assignment. Up to four physical Bluetooth controllers are supported. Ordinary controllers each occupy one emulated Switch Pro interface; a merged Joy-Con 2 pair consumes two Bluetooth connections but occupies one USB player slot.

With no active controller, the Pico runs Bluepad32 discovery and autoconnect. After any controller becomes active, active discovery pauses to protect input, motion, and rumble latency; bonded controllers may still initiate incoming reconnects. Pairing keys persist across Pico power cycles, so reconnect a previously paired controller by pressing its normal Home, PS, or Xbox power button. Hold BOOTSEL for the bounded pairing window before pairing a new controller or a controller that requires host-side discovery. Outside that window, BTstack remains non-bondable and rejects new Classic and BLE authentication.

Exception: a ready solo Joy-Con 2 keeps a low-duty passive BLE scan running for a remembered opposite half while physical capacity remains and no controller setup is pending. This reconnect does not require BOOTSEL, start Classic inquiry, or enable fresh pairing. Scanning stops when the pair completes; an explicit pairing window restores normal discovery.

To clear every stored Classic, BLE and proprietary Switch 2 pairing without a PC, hold BOOTSEL continuously for 10 seconds. The normal pairing window opens after two seconds; continuing to hold until the LED changes to a rapid blink clears remembered controllers, disconnects active controllers, publishes neutral state to every slot, and closes new authentication. Release BOOTSEL, open a new pairing window, and pair controllers again. A persistent-storage failure is reported rather than acknowledging a successful clear.


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
- **Pairing window expires**: new authentication and full discovery stop while a controller is active; remembered controllers may still initiate reconnects, and a solo Joy-Con 2 resumes low-duty discovery of its remembered opposite half.
- **Clear all pairings**: hold BOOTSEL continuously for 10 seconds, through the initial double blink, until the rapid confirmation blink starts. All controllers are disconnected and must be paired again.

### Joy-Con 2 player arrangement

In Controller Studio, open **Adapter settings** in the top bar, then use
**Joy-Con 2 default mode** and **Apply default mode**. Closing the settings
dialog preserves unapplied selections and does not change your profile draft.
This is a real adapter-wide setting, separate from the artwork preview:

- **Paired:** ready opposite halves form one player and use their L+R bank.
- **Individual:** both halves can remain connected as separate sideways players,
  each using its own solo bank. Select the L or R owner to edit its settings.

Changes apply live without Bluetooth disconnection or fresh pairing. Affected
input, macro/capture and rumble epochs are cleared; unrelated players and all
saved banks remain intact. Unmatched halves stay solo. The default persists
across reboot:

```sh
uv run switch-pico-config joycon-mode
uv run switch-pico-config joycon-mode individual
uv run switch-pico-config joycon-mode paired --json
```

#### Join or split a specific pair from the controllers

- On the **left** Joy-Con 2, hold **ZL + Minus**.
- On the **right** Joy-Con 2, hold **ZR + Plus**.
- Hold both chords together for **2 seconds**, until the short confirmation
  pulse, then **release all four buttons**.

Two solo halves join; an existing pair splits. Other controllers are untouched.
An explicit join keeps the lower of the two participating player slots, not
whichever slot belongs to the left half. Both LEDs follow that retained slot;
the other slot is neutralized. It never takes a slot from an unrelated player.
The shortcut does not change the saved default: its connection-only override
ends when either participating half disconnects, or when the adapter default
changes. Existing solo/pair banks and active selections are reused.

Each half's complete two-button chord is reserved while held and until both
buttons on that half release, even if the other half has not joined the
attempt. Those inputs do not reach profile macros or the console. Other inputs
remain available. Fresh held reports from both halves are required; stale,
failed or ambiguous attempts require release before retry. When several solo
halves are armed ambiguously, no arbitrary pair is chosen. A chord cannot steal
a member from a different already-joined pair.

For a non-destructive disconnect/reset, Nintendo documents pressing
[SYNC once, then a normal button to wake the Joy-Con 2](https://en-americas-support.nintendo.com/app/answers/detail/a_id/68521/).

Configuration schema 4 keeps the 232-byte object and stores the default mode
in byte 4 (`0` Paired, `1` Individual); bytes 5–7 remain reserved. Schemas 1–3
migrate to Paired, preserving USB mode, pairing-window duration and existing
native approvals. Older firmware cannot save this setting.

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

Switch 2's **C, GL, GR, Left SL/SR and Right SL/SR** are additional source-only controls. Map each to a normal button or trigger, assign a button-only alternate Shift mapping, or use it in action/macro chords, cancellation and modifiers. Extra mappings default to disabled. The live playtest shows raw extras separately; they are not fictitious output channels on the emulated Switch Pro/XInput controller.

Controller Studio uses the supplied lightweight SVGs for Switch 2 Pro, Joy-Con 2 left/right solo and paired layouts, original Switch Pro, DualSense, Xbox, and Wii Remote/Nunchuk views. Hotspots follow the artwork's actual coordinates; solo Joy-Con views rotate with their firmware input mappings. Rear buttons and rails are labeled below the front view rather than drawn in fictitious positions. On narrow screens, pan the diagram or use the **Source control** menu.

**Auto** uses matching-owner live metadata to distinguish Joy-Con pairs/solo halves and Wii horizontal, vertical, and Nunchuk layouts. **Preview** changes only the editor's diagram and source labels; it does not pair controllers or change saved mappings, and physical highlighting is disabled. Source choices reflect the layout while stored unavailable mappings are retained. Legacy Wii metadata without orientation uses a visibly labeled horizontal reference with physical highlighting disabled.

**Wii orientation** is separate from layout preview. Select a connected Wii Remote owner, choose **Horizontal** or **Vertical**, and click **Apply orientation**. Studio waits for firmware confirmation before reporting success. This changes the current connection's physical button mapping, not saved profiles or adapter configuration. Nunchuk mappings remain vertical while attached; unplugging restores the selected standalone orientation. Changing orientation clears old held-input/macro/capture state and advances the logical input generation without reconnecting Bluetooth. Reconnect uses the horizontal default, or vertical if + is held while connecting.

**Joy-Con 2 pair profiles:** the first successful join of an L + R combination creates a separate **Nintendo Joy-Con 2 (L + R)** owner with eight profiles. It initially copies the left bank's profiles, names, and active selection; its alias starts empty. Both solo banks stay unchanged. Pair edits, names, and active selections are independent thereafter. Splitting or losing a half restores solo banks; joining the same members again restores their existing pair bank without copying. Different member combinations have different banks. Select the L + R owner—not either solo owner—to edit paired settings.

Pair keys contain both complete Bluetooth addresses and address types, in canonical L/R order. They retain the 14-byte identity size: transport byte 1 is 3; byte 0 contains the stable bit plus the left/right static-random flags in bits 1/2; bytes 2–7 and 8–13 contain the left and right addresses. Pair keys are profile identities, not Bluetooth peers or native-output approvals. Update host tools with firmware when using this identity kind.

The read-only playtest endpoint (`0x39`) uses schema 5, 56 bytes: byte 55 identifies unspecified (0), Joy-Con 2 left solo (1), right solo (2), pair (3), legacy Wii Remote with unknown orientation (4), Wii Remote + Nunchuk (5), Wii horizontal (6), or Wii vertical (7). Host tools still read schema 2/54-byte, schema 3/55-byte and schema 4/56-byte payloads. Orientation requests (`0x3d`) contain the 14-byte controller identity, four-byte little-endian connection generation, and orientation byte (0 horizontal, 1 vertical); stale/replaced/non-Wii/extension targets are rejected before Bluetooth-core dispatch. These operations do not change profile records or persistent configuration.

Profile names and controller aliases are stored as independently checksummed
catalog metadata. Runtime profiles use schema 9 and unchanged 384-byte records. Schemas 1–8 retain existing settings, including schema-8 Remote swing mappings; new Nunchuk and combined actions default to disabled. Names remain separate. The
editor can rename and copy profiles across controllers and slots, import or
export JSON backups, and reset one section without discarding the rest of the
draft. Its response-curve cards provide named presets, exact Q8.8 fine
adjustment, live curve markers, and one-click application to the opposite
stick or trigger. Connected-controller details include transport, battery,
and supported feedback/motion capabilities; **Identify** sends one bounded
rumble/light pulse only to the selected live controller.

`profiles list` prints identity index `0` for the global fallback plus each stable Bluetooth identity observed by the firmware. Each identity owns eight persistent profiles and one active index. The JSON export/import commands remain available for version-controlled or scripted profiles. Profile numbers shown to users are `1` through `8`; `--identity` uses the zero-based index from `profiles list`.

`pairings list` refreshes and prints stored Bluetooth Classic and BLE addresses, including Switch 2's application-level authorizations. `pairings clear --yes` forgets them all, disconnects active controllers, closes new authentication, and resumes discovery because no controllers remain. Destructive commands require `--yes`. If multiple compatible Picos are attached, select one with `--bus N --address N`; the error lists their locations. USB access errors require permission to the matching `/dev/bus/usb` device.

`diagnostics` reports Bluetooth initialization stage, real BTstack timer
callbacks, controller report traffic, host/local rumble requests and
dispatches, active/rumble-capable slot counts, and pending feedback. The AIO
build services one CYW43 packet per poll and explicitly reschedules remaining
input. Packet-level ring reads and bounded incoming-credit batching reduce
bus work without disabling flow control. `haptics-experiment profile --json`
adds transport timings, clock/voltage settings and packet-size diagnostics.
Switch 2 native output adds separate ingress and output-stage drop counters.
The management response extends from 32 to 40 bytes; the updated host tool
still reads older 32-byte responses and treats their missing counters as
unreported, not zero. These count firmware queue discards/rejections, not
physical actuator-delivery receipts.

Receive-credit commands use a separate, word-aligned buffer so pending outgoing
ACL fragments cannot block receive-credit returns. This is enabled by default
with credit batching; `-DSWITCH_PICO_HCI_CREDIT_BUFFER=OFF` restores the shared
buffer for comparison. It does not change radio scheduling, pairing policy, or
the controller's credit limits. See the
[credit-buffer results](HAPTICS_EXPERIMENT.md#dedicated-receive-credit-buffer)
for the distinction between host-side progress and measured gameplay latency.


### Per-controller profiles

The profile editor lists **Cycle active profile**, **Toggle motion**, and **Run custom macro** as separate editable actions. Every action chord can contain any combination of the 16 buttons, L2/R2 analog triggers and seven Switch 2 extra inputs. The default profile-switching chord is **L + R + Select + Start**; on DualSense, use **L1 + R1 + Create + Options**. A stored empty chord selects that default.

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
| Switch 2 Pro | Yes, including remappable C/GL/GR | Yes, amplitude translation | Yes |
| Joy-Con 2 solo / merged pair | Implemented; hardware qualification pending | Implemented | Implemented |
| PS Move ZCM1/ZCM2 | Buttons/trigger | Yes | Yes, after calibration |
| Wii Remote / Remote Plus | Orientation-dependent buttons; Nunchuk stick/C/Z | Yes | Factory-calibrated accelerometer; gyro with MotionPlus |
| 8BitDo in Switch-compatible Bluetooth mode | Yes | Model-dependent | Yes when the mode exposes IMU |
| Xbox Bluetooth controller | Yes | Grip + impulse-trigger motors (Microsoft Xbox parser) | No hardware IMU |

Motion-producing Bluepad32 parsers normalize to 1024 units per degree/second and 8192 units per g in SDL-oriented axes before conversion to Nintendo samples. PS Move motion remains neutral until all model-specific calibration blocks have been received and validated; buttons and rumble remain available while calibration is pending or unavailable. The latest normalized sample is duplicated across the report's three nominal 5 ms slots and remains pending until a regular `0x30` USB report successfully consumes it.

### Wii Remote and MotionPlus input

The AIO firmware enables motion automatically for original Wii Remotes with an external MotionPlus and Remote Plus models with integrated MotionPlus, including `RVL-CNT-01-TR` (`057E:0330`). Holding A while connecting is no longer required. MotionPlus is probed independently of the status report's extension-present bit; integrated units can return an `A4` identifier from the inactive `A600FA` address.

- **Buttons:** standalone remotes retain horizontal mappings by default: 1/2/A/B → south/east/west/north, with the D-pad rotated for sideways use. Hold **+ while connecting** for vertical mappings: B/A/1/2 → south/east/west/north and upright D-pad. With a Nunchuk, B/A → south/east, 1/2 → L/R, C/Z → west/north, and the Nunchuk supplies the **left stick**; the right stick stays neutral. Minus/Plus/Home remain Select/Start/Home. Profiles can remap these sources.
- **Nunchuk stick travel:** the parser reads the 16-byte extension calibration at `A40020` before activating MotionPlus. It validates both checksum bytes and the X/Y min/center/max ordering, then scales each side of each axis independently into the full normalized range, clamps overshoot, and inverts Y. It preserves the notched gate rather than expanding diagonals into square corners. Unreadable/invalid calibration uses documented nominal travel (center 128, ±96), not the full 0–255 byte range. Replacement Nunchuks reload their own calibration. Existing left-stick profile tuning applies; saved right-stick tuning is not copied or overwritten.
- **Motion:** ordinary remotes use continuous `0x31`; MotionPlus and Nunchuk use combined `0x35` reports. The parser checks primary/backup accelerometer factory calibration and both fast/slow gyro calibration blocks, including checksums. Invalid calibration disables only the affected sensor rather than inventing readings. MotionPlus selects sensitivity independently for each axis; factory-only zero correction can retain temperature-dependent gyro drift.
- **Nunchuk passthrough:** MotionPlus and Nunchuk samples alternate. Held Nunchuk controls persist across gyro packets, moved C/Z bits are decoded, and the remote's accelerometer remains the motion source. Attaching or removing a Nunchuk triggers serialized extension discovery, updates Studio's layout, and clears detached stick/C/Z state. Extension identifiers accept the upstream-compatible type suffix rather than requiring a vendor-specific prefix; initializing extensions get bounded reply-paced retries. Periodic discovery of an inactive external MotionPlus attached alone and Classic Controller MotionPlus passthrough remain unimplemented. Existing ordinary Classic Controller, Wii U Pro, Balance Board and uDraw paths remain separate.
- **Motion gesture bindings:** Studio's Macro section provides **Wii Remote swing**, **Nunchuk swing**, and **Both together**. Each can select a final output button or one of the four configured macros, plus an optional held modifier. Remote and Nunchuk have independent Low/Medium/High sensitivity. All new bindings default to disabled; existing Remote bindings are preserved. No sensor bar or MotionPlus is required, and gestures remain available when gyro output is disabled. For Zelda on Switch, choose **Y** (logical `west`; use the Switch preview for Nintendo button labels).
- **Combined priority:** when Both together is enabled and both calibrated sensors are fresh, individual gestures wait for the combination window (30–200 ms, default 100 ms). One full swing can pair with smaller sustained movement from the other sensor, before or after it within the inclusive window. Confirmation uses half that sensor's normal acceleration threshold while retaining the force/tilt guard and fresh-sample evidence requirement; individual thresholds are unchanged. Two small movements alone do not trigger an action. Confirmation is consumed once, including the confirming source's rearm/cooldown, so a later full swing from that same stroke cannot leak an individual action. Without an eligible combined binding or a fresh Nunchuk stream, Remote actions do not acquire the delay. Disconnects, lost modifiers, stale samples and active macros discard pending evidence rather than replaying it.
- **Stroke detection and macros:** initially settle each enabled sensor for 120 ms. Further strokes need only a 20 ms lower-force gap and at least 200 ms between presses; early rebounds are discarded. Buttons generate 80 ms presses. Gesture macros run one cycle regardless of their physical-trigger playback mode and must contain a positive-duration step before binding. Physical macro triggers and cancel controls take priority; gestures during active macros are ignored, not queued. The detector retains stroke history so repeated gesture macros do not require full stops. Profile changes and reserved hotkeys cancel pending gesture output; physical button holds remain intact.
- **Independent Nunchuk acceleration:** plain and MotionPlus passthrough reports use separate calibrated samples and freshness counters. Nunchuk acceleration never replaces the Remote's console IMU. Invalid factory accelerometer calibration disables Nunchuk/combined gestures without disabling the stick, C/Z buttons, or Remote gestures; unplugging and replacement discard the old Nunchuk sample.
- **Experimental IR aiming:** `SWITCH_PICO_WII_IR_GYRO=ON` enables camera-derived aiming instead of the mutually exclusive IR mouse experiment. Hold **1 + 2 for two seconds** to switch sources: one rumble pulse selects physical gyro, two select IR. Physical gyro is the connection default. With a Nunchuk, hold **C + 1** to reposition without sending aim motion; 1 alone remains usable. IR horizontal motion uses Nintendo **gyro Z**, matching a captured physical MotionPlus yaw reference; gyro X remains neutral. Vertical output uses gyro Y with **1.5x pitch gain**, conservatively tuned against a physical-gyro capture without changing horizontal gain, camera geometry or filtering. The real accelerometer is retained. Tracking loss stops IR aiming rather than silently switching to physical gyro. Horizontal aiming is user-confirmed in-game; the increased vertical gain still requires in-game qualification.
- **IR tracking:** gyro mode follows relative camera-frame movement of one to four persistent spots, without requiring both ends of the bar or expiring a continuously visible single spot after 80 ms. Mutual nearest matches tolerate camera-slot changes; conflicting motion, large jumps, stale reports and complete visibility loss rebaseline rather than replay missing movement. The IR mouse experiment retains its two-end bar model. Relative single-spot tracking cannot distinguish remote roll from pointing motion, so keep the remote upright for aiming.
- **Hardware verification:** Remote Plus `B8:AE:6E:21:8F:C2` reconnected after flashing and delivered 100 real USB `0x30` reports with changing accelerometer and gyro values on all axes. Identify dispatched rumble without losing the link. All 72 profile records, names/aliases, active selections, nine pairings and adapter configuration generation 21 / CRC `b58672ac` were unchanged. The subsequent Studio fix detects the attached Nunchuk with live stick and gyro data and renders the matching layout and vertical lock. Native tests cover integrated/external detection, calibration/error boundaries, hotplug ordering, interleaved controls, orientation changes and stale-request rejection. External MotionPlus combinations still need physical qualification.

Protocol references: [WiiBrew Wiimote](https://wiibrew.org/wiki/Wiimote), [MotionPlus registers/calibration](https://wiibrew.org/wiki/Wiimote/Extension_Controllers/Wii_Motion_Plus), [Dolphin MotionPlus calibration](https://github.com/dolphin-emu/dolphin/blob/master/Source/Core/Core/HW/WiimoteEmu/MotionPlus.cpp), and [SDL Wii sensor axes](https://github.com/libsdl-org/SDL/blob/main/src/joystick/hidapi/SDL_hidapi_wii.c).

### Experimental native Joy-Con 2 USB output

`SWITCH_PICO_SWITCH2_USB_BRIDGE=ON` selects the separate USB protocol probe in
`tools/switch2_usb_probe`, not the ordinary four-Pro-controller AIO output.
`SWITCH2_BRIDGE_INPUT=JOYCON2` preserves complete packets from one selected
Joy-Con 2. Choose `SWITCH2_PROBE_SIDE=LEFT` or `RIGHT` (default), with matching
identity, firmware and calibration captures, and select the physical Bluetooth
address with `SWITCH2_BRIDGE_SOURCE_ADDRESS`. Left uses USB PID `2067`/report
`07`; right uses PID `2066`/report `08`. `SWITCH2_BRIDGE_INPUT=WII` generates
native right-Joy-Con reports and rejects `LEFT`.

The Wii source requires Pico 2 W, the Bluepad32 backend, Bluetooth `MIXED` mode,
and the bridge's native capture prerequisites
(`SWITCH_PICO_SWITCH2_MOUSE_CAPTURE=ON` and
`SWITCH_PICO_SWITCH2_MOUSE_CAPTURE_NATIVE=ON`). It enables the Wii camera parser
without the mutually exclusive legacy `SWITCH_PICO_WII_IR_MOUSE` or
`SWITCH_PICO_WII_IR_GYRO` USB experiments.

This research target also requires the probe's validated identity, firmware,
factory-memory and user-calibration inputs, a distinct virtual controller address,
`SWITCH2_PROBE_ACK_SETUP04=ON`, and `SWITCH2_PROBE_USB_INIT=ON`. The
`SWITCH2_PROBE_*_FILE` inputs are checked by `probe_build.cmake`; private captures,
pairing records and firmware backups are not bundled with the source.
Keep a known-good UF2 and use a separate build directory for experiments.

For read-only **donor capture**, use a separate ordinary Bluepad32 build with
`SWITCH_PICO_SWITCH2_USB_BRIDGE=OFF`, `SWITCH_PICO_LOG=ON`,
`SWITCH_PICO_SWITCH2_MOUSE_CAPTURE=ON`, and
`SWITCH_PICO_SWITCH2_MEMORY_CAPTURE=ON`. After normal pairing/calibration,
each connected Joy-Con reads 192 acknowledged 64-byte pages covering factory
`0x13000..0x14fff` and user calibration `0x1fc000..0x1fcfff`, logged as
`SW2_MEMORY_<address>`. Setup identity/version logs identify the donor.
The capture adds no memory writes or erases; normal pairing rules still apply.
Keep these private captures out of commits. Add
`SWITCH_PICO_SWITCH2_MOUSE_CAPTURE_NATIVE=ON` to capture raw left `07` or right
`08` input through USB management after setup. This is capture firmware, not
left-side or dual-Joy-Con USB emulation; composite L/R acceptance is unverified.

**Left-only passthrough:** firmware `0.34-left-trace` has enumerated as a left
Joy-Con on Linux. A live USB check verified all 192 factory/user memory pages
and received 402 native `07` packets, including 400 motion-bearing packets whose
IMU blocks decoded and reconstructed exactly. The donor's separate stationary
capture also reconstructed all 539 blocks and measured approximately 0.995 g;
left directional axes and console gameplay remain unqualified. Fifteen targeted
tests pass, and left, right and Wii variants build.
The Switch subsequently completed left-side pairing and activation (runtime
`03/0C=1`, player LED mask 1), received motion-bearing `07` reports, and requested
its connection vibration cue, acknowledged by the physical donor. All 54 sampled
console motion blocks reconstructed exactly. The user confirmed menu navigation
with the left stick. Perceived vibration and directional IMU behavior remain
unconfirmed.

Left and right native USB pairing records use independent two-sector banks.
On the 4 MiB Pico 2 W, left occupies flash offsets `0x3b7000..0x3b8fff`; the
existing right bank stays at `0x3b9000..0x3bafff`. Profiles, configuration and
Bluetooth storage do not move. Host fault-injection checks verified old-right
record recovery, opposite-bank preservation, torn-write recovery and refusal
of foreign sector ownership.

**Simultaneous L/R experiment:** `SWITCH2_PROBE_COMPOSITE=ON` builds
`0.35-pair[-trace]` with two live native donor paths. It requires
`SWITCH2_PROBE_SIDE=RIGHT`, `SWITCH2_BRIDGE_INPUT=JOYCON2`, distinct physical
`SWITCH2_BRIDGE_SOURCE_ADDRESS` / `SWITCH2_BRIDGE_SECOND_SOURCE_ADDRESS`, and
distinct advertised controller addresses. The existing capture inputs describe
R; `SWITCH2_PROBE_SECOND_IDENTITY_FILE`, `SWITCH2_PROBE_SECOND_VERSION_FILE`,
`SWITCH2_PROBE_SECOND_FACTORY_FILE`, `SWITCH2_PROBE_SECOND_USER_CALIBRATION_FILE`
and `SWITCH2_PROBE_SECOND_CONTROLLER_ADDRESS` describe L.

The 151-byte USB configuration exposes R HID/vendor interfaces 0/1 and L
interfaces 2/3, using endpoint pairs 1/2 and 3/4 respectively. Both functions
have independent protocol state, native report consumption, feature gates,
command/reply queues, cue tokens and pairing records. Device-level VID/PID
remains `057e:2066`; explicit control indexes 2/3 address L, while index 0
continues to identify R. No identity is inferred from request timing.
Composite discovery uses the native device class `EF/02/01` and interface
associations. Single-side builds retain their published 80-byte configuration.
The flat per-interface trial described below was reverted in source.

Seventeen targeted tests pass across single and composite modes. A host smoke
through the actual USB callbacks exercised simultaneous report delivery,
cross-interface backpressure, deferred cue replies, fragmented commands,
indexed identities and disconnect/reset boundaries. Indexed storage
fault-injection and right, left, Wii, donor-capture, standalone-probe and
composite builds also pass. These checks do not establish Switch acceptance
of both functions; that requires the console enumeration trial.

Full-controller input splitting and continuous USB HD-rumble forwarding are
not implemented yet. This experiment relays two genuine Joy-Cons, including
their opaque motion/mouse packets and acknowledged built-in vibration cues.

The composite image has now been flashed with both pairing banks and all other
persistent storage verified unchanged. Linux enumerates all four interfaces;
indexed R/L identity reads and independent initialization succeed. A live check
received 250 reports from each function: L carried 248 motion blocks, while R
correctly remained neutral because its physical donor was not connected.
Both saved virtual pairing records restored. After reconnecting R, a simultaneous
live USB check received 376 reports from each donor, all 752 carrying motion
blocks that decoded and reconstructed exactly. On Switch, however, 0.35 only
initialized and displayed R: L was Bluetooth-active but USB-uninitialized, with
no commands or reports on its function. Connection order is not an adequate
explanation for the missing USB initialization.

Firmware 0.36 tested device class `00/00/00` without association descriptors.
Its interfaces, endpoints, reports, identities and protocol behavior were
unchanged. Binary comparison, 17 targeted tests, USB callback smoke and Linux
descriptor checks passed, but the user reported neither controller appearing on
Switch. R completed bulk initialization yet its input count stayed at one;
L remained USB-uninitialized, despite both Bluetooth sources being active.
The source was restored to 0.35 and rebuilt byte-identically to its saved image.
A later capture included USB restart and grip-screen activity: R resumed reports
and player assignment, but L remained uninitialized and its L press was not
detected by the console. This does not establish that opening the grip screen
alone caused R to recover.

The user-requested `SWITCH2_PROBE_JOIN_CHORD_GATE=ON` experiment builds
`0.37-pair-chord[-trace]` on the 0.35 native layout. It requires composite output
and suppresses each real L/R shoulder bit until both active physical sources
hold their shoulders. Release or stale/disconnected input closes the gate.
Both sources are polled before USB submissions. Native and common GET_REPORT
paths are gated; other buttons and opaque motion bytes are retained. The gate
does not synthesize presses, force initialization or make two USB device PIDs.
`JOIN_CHORD` traces record raw shoulder states and initialization status.
Host smoke checks covered these boundaries; the ungated firmware remained
byte-identical to 0.35. Firmware 0.37 was flashed with persistent storage
unchanged and both pairing records restored; its console chord test is pending.

- **Motion:** factory-calibrated Wii acceleration and MotionPlus gyro have
  independent freshness counters. Motion starts with the first usable fresh
  sensor pair; residual bias is refined in the background during quiet periods.
  There is no mandatory startup settling period. The encoder
  integrates real gyro into an orientation quaternion, using fresh near-1g
  acceleration to correct tilt drift, and emits the recovered 30-byte native
  IMU format. Fresh full-bar observations gently correct relative heading
  around world-up, following the sensor-fusion approach used by Dolphin.
  The first optical observation anchors the current heading without a jump;
  missing/inferred observations do not supply heading corrections. Translation
  changes optical bearing too, so this is not an absolute world-yaw reference.
  A fixed mounting transform makes a face-up Wii
  correspond to the virtual right Joy-Con's rail-down mouse pose. Missing,
  stale or uncalibrated sensors withhold IMU/mouse output rather than inventing it.
- **IR:** firmware 0.32 runs the actual libogc Wiiuse IR math pipeline from
  [commit `a4064a8`](https://github.com/devkitPro/libogc/blob/a4064a86487c46d8ab76d4fdf99e8059a62c4fa2/wiiuse/ir.c),
  not a separately implemented approximation. `tools/prepare_libogc_ir.py`
  checks pinned source hashes and extracts seven unchanged functions and their
  algorithm constants into a build-local C translation unit. The original
  sources remain untouched under `external/libogc_ir/upstream/`.
  Upstream owns bar selection, missing-dot recovery, smoothing, glitch counters
  and bounded screen mapping. It runs once per new camera report, including
  while USB output is disabled; its frame-count behavior is not replaced by
  custom timing or association thresholds.
  The separate native adapter supplies mirrored raw X, unchanged raw Y, and
  gravity roll in degrees. It converts upstream smoothed `sx/sy` changes into
  signed16 relative mouse reports with once-only consumption. A first native
  baseline waits for an accepted upstream position rather than its initial
  glitch-held origin; USB stalls, freshness loss and reconnects discard stale
  native movement. Brief upstream missing/glitch holds produce no invented
  movement. Relative output is not gated on upstream's bounded `ir.valid`,
  because the adapter does not know the actual host cursor position.
  Optical heading confidence excludes upstream-rejected glitches and inferred
  endpoints. The native IMU encoder and heading observer remain project code;
  they are not a verbatim Dolphin port. Legacy IR modes retain their old trackers.
  Native USB still provides no absolute cursor-position feedback or automatic
  synchronization; host sensitivity and initial cursor location still matter.
- **Controls:** stored profiles map ordinary controller buttons; IR does not
  create left/right clicks or turn button 1 into a desktop-mouse clutch.
  This is one virtual **right** Joy-Con, so left-only controls require profile
  remapping if needed. A mapped right stick takes precedence; otherwise a
  Nunchuk's mapped left stick supplies the single virtual stick using the
  calibration advertised to the console. Configure profiles in normal AIO
  firmware before using this probe.
  After Bluetooth setup, press a mapped face button (A with the tested profile)
  if the Switch has not assigned the controller. The button used to wake the
  Wii may be consumed during setup; observed activation changes player LEDs
  from mask 0 to 1. No automatic button press is injected.
- **Feedback/pairing:** built-in cue commands become bounded Wii on/off rumble
  patterns, not HD frequency/audio emulation. USB completion follows Core 1
  driver dispatch. Holding BOOTSEL for two seconds opens pairing; this probe
  never routes a long hold to clear pairings.
- **Qualification:** genuine Joy-Con passthrough mouse operation is confirmed
  on Switch with firmware 0.24. Earlier Wii builds deliver accepted native mouse
  and IMU reports, but pointing remained unreliable. Their custom tracker tests
  did not establish equivalence to upstream. Firmware 0.32 replaces that native
  tracker with the pinned upstream pipeline. Source identity and all seven
  retained function bodies are verified; the actual C code has been replayed
  on 495 recorded camera snapshots. All final 100 steady observations produced
  valid smoothed full-bar output. That 10Hz, zero-roll replay does not establish
  full-rate timing or improved console behavior; hardware qualification remains pending.
  Firmware 0.33 adds a camera-only sensitivity-level-2 trial after an on/off
  comparison retained one detection with the bar off. The upstream tracking
  code is unchanged. Both default level 3 and selected level 2 have been
  exercised through the real parser's complete camera-register setup; reduced
  interference and usable range still require a hardware comparison.
  **In progress; Wii pointing work is paused.** Firmware 0.33 was flashed and
  verified with persistent storage unchanged, but the level-2 on/off comparison
  has not been run. Erratic tracking, tracking loss and ineffective vertical
  movement remain unresolved; this checkpoint is not a completed Wii pointer.

Native Wii camera and viewport settings are build-time parameters, not stored profile changes:

| CMake option | Default | Meaning |
| --- | ---: | --- |
| `SWITCH2_WII_IR_SENSITIVITY` | `3` | Standard camera preset `2` or `3`; lower sensitivity may reduce interference and range |
| `SWITCH2_WII_IR_VIEW_WIDTH` | `660` | Viewport width in camera pixels |
| `SWITCH2_WII_IR_VIEW_HEIGHT` | `370` | Viewport height in camera pixels |
| `SWITCH2_WII_IR_OFFSET_X` | `0` | Horizontal offset from the camera center |
| `SWITCH2_WII_IR_OFFSET_Y` | `-115` | Below-screen bar; use `115` for above-screen |
| `SWITCH2_WII_IR_SPAN_X` | `1920` | Native mouse counts across viewport width |
| `SWITCH2_WII_IR_SPAN_Y` | `1080` | Native mouse counts across viewport height |

The 660 x 370 viewport and +/-115 vertical placement follow libogc's 16:9
defaults; they are not a measurement of the attached camera or screen. Spans
are **mouse counts**, not guaranteed display pixels. The viewport must remain
inside the 1024 x 768 camera image. For example, configure the existing native
Wii build with `cmake -S . -B build-switch2-usb-wii -DSWITCH2_WII_IR_OFFSET_Y=-115`,
then build normally. Trace builds sample raw IR diagnostics at 10Hz; diagnostic
flag bits 3/4 additionally indicate viewport inclusion and full optical reference.
Offsets locate the nominal screen rectangle and its diagnostics; a constant
offset alone cannot recenter a relative host cursor. Viewport dimensions and
mouse-count spans determine movement scale, but absolute pointing remains unsynchronized.
For the lower-sensitivity trial, configure with
`cmake -S . -B build-switch2-usb-wii -DSWITCH2_WII_IR_SENSITIVITY=2`, then build.
The parser logs the selected level after camera setup completes. Other firmware
builds retain standard level 3 unless explicitly configured otherwise.


**Port provenance and licensing:** see `external/libogc_ir/UPSTREAM.json` and
`external/libogc_ir/NOTICE.txt`. The component retains its full GPLv3 license
and libogc-specific independent-module linking exception; independent project
code is not relicensed. Preserve component notices and corresponding source
when distributing the firmware. The only C-language compatibility adjustment
is an equivalent disabled debug macro accepting one-argument calls in strict
C11; algorithm bodies and constants are unchanged. Transport/report decoding
is provided by the existing Bluepad32 path, not copied Wiiuse I/O stubs.
Do not edit generated `build*/libogc_ir.c` or the pinned original files to tune
tracking. Changes to upstream require an explicit pin/manifest update; native
protocol adaptations belong in the separate adapter.


Native bridge builds support the existing software **BOOTSEL reboot**
without erasing pairings, profiles or configuration. The standalone USB diagnostic
probe does not. Connect the bridge to a PC and disconnect any genuine USB
Joy-Con 2 before running this from the repository:

```sh
uv run python - <<'PY'
import usb.core
from switch_pico_bridge.config_manager import request_bootsel_reboot

product_id = 0x2066  # Use 0x2067 for a LEFT bridge build.
devices = list(usb.core.find(find_all=True, idVendor=0x057e, idProduct=product_id))
if len(devices) != 1:
    raise SystemExit(f"Connect exactly one native bridge (057e:{product_id:04x}).")
request_bootsel_reboot(devices[0])
print("Rebooting into USB BOOTSEL mode.")
PY
```

USB access requires permission to the matching `/dev/bus/usb` device. With
multiple bridges, select the intended PyUSB device by its `bus` and `address`
instead of sending to every matching device. Standalone native identities still
require the direct helper above, with no discovery request or interface claim.
Native **hub** builds from 0.72 expose management on their `057e:2068` root, so
the ordinary `switch-pico-config reboot bootsel` CLI and profile editor work there.

The standalone bridge's only management command is vendor-device OUT `0x40`,
request `0x04`, value `0x5350`, index `1`, with the validated 16-byte envelope.
Reboot is scheduled only after its control status ACK, followed by the existing
50 ms guard. Invalid envelopes cannot schedule it. Nintendo's separate request
`0x04`, value `0x0276`, index `0`, length `0` remains an ordinary setup
acknowledgement. Hub 0.72 additionally exposes the existing profile/configuration
management protocol on the root only; native child identities remain separate.

### Experimental stock-socket native Joy-Con 2 hub

`SWITCH2_PROBE_HUB=ON` exposes a `057e:2068` hub with separate right
`057e:2066` and left `057e:2067` devices through the unchanged Pico 2 W USB
socket. It uses the native USB PHY/SIE and a Core 1 SIO observer, not USB
wiring on GPIO pins. Each child retains its own native HID/vendor interfaces,
EP1/EP2 state, identity, protocol state and pairing bank.

This mode requires `SWITCH_PICO_SWITCH2_USB_BRIDGE=ON`,
`SWITCH2_PROBE_SIDE=RIGHT`, `SWITCH2_PROBE_COMPOSITE=OFF`, and
`SWITCH_PICO_SYS_CLOCK_MHZ=240`. `SWITCH2_BRIDGE_INPUT=JOYCON2` forwards the
two selected physical Joy-Cons; `DUALSENSE` translates one full DualSense into
the same virtual pair. Both require the private R/L identity/factory captures.
Joy-Con input additionally requires BLE/native capture and both source addresses.
Bluetooth runs cooperatively on Core 0; Core 1 is reserved for USB observation.
Receive PID state is selected before accepting OUT traffic.
Transmit payloads are prepared outside the bank lock and published by Core 0;
unavailable IN buffers NAK rather than expose another device's packet.

**Qualification history:** the earlier RAM-only
probe established three-address EP0 routing, not Joy-Con output. The
`0.65-native-hub-ready` bridge subsequently passed interleaved native descriptor,
identity and short control reads, both initialization sequences and bulk
isolation. Two consecutive 60-second captures received 7,504 and 7,496 native
HID packets, with correct R/L report IDs and lengths and no USB protocol error
or hub reset. All packets lacked live donor IMU, so both captures correctly
failed the live-input requirement.

An awake-controller trial exposed a separate hub-mode bug: the input capture
mailbox still allocated one channel unless composite mode was enabled, silently
rejecting L registration. Firmware `0.66-native-hub-input` enables both capture
channels for hub mode and checks that their count matches the controller models.
The dual-source BLE/capture regression failed on L packet delivery before this
fix; its new hub case and all 16 focused regression cases now pass.

Live PC qualification then passed with 575 R and 703 L decoded IMU reports and
changing sensor counters. A follow-up run received 587 R and 588 L live IMU
reports while completing 37 interleaved read-isolation rounds and matching the
Bluetooth-backed built-in motor-sample-0 acknowledgement independently on each
side. Neither run reported malformed or wrong-side packets or qualification
errors. These captures did not exercise deliberate button presses or establish
physical motor feel. The user subsequently confirmed that 0.66 works on Switch,
with some noticeable input lag. This is console smoke-test evidence, not a
latency measurement or exhaustive compatibility test. This mode does not add
arbitrary full-controller splitting or continuous USB HD-rumble forwarding.

**Queue latency follow-up:** `0.67-native-hub-latency` coalesces adjacent analog/
IMU-only Joy-Con updates when buttons, status, opaque fields and IMU format are
unchanged and neither packet contains relative mouse motion. Discrete transitions
and mouse packets keep their order, and a peeked packet is pinned until commit.
A reproducible 125Hz producer/62.5Hz consumer simulation of the actual capture
code reduced maximum queue age from 252ms to 4ms (mean 129.968ms to 4ms).
This is a same-format continuous-state workload, not measured Bluetooth-to-Switch
latency; different formats, discrete events and sustained mouse traffic still
use the bounded FIFO. The console lag improvement remains to be compared.

**One DualSense, two native halves:** use a separate private build configured
with `SWITCH2_BRIDGE_INPUT=DUALSENSE` and Classic Bluetooth enabled. The trial
uses `SWITCH_PICO_BLUETOOTH_MODE=CLASSIC` with
`SWITCH_PICO_SWITCH2_MOUSE_CAPTURE=OFF` and
`SWITCH_PICO_SWITCH2_MOUSE_CAPTURE_NATIVE=OFF`. It does not require Joy-Con
donors. Empty `SWITCH2_BRIDGE_SOURCE_ADDRESS` selects the uniquely eligible
ready DualSense/Edge; an explicit address filters that source. Multiple eligible
pads fail closed instead of mixing players. The secondary source address is unused.

In the dedicated DualSense mode, transport connections awaiting classification
count against Bluetooth capacity but do not reserve logical player/colour slots.
Only a supported PS5-parser source can enter those slots. Logical allocation
uses the first free slot rather than the Bluetooth device index, so an earlier
Pro Controller reconnect cannot move the first DualSense to the second colour.
Unsupported ready devices are disconnected without deleting their bonds;
normal AIO admission and slot assignment are unchanged.

The existing profile transform runs once for the full pad. R gets face buttons,
right stick/shoulder/trigger, plus and home; L gets the D-pad, left stick/shoulder/
trigger, minus and capture. Each uses its own advertised stick calibration.
One shared motion integrator consumes only fresh, complete, CRC-checked and
factory-calibrated DS5 sensor data. DS5 initializes from its first usable fresh
sensor pair. From 0.71, Wii also starts immediately and refines residual bias in
the background, without blocking IMU or resetting orientation. Invalid/stale sensors still
withhold IMU while controls remain available. Each USB half has independent
peek/commit, reset and backpressure state. No mouse movement or rail presses
are invented.

Built-in cue requests become bounded compatibility vibration on the corresponding
DualSense actuator, not Joy-Con HD waveforms or adaptive-trigger effects.
Completion means accepted L2CAP submission to the source driver, **not** a
DualSense application ACK or measured motor onset. Stop attempts are bounded; a persistently
blocked OFF path disconnects the stuck link without deleting its bond.

`0.67-native-hub-dualsense` built and was flashed with current persistent storage
verified unchanged. Its first PC run passed hub/child enumeration, native control
reads, initialization and bulk isolation, but had no real DualSense input and
therefore failed live-IMU qualification. Physical controls, native motion axes,
motor feel and Switch acceptance for this source remain pending. For a new bond,
open the Pico's two-second BOOTSEL pairing window, release it, then hold
DualSense Create + PS. Previously bonded pads normally reconnect with PS.

`0.68-native-hub-slot` fixes the observed red/second-slot case: the trace showed
a Pro Controller connecting first and the DualSense using Bluetooth index 1.
The regression reproduces that ordering and verifies logical slot 0 and its
lightbar colour, stable identity across a new Bluetooth index, unrelated
connection churn, pending-capacity accounting and rejection of late ready
callbacks. The update was flashed after a fresh full backup, with persistent
storage verified unchanged. USB transport checks pass; the post-update physical
DualSense reconnect and steady lightbar colour still need observation.

The subsequent Tears of the Kingdom wire trace showed IMU on both completed
USB endpoints after the delayed startup: 38 sampled R blocks and 37 L blocks
decoded with changing counters and quaternions. The Wii-style stationary gate
had delayed readiness until about 30 seconds after boot in that run. Firmware
0.69 removes that extra gate for factory-calibrated sources; the regression
checks first-sample output even while rotating, fresh-data recovery, and the
then-current Wii settling behavior. From 0.71, Wii no longer waits for that
estimate before emitting motion; it uses the nonblocking policy described below.
Factory calibration still applies, and bias is not cached across boots.

Translated full-controller builds expose `SWITCH2_BRIDGE_IMU_TARGET`:
`LEFT`, `RIGHT`, or `BOTH` (default). For example, configure the existing private
DualSense build with `-DSWITCH2_BRIDGE_IMU_TARGET=RIGHT` and rebuild/reflash.
This routes only IMU; both halves retain their controls. It consumes no controller
chord and changes no saved profile or pairing. The full dual-IMU PC checker
requires `BOTH`; use the USB-completion UART trace for single-target comparisons.

For the DualSense trial, pass `--build-dir build-switch2-native-dualsense` to
the checker below. Its configured shared-source policy permits identical IMU
blocks across the halves while retaining per-child identity, report-ID,
fresh-counter and control/bulk isolation checks.

**Any supported gamepad (0.70):** `SWITCH2_BRIDGE_INPUT=GAMEPAD` uses the same
native R/L hub and private identity/calibration captures, but accepts the normal
Bluepad32 gamepad families instead of filtering for a DualSense. Use a separate
private hub build with `SWITCH_PICO_BLUETOOTH_MODE=MIXED` to enable both Classic
and BLE controllers, and disable both `SWITCH_PICO_SWITCH2_MOUSE_CAPTURE` and
`SWITCH_PICO_SWITCH2_MOUSE_CAPTURE_NATIVE`. This is one logical controller
feeding one virtual R/L pair, not additional players. An empty source address
requires one uniquely eligible logical controller; multiple eligible sources
fail closed. An explicit address selects that controller (either member of an
existing Switch2 Joy-Con pair). Ordinary AIO pairing, identity, layout and
profile behavior is retained; original Switch Joy-Con grouping is not added.

- Buttons, sticks and profiles work independently of motion capability.
- Calibrated motion providers cover DS4, DS5/Edge, Switch/Joy-Con-compatible
  parsers, Switch2 Pro/Joy-Con, PS Move, and Wii/MotionPlus. Motion requires actual
  supported, calibrated and fresh acceleration **and** gyro samples. A pad with
  absent/invalid sensors remains usable for controls; no IMU is invented.
- Sensor counters advance at parser ingress, not when polled or when buttons
  arrive. A paired left Joy-Con cannot refresh the right-owned sensor stream.
  Wii acceleration cannot refresh a stalled MotionPlus gyro stream.
- `SWITCH2_BRIDGE_IMU_TARGET=LEFT|RIGHT|BOTH` also applies to `GAMEPAD`; Wii alone
  refines residual bias in the background while apparently stationary, without
  withholding valid IMU. Factory calibration still runs; no bias is saved across boots.
- Native cue requests use each source driver's bounded compatibility vibration.
  Mono drivers combine the two logical contributions; paired Switch2 Joy-Cons
  target their actual halves. This does not promise stereo, HD-waveform fidelity
  or physical actuator onset. Completion means driver dispatch (accepted L2CAP
  submission for DS5), not a remote application ACK. Missing rumble capability
  fails the request rather than claiming a motor response.
- The dedicated `WII` source remains the IR/native-mouse path. `GAMEPAD` does not
  synthesize mouse movement or rail buttons.

The private `build-switch2-native-gamepad` image uses mixed Bluetooth and the
unchanged stock USB socket. Software regressions cover real parser calibration,
report integrity/freshness, source selection, split/reset/backpressure, Wii
background correction and cue lifetimes. Version 0.70 was flashed with saved
storage verified unchanged, and the user confirmed DualSense operation.
Other controller-family motion orientation and motor response still need
physical qualification.

**Nonblocking Wii motion (0.71):** both the `GAMEPAD` and dedicated `WII` paths
emit motion on the first usable fresh acceleration/gyro pair. Bias collection
requires 1.5 seconds, at least 64 distinct gyro samples, low sensor variation and
stable gravity direction, but runs alongside output rather than gating it.
Accepted targets are applied at no more than 5 dps of correction per second;
they never reset the quaternion or undo accumulated yaw. The absolute candidate
gyro-vector limit is 30 dps, retaining headroom for the recorded Wii residual of
roughly 13 dps per axis without permitting unbounded learning or ratcheting.
Large rates, shaking and changing tilt discard the candidate; invalid/stale
sensors retire both the learned correction and target. Fresh recovery starts
immediately. DualSense and other factory-only sources do not run this tracker.

Quiet periods still improve drift; initial drift can be substantial with a large
offset. A sufficiently steady rotation about gravity below the candidate limit
cannot be distinguished from bias using these sensors alone. This is not a
guarantee of drift-free aiming while continuously moving. Built-in MotionPlus
needs no accessory handling or manual calibration command.

All 31 focused regressions pass, including immediate Wii output, bounded
background convergence without a pose reset, motion rejection, duplicate-poll
invariance and lifecycle recovery. A throwaway production-estimator smoke run
kept output ready from its first sample while converging to the recorded-scale
offset by six seconds. Generic, dedicated Wii, DualSense and mixed AIO builds pass.
Version 0.71 was then flashed and verified, with the saved-storage region
byte-for-byte unchanged. The hub and both native children enumerated, and UART
confirmed the nonblocking policy. Physical Wii startup/drift qualification is
still pending.

**Native hub profile editor (0.72):** connect the Pico's built-in USB socket to
the computer, then run:

```sh
uv run switch-pico-config profiles edit
```

The local editor runs at `http://127.0.0.1:8765/`. Save the profile before moving
the USB cable back to the Switch. The editor discovers only the hub root
`057e:2068`, validates its management response, and does not mistake the two
native children for extra adapters. Linux access is covered by the updated
`udev/99-switch-pico.rules`. USB output remains fixed to the native hub; ordinary
output-mode switching/reboot-to-mode is unavailable.

Select the Wii profile owner and its active profile. With the Nunchuk connected,
Auto uses its live layout; while offline, choose **Preview · Wii Remote + Nunchuk**.
The physical Nunchuk **C** is logical `west` and **Z** is logical `north`, not the
unrelated Switch2 extra control named `c`. Both can target buttons or triggers.
For example, **Z → L**, **C → ZL**, with **Remote 2 → R**, makes **Z + 2** the
physical L+R combination. Save changes to that Wii profile, not the global
default or another controller's profile.

This fixes the omitted editor integration: earlier hub builds accepted only
the private BOOTSEL management command, and host discovery excluded their root.
The existing profile service and storage transactions are reused. Root requests
cannot borrow child EP0 buffers; aborted/short/corrupt transfers and reset-stale
status completions cannot dispatch profile writes. Valid status ACKs preceding
a subsequent SETUP remain valid.

Qualification: 390 focused tests pass. The actual browser editor saved the
Wii C/Z example and read it back after a Pico reboot. All 80 stored profiles
were compared: only the two intended mappings changed; the other 79 profiles,
metadata and active selections were unchanged. The configuration/pairing flash
region matched the pre-update backup. Native R/L descriptors, EP0 identity,
initialization and bulk-isolation checks passed while editor traffic was active.
No physical Switch L+R button press was claimed by that transport check.

For sensorless hardware, the checker supports `--input-only`: press real buttons
and keep changing controls on both halves during the run. Neutral fallback
alone cannot qualify. The result explicitly records that IMU was not required;
omit this option to retain the strict dual-IMU check.

With the existing private build configured, qualify on a PC using:

```sh
uv run python tools/native_joycon_hub_check.py \
  --build-dir build-switch2-native-hub \
  --output build-switch2-native-hub/qualification.json
```

Wake both physical Joy-Cons and move them during the manual-wake window.
The checker rejects neutral/zero-length IMU reports and requires fresh,
decodable motion with changing counters from both devices. It does not pair,
reset, change profiles or write flash. `--rumble-sample 0` is an explicit
optional motor-cue test, not a continuous HD-rumble test. Captures and flash
backups contain private device data and must remain untracked.

`HUB_RADIO reports` counts normal parsed gamepad callbacks, which native packed
input bypasses. Zero is not evidence that a native donor is asleep or inactive;
use per-source activation and the host's fresh native IMU results instead.

The hardware trials verified the complete persistent region
`0x103b7000..0x10400000` unchanged before and after application-only flashing.
Software BOOTSEL recovery uses the existing helper above on the verified
`057e:2068` root, not either child. UART remains available during qualification.
Watchdog recovery and failure to configure the initial root hub enter BOOTSEL
without erasing storage; neither mechanism proves successful controller output.

### Switch 2 controller input

The AIO firmware implements the proprietary BLE protocol for Nintendo `057E:2069` (Pro), `057E:2067` (left Joy-Con 2), and `057E:2066` (right Joy-Con 2). This is controller **input** support, distinct from the existing Switch 2 console-wake feature and from emulating a native Switch 2 USB controller.

- **Pairing:** fresh SYNC pairing requires the existing bounded pairing window. A directed reconnect must target this adapter's Bluetooth address and match its persistent application-level authorization. These links are unencrypted and are **not authenticated SMP bonds**. No global Bluetooth security downgrade is made; automatic SMP requests for these devices fail closed while other controllers retain their existing policy. Public/static addresses can own profiles; transient private addresses are not promoted to persistent identities.
- **Joy-Con ownership:** the saved player-mode default and connection-only shortcut determine whether ready opposite halves join. Enrollment must finish before a pair is published; a failed join leaves the solos intact. Pair identity includes both typed member addresses, and the right half supplies paired motion. Splitting clears stale input/effects and restores sideways solo banks. Changing the default does not disconnect Bluetooth; a participant disconnect clears its temporary grouping override. Two pairs exhaust the four physical Bluetooth connections. New authentication remains restricted to the normal pairing window.
- **Protocol:** service, characteristic and CCCD UUIDs are discovered rather than trusting fixed ATT handles. Setup requires matching acknowledgements, reads user/factory stick calibration and gyro bias, and rejects malformed/failed transactions. Motion is normalized to the existing SDL-oriented units; sensor clock/range classification and physical axis accuracy still need wider model qualification.
- **Rumble:** Switch HD commands now retain independent left/right frequency/amplitude fields and up to three ordered subframes through the native Switch 2 encoder and bounded queues described below. XInput and local feedback retain their conventional fixed-carrier behavior. This is separate from the original Switch-native opt-in backend.
- **Not implemented:** Joy-Con mouse output, native GameChat signaling, NFC/IR and Switch 2 NSO GameCube support. C and back/rail inputs can instead be remapped to controls the selected USB mode supports.

This is a scoped reimplementation informed by [Bluepad32 PR #219](https://github.com/ricardoquesada/bluepad32/pull/219), reviewed at `9c95e43a87d3bd8a68565da0836d8a758bd8d8af`, not a wholesale fork import. Protocol references: [ndeadly's research](https://github.com/ndeadly/switch2_controller_research), [Nadeflore](https://github.com/Nadeflore/switch2-controllers), [Switch2Connect](https://github.com/TommyWabg/Switch2Connect), and [SDL's Switch 2 sensor implementation](https://github.com/libsdl-org/SDL/blob/main/src/joystick/hidapi/SDL_hidapi_switch2.c).

**Verification:** 349 tests passed; AIO, XInput/feasibility, HD-rumble, haptics and UART firmware variants built. Native protocol tests use real BTstack types/accessors. Lifecycle/storage tests cover player-mode transitions, both connection orders, two-pair membership, bank isolation and interrupted initialization. Gesture cases cover timing/freshness/wrap, input masking, real macro/capture cancellation, ambiguity, failures, and disconnect/default restoration. Studio controls, draft preservation, CLI refresh and desktop/mobile layouts were exercised in Chromium.

On hardware, the new L + R owner copied all eight left-bank profiles and names, bringing the inventory to eight owners / 64 profiles without changing the original 56. A temporary edit and name on inactive pair profile 8 survived reboot while both solo banks stayed unchanged; pair activation was also independent. Test edits, names and active selections were restored. Studio selected the real composite owner and its Identify action produced two physical rumble dispatches.

Hardware mode changes split the pair into two live USB slots and recombined it without fresh pairing; Individual remained selected after reboot. The user felt join and split confirmation pulses from the physical shortcut. Saved configuration generation stayed unchanged during each gesture. A temporary split under a Paired default ended after resetting/reconnecting the right half, restoring the original pair. The final saved default is Individual; all 64 profiles, names, aliases and active selections were preserved.

At the initial Switch 2 input checkpoint, a real Pro (`3C:A9:AB:65:73:12`) completed setup, appeared in persistent pairing/profile inventories, and delivered live sticks, accelerometer, gyro and independent C/GL/GR presses. A 100-report USB rumble exercise retained its connection while 3,033 controller reports arrived. Schema-7 extra mappings were written/read and restored; all 32 then-existing profiles, metadata and active selections were preserved, with adapter configuration generation 13 / CRC `3af5ee18` unchanged. Subsequent native-rumble and pair measurements are documented here and below; wider sensor-axis, perceptual-equivalence and long-duration transport qualification remains open. Use schema-7-capable firmware after saving expanded profiles.

### Switch 2 native HD rumble

Switch-mode host commands use decoded HD parameters, not the compatibility
strong/weak peak values. Pro output preserves two independent actuators;
paired Joy-Con 2 output routes each source side to its physical half.
A solo Joy-Con uses the louder source independently for each band, retaining
that band's frequency, with left winning ties and shorter sequences holding
their final sample. Profiles scale amplitudes before this conversion.

Physical microphone characterization on the Pro Controller established:

- Each five-byte sample contains two **10-bit frequency + 10-bit amplitude**
  fields. The measured frequency model is
  `Hz ~= 10 * 2^((code - 1) / 96)`.
  Original low/high indices map to `193 + 3*index` / `289 + 3*index`;
  index 64 therefore produces codes 385/481 (160/320 Hz).
- Block headers `0x50`, `0x60`, `0x70`, plus the four-bit sequence counter,
  select **one, two or three** valid samples. Unused slots are zeroed.
  Filling three slots under `0x50` does not play the later slots.
- A randomized 90-packet run using the other actuator as an acoustic timing
  reference measured **5.27 ms/frame, ±0.16 ms statistical 95% interval**.
  Acoustic/threshold systematic error is not included. The sender uses
  conservative **6/11/16 ms** submission guards, not a claim of exact onset.

Linear Q0.15 amplitudes use SDL's conservative `29000/65535` envelope, producing
native codes 0–453. This preserves a linear input curve but is not calibrated
physical-force equivalence; it can feel different from compatibility rumble.
Source frequency indices are bounded to 1–127.

Each logical slot has a 16-command cross-core ingress FIFO; each physical
Switch 2 controller has a 16-command transport FIFO. Commands keep their
original receipt time and connection/output generation. Native commands expire
after 50 ms; a batch that cannot fit its complete playback guard before that
deadline is discarded rather than started halfway stale. Expiring unplayed
history does not interrupt current playback or force a useless HOLD ahead of
fresh work. Consecutive identical one-sample holds may refresh a pending
command; multi-sample sequences are never coalesced.

Stops flush older host work, including under backpressure. Local feedback owns
a separate bounded override while host state advances underneath it; resuming
uses the current valid final sample, not a replay of masked history.
Keepalives likewise send only the final sample with count 1. Pending ATT
write-request buffers remain immutable; late completion cannot resurrect an
old epoch after stop, reconnect or Joy-Con topology change.
Normal GATT-client busy responses are treated as transient backpressure, not
as controller disconnects.

Final single-Pro hardware runs:

| Workload | Result |
|---|---|
| Stereo, frequency sweep, and three-subframe patterns (195 USB reports) | Zero ingress/output drops; 1,193 input reports continued |
| 512 changing one-subframe commands at 125.14 Hz | Zero ingress/output drops; clean stop |
| 128 changing three-subframe commands at 125.11 Hz | Zero ingress drops; 69 output-stage commands discarded/superseded; clean stop and empty ingress |

Paired Joy-Con 2 hardware runs exercised left-only, right-only, stereo and
ordered-subframe effects: 197 host requests produced 394 physical dispatches
with zero ingress/output drops. The user confirmed both sides vibrated as
intended. Held one-subframe commands at 125 Hz, changing one-subframe commands
at 62.5 Hz, and three-subframe commands at 50 Hz completed without drops.
Higher changing workloads discarded output-stage commands, but the pair
remained connected after the GATT backpressure fix.

Three-subframe commands at 125 Hz exceed the native playback budget. These
results do **not** establish lossless arbitrary workloads or perceptual
equivalence. All 40 profiles, names, active selections and adapter
configuration were preserved during this upgrade.

**Switch 2 connection timing:** all supported Switch 2 BLE controllers now request
**7.5 ms connection intervals**, including Switch 2 Pro and both Joy-Con 2 halves
in Paired or Individual mode. The earlier 30 ms policy for multiple Switch 2 links
is removed; additional BLE or Classic controllers do not slow these requests.
Unrelated BLE controllers are not retimed. Bonds, profiles, HD encoding and the
DualSense timeout are unchanged. Initial setup retains ownership of its interval
request; ready links reconcile negotiated intervals with at most one retry per
second. More frequent connection events favor responsiveness but can increase
shared-radio contention; this does not establish lower measured gameplay latency.

The BLE connection interval is not the rumble packet cadence: multiple packets
can travel per connection event. The normal active-output algorithm is unchanged
by this checkpoint; the direct-packet lab fixtures are not release features.
Lifecycle coverage includes Paired/Individual mode changes, Classic arrival and
departure, handle reuse, multiple Switch 2 Pro links, correcting slow negotiated
intervals, unrelated BLE isolation, asynchronous settlement across clock wrap
and bounded retries after rejected requests.

**Idle Switch 2 rumble traffic:** the parser sends three successful neutral
writes, then suppresses further idle output. Any successful non-neutral packet
re-arms that stop budget, including a late completion from an older logical
epoch. Failed or pending ATT writes do not count as completed stops. Repeated
host stops still discard queued history without restarting settled idle traffic.
Active-effect cadence, native subframes and watchdogs are unchanged; the parser
timer remains available for control and expiry work.

**Qualification and known mixed-radio limitation:** this default does not claim
reliable native DualSense PCM alongside two fast Joy-Con links.

- Earlier 15/30 ms trials favored 30 ms for transport continuity: at 15 ms the
  same 125 Hz-per-slot workload timed out around four seconds; at 30 ms it
  completed 30 seconds with 1,384 PCM sends, 47 skips and no send failures.
  All three links stayed connected, but the user later clarified that Joy-Con
  rumble felt weak at 30 ms and DualSense rumble was also weak/inconsistent.
- Idle suppression was measured at 30 ms: outgoing HCI ACL writes fell from 974
  to 469 per ten-second idle sample, while both samples sent 469 DualSense PCM
  packets without skips. Total HCI writes did not fall because receive-credit
  traffic increased. A subsequent 30-second workload sent 1,431 PCM packets
  with zero skips/failures, two Switch 2 ingress drops and nine output-stage
  drops. This was a transport improvement, not full perceptual qualification.
- In the isolated right-Joy-Con fixture, one- and three-sample packets at a
  20 ms packet cadence both felt weak. One-sample packets at a nominal 7.5 ms
  cadence, with the same 320 Hz frequency and amplitude, produced user-confirmed
  clear tone and good feel. Each two-second burst sent 267 packets with a
  7.504 ms average submission gap, not a measured on-air or actuator interval.
- Adding native DualSense PCM to that fast fixture failed in about 0.66 seconds,
  both with one Joy-Con vibrating and with both vibrating. Both Joy-Cons remained
  connected in each test. DualSense sent 11 PCM packets before timing out; with
  both vibrating, Joy-Con submission gaps reached 32 ms. The fixture aborted and
  completed explicit stop writes.

The shared Classic/GATT scheduler trial was not qualified and is excluded from
this checkpoint. Temporary packet fixtures are also excluded. Native DualSense
buffer/cadence qualification remains separate; no gain increase or fallback mode
is silently applied to conceal transport loss.

### Rumble per controller

Commands remain bound to a USB slot and Bluetooth connection generation. Compatibility output uses a latest-value mailbox; native output keeps a bounded timestamped command history instead of collapsing substeps.

Microsoft controllers (`045E`) using Bluepad32's Xbox parser now add **impulse-trigger rumble** while retaining the existing strong/weak grip output. Switch high-band amplitude drives the corresponding left/right trigger, taking the peak across each command's substeps and capping the added output at half scale. Conventional/XInput high-frequency magnitude drives both triggers at half strength. Profile rumble scaling applies before this mapping; local confirmation/identify pulses remain grip-only. All four motors share the existing duration/stop handling. This is amplitude-only translation, not HD/PCM playback or adaptive-trigger resistance; the compatibility mailbox and transport cadence are unchanged.

The connected Classic Xbox (`045E:02E0`) was exercised with 307 USB reports: two rounds of left high-band, right high-band, and both, with intervening stops. It remained connected, and configuration generation 13 / CRC `3af5ee18` was preserved. Firmware counters confirmed host rumble dispatch; the user tested the effect and accepted it as good. The 261-test suite passed, including Xbox side isolation, amplified trigger-only output, conventional mapping, stop, and disconnect coverage.

The standard AIO and XInput builds use **300 MHz at 1.3 V**, packet-level CYW43 reads, bounded HCI credit returns, and native DualSense haptics by default. The first eligible DualSense/DualSense Edge that becomes ready can occupy the one native PCM stream, in any slot; later controllers do not steal it. Nintendo native output is a separate, explicit per-controller opt-in described below. Unapproved and unsupported controllers retain their existing parser-specific output. To change the selected DualSense manually, stop the current run and use `haptics-experiment gameplay --slot N` (API slots are zero-based).

In Switch mode, that stream preserves decoded left/right, low/high-band HD commands. In XInput mode, strong/low magnitude drives the left 160 Hz carrier and weak/high drives the right 320 Hz carrier; these commands stay active until changed or stopped. XInput does not supply Nintendo frequency/substep detail. USB reset, unmount, and suspend stop held host rumble. Auto-mode XInput additionally reboots to Switch probe after unmount, by the existing one-attachment policy; manual XInput is exempt.

Standard native gameplay uses **32 stereo frames at 3 kHz** per Bluetooth report (93.75 reports/s), with 10.667 ms causal lookback. Startup first writes a state-only AudioControl-enable report, then sends the full control header and one 64-byte PCM block used by the physically accepted reference. This requires packet-level reads, credit batching and at least 300 MHz; the normal AIO/XInput defaults already provide them. The user confirmed strong, distinct left/right native output and clean stops. The gain curve is unchanged.

`SWITCH_PICO_HD_PACKET_FRAMES=64` retains the compact two-block format only as an explicit, physically unqualified experiment: it felt worse despite clean transport counters. Mixed-controller and long-duration fidelity still need qualification. Native streaming continues silence while idle; no physical actuator-onset bound or lossless-radio claim is made. See [HAPTICS_EXPERIMENT.md](HAPTICS_EXPERIMENT.md) for exact initialization, packet formats, and the distinction between current acceptance and historical measurements.

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
Approvals persist in adapter configuration schema 4 (232 bytes), alongside the
Joy-Con default mode. Schema 1/2 migration starts with no approvals; schema 3
migration preserves its approval list. Profile schema 9/catalog 3 are separate.

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
existing regular, AIO (including BLE/Classic), and wake-capture CMake caches, then from
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
| Executable flash image | 833,696 bytes | 4 MiB |
| Indexed profile arenas | 256 KiB | 4 MiB flash |
| Adapter configuration | 8 KiB | 4 MiB flash |
| BTstack bonds and Switch 2 application authorizations | 8 KiB | 4 MiB flash |
| RP2350 terminal sector | 4 KiB | 4 MiB flash |
| Allocated/reserved SRAM, including heap and stacks | 151,844 bytes | 520 KiB |

The executable plus persistent reservations consume 1,116,320 bytes of flash,
leaving 3,077,984 bytes. Allocated SRAM sections leave 380,636 bytes of link-time
headroom; this is not a runtime heap high-water measurement. Core 0 has a
4 KiB stack, and Core 1 uses a dedicated 16 KiB stack in main SRAM for nested
catalog migration/compaction rather than overflowing its 4 KiB scratch bank.

Profiles use two 128 KiB append-only arenas and retain 248 physical record
slots. Catalog 3 uses a 128-byte header plus a 384-byte profile in the same
512-byte stride. The second page is programmed before the header-containing
first page, and records are read back before publication. Profile names
remain 256-byte metadata payloads and aliases remain 32 bytes. The compact
index stores locations and generations; active/fallback profiles for observed
identities and the selected profile are decoded, not the entire database.
Pair initialization commits one seed record that snapshots the left bank's
immutable profile/name record references and active selection. Subsequent
writes are independent, and compaction materializes pair-owned records.
Interrupted initialization exposes either no pair bank or the complete bank,
never a partly copied bank. An ambiguous storage write/readback failure freezes
catalog access until reboot/replay; existing live profiles retain their last
committed cached settings.

The catalog supports eight profiles for the global fallback and each of 16
stable identities, including pair owners. Missing records resolve to defaults, so profiles 5–8 do
not consume flash until changed. When an arena fills, the latest indexed
records are compacted into its peer and the new superblock is published last.
Interrupted or corrupt appends leave the previous valid record available.
Catalogs 1/2 and retired four-profile banks migrate through the alternate arena;
the old published data is retained until all copies and the new superblock
verify. Schema 1–8 profiles retain their meaning when decoded as schema 9.
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
