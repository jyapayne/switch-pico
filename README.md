# Switch Pico Controller Bridge

Raspberry Pi Pico firmware that emulates one or more Switch Pro controllers over USB. Input can come from the SDL3-to-UART computer bridge or, on Pico 2 W, directly from Bluetooth controllers through Bluepad32.

## What you get
- **Firmware** (`switch-pico.cpp` + `switch_pro_driver.*`): acts as a Switch Pro controller (one on standard Pico, four on Pico 2 W AIO), accepting either UART bridge reports or the optional Pico 2 W Bluepad32 backend.
- **Python bridge** (`switch_pico_bridge.controller_uart_bridge` / CLI `controller-uart-bridge`): reads SDL3 controllers on the host, sends reports over UART, and applies rumble locally. Hot‑plug friendly and cross‑platform (macOS/Windows/Linux).
- **Color configuration** (`controller_color_config.h`): compile-time RGB colors for emulated controller grips and supported Bluetooth controller LEDs.
- **Pico 2 W AIO firmware** (`firmware/switch-pico-aio.uf2`): hosts four concurrent Bluetooth controllers and sends their controls, calibrated motion, rumble, and slot identity through four separate Switch Pro USB interfaces without a computer.

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

Both `build.py --aio` and direct AIO CMake configuration apply `patches/bluepad32-sdl3-imu.patch` idempotently before compiling Bluepad32. The patch makes supported motion controllers use SDL3-equivalent axes and fixed-point units before conversion to Nintendo samples. It intentionally leaves the dependency worktree dirty; the committed submodule revision remains Bluepad32 4.2.0.

### Pairing up to four controllers

1. Flash and connect the Pico 2 W to the Switch.
2. Enable `System Settings → Controllers and Sensors → Pro Controller Wired Communication`.
3. Hold the Pico's BOOTSEL button for about two seconds, until the onboard LED starts double-blinking. This opens a 60-second pairing window.
4. Put a controller into Bluetooth pairing mode:
   - DualSense: hold Create + PS.
   - DualShock 4: hold Share + PS.
   - Switch Pro: press its sync button.
   - Xbox Bluetooth controller: hold its pair button.
   - 8BitDo: use a Bluetooth mode supported by Bluepad32; use Switch/S mode when motion is required.
5. Wait for the controller's player light to settle. Repeat step 4 for additional controllers while the window remains open. Holding BOOTSEL again extends the window by 60 seconds from that point.

Pairing order determines the initial USB slot assignment. Up to four controllers map 1:1 to the four emulated Switch Pro Controller interfaces.

Outside the BOOTSEL-open window, Bluetooth discovery and incoming connections are disabled. The Pico does not scan for or reconnect disconnected controllers while locked. Pairing keys still persist, but reconnecting a previously paired controller also requires opening the BOOTSEL window before pressing its normal power button.

### LED meanings and device state

The Pico 2 W onboard LED reports the overall Bluetooth state:
- **Double blink**: the bounded pairing window is open.
- **Fast blink**: a controller connection is still completing its handshake.
- **Solid**: at least one controller is active.
- **Slow blink**: no controller is active and pairing is locked.
- **Solid immediately after boot that never transitions**: Bluepad32 initialization did not complete; check firmware flashing and UART logs.

### Managing controller disconnect and reconnect

- **Disconnect a controller**: its slot immediately publishes neutral buttons, sticks, and motion. Other connected controllers are unaffected.
- **Reconnect a paired controller**: hold BOOTSEL until the LED double-blinks, then power on the controller normally.
- **Pair a new controller**: hold BOOTSEL until the LED double-blinks, then put the controller into its explicit Bluetooth pairing mode.
- **Pairing window expires**: scanning and incoming connections stop; already connected controllers remain connected.

### Per-slot controller colors

Each AIO slot has one color shared by its emulated Switch Pro grips and its physical Bluetooth controller:

1. Blue `#0089EB`
2. Red `#E63946`
3. Yellow `#F6C945`
4. Green `#2ECC71`

When a controller becomes ready, RGB-capable devices such as DualSense and DualShock 4 receive a darker, more saturated RGB value derived automatically from the slot's Switch grip color. Controllers without an RGB light use player indicator 1, 2, 3, or 4 when Bluepad32 exposes player-LED control. Devices without either capability are left unchanged. Edit only the four grip colors in `controller_color_config.h`; rebuilding automatically recalibrates their lightbar colors.


### Controller capabilities

| Controller | Buttons/sticks | Rumble | Motion |
|---|---:|---:|---:|
| DualSense / DualShock 4 | Yes | Yes | Yes |
| Switch Pro | Yes | Yes | Yes |
| 8BitDo in Switch-compatible Bluetooth mode | Yes | Model-dependent | Yes when the mode exposes IMU |
| Xbox Bluetooth controller | Yes | Yes | No hardware IMU |

Motion is normalized to 1024 units per degree/second and 8192 units per g in SDL3 axes, then converted to Nintendo axes and raw counts. The latest normalized sample is duplicated across the report's three nominal 5 ms slots; it remains pending until a regular `0x30` USB report successfully consumes it.

### Rumble per controller

Rumble effects are per-slot and independent. The Switch sends rumble commands to a specific USB interface, and the Pico routes each command to the Bluetooth controller in the matching slot. Each slot has a critical-section-protected latest-value mailbox tagged with its connection generation; a newer pending command replaces the older one, and disconnect invalidates commands from the prior controller.

### Hardware validation

The four-interface AIO build has been verified on a real Switch with two DualSense controllers: the Switch assigned independent controller slots, and buttons, sticks, calibrated motion, rumble, and disconnect isolation worked per controller. Fresh DualSense pairing through the BOOTSEL-open window has also been verified on hardware.

To reproduce the validation:

1. **Verify USB enumeration**: Connect the Pico 2 W to a USB host or analyzer. Confirm that four HID interfaces are present, using IN/OUT endpoint pairs `0x81/0x01` through `0x84/0x04`.
2. **Verify Bluetooth pairing**: Hold BOOTSEL until the LED double-blinks, put a controller into explicit pairing mode, and confirm its player light settles.
3. **Verify input on one controller**: Move sticks and press buttons; confirm only its assigned Switch slot changes.
4. **Verify input on two controllers**: Move the second controller independently and confirm the first controller's slot is unaffected.
5. **Verify the pairing gate**: Disconnect a controller and confirm it does not reconnect while locked. Open the BOOTSEL window, power it on, and confirm it can connect.
6. **Verify rumble per slot**: Send rumble to interface 0 and confirm only the slot 0 controller vibrates. Send rumble to interface 1 and confirm only the slot 1 controller vibrates.
7. **Verify motion**: Enable gyro/accel on both controllers. Rotate each controller independently and confirm that motion is per-slot (rotating controller 0 does not affect controller 1's IMU output).

On the tested Linux host, all four HID interfaces enumerated, but `hid-nintendo` timed out (`-110`) while requesting controller information from the composite device and removed the transient hidraw nodes. This is an observed, undiagnosed composite interoperability limitation; its root cause has not been established. The timeout was not observed on the Switch, so successful `hid-nintendo` binding is not the release criterion for the four-interface AIO firmware.

Bluepad32 is Apache-2.0. BTstack use on Pico W/Pico 2 W is covered by Raspberry Pi's BTstack license.

## Planned features

## Limitations
- No NFC/amiibo/IR support.
- Rumble is best-effort: the UART build depends on SDL3 haptics; the AIO build depends on the connected controller's Bluepad32 rumble implementation.
- The UART firmware requires a host computer running the bridge. The Pico 2 W AIO firmware does not; it hosts controllers over Bluetooth, not USB.

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

Nintendo sends two stateful four-byte HD-rumble actuator words. Each word can carry full or relative high/low frequency and amplitude commands with up to three subsamples; amplitude uses a logarithmic curve. The Pico decodes both words once in `SwitchHapticsDecoder`, retains actuator state across packets, and reduces the result to conventional low/strong and high/weak motor magnitudes. SDL3 and Bluepad32 cannot reproduce the original linear-actuator frequencies or left/right spatial effects, but they receive the correct nonlinear band amplitudes.

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
Prereqs: Pico SDK + CMake toolchain set up.

### Using `build.py`

`build.py` configures CMake, builds the firmware, checks that both output formats
were created, copies the release artifacts into `firmware/`, and flashes the ELF
with `picotool`.

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
`controller_color_config.h` before building. With no color option, the
per-slot blue/red/yellow/green palette is left unchanged. Run
`python3 build.py --help` to see the available command-line options.

If the tools or artifacts are in non-default locations, use these environment
variables:

```sh
PICOTOOL_PATH=/path/to/picotool \
ELF_PATH=/path/to/switch-pico.elf \
UF2_PATH=/path/to/switch-pico.uf2 \
python3 build.py
```

`PICOTOOL_PATH` selects the flashing tool, `ELF_PATH` selects the ELF that is
checked and flashed, and `UF2_PATH` selects the UF2 that is checked after the
build. Their defaults are `picotool` from `PATH`, `build/switch-pico.elf`, and
`build/switch-pico.uf2`, respectively.

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

`switch_pro_driver.cpp` implements this in `integrate_motion_sample()` and `fill_quaternion_imu_report_data()`:

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
