# Switch Pico Controller Bridge

Raspberry Pi Pico firmware that emulates a Switch Pro controller over USB and a host bridge that forwards real gamepad input over UART (with rumble round-trip).

## What you get
- **Firmware** (`switch-pico.cpp` + `switch_pro_driver.*`): acts as a wired Switch Pro. Takes controller reports over UART1 and passes rumble from the Switch back over UART.
- **Python bridge** (`switch_pico_bridge.controller_uart_bridge` / CLI `controller-uart-bridge`): reads SDL2 controllers on the host, sends reports over UART, and applies rumble locally. Hot‑plug friendly and cross‑platform (macOS/Windows/Linux).
- **Colour override** (`controller_color_config.h`): compile‑time RGB overrides for body/buttons/grips as seen by the Switch.

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
   - Any SDL-compatible gamepads you want to use should also be plugged into (or paired with) the same host computer that runs the Python bridge; the bridge is the one reading them.

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
```sh
cmake -S . -B build -DSWITCH_PICO_AUTOTEST=OFF -DSWITCH_PICO_LOG=OFF
cmake --build build -j
```
Flash the UF2 to the Pico (e.g., bootsel + drag-drop or `picotool load`).
Flags:
- `SWITCH_PICO_AUTOTEST` (default ON in some configs): disable autopilot/test replay with `OFF`.
- `SWITCH_PICO_LOG`: enable/disable UART logging on the Pico.

### Changing controller colours
`./build.sh` now writes a random colour into `controller_color_config.h` before building so the Switch shows a fresh colour when you flash.
- Use `./build.sh --color FF00AA` (or `--color 12,34,56`) to pick a specific colour applied to the body/buttons/grips.
- Use `./build.sh --keep-color` if you want to build without touching `controller_color_config.h`.

## Python bridge (recommended)
Works on macOS, Windows, Linux. Uses SDL2 + pyserial.

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

- SDL2 runtime: install via your OS package manager (macOS: `brew install sdl2`; Windows: place `SDL2.dll` on PATH or next to the script; Linux: `sudo apt install libsdl2-2.0-0` or equivalent).

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
- Ensure SDL2.dll is on PATH or alongside the script.

### Linux tips
- You may need udev permissions for `/dev/ttyUSB*`/`/dev/ttyACM*` (add user to `dialout`/`uucp` or use `udev` rules).

## Troubleshooting
- **No input on Switch**: verify UART wiring (Pico GPIO4/5), baud matches both sides, Pico flashed with current firmware, Switch sees a “Pro Controller”.
- **Constant buzzing rumble**: the bridge filters small rumble payloads; ensure baud isn’t dropping bytes. Try lowering rumble scale in `switch_pico_bridge.controller_uart_bridge` if needed.
- **Guide/Home triggers system menu (macOS)**: try different controller mode (XInput/DInput), disable Steam overlay/controller support, or connect wired.
- **SDL can’t see controller**: load `switch_pico_bridge/controller_db/gamecontrollerdb.txt` (default), add your own mapping, or try a different mode on the pad (e.g., XInput).
