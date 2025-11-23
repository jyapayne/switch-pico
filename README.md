# Switch Pico Controller Bridge

Raspberry Pi Pico firmware that emulates a Switch Pro controller over USB and a host bridge that forwards real gamepad input over UART (with rumble round-trip).

## What you get
- **Firmware** (`switch-pico.cpp` + `switch_pro_driver.*`): acts as a wired Switch Pro. Takes controller reports over UART1 and passes rumble from the Switch back over UART.
- **Python bridge** (`controller_uart_bridge.py`): reads SDL2 controllers on the host, sends reports over UART, and applies rumble locally. Hot‑plug friendly and cross‑platform (macOS/Windows/Linux).
- **Optional Rust bridge** (`fast_uart_bridge/`): higher‑performance alternative if the Python bridge has rumble latency.
- **Colour override** (`controller_color_config.h`): compile‑time RGB overrides for body/buttons/grips as seen by the Switch.

## Hardware wiring (Pico)
- UART1 pins (fixed in firmware):
  - **TX**: GPIO4 (Pico pin 6) → RX of your USB‑serial adapter.
  - **RX**: GPIO5 (Pico pin 7) → TX of your USB‑serial adapter.
  - **GND**: common ground between Pico and adapter.
- Baud rate: **921600** (default). Some adapters only handle 500,000; both bridges accept a `--baud` flag.
- Keep logic at 3.3V; do not feed 5V UART into the Pico.

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
Edit `controller_color_config.h` (RGB values) and rebuild; the Switch will show the new colours on reconnect.

## Python bridge (recommended)
Works on macOS, Windows, Linux. Uses SDL2 + pyserial.

### Install dependencies with uv (or pip)
```sh
# from repo root
uv venv .venv
uv pip install pyserial pysdl2
```
- SDL2 runtime: install via your OS package manager (macOS: `brew install sdl2`; Windows: place `SDL2.dll` on PATH or next to the script; Linux: `sudo apt install libsdl2-2.0-0` or equivalent).

### Run
```sh
source .venv/bin/activate  # or .venv\Scripts\activate on Windows
python controller_uart_bridge.py --interactive
```
Options:
- `--map index:PORT` (repeatable) to pin controller index to serial (e.g., `--map 0:/dev/cu.usbserial-0001` or `--map 0:COM5`).
- `--ports PORTS...` or `--interactive` for auto/interactive pairing.
- `--baud 921600` (default 921600; use `500000` if your adapter can’t do 900K).
- `--frequency 1000` to send at 1 kHz.
- `--sdl-mapping path/to/gamecontrollerdb.txt` to load extra SDL mappings (defaults to `controller_db/gamecontrollerdb.txt`).

Hot‑plugging: controllers and UARTs can be plugged/unplugged while running; the bridge will auto reconnect when possible.

### macOS tips
- Ensure the USB‑serial adapter shows up (use `/dev/cu.usb*` for TX).
- Some controllers’ Guide/Home buttons are intercepted by macOS; using XInput/DInput mode or disabling Steam’s controller handling helps.

### Windows tips
- Use `COMx` for ports (e.g., `COM5`). Auto‑detect lists COM ports.
- Ensure SDL2.dll is on PATH or alongside the script.

### Linux tips
- You may need udev permissions for `/dev/ttyUSB*`/`/dev/ttyACM*` (add user to `dialout`/`uucp` or use `udev` rules).

## Rust bridge (optional)
Note: Rust toolchain not currently present in this setup. If you install Rust + SDL2 dev libraries, an experimental Rust bridge lives in `fast_uart_bridge/`.

## Troubleshooting
- **No input on Switch**: verify UART wiring (Pico GPIO4/5), baud matches both sides, Pico flashed with current firmware, Switch sees a “Pro Controller”.
- **Constant buzzing rumble**: the bridge filters small rumble payloads; ensure baud isn’t dropping bytes. Try lowering rumble scale in `controller_uart_bridge.py` if needed.
- **Guide/Home triggers system menu (macOS)**: try different controller mode (XInput/DInput), disable Steam overlay/controller support, or connect wired.
- **SDL can’t see controller**: load `controller_db/gamecontrollerdb.txt` (default), add your own mapping, or try a different mode on the pad (e.g., XInput).
