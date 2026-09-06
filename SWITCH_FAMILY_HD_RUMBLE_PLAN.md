# Switch-family native HD-rumble implementation handoff

Status: **planned, not implemented**. This document is self-contained for another coding agent. The shorter project-roadmap version is in `ADAPTER_PARITY_PLAN.md` under “Native Switch-family HD rumble — Planned”.

## Goal

Deliver native Nintendo rumble commands to qualified Switch-family Bluetooth controllers without collapsing left/right and low/high-band information to conventional motor magnitudes. This is a separate backend from DualSense PCM synthesis.

Start with a genuine original Switch Pro Controller, then original standalone Joy-Con L/R. Keep third-party Switch-mode controllers on their existing tested fallback until individually qualified. Do not implement Joy-Con logical pairing or assume Switch 2/NSO controllers share the same capabilities.

## Current project state and coordination

- Branch at handoff: `feature/controller-profiles`.
- `44a474e`: roadmap plan for native Switch-family HD rumble.
- `3c2d9fa`: Set B profiles, catalog migration, recording, optimized transport and native DualSense/XInput work.
- Other transport qualification/artifact work may still be in progress. Coordinate with the active agent before editing shared files; do not reset, stash, or overwrite its changes. Re-read current code rather than relying on line numbers here.
- Standard AIO/XInput builds now use 300 MHz/1.3 V and the optimized CYW43 transport. The first eligible DualSense may own one native PCM stream, regardless of slot. Other models retain their controller-specific rumble path.
- Profiles are schema 6 / 384 bytes; catalog 2 keeps a 512-byte record stride and two 128 KiB arenas. Preserve migration, identity keys, names, active indices and atomic publication.
- Preserve Bluetooth bonds, calibration, the UART wire protocol and the private `src/firmware/platform/pico/switch2_wake_config.h`. Do not expose that file's contents or change the configured wake identity.

### Important timing qualification caveat

Do not generalize single-controller DualSense results to mixed-controller loads. A 32-frame/93.75-packet-per-second run passed roughly 65 seconds with one controller, but a later Switch Pro + DualSense test with continuous USB motion reads recorded **80 skipped audio slots over 16.6 seconds**, despite receiving all 2,050 USB commands with no command drops or send failures. Maximum permission wait was 17,180 us and the eight outgoing ACL credits were observed exhausted. The standard native cadence is consequently **64 frames / 46.875 packets per second** at the same 300 MHz/1.3 V, with 32 frames an explicit experiment. Preserve the current cadence choice and coordinate before changing it as part of this Nintendo backend task.

The Nintendo path should not need PCM packets at all. Its small native commands have a different bandwidth budget, which still needs real multi-controller measurement.

## Read these code paths first

| Area | Files / symbols | Relevant facts |
|---|---|---|
| Switch host decoder | `src/firmware/usb/switch/switch_haptics.h/.cpp`, `SwitchHapticsDecoder`, `ControllerRumbleOutput`, `SwitchHapticsFrame` | Two sides, each with up to three decoded low/high frequency/amplitude substeps. Original eight wire bytes are not retained in the output envelope. |
| Intensity scaling | `src/firmware/profile/controller_profile_transform.cpp`, `controller_profile_scale_host_rumble` | Strong scales low band and weak scales high band on both sides. Q15 amplitudes are decoder-normalized values, not raw Nintendo amplitude codes. |
| Routing and lifetime | `src/firmware/input/bluepad32_input_backend.cpp` | Generation-tagged slots, compatibility mailbox, native submission, local/profile feedback, controller ready/disconnect events. Preserve slot isolation. |
| Existing Nintendo output | `patches/bluepad32-sdl3-imu.patch`; generated `build-aio/_deps/bluepad32-src/src/components/bluepad32/parser/uni_hid_parser_switch.c` | Modify the patch/build-local copy, not the upstream checkout or arbitrary SDK files. |
| Parser functions | `send_subcmd`, `switch_encode_rumble`, `switch_send_dual_rumble_now`, `switch_stop_rumble_now`, `set_led`, `fsm_enable_rumble` | Existing conventional path enables vibration with 0x48, uses fixed frequencies and a 40 ms refresh. |
| Transport | `src/firmware/input/haptics_transport_probe.*`, `src/firmware/platform/pico/cyw43_packet_transport.c`, `patches/btstack-credit-batch.patch` | Bounded receive fairness, packet-level reads, real per-handle credit accounting. Do not remove incoming flow control or invent extra controller credits. |
| Prior evidence | `HAPTICS_EXPERIMENT.md` | Distinguishes measured submission timing from physical actuator onset and preserves the accepted controlled-effect reference. |

### Existing parser issues the new owner must resolve

1. `send_subcmd()` uses a process-global four-bit packet counter. Move sequence state to each physical parser/device instance and share it across that device's rumble and subcommand reports.
2. Player-LED requests are built with zeroed rumble fields. All applicable subcommands must carry the current effective rumble state while native rumble is active.
3. The parser has duration, delayed-start and refresh timers. A second independent native writer cannot safely coexist with those timers or their stale callbacks.
4. `switch_encode_rumble()` currently takes one amplitude for both bands of an actuator. It is not a complete encoder for independent low/high amplitudes or all compressed multi-substep forms.
5. Bluepad32 explicitly treats Joy-Cons as separate, horizontally mapped controllers. There is no existing two-Joy-Con logical pair to route stereo output into.

## Model policy

- Initial target: original genuine Pro Controller `057E:2009`.
- Next: original Joy-Con L `057E:2006`, Joy-Con R `057E:2007`.
- A Pro-like name, VID/PID or parser type is not sufficient proof of full native compatibility; clones can present the same identity. Use qualified model/firmware evidence and conservative handling of ambiguous devices.
- Preserve the 8BitDo Ultimate's tested enable/fixed-frequency/refresh behavior until that exact model passes native qualification.
- Do not send DualSense 0x32 PCM packets to Nintendo devices.
- Do not apply the DualSense 2x/0.8-power response curve to Nintendo actuators.
- Switch 2 controllers and NSO/retro models require separate protocol and actuator qualification. Some Nintendo-family devices have no HD-rumble actuators.

## Implementation sequence

### 1. Establish protocol truth with a genuine Pro Controller

Capture real console USB rumble words and Bluetooth reports for:

- neutral and explicit stop;
- each actuator independently;
- low and high bands independently and together;
- repeated words, absolute and relative commands;
- all observed one/two/three-substep forms;
- frequency sweeps and safe amplitude changes.

Validate accepted Bluetooth encodings, byte order, amplitude normalization and repeated-word behavior. The older public fixed-state tables do not prove every compressed command's behavior. Use independent golden data and real controller acceptance, not only `decode(encode(x))` against the same implementation.

### 2. Preserve command information and add bounded delivery

Extend the existing rumble envelope with original eight-byte data and explicit validity/unmodified state while retaining the decoded, profile-scaled timeline. Conventional XInput/UART/local feedback must not accidentally acquire valid raw Nintendo bytes.

Use a fixed-capacity, generation-tagged Core-0-to-Core-1 command queue. Add one native output owner per physical Switch device, integrated with the existing parser's sender. No heap allocation, PCM FIFO or unbounded catch-up loop.

Nintendo rumble-only Bluetooth output is report `0x10`: transaction byte, report ID, four-bit counter and eight rumble bytes, **11 bytes before L2CAP/HCI/radio overhead**. At 125 reports/s that is about 1,375 payload bytes/s. If the original command can be sent directly, it need not inherit DualSense's PCM lookback. Measure actual queue/submission delay before making latency claims.

### 3. Implement scaling and state recovery, not just unity passthrough

Raw forwarding is a fast path only when profile gain is unity and the physical controller's command state is synchronized. For changed gain, encode the scaled low/high-band timeline using verified Nintendo encodings and safe amplitude limits.

- Preserve exact silence and valid unity behavior.
- Quantize against the supported Nintendo amplitude/frequency representation; do not treat normalized Q15 amplitudes as wire codes.
- Retain all representable substeps.
- If scaling makes a multi-step sequence impossible to encode in one word, explicitly qualify a bounded legal packet schedule or documented quantization policy. Do not silently replace it with peak/latest magnitude output.
- After overflow, dropped history, feedback or reconnect, establish a legal absolute current-state/neutral baseline before forwarding commands that depend on prior state.
- Never replay an obsolete vibration backlog to catch up.

### 4. Centralize LEDs, local feedback, stop and disconnect

The same per-device owner must serialize rumble-only reports and subcommands. Piggyback current effective rumble into applicable `0x01` reports rather than interrupting an effect with default/zero fields.

Local confirmation temporarily overrides host output, while the host timeline keeps advancing. After confirmation, resume the current host state, not an expired effect.

Preserve the existing 50 ms Switch-command expiry policy. Prioritize explicit stop. Cancel all parser rumble timers when transferring ownership or disconnecting; callbacks and queued commands from an old generation must never touch a replacement device. Preserve ordinary setup/calibration and subcommand replies.

For XInput input, use two-magnitude stateful effects with fixed native carriers and explicit zero/host-lifetime stop. XInput does not contain Nintendo frequency or substep detail.

### 5. Add explicit single-actuator and third-party behavior

A standalone Joy-Con has one actuator but is currently exposed as a standalone controller. Do not simply discard whichever host side is absent. Define a deterministic mono downmix preserving each band's dominant contribution, with a documented frequency/tie rule and safe amplitude limits. Spatial fidelity is necessarily lost.

Routing a stereo left/right pair belongs with a separate logical Joy-Con pairing feature; do not bundle that input-topology change into this task.

Third-party models remain on their proven compatibility policy unless their native behavior is independently qualified. Retain useful existing rumble rather than broadly enabling an unverified protocol.

### 6. Qualify and enable per model

Permanent regression cases should defend observable behavior:

- independent golden packet/codec vectors and neutral values;
- zero, unity and intermediate profile intensity;
- band/actuator isolation and substep order;
- per-device counter wrap and multiple devices;
- LED/subcommand coexistence during effects;
- local feedback resume, timeout and explicit stop;
- queue pressure, loss/resynchronization, disconnect/reuse;
- compatibility fallback and unsupported models.

Run existing relevant suites, then the complete repository suite:

```sh
uv run pytest -q tests/test_switch_haptics_native.py \
  tests/test_controller_profile_transform_native.py \
  tests/test_bluepad32_backend_lifecycle_native.py \
  tests/test_prepare_bluepad32.py tests/test_usb_output_driver_native.py
uv run pytest -q
```

Build affected AIO, XInput and UART configurations. `build.py` flashes by default; use CMake for build-only checks and coordinate hardware access before flashing.

Hardware acceptance: safe frequency/amplitude sweeps; actual captured game effects; one versus four controllers; mixed Nintendo/DualSense traffic; simultaneous motion/input, LEDs and profile writes; reconnect and stop. Report p50/p95/p99/worst host-receipt-to-HCI submission, lost/resynchronized commands and physical actuator onset where instrumentation exists. HCI acceptance is not proof of playback.

Enable native output only for models that pass qualification. Keep the fallback available for ambiguous/unsupported devices. Do not claim full Switch-family support from testing a single Pro Controller.

## Non-goals

- Nintendo controller firmware updating or modifying calibration flash.
- Bond resets or wake-identity changes.
- USB host support, audio/headphone output, NFC/IR or adaptive triggers.
- New per-game detection, scripts or unbounded action layers.
- Re-tuning DualSense gain or general overclock experiments as a side task.
- Pretending unmeasured physical latency or cross-actuator force equivalence.

## Protocol references

- Reports, neutral values and safe amplitude notes: https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/bluetooth_hid_notes.md
- Frequency/amplitude encoding tables: https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/rumble_data_table.md
- Current implementation patches and investigation: `patches/bluepad32-sdl3-imu.patch`, `HAPTICS_EXPERIMENT.md`, `ADAPTER_PARITY_PLAN.md`.

Treat community reverse-engineering references as evidence to validate, not an official guarantee. Preserve applicable source licenses and do not copy noncommercial reference implementations into this project without resolving their licensing.
