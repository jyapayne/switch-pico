# Switch-family native HD-rumble implementation handoff

Status: **implemented; hardware qualification incomplete**. This handoff now records the implementation and remaining acceptance work. The roadmap entry is in `ADAPTER_PARITY_PLAN.md` under “Native Switch-family HD rumble — Implemented, qualification incomplete”.

## Goal

Deliver native Nintendo rumble commands to qualified Switch-family Bluetooth controllers without collapsing left/right and low/high-band information to conventional motor magnitudes. This is a separate backend from DualSense PCM synthesis.

Start with a genuine original Switch Pro Controller, then original standalone Joy-Con L/R. Keep third-party Switch-mode controllers on their existing tested fallback until individually qualified. Do not implement Joy-Con logical pairing or assume Switch 2/NSO controllers share the same capabilities.

## Current project state and coordination

- Branch at handoff: `feature/controller-profiles`.
- `44a474e`: roadmap plan for native Switch-family HD rumble.
- `3c2d9fa`: Set B profiles, catalog migration, recording, optimized transport and native DualSense/XInput work.
- Other transport qualification/artifact work may still be in progress. Coordinate with the active agent before editing shared files; do not reset, stash, or overwrite its changes. Re-read current code rather than relying on line numbers here.
- Standard AIO/XInput builds use 300 MHz/1.3 V and optimized CYW43 transport. DualSense retains its separate 64-frame PCM stream. Nintendo native output requires explicit approval of the stable physical Bluetooth identity; unapproved devices keep compatibility output.
- Profiles are schema 6 / 384 bytes; catalog 2 keeps a 512-byte record stride and two 128 KiB arenas. Preserve migration, identity keys, names, active indices and atomic publication.
- Adapter configuration is now schema 3 / 232 bytes: up to 16 physical Nintendo approvals, independent of profiles. Old schemas 1/2 migrate with no approvals and preserve their existing settings. No controller is approved merely by its name, VID/PID or parser.
- Preserve Bluetooth bonds, calibration, the UART wire protocol and the private `src/firmware/platform/pico/switch2_wake_config.h`. Do not expose that file's contents or change the configured wake identity.

### Important timing qualification caveat

Do not generalize single-controller DualSense results to mixed-controller loads. A 32-frame/93.75-packet-per-second run passed roughly 65 seconds with one controller, but a later Switch Pro + DualSense test with continuous USB motion reads recorded **80 skipped audio slots over 16.6 seconds**, despite receiving all 2,050 USB commands with no command drops or send failures. Maximum permission wait was 17,180 us and the eight outgoing ACL credits were observed exhausted. The standard native cadence is consequently **64 frames / 46.875 packets per second** at the same 300 MHz/1.3 V, with 32 frames an explicit experiment. Preserve the current cadence choice and coordinate before changing it as part of this Nintendo backend task.

Nintendo uses no PCM stream. Its small commands still contend for radio scheduling and HCI credits; payload byte rate alone did not predict the measured mixed-controller limit.

## Implemented behavior and current evidence

- `input/switch_native_output.*` owns four bounded, generation-tagged queues.
  USB publication uses an IRQ-safe BTstack wake rather than taking the radio
  async-context lock. Encoding waits for L2CAP can-send permission. A prepared
  schedule that becomes obsolete is discarded and resynchronized, not replayed.
- `usb/switch/switch_native_haptics.*` preserves independent sides/bands and all
  representable 1/2/3-substep forms. Safe synchronized unity can retain raw bytes.
  Profile changes revoke unmodified provenance. XInput parsing clears reused
  HD/raw state rather than accidentally inheriting Nintendo data.
- Safe amplitude codes stop at 100 (decoder LUT index 228 / Q15 17867).
  Q15 is not a wire amplitude. Frequencies clamp to indices 1..127. Exact
  one-packet forms are preferred, then bounded two-packet prefixes/baselines.
  Unrepresentable scaled sequences retain their time slots and choose legal
  commands minimizing `abs(Q15 error) + 128 * abs(frequency-index error)`,
  with lower-command ties and exact silence. This is documented quantization,
  **not** a promise of lossless or perceptually equivalent arbitrary scaling.
- Mono selects the dominant contribution per band before rounding, with left
  ties. Unequal side counts use the larger count and hold the shorter side's
  final sample; that temporal quantization is reported. No Joy-Con pairing.
- Parser counters and effective rumble are per physical device. Queued
  subcommands refresh rumble and LED state at actual submission. Native
  ownership cancels duration/delayed/refresh compatibility timers; detach
  retires state without writing to a dead connection.
- Local feedback overrides output without freezing the host timeline.
  Switch commands expire after 50 ms. Unchanged held states coalesce while
  extending that watchdog; active states retain a 40 ms refresh. XInput holds
  use left-low 160 Hz / right-high 320 Hz until explicit stop.
- Read-only management operation `0x43`, diagnostic schema 2, returns four
  80-byte rows. It separates received, HCI-completed, coalesced and dropped
  commands. Latency percentiles are 250-us histogram upper bounds for actual
  submissions; coalesced holds are excluded. No physical-onset claim.

Opt-in commands:

```sh
uv run switch-pico-config profiles list
uv run switch-pico-config config native-rumble approve --identity N --yes
uv run switch-pico-config config native-rumble status --json
uv run switch-pico-config config native-rumble revoke --identity N
```

`--identity N` uses the physical row from `profiles list`. The separate
`native-rumble list` command lists persisted approval indices; revocation by
`--approval N` works even after a controller leaves the profile catalog.

### Qualification checkpoint

- **Software:** 260 repository tests pass, including independent absolute
  packet vectors, scaled/relative codec cases, actual patched parser/queue
  tests, and 17 owner lifecycle/credit/coalescing scenarios. All five final
  firmware variants build; AIO, feasibility and UART artifacts are refreshed.
- **Persistent data:** all 24 profiles, active indices, aliases and names
  matched the pre-migration hardware checkpoint. Configuration migrated
  generation 9 → 10; explicit approval of the attached Pro produced 11.
  Profile schema/catalog, bonds and wake identity were not changed.
- **Pro-only radio:** genuine Pro reply firmware bytes `03 48`; a controlled
  pre-coalescing 125-Hz run completed all **1,025 commands**, with **1,025
  submitted reports**, **zero new drops**, and **zero congestion attempts**.
- **Optimized Pro hardware:** the reconnected Pro received 513 commands for
  separated left/right 160-Hz and 320-Hz pulses. Eight state changes completed,
  505 unchanged holds coalesced, and 56 reports were submitted including
  refreshes, with zero loss/congestion. The user confirmed all four pulses
  worked and stopped cleanly. Worst observed submission latency was 1,031 us.
  A subsequent 125-Hz run changed amplitude every command: all 1,025 commands
  completed as 1,025 reports, with zero coalescing, loss or congestion. The
  diagnostic maximum reached 1,576 us; neither figure is actuator onset.
- **Mixed radio before coalescing:** can-send-driven Pro isolation with
  DualSense connected completed 195 of 513 commands and dropped 318; the
  idle DualSense PCM stream skipped 30 slots. Stopping its PCM stream but
  keeping its input connection completed 276/513 and dropped 237. These
  failures must not be relabeled as successful fidelity qualification.
- **Optimized mixed holds:** concurrent USB writers delivered 2,049 commands
  per controller at approximately 125 Hz each. Pro completed 65 state changes,
  coalesced 1,984 holds and lost none; DualSense accepted all 2,049 commands
  with no audio skips/send failures. An earlier sequential-writer test ran
  only about 62 Hz per controller and must not be cited as a 125-Hz result.
- **Optimized mixed distinct updates:** changing amplitude every 8 ms on both
  controllers completed 404/1,025 Pro commands and superseded 621; DualSense
  accepted all host commands but skipped 46 audio slots. This remains a
  failed high-rate qualification, not evidence of lossless mixed HD rumble.
- **Remaining checks:** resolve mixed distinct-update saturation, then qualify
  approval revocation/resume, LEDs, scaling and stateful XInput on hardware.
  Pro approval survived firmware reboot and reconnect.
- **Unavailable evidence:** no real-console USB/BT rumble capture corpus,
  original Joy-Con L/R qualification, four-controller hardware result or
  instrumented actuator onset has been obtained. Do not infer these from
  synthetic vectors or HCI acceptance.

### Active transport experiments

Native rumble remains enabled at the user's request. A reported controller
power-off after a test is accepted as a failure observation; its cause is not
established. Pro approval was briefly revoked at configuration generation 12,
then explicitly restored at 13. Do not revoke it as part of further experiments.

Passive HCI observations found both links in master role, with Pro in sniff
mode at interval 24 (15 ms) and DualSense active. During saturation, the Pro
held four or five of the eight outgoing ACL buffers. Those eight buffers are
distinct from the three controller-to-host incoming flow-control credits.
Remote feature responses were Pro `bff8cbfecbef7b87` and DualSense
`bf3a8dfedbff7b87`; the local HCI packet-type mask was `0xcc18`.

| Experiment | Commands per controller | Pro completed / dropped | DS audio skips |
|---|---:|---:|---:|
| Distinct nonzero effects, normal policy | 1,025 | 404 / 621 | 46 |
| Distinct nonzero effects, Pro no-sniff policy, receive bound 1 | 1,025 | 882 / 143 | 2 |
| Distinct nonzero effects, Pro no-sniff policy, receive bound 2 | 1,025 | 947 / 78 | 5 |
| Changing zero-amplitude commands, normal policy, receive bound 1 | 513 | 214 / 298 | 21 |
| Changing zero-amplitude commands, Pro no-sniff policy, receive bound 1 | 513 | 491 / 21 | 1 |
| Changing zero-amplitude commands, Pro no-sniff policy, receive bound 4 | 1,025 | 988 / 36 | 4 |

The zero-amplitude runs each coalesced one command; the other commands
exercise the 11-byte native transport without driving actuators. They do not
prove nonzero-effect fidelity. Exit-sniff requests alone failed to keep the
Pro active; that unproven automatic-wake code has been removed. The no-sniff
policy is an experimental per-link setting, not a qualified production default.

The current diagnostic build supports **same-boot** receive bounds 1/2/4 and
incoming-credit thresholds 2/3, retaining the existing credit timer and three
advertised receive buffers. Runtime controls were exercised and returned to
baseline receive bound 1 / credit threshold 2; the radio workload matrix is
awaiting normal Home/PS reconnects after flashing. Native approval remains on.

Temporary `build-aio/link_probe.cpp`, `link_probe_config.h`,
`link_probe_credit.patch`, and `SWITCH_PICO_LINK_PROBE` CMake/management hooks
are **uncommitted experiment code**, not packaged release firmware.
Operation `0x44`, diagnostic schema 2, returns 468 bytes containing link
snapshots, selected HCI command/mode events, disconnect reasons, and control
completion state. OUT payload `<HH>` supports:

- connected Pro handle + policy 1 (role-switch only), 5 (normal role-switch
  plus sniff), or `0xffff` (readback);
- handle `0xffff` + value 2/3 to set incoming-credit batching threshold;
- handle `0xffff` + value `0x101`/`0x102`/`0x104` to set receive bound 1/2/4.

No link policy changes automatically. The prepared comparison matrix uses
active Pro policy, tests `(receive, credit)` pairs `(1,2), (2,2), (4,2),
(4,3), (2,3), (1,3), (1,2)`, and restores baseline controls afterward.
Keep approvals/bonds/profiles unchanged. Remove diagnostic hooks and restore
or qualify settings before publishing another production build.

Linux's current Nintendo driver also documents disconnect risk from excessive
output traffic and uses input-report-aware throttling. This is corroborating
timing evidence, not code incorporated into this project:
https://github.com/torvalds/linux/blob/master/drivers/hid/hid-nintendo.c

## Read these code paths first

| Area | Files / symbols | Relevant facts |
|---|---|---|
| Switch host decoder | `src/firmware/usb/switch/switch_haptics.h/.cpp`, `SwitchHapticsDecoder`, `ControllerRumbleOutput`, `SwitchHapticsFrame` | Two sides with up to three substeps; original eight bytes now have explicit validity/unmodified provenance. |
| Intensity scaling | `src/firmware/profile/controller_profile_transform.cpp`, `controller_profile_scale_host_rumble` | Strong scales low band and weak scales high band on both sides. Q15 amplitudes are decoder-normalized values, not raw Nintendo amplitude codes. |
| Routing and lifetime | `src/firmware/input/bluepad32_input_backend.cpp` | Generation-tagged slots, compatibility mailbox, native submission, local/profile feedback, controller ready/disconnect events. Preserve slot isolation. |
| Native Nintendo owner/encoder | `src/firmware/input/switch_native_output.*`, `src/firmware/usb/switch/switch_native_haptics.*` | Per-physical opt-in, bounded queues, can-send-driven serialization, safe encoding and coalescing. |
| Existing Nintendo output | `patches/bluepad32-sdl3-imu.patch`; generated `build-aio/_deps/bluepad32-src/src/components/bluepad32/parser/uni_hid_parser_switch.c` | Modify the patch/build-local copy, not the upstream checkout or arbitrary SDK files. |
| Parser functions | `send_subcmd`, `switch_encode_rumble`, `switch_send_dual_rumble_now`, `switch_stop_rumble_now`, `set_led`, `fsm_enable_rumble` | Existing conventional path enables vibration with 0x48, uses fixed frequencies and a 40 ms refresh. |
| Transport | `src/firmware/input/haptics_transport_probe.*`, `src/firmware/platform/pico/cyw43_packet_transport.c`, `patches/btstack-credit-batch.patch` | Bounded receive fairness, packet-level reads, real per-handle credit accounting. Do not remove incoming flow control or invent extra controller credits. |
| Prior evidence | `HAPTICS_EXPERIMENT.md` | Distinguishes measured submission timing from physical actuator onset and preserves the accepted controlled-effect reference. |

### Parser ownership decisions

1. Sequence state is per physical parser instance, shared by rumble/subcommands.
2. Applicable subcommands carry the exact last-successful rumble bytes at send.
3. Ownership transitions cancel compatibility timers and retire queued rumble.
4. `switch_encode_rumble()` remains the conventional fixed-frequency fallback;
   independent native bands and compressed substeps use the new C++ encoder.
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
