# DualSense native HD rumble and transport experiment

## Goal and evidence

Native Nintendo HD-rumble decoding is connected to the proven Bluetooth PCM transport through opt-in gameplay mode. A dedicated HD image auto-arms one DualSense in slot 0. Normal builds retain compatibility rumble, and the deterministic transport fixture remains available. Console gameplay was user-tested; precise actuator-onset latency and perceptual equivalence to Nintendo hardware are not claimed.

The user observed 1–2 seconds of gameplay-to-haptics delay in OMP session `01a06fa9-cdc7-72de-ac0e-7de08c355f06`. Both a DS5Dongle-style 0x39 stream and a short 0x32 stream failed after continuous silence, latest-state replacement and can-send callbacks were tried. Do not repeat those changes as newly discovered fixes or attribute the observed delay to profile feedback.

Recovered source and retained object identify three concrete issues:

- `request_can_send()` armed `send_requested` after `l2cap_request_can_send_now_event()`. This BTstack can deliver the event synchronously; the callback discarded it and left a pending flag with no notification. An isolated reproduction produced zero sends / pending=true, versus one send / pending=false when armed before requesting.
- The final short report used haptic descriptor 0xd2. SAxense uses 0x92 for its single 64-byte block. The extra bit is undocumented; the mismatch is established, its contribution to the physical delay is not.
- Relative Pico BTstack timers add one millisecond tick. Rescheduling `{11,11,10}` relative to each callback does not implement an exact 3 kHz clock and accumulates callback lateness.

The original diagnostic captures were not from a live native-haptic stream. CPU saturation, ACL-credit starvation, and controller-internal latency were not measured.

Sources:

- https://github.com/egormanga/SAxense/blob/master/SAxense.c
- https://github.com/awalol/DS5Dongle/blob/master/src/audio.cpp
- https://github.com/awalol/DS5Dongle/blob/master/src/bt.cpp
- https://github.com/awalol/DS5Dongle/blob/master/CMakeLists.txt
- Local SDK `lib/btstack/src/l2cap.c`, `src/rp2_common/pico_btstack/btstack_run_loop_async_context.c`.
- Frequency reference: https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/rumble_data_table.md
- Reconstructed substep reference: https://github.com/HandHeldLegend/NS-LIB-HID/blob/becc24f0841bbb875da24ea622cc1ada00cb8492/docs/hd-rumble-implementation-guide.md
- Eight-millisecond playback-window reference: https://github.com/HandHeldLegend/HOJA-LIB-RP2040/blob/238f66d1c4aae87fc320d94d8abd38229e7da2d0/src/utilities/pcm.c

## Implementation contract

1. Build-only opt-in `SWITCH_PICO_HAPTICS_EXPERIMENT`; separate build directory/artifacts. Preserve wake identity, pairing storage, USB modes and ordinary firmware artifacts. Use stock clock/voltage. Experimental builds use the controller's advertised outgoing ACL capacity and one-packet receive batches with explicit rescheduling; normal builds retain the three-credit cap and sixteen-packet batches. Incoming flow control and all FIFO sizes remain unchanged.
2. One selected connected Sony DualSense/DualSense Edge, Bluetooth Classic, sufficient negotiated MTU. The fixture requires explicit start. `SWITCH_PICO_HD_RUMBLE=ON` additionally auto-arms continuous gameplay for a DualSense in slot 0; it emits silence until host commands or local confirmation arrive. Other controllers retain compatibility output.
3. 142-byte report 0x32 plus A2 transaction = 143-byte L2CAP SDU. The first report selects native mode using sized state block 0x90, length 63, with rumble-selection bits and other write flags clear; it carries one silent 64-byte haptic block (0x92). Subsequent reports use compact audio control `{0x91,3,0x62,16,counter}` and **two** 64-byte haptic blocks: descriptor 0xd2, length 64, followed by 128 sample bytes. Thus 0xd2 is valid here, unlike the original spike's single-block mismatch. Deterministic padding and Bluetooth CRC. No speaker, microphone, USB audio endpoint, Opus or resampler.
4. Steady-state 64 stereo frames per report at 3 kHz, 46.875 reports/s. Absolute microsecond/sample deadlines use integer rational arithmetic; preserve fractional time and skip obsolete packets after stalls rather than burst-replaying them. Timer wakeups account for SDK +1 tick. Can-send permission and audio deadlines are separate. Arm flags before requests and handle synchronous callbacks without recursive stream generation.
5. The deterministic fixture remains a finite 288-report / 6.144-second sequence: 48 priming intervals, four cycles of left 100 Hz / silence / right 200 Hz / silence (12 reports = 256 ms per phase), then 48 trailing-silence reports. Its peak remains 32/127. Gameplay is continuous, has no one-second priming pattern, and uses timestamped Switch commands instead. Stop restores compatibility output; disconnect cancels without stale-pointer use.
6. No historical PCM FIFO. Generate only the current due block when transmission is permitted; bounded control mailbox across cores. Record packet counts, skipped blocks, failed sends, synchronous callbacks, generation cost, send gaps, lateness, request wait and first-tone timestamps. HCI submission is not physical actuator onset.
7. Host `haptics-experiment start`, `gameplay`, `status`, `stop`, and `profile` use existing USB management framing. Ordinary builds report unsupported. General runtime diagnostics remain unchanged. The experiment response is schema 3; transport profiling remains schema 2. Update the host and experimental firmware together.
8. Regression coverage must include synchronous callback delivery, rational clock and late wakeups, reference packet interpretation, finite completion/stop, disconnect/reconnect and compatibility restoration. Native probes cannot prove controller acceptance or physical latency.

## Gameplay mode

Build using the provisioned Pico SDK/toolchain environment:

```sh
cmake -S . -B build-hd-rumble -DPICO_BOARD=pico2_w \
  -DSWITCH_PICO_INPUT_BACKEND=BLUEPAD32 \
  -DSWITCH_PICO_HD_RUMBLE=ON \
  -DSWITCH_PICO_HAPTICS_EXPERIMENT_RAM=ON -DSWITCH_PICO_LOG=OFF
cmake --build build-hd-rumble
```

Load `build-hd-rumble/switch-pico.elf` or `.uf2`. The HD option implies the experimental transport and auto-arms slot 0 on connection. On the Switch, reconnect the DualSense with PS. Manual PC arming is `uv run switch-pico-config haptics-experiment gameplay --slot 0`; it does not persist across power cycles. `stop` disarms the native stream, not ordinary compatibility rumble. The standard `build.py` commands explicitly disable both HD and experiment options.

The decoder preserves each actuator's one-to-three ordered substeps and frequency indices. Amplitudes become linear Q0.15 values via precomputed lookups; compatibility magnitudes retain their previous mapping. Profile strong/weak scales apply to the low/high bands of both actuators without discarding substeps.

The synthesizer has independent left/right low/high phase accumulators. Frequencies are `40 * 2^(index/32)` and `80 * 2^(index/32)` Hz. Each command occupies an 8 ms window, split into 24/12/8 PCM samples per substep for counts 1/2/3 at 3 kHz. New reports supersede unplayed old substeps; identical compressed words hold final state rather than replaying deltas. Each side expires 50 ms after its last update, matching the existing conservative Switch-rumble timeout policy.

One report interval (21.333 ms) of causal lookback preserves commands received between Bluetooth sends without predicting future input. Fixed 16-entry cross-core and synthesis command histories contain decoded states, not PCM. Overflow is counted; stale sample intervals are skipped, not replayed as a backlog. `host_updates` and `dropped_updates` expose command ingestion and loss.

Native gameplay gain is **1.5x after profile scaling**, following console feedback that the initial gain was weak. When the requested combined band weights exceed output headroom, both are reduced proportionally. This retains band balance and bounds samples to signed PCM range without clipping waveform peaks. Zero profile gains remain zero. Local profile confirmations retain their previous strength and temporarily override, rather than erase, the current host timeline.

The gameplay stream continues with silence while idle. It is stopped on disconnect, explicit stop, or a stalled send-permission watchdog; it yields to compatibility behavior in XInput mode. Existing LED feedback can drain without switching the controller out of native haptics. Continuous idle streaming trades power for avoiding repeated audio-mode startup.

## Building and running

Use the repository's provisioned Pico SDK/toolchain environment. These CMake commands only build; they do not flash the adapter:

```sh
cmake -S . -B build-haptics -DPICO_BOARD=pico2_w \
  -DSWITCH_PICO_INPUT_BACKEND=BLUEPAD32 \
  -DSWITCH_PICO_HAPTICS_EXPERIMENT=ON \
  -DSWITCH_PICO_HAPTICS_EXPERIMENT_RAM=ON \
  -DSWITCH_PICO_LOG=OFF
cmake --build build-haptics
```

The experimental artifacts are `build-haptics/switch-pico.elf` and `.uf2`. For the same-clock flash comparison, use another build directory and `-DSWITCH_PICO_HAPTICS_EXPERIMENT_RAM=OFF`. The standard `build.py` entry points explicitly disable the experiment, including when reusing an old CMake cache.

After loading the chosen image and connecting a DualSense:

```sh
uv run switch-pico-config haptics-experiment status
uv run switch-pico-config haptics-experiment start --slot 0 --watch --json
uv run switch-pico-config haptics-experiment profile --json
uv run switch-pico-config haptics-experiment stop --slot 0
```

For the deterministic fixture only, the first tone is intentionally scheduled 1.024 seconds after start. Gameplay instead renders the timestamped host timeline with one report interval of lookback. First-tone fields identify the logical first nonsilent sample and the containing report's submission, not actual actuator onset.

## Protocol

USB vendor management operation 0x40: OUT two-byte payload `{action, slot}` (0=stop, 1=finite fixture, 2=continuous gameplay; slot 0..3); existing request envelope. IN is schema 3, 84 bytes. The first 72 bytes retain the previous layout; mode and gameplay counters follow.

- Seventeen little-endian u32 fields: run_id, connection_generation, start_us, generated_packets, sent_packets, skipped_packets, send_failures, can_send_requests, synchronous_callbacks, max_generate_us, max_send_gap_us, max_lateness_us, max_request_wait_us, first_tone_due_us, first_tone_sent_us, last_sent_us, elapsed_us.
- Four u8 fields: state, slot, last_error, reserved (zero).
- Byte 72: mode (0=fixture, 1=gameplay); bytes 73–75: zero reserved bytes.
- Little-endian u32 at 76: `host_updates`; at 80: `dropped_updates`.
- State: idle=0, pending=1, running=2, completed=3, stopped=4, disconnected=5, unsupported=6, error=7. Disabled build reports unsupported.
- Microsecond timestamps are low 32 bits of Pico uptime; use unsigned modular differences for this bounded experiment. Host receipt time is not a hardware onset measurement.
- Error: none=0, unsupported controller=1, insufficient MTU=2, disconnected=3, timeout=4, transport failure=5, queued conventional output=6. The fixture rejects a queued start. Gameplay allows a bounded startup interval for prior output to drain; it does not discard LED/control reports.

### Transport timing probe

Operation `0x41` is IN-only. Schema 2 contains 32 little-endian u32 fields in the declaration order of `HapticsTransportProbe` in `input/haptics_transport_probe.h` (128 bytes). The final two fields expose ACL packet size/count from the controller's raw HCI Read Buffer Size response, before the SDK's software cap. Measurements reset for each run, retain their final values, and correlate by run ID and connection generation. Ordinary firmware reports this operation as unsupported.

```sh
uv run switch-pico-config haptics-experiment profile --json
```

The probe measures scheduled timer lateness before choosing a current packet, permission wait before generating/sending, full synchronous `l2cap_send` duration, CYW43 write/read duration, data-source polling duration/gaps, selected-handle Number Of Completed Packets events, and observed ACL credit extrema. The first successful tone's send-return timestamp complements the original pre-send timestamp. Neither is radio transmission or actuator onset.

Timing totals are inclusive and overlap: a data-source poll can dispatch a completion callback that sends a packet, and a synchronous send can call the CYW43 writer. Do not add those totals as independent CPU costs. Read/write/poll measurements cover all controller traffic during the selected run; completion counts and outstanding packets are filtered to the selected connection. Free ACL slots come from the shared controller pool. Extrema are observations, not a complete occupancy trace.

The instrumentation stores bounded counters in memory, not per-packet UART logs. GNU linker wrappers preserve each transport call's arguments, return values and call count. After a bounded poll returns nonempty input, the wrapper marks future receive work pending rather than draining recursively; this fairness behavior runs even while measurement is inactive. Verify wrapper call sites in the actual ELF and nonzero live counters; native tests alone cannot establish that a differently optimized SDK build retained the boundaries.

## SRAM relocation (not RAM replacement)

DS5Dongle executes selected hot code from SRAM instead of external XIP flash. Its full audio build also relocates roughly 220 KB of Opus code/data; that does not belong in this haptics-only experiment.

Start by placing this experiment's packet synthesis/send callback and small waveform constants in SRAM using the Pico SDK's time-critical sections; verify symbol placement and SRAM cost in the linked ELF. Do not claim that this alone fixes transport latency. Keep a build switch for flash-versus-SRAM comparison without changing the protocol or clock.

Only after timing evidence, consider selected L2CAP/HCI/CYW43 and USB hot call chains. Relocating only a wrapper leaves callees in flash. Broad object-section rewriting is SDK/compiler sensitive, consumes SRAM needed by stacks and buffers, and must be checked in the map. Do not wholesale replace memcpy/memset or move the entire stack without measurements. Shared flash-safe operations and BOOTSEL sampling remain legitimate jitter sources to measure, not disable unsafely.

## Optional overclocking

Overclocking is permitted, but the first transport run stays at the Pico 2 W's stock 150 MHz and stock regulator setting. Compare flash versus SRAM at the same clock first; then compare clocks with identical packet contents and scheduling. An overclock cannot repair a malformed block descriptor or a lost synchronous callback, and it does not increase Bluetooth's negotiated air rate or controller ACL credits.

The [ClockworkPi thread](https://forum.clockworkpi.com/t/overclocking-pico-2/18226) reports successful 300 MHz and higher configurations, alongside warnings about peripheral limits. [Pimoroni's measurements](https://learn.pimoroni.com/article/overclocking-the-pico-2) found 312 MHz at 1.1 V on one sample under their initial benchmark. Those are experimental observations, not a stability guarantee for this board running both cores, USB and CYW43 simultaneously. Do not copy the article's extreme voltages, voltage-limit removal, or dry-ice setup.

For a subsequent opt-in clock experiment, record requested/measured system clock, regulator voltage, flash divider, CYW43 PIO divider, temperature/environment, and the same packet diagnostics. Preserve the 48 MHz USB clock; adjust flash and CYW43 dividers before raising the system clock so neither bus is inadvertently overclocked. Validate USB enumeration/control transfers, sustained controller input, reconnect, flash-safe persistence and packet timing—not just a CPU benchmark. Keep a stock UF2 and BOOTSEL recovery path. No clock or voltage change is applied by this transport experiment.

## Verification and acceptance

Build normal and opt-in firmware, run focused regressions, verify time-critical symbols and retained wake configuration. Exercise the actual USB CLI and connected controller. Capture live counters during the finite run. The current two-block format requires 46.875 reports/s and nominal HID+A2 traffic of 6,703.125 bytes/s, before L2CAP/HCI/radio overhead. Acceptance is 288/288 submissions with zero skipped slots and failures, repeated runs, bounded stop and continuing controller input.

Physical acceptance requires correlating first-tone scheduling/sending with actuator onset using an accelerometer/contact microphone or a synchronized observation. Record controller model/firmware, packet gaps and CPU generation time. A successful `l2cap_send` is not an acknowledgement of playback. Only a measured low-latency result permits integration with Nintendo's per-side, per-band timeline.

## Results

### Initial single-block baseline (superseded)

- Normal all-in-one, experimental SRAM, and experimental flash variants built successfully for Pico 2 W / RP2350 Arm, using the provisioned Pico SDK and GNU Arm 15.2.1 toolchain.
- 99 focused host-control, USB-management, build-helper and Bluepad32-preparation tests passed. Both native sender variants passed, including independent zlib validation of all 576 generated reports.
- The first hardware start exposed an additional integration defect: the Classic path leaves Bluepad32's cached `conn.protocol` unset. Eligibility now uses `gap_get_connection_type(handle)`, matching the existing backend identity code. Modeling the real unset field made the native test fail before this correction and pass afterward; the corrected hardware accepted the run.
- The initial SRAM image put the callback at `0x20000174` and waveform at `0x20000910`, with 2,024 bytes extra SRAM versus its flash counterpart. Final-image placement is recorded below.
- At this initial stage no wake capture, pairing reset, clock, voltage, FIFO-size or HCI-credit-limit change was performed. Wake transmission itself was not exercised.

### Initial connected-controller run

The SRAM variant was flashed and exercised over the actual USB management endpoint with a reconnected Sony DualSense (`054c:0ce6`) in slot 0, using the configured stock 150 MHz clock and unchanged regulator settings. Controller firmware revision and physical actuator onset were not measured.

```sh
uv run switch-pico-config --timeout 15 haptics-experiment start --slot 0 --watch --json
```

The host captured 63 snapshots during run 1:

| Measurement | Result |
| --- | ---: |
| Scheduled packet slots | 576 |
| Successful PCM submissions | 113 |
| Obsolete packet slots skipped | 463 |
| Send failures | 0 |
| CAN_SEND_NOW requests | 113 |
| Synchronous callbacks | 58 |
| Maximum packet generation time | 55 us |
| Maximum permission wait | 72,231 us |
| Maximum submission gap | 108,791 us |
| First tone submission after scheduled first tone | 15,618 us |
| Terminal elapsed time, including restoration | 6,264,335 us |

The finite lifecycle reached `completed`, but **the PCM cadence acceptance criterion failed**: 113 submissions over a 6.144-second pattern window is about 18.4 packets/s, versus the required 93.75. `completed` means the finite run and compatibility restoration finished, not that every packet was delivered or low-latency playback was proved. There was no historical PCM queue replay; old sample slots were discarded.

Run 2 exercised early stop: firmware-confirmed `stopped`, 37 submissions, 142 skipped slots, zero send failures. Controller input remained live afterward. Configuration generation 9 / CRC `b740995b` matched the pre-flash baseline.

This established that synthesis was inexpensive but did not separate synchronous I/O, receive-loop work and credit wait. The follow-on measurements below did so. The flash-resident comparison was built, not flashed or timed; no SRAM latency improvement was established by this baseline.

### Transport diagnosis and corrections

1. **Instrumented sixteen-packet baseline:** 106/576 submissions, 470 skipped slots. Every poll consumed all 16 reads: 2,624 reads across 164 polls. Maximum poll duration 37,834 us; timer lateness 38,544 us. In contrast, synchronous send peaked at 1,429 us and CYW43 write at 1,090 us. Nested timing totals must not be summed.
2. **Receive fairness:** cap each experimental poll at one packet and explicitly reschedule further work after nonempty reads. Maximum timer lateness fell to 4,126 us. However, the three-slot outgoing cap still filled and permission waits reached 127,578 us; this alone did not sustain PCM.
3. **Negotiated credits:** capture the controller's raw initialization response: **8 ACL packets of 1,021 bytes**. Remove only the experimental software clamp, allowing BTstack to use the advertised capacity. Keep controller-to-host flow control enabled. The single-block sender improved to 356/576 submissions but still skipped 220 slots.
4. **Native mode handoff:** the user felt no vibration. Compatibility restoration had explicitly selected rumble; audio reports did not clear that selection. Sending the sized 0x10 native-state block first produced user-confirmed vibration, albeit weak and still interrupted.
5. **Compact two-block format:** [DS5Dongle's audio implementation](https://github.com/awalol/DS5Dongle/blob/master/src/audio.cpp) documents that audio controls can be reduced to a buffer-length value and packet counter. With microphone streaming disabled, mask 0x62 and length 3 leave room for two 64-byte blocks in the same 143-byte SDU. Buffer length 16 is the reference configuration's minimum. The counter advances by two. This halves required report cadence without reducing the 3 kHz sample rate or changing the audible pattern.

### Final live acceptance

Four consecutive six-second runs on the connected DualSense completed with **288/288 reports, zero skipped slots and zero send failures in every run**. Across those runs:

| Measurement | Worst observed value |
| --- | ---: |
| Packet generation | 57 us |
| Send-permission wait | 191 us |
| Report gap (nominal 21,333.33 us) | 24,081 us |
| Submission lateness | 2,988 us |
| First tone send-return lateness | 3,095 us |

The user confirmed **alternating pulses** and described them as still a bit weak. Peak 32/127 is intentional; it remained unchanged across packing comparisons. This establishes native haptic acceptance and recognizable channel alternation, not a precise physical latency measurement.

Early-stop run 5 reached `stopped` with 72 submissions, zero skipped slots and zero failures. The host observed stop confirmation in approximately 111 ms, including USB querying/restoration. During that check 605 controller reports arrived, and the live input state advanced by 565 generations while remaining connected. Button-to-Switch latency and actuator-stop latency were not measured.

Final verification: **122 focused tests passed**; normal, experimental SRAM and experimental flash images built. The SRAM callback is at `0x20000174`, waveform at `0x20000920`; the flash variant places them at `0x10014624` and `0x10068534`. SRAM relocation adds **2,056 bytes** versus the final flash variant. Real SDK call sites were verified to enter the transport wrappers.

Configuration remains generation 9 / CRC `b740995b`; wake configuration remains included. Clock stays at the configured stock 150 MHz, voltage unchanged. No FIFO enlargement, incoming flow-control removal, speaker/microphone stream or broad stack relocation was needed.

### Gameplay integration verification

The initial gameplay image passed 513 real Switch-format USB OUT reports through the decoder, profile scaling, cross-core history and native sender: 513 observed host updates, zero dropped updates, zero Bluetooth skips and zero send failures. A stronger-command repeat also passed; the user confirmed correct alternating left/right effects. A simultaneous local-profile identification test retained all 513 updates, dispatched local confirmation, and stayed in native gameplay with no skips or failures.

The user then tested an actual Switch game and reported that it **worked well but needed more strength**. Gameplay gain was increased to 1.5x with joint headroom limiting, then built, flashed and exercised again: all 513 commands arrived, 200 Bluetooth reports were submitted during the test, zero command drops/skips/send failures, and 1,443 controller input reports continued. Maximum observed packet-generation time was 680 us, permission wait 292 us, and report gap 24,343 us. These are firmware/transport measurements, not a physical latency bound.

Stop and re-arm were also exercised: stop confirmation was observed in about 19.5 ms; the new run resumed continuous silence and controller input. That re-arm recorded one skipped silent startup slot, with no host commands or send failures; the active USB-driven tests above had no skipped slots.

Final focused verification: **148 tests passed**, covering decoder fidelity, frequency/phase behavior, substeps, gain/headroom, watchdogs, overflow, startup/stop, profile gain/feedback, host controls, existing backend lifecycle, UART and build helpers. HD gameplay, normal all-in-one, deterministic experiment, and Pico/UART firmware builds succeeded. Configuration remained generation 9 / CRC `b740995b`; wake identity, clock and voltage were not changed.

The stronger HD gameplay image remains loaded and armed. This is a translation to DualSense actuators, not a promise of identical Nintendo force response. Physical onset still needs synchronized measurement. The user subsequently reported slight IMU aiming lag; that is being investigated separately from this completed haptics integration.
