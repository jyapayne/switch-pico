# DualSense native HD rumble and transport experiment

## Goal and evidence

Native Nintendo HD-rumble decoding and stateful XInput motor strengths feed the DualSense Bluetooth PCM backend. Standard AIO/XInput builds enable the qualified 300 MHz/1.3 V transport by default and auto-arm the first eligible DualSense that becomes ready, in any slot. One native stream is selected at a time; other controllers retain compatibility rumble. The deterministic transport fixture remains available. Console gameplay was user-tested; precise actuator-onset latency and perceptual equivalence to Nintendo hardware are not claimed.

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

## Current accepted native format

The current default is **32 stereo frames at 3 kHz**, with explicit state-only
audio initialization and the full SAxense-style control header described below.
Isolated hardware capture verified independent left/right PCM peaks of 32, 63
and 96, with the opposite channel zero and no compatibility-selector reports
during the tones. The user confirmed strong, distinct sides and clean stops at
peak 96/127. The gain curve was not increased to obtain this result.

The compact 64-frame/buffer-16 candidate felt worse despite zero skipped packets.
It is retained only as an explicit, physically unqualified experiment. That
comparison changed the control header and buffer field as well as frame count;
it does not establish that batching alone caused the difference. The accepted
32-frame trial had ten skipped slots across its strong-pulse run and no send
failures, so mixed-radio and long-duration qualification remain outstanding.
Earlier results below are historical transport measurements, not approval of
the current or rejected formats' physical fidelity.

## Implementation contract

1. AIO/XInput defaults enable `SWITCH_PICO_HAPTICS_EXPERIMENT`, `SWITCH_PICO_HD_RUMBLE`, packet-level CYW43 reads and bounded HCI credit batching at 300 MHz/1.3 V. UART is unchanged. Preserve wake identity, pairing storage and USB modes. Incoming flow control and FIFO capacities remain unchanged; the controller's advertised outgoing capacity is eight ACL packets on this hardware.
2. One selected Sony DualSense/DualSense Edge, Bluetooth Classic, sufficient negotiated MTU. Auto-arm chooses the first eligible ready controller, not necessarily slot 0, and later controllers do not steal an active stream. The fixture requires explicit start. Idle native output remains silent. Other devices use compatibility output unless explicitly approved for the separate Nintendo-native backend described in `SWITCH_FAMILY_HD_RUMBLE_PLAN.md`.
3. Report 0x32 plus A2 remains a 143-byte L2CAP SDU. The first report is state-only: sequence/tag byte 0x10, sized state block 0x90/63, and valid flag0 0x80 to write AudioControl with default route/MicSelect. Other state validity flags stay clear: no volume, preamp, mute, trigger or LED change. It carries no PCM. Default subsequent controls are `{0x91,7,0xfe,0,0,0,0,0xff,counter}`, followed by `{0x92,64}` and one 64-byte PCM block (32 stereo frames). The data counter begins at zero after initialization and advances by one. The 0xff field is a reference parameter, not an established millisecond duration. Explicit 64-frame mode retains compact controls `{0x91,3,0x62,16,counter}`, two blocks under 0xd2, and a counter advancing by two; it is not the accepted default. Padding and Bluetooth CRC remain deterministic. No speaker/microphone stream, USB audio endpoint, Opus or resampler is added.
4. At 3 kHz, 32/64 stereo frames require 93.75/46.875 reports/s. Absolute rational deadlines preserve fractional time and skip obsolete packets after stalls rather than burst-replaying them. Timer wakeups account for SDK +1 tick. Can-send permission and audio deadlines remain separate; flags are armed before requests and synchronous callbacks cannot recursively generate a stream.
5. The default deterministic fixture is 576 reports over 6.144 seconds: 96 priming slots, four cycles of left 100 Hz / silence / right 200 Hz / silence (24 reports = 256 ms per phase), then 96 trailing-silence reports. The state-only initialization occupies the first priming slot and counts as one report, with zero PCM frames. Explicit 64-frame mode preserves the same timeline with 288 total reports, 48 priming/trailing slots and 12 reports per phase. Peak remains 32/127, not full-strength rumble or a calibrated physical-force percentage. Gameplay has no one-second priming pattern and uses timestamped Switch commands. Stop restores compatibility output; disconnect cancels without stale-pointer use.
6. No historical PCM FIFO. Generate only the current due block when transmission is permitted; bounded control mailbox across cores. Record packet counts, skipped blocks, failed sends, synchronous callbacks, generation cost, send gaps, lateness, request wait and first-tone timestamps. HCI submission is not physical actuator onset.
7. Host `haptics-experiment start`, `gameplay`, `status`, `stop`, and `profile` retain USB management framing. AIO builds enable these operations; explicitly disabled/UART builds do not. Operation 0x40 uses schema 5 and transport profiling uses schema 3. Both fixture and gameplay diagnostics report the actual configured frame count; host metadata derives packet counts and timing from it. Update firmware and host tools together.
8. Regression coverage must include synchronous callback delivery, rational clock and late wakeups, reference packet interpretation, finite completion/stop, disconnect/reconnect and compatibility restoration. Native probes cannot prove controller acceptance or physical latency.

## Gameplay mode

Build using the provisioned Pico SDK/toolchain environment:

```sh
cmake -S . -B build-hd-rumble -DPICO_BOARD=pico2_w \
  -DSWITCH_PICO_INPUT_BACKEND=BLUEPAD32 \
  -DSWITCH_PICO_HD_RUMBLE=ON \
  -DSWITCH_PICO_SYS_CLOCK_MHZ=300 -DSWITCH_PICO_OVERCLOCK_MV=1300 \
  -DSWITCH_PICO_CYW43_PACKET_READ=ON -DSWITCH_PICO_HCI_CREDIT_BATCH=ON \
  -DSWITCH_PICO_HD_PACKET_FRAMES=32 \
  -DSWITCH_PICO_HAPTICS_EXPERIMENT_RAM=ON -DSWITCH_PICO_LOG=OFF
cmake --build build-hd-rumble
```

Load `build-hd-rumble/switch-pico.elf` or `.uf2`, or use the standard `build.py --aio` entry point and `firmware/switch-pico-aio.uf2`. Both use the optimized native path. Reconnect a DualSense with PS if needed. Manual selection uses `haptics-experiment gameplay --slot N` after stopping any active run; CLI slots are zero-based, so the second controller is `--slot 1`. Manual arming does not persist across power cycles. `stop` disarms the native stream and restores compatibility output. A new eligible connection may auto-arm; there is no periodic re-arm that defeats an explicit stop.

The decoder preserves each actuator's one-to-three ordered substeps and frequency indices. Amplitudes become linear Q0.15 values via precomputed lookups; compatibility magnitudes retain their previous mapping. Profile strong/weak scales apply to the low/high bands of both actuators without discarding substeps.

The synthesizer has independent left/right low/high phase accumulators. Frequencies are `40 * 2^(index/32)` and `80 * 2^(index/32)` Hz. Each Switch command occupies an 8 ms window, split into 24/12/8 PCM samples per substep for counts 1/2/3. New reports supersede unplayed old substeps; identical compressed words hold final state rather than replaying deltas. Each Switch-updated side expires after 50 ms, matching the existing conservative timeout policy.

Standard gameplay uses 10.667 ms causal lookback. The explicitly selected, unqualified 64-frame experiment uses 21.333 ms. Fixed 16-entry cross-core and synthesis histories contain commands, not PCM. Overflow is counted and obsolete sample intervals are not replayed. XInput holds use a distinct persistent command: strong/low magnitude drives the left 160 Hz band, weak/high magnitude drives the right 320 Hz band, until a new command or zero stop. They do not fake refreshes to evade the 50 ms Switch watchdog. Retained XInput state is seeded once per native run, including manual re-arming after compatibility output.

Native gameplay uses balanced **2x low/high gain after profile scaling**, followed by a gentle **0.8-power curve** on the combined amplitude. This lifts quiet and medium effects while retaining their low/high ratio. The curve is a 257-entry lookup with integer interpolation, not per-sample floating-point math. Combined weights are capped at 65535 to avoid overflow and clipping. Zero remains zero. The amplitude curve does not alter carrier frequencies or local-confirmation gain; packet timing follows the transport configuration above. This response replaced the initial 1.5x and low-band-only experiments after user comparison.

Local confirmation remains a transient overlay and resumes the current host state. USB reset/unmount/suspend clears held XInput output. In Auto mode, an unmount intentionally watchdog-reboots to Switch probe; this includes a reset that clears TinyUSB's configured/mounted state. A libusb reset can consequently report “Entity not found” while the device re-enumerates. Persistent manual XInput mode is exempt from that Auto-mode reboot policy.

The gameplay stream continues with silence while idle. It stops on disconnect, explicit stop, or a stalled send-permission watchdog. Existing LED feedback can drain without switching the controller out of native haptics. Continuous idle streaming trades power for avoiding repeated audio-mode startup.

## Building and running

Use the repository's provisioned Pico SDK/toolchain environment. These CMake commands only build; they do not flash the adapter:

```sh
cmake -S . -B build-haptics -DPICO_BOARD=pico2_w \
  -DSWITCH_PICO_INPUT_BACKEND=BLUEPAD32 \
  -DSWITCH_PICO_HAPTICS_EXPERIMENT=ON \
  -DSWITCH_PICO_HD_RUMBLE=OFF \
  -DSWITCH_PICO_HAPTICS_EXPERIMENT_RAM=ON \
  -DSWITCH_PICO_LOG=OFF
cmake --build build-haptics
```

The fixture artifacts are `build-haptics/switch-pico.elf` and `.uf2`; disabling auto-arm lets `start` run without first stopping gameplay. For a same-clock flash comparison, use a separate directory and `SWITCH_PICO_HAPTICS_EXPERIMENT_RAM=OFF`. Standard AIO/XInput `build.py` entry points explicitly enable the qualified defaults even with an old CMake cache; UART entry points explicitly disable them.

After loading the chosen image and connecting a DualSense:

```sh
uv run switch-pico-config haptics-experiment status
uv run switch-pico-config haptics-experiment start --slot 0 --watch --json
uv run switch-pico-config haptics-experiment profile --json
uv run switch-pico-config haptics-experiment stop --slot 0
```

For the deterministic fixture only, the first tone is intentionally scheduled 1.024 seconds after start. Gameplay instead renders the timestamped host timeline with one report interval of lookback. First-tone fields identify the logical first nonsilent sample and the containing report's submission, not actual actuator onset.

## Protocol

USB vendor operation 0x40: OUT `{action, slot}` (0=stop, 1=finite fixture, 2=gameplay; slots 0..3), using the existing envelope. IN is schema 5, 84 bytes.

- Seventeen little-endian u32 fields: run_id, connection_generation, start_us, generated_packets, sent_packets, skipped_packets, send_failures, can_send_requests, synchronous_callbacks, max_generate_us, max_send_gap_us, max_lateness_us, max_request_wait_us, first_tone_due_us, first_tone_sent_us, last_sent_us, elapsed_us.
- Four u8 fields: state, slot, last_error, reserved (zero).
- Byte 72: mode (0=fixture, 1=gameplay); byte 73: actual stereo frames per packet (32 or 64); byte 74: whether the last successfully submitted PCM packet was nonzero (0/1); byte 75: zero reserved. Nonzero PCM is firmware output evidence, not measured actuator motion.
- Little-endian u32 at 76: `host_updates`; at 80: `dropped_updates`.
- State: idle=0, pending=1, running=2, completed=3, stopped=4, disconnected=5, unsupported=6, error=7. Disabled build reports unsupported.
- Microsecond timestamps are low 32 bits of Pico uptime; use unsigned modular differences for this bounded experiment. Host receipt time is not a hardware onset measurement.
- Error: none=0, unsupported controller=1, insufficient MTU=2, disconnected=3, timeout=4, transport failure=5, queued conventional output=6. The fixture rejects a queued start. Gameplay allows a bounded startup interval for prior output to drain; it does not discard LED/control reports.

### Transport timing probe

Operation `0x41` is IN-only. Schema 3 contains 44 little-endian 32-bit fields in `HapticsTransportProbe` declaration order (176 bytes). The first 32 retain the prior counters and raw advertised ACL capacity. Appended fields report requested/measured system clock, measured USB clock, regulator setting, flash divider, CYW43 PIO divider in 1/256 units, estimated temperature (signed milli-Celsius), credit/ACL/other HCI write counts, write failures, and packet-read optimization. Regulator voltage is a setting, not a multimeter reading; temperature assumes a 3.3 V ADC reference and is uncalibrated. Counters correlate by run ID/generation; snapshots are not an atomic cross-operation instant.

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

The initial investigation used stock 150 MHz. The qualified AIO default is now **300 MHz at 1.3 V**. CPU clocks do not increase Bluetooth air rate or repair protocol/scheduling errors.

The first 300 MHz image stopped at its flash-divider guard: RP2350 had discarded optional boot2/XIP setup, so `PICO_FLASH_SPI_CLKDIV` was not applied. `PICO_EMBED_XIP_SETUP=1` fixed that boot path; its linked copy/execute path was checked. The corrected 300 MHz/1.3 V image booted and passed input-plus-rumble tests. On this board 400 MHz/1.3 V did not boot; the user-requested 400 MHz/1.4 V variant booted and passed a short run but was not faster in the measured workload.

An explicit `SWITCH_PICO_SYS_CLOCK_MHZ=400` plus `SWITCH_PICO_OVERCLOCK_MV=1400` permits that experiment; it alone lifts the regulator's 1.3 V limit. Lower-voltage builds restore the limit after lowering voltage, including warm reboot. Stock 150 MHz is an explicit option, not the default AIO image. USB remains 48 MHz. Flash/PIO dividers are 4/4 at 300 MHz and 6/(5+86/256) at 400 MHz, keeping flash at 75/66.67 MHz and CYW43 SPI at about 37.5 MHz.

[Pimoroni's measurements](https://learn.pimoroni.com/article/overclocking-the-pico-2) are useful experimental evidence, not a stability or lifetime guarantee for every Pico 2 W. Do not generalize their extreme-voltage/cooling experiments. Keep a recovery image that understands the current profile schema; old pre-migration firmware does not understand catalog 2/schema 6.

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

The HD gameplay image auto-arms on connection. This is a translation to DualSense actuators, not a promise of identical Nintendo force response. Physical onset still needs synchronized measurement. Follow-on IMU and rumble-response refinements are recorded below.

### IMU scheduling correction

After HD integration was committed as `6188fcb`, the user reported a small aiming lag. The USB scheduler had two avoidable delays: unsuccessful motion sends advanced the shared timer, and control replies reset the same timer used for motion. It also mutated quaternion/timestamp state before successful USB submission.

The scheduler now uses an independent logical 15 ms motion clock, checks endpoint readiness before integrating, retries overdue data at the next ready opportunity, and rolls back motion state if queuing fails. Successful sends advance the logical clock rather than drifting with 8 ms USB polling. Long stalls skip missed periods instead of replaying a catch-up burst. Control replies and overdue motion get bounded service without one starving the other. No gyro scaling or smoothing was changed.

New regressions failed on the old implementation and passed with the correction. Live USB checks measured about 15 ms average motion-report spacing, including during control-reply traffic; four deliberate backpressure trials recovered with successive-read gaps of about 7.96–7.98 ms. The user retested aiming on the Switch and reported it was “pretty good now.”

### Accepted rumble response

The user described the 1.5x mix as thin/hollow and lacking body. A 2x low / 1.5x high comparison was still insufficient; the user requested more high-band response and selected a fuller amplitude curve. The final balanced 2x / 0.8-power response was tested on the device and in the same game, and the user selected **“Fuller and good.”**

The final USB-driven test delivered all **513 host commands**, with **zero dropped updates**, **zero Bluetooth skips**, and **zero send failures**. Controller input continued with 1,408 reports during the test. Maximum observed packet generation was 687 us and report gap 23,925 us. Local-feedback gain, frequencies, the 50 ms watchdog, the 21.333 ms lookback, and controller buffering remain unchanged.

Final checks: **151 focused tests passed**. HD gameplay, deterministic experiment, normal all-in-one, and Pico/UART builds succeeded. The accepted response and IMU scheduler correction are flashed; image files are `build-hd-rumble/switch-pico.elf` and `.uf2`. The stream remains armed. These follow-on refinements are separate from the committed HD integration.

### Qualified lower-latency transport and standard-image cutover

The following comparisons used real 8 ms USB rumble output while continuously
reading USB motion reports, not only an idle PCM fixture. Read/poll means
include empty calls and nested work and must not be interpreted as additive
CPU utilization.

| Configuration | Mean read call | Mean poll call | Result |
|---|---:|---:|---|
| Stock 150 MHz | 971 us | 2,494 us | 2,050 commands received; six skipped audio slots |
| 300 MHz / 1.3 V | 665 us | 1,370 us | 2,050 commands; zero drops/skips/send failures |
| 400 MHz / 1.4 V | 730 us | 1,485 us | 2,050 commands; zero drops/skips/send failures |
| 300 MHz + packet-level reads | 421 us | 1,139 us | 2,050 commands; zero drops/skips/send failures |
| 300 MHz + packet reads + credit batching | 274 us | 572 us | 2,050 commands; zero drops/skips/send failures |

Packet-level reads use one published ring-index snapshot and one consumer
publication/notification per complete packet. Partial packets stay
unconsumed; a build-local SDK patch propagates SPI read errors. Credit
batching retains controller-to-host flow control and three host ACL credits,
returning at two completed packets or a bounded timer deadline. SCO remains
immediate. Lifecycle cancellation and synchronous callbacks have regressions.

The packet-read stress run passed 8,194 commands with zero drops/skips/failures.
The subsequent 32-frame run also passed all 8,194 commands over about 65 s,
submitted 6,165 audio reports, and had a worst observed report gap of
12,594 us (nominal 10,666.67 us). No physical actuator-onset bound is claimed.

The user described the controlled test effects as notably good. Preserve this
reference separately from actual game effects: alternate 256 ms of left word
`0x68402100` / neutral right, then neutral left / right word `0x4840a100`,
refreshing every 8 ms. Neutral is `0x40400100`; words are little-endian in the
eight-byte rumble payload of USB report `0x10`. These are fixed 160/320 Hz
bands with primary/secondary amplitude codes 80/16, using the normal profile
scaling and 2x/0.8 response—not an extra test-only boost. Sustained reference
effects do not establish that short or swept game effects will feel identical.

Native XInput was exercised with the DualSense on API slot 1 while a Switch
Pro occupied slot 0. The Pro received two compatibility dispatches without
entering the native stream; four DualSense commands held left/right output
for 700 ms and explicit zeros stopped it. The stream stayed selected on the
DualSense and configuration generation remained 9. The former slot-0-only
auto-arm condition was removed and a mixed-controller regression verifies
first-eligible selection without a later controller stealing ownership.

Final integrated verification includes 228 passing repository tests, standard
AIO/XInput builds with the native defaults, the manual fixture and unchanged
Pico/UART builds. The profile catalog was migrated and all sixteen stored
profiles were compared with the pre-migration backup; the temporary editor
profile and name were restored. No configuration, bond, or wake-identity reset
was part of the transport work.

### Historical mixed-controller cadence limit

Final testing with a Switch Pro plus a DualSense and continuous USB motion
reads changed the cadence decision. Rumble commands targeted only the
DualSense; the Pro supplied concurrent input traffic, not a Pro rumble
qualification. The 32-frame stream received all 2,050
host commands but skipped **80 audio slots in 16.6 s**. Maximum permission
wait reached **17,180 us**, exceeding its 10,667 us interval, with all eight
outgoing credits observed in use. CPU clock remained 300 MHz/1.3 V.

Those measurements led the earlier standard build to use **64 frames / 46.875
reports per second**, without reverting its CPU or transport improvements.
The same 2,050-command
mixed-controller comparison passed with zero drops/skips/send failures and
778 audio reports. Its worst observed report gap was 26,588 us. A subsequent
roughly 65-second mixed-controller stress run received all **8,194 commands**
and submitted **3,082 audio reports**, with **zero drops, skipped audio slots,
or send failures**. It processed 41,738 input reports and its worst observed
audio report gap was 26,655 us.

That earlier transport-only choice is superseded by the current accepted
32-frame format and explicit audio initialization above. Neither configuration
is advertised as qualified for mixed/four-controller physical fidelity.

The later Nintendo-native implementation adds output traffic that was absent
from this cadence comparison. Its Pro-only controlled run delivered all 1,025
commands at 125 Hz, but early mixed Pro/DualSense runs exposed shared-radio
congestion. Nintendo can-send-driven delivery and held-state coalescing are
separate from that earlier DualSense 64-frame policy. Consult
[SWITCH_FAMILY_HD_RUMBLE_PLAN.md](SWITCH_FAMILY_HD_RUMBLE_PLAN.md) for measured
results and outstanding qualification; do not treat the DualSense-only output
benchmark above as proof that simultaneous native streams are lossless.
