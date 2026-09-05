# DualSense low-latency haptics experiment

## Goal and evidence

Prove a bounded-latency native Bluetooth PCM transport on one Pico 2 W / DualSense connection before reconnecting Nintendo HD-rumble decoding. Normal builds retain compatibility rumble. This is an opt-in deterministic transport experiment, not a claim of complete HD Rumble support.

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

## Implementation contract

1. Build-only opt-in `SWITCH_PICO_HAPTICS_EXPERIMENT`; separate build directory/artifacts. Preserve wake identity, pairing storage, USB modes and ordinary firmware artifacts. Use stock clock/voltage. Experimental builds use the controller's advertised outgoing ACL capacity and one-packet receive batches with explicit rescheduling; normal builds retain the three-credit cap and sixteen-packet batches. Incoming flow control and all FIFO sizes remain unchanged.
2. One selected connected Sony DualSense/DualSense Edge, Bluetooth Classic, sufficient negotiated MTU. Explicit management start/stop; no tones at pairing or boot.
3. 142-byte report 0x32 plus A2 transaction = 143-byte L2CAP SDU. The first report selects native mode using sized state block 0x90, length 63, with rumble-selection bits and other write flags clear; it carries one silent 64-byte haptic block (0x92). Subsequent reports use compact audio control `{0x91,3,0x62,16,counter}` and **two** 64-byte haptic blocks: descriptor 0xd2, length 64, followed by 128 sample bytes. Thus 0xd2 is valid here, unlike the original spike's single-block mismatch. Deterministic padding and Bluetooth CRC. No speaker, microphone, USB audio endpoint, Opus or resampler.
4. Steady-state 64 stereo frames per report at 3 kHz, 46.875 reports/s. Absolute microsecond/sample deadlines use integer rational arithmetic; preserve fractional time and skip obsolete packets after stalls rather than burst-replaying them. Timer wakeups account for SDK +1 tick. Can-send permission and audio deadlines are separate. Arm flags before requests and handle synchronous callbacks without recursive stream generation.
5. Finite sequence: 48 report intervals of priming silence (1.024 s), four cycles of left 100 Hz tone / silence / right 200 Hz tone / silence (12 reports = 256 ms per phase), then 48 reports of trailing silence. Total 288 reports / 6.144 s. The initial mode handoff contains 32 silent frames; normal two-block streaming follows. Signed sample peak 32/127 is deliberately gentle, not a claim of 25% perceived force. Stop preempts the pattern, emits silence when sendable and restores compatibility output. Disconnect cancels without stale-pointer use. Only the selected controller's conventional outputs are overridden.
6. No historical PCM FIFO. Generate only the current due block when transmission is permitted; bounded control mailbox across cores. Record packet counts, skipped blocks, failed sends, synchronous callbacks, generation cost, send gaps, lateness, request wait and first-tone timestamps. HCI submission is not physical actuator onset.
7. Host `haptics-experiment start --slot 0`, `status`, `stop`, and `profile` use existing USB management framing. The experiment reports unsupported on ordinary builds. Existing general runtime diagnostics remain unchanged. Both experiment and transport-profile responses now require schema 2; update the host and experimental firmware together.
8. Regression coverage must include synchronous callback delivery, rational clock and late wakeups, reference packet interpretation, finite completion/stop, disconnect/reconnect and compatibility restoration. Native probes cannot prove controller acceptance or physical latency.

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

The first tone is intentionally scheduled 1.024 seconds after the stream starts; that priming silence is not transport delay. Compare physical onset against `first_tone_due_us` / `first_tone_sent_us`, not the time the start command was entered. Each 256 ms tone/silence phase is a second timing marker. Host USB polling can miss intermediate state but the firmware retains maxima and final counters.

## Protocol

USB vendor management operation 0x40: OUT two-byte payload `{action, slot}` (0=stop, 1=start, slot 0..3); existing request envelope. IN diagnostics, existing response envelope, schema 2, 72-byte payload. Schema 2 identifies the two-block/288-report pattern; schema 1 used one block/576 reports.

- Seventeen little-endian u32 fields: run_id, connection_generation, start_us, generated_packets, sent_packets, skipped_packets, send_failures, can_send_requests, synchronous_callbacks, max_generate_us, max_send_gap_us, max_lateness_us, max_request_wait_us, first_tone_due_us, first_tone_sent_us, last_sent_us, elapsed_us.
- Four u8 fields: state, slot, last_error, reserved (zero).
- State: idle=0, pending=1, running=2, completed=3, stopped=4, disconnected=5, unsupported=6, error=7. Disabled build reports unsupported.
- Microsecond timestamps are low 32 bits of Pico uptime; use unsigned modular differences for this bounded experiment. Host receipt time is not a hardware onset measurement.
- Error: none=0, unsupported controller=1, insufficient MTU=2, disconnected=3, timeout=4, transport failure=5, queued conventional output=6. Start rejects a nonempty conventional output queue rather than discarding LED/control reports or interleaving them with PCM; let prior output drain before retrying.

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

The SRAM experimental image remains loaded, with the experiment stopped. Nintendo HD-rumble decoding is **not yet connected** to this PCM sender; normal gameplay retains compatibility rumble. Physical onset still needs a synchronized sensor/contact-microphone measurement before claiming a gameplay-to-actuator latency bound.
