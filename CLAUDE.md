# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Automated shop dust collection controller. Migrating from **Arduino Mega 2560** to **Arduino Giga R1 WiFi + Giga Display Shield**. Controls blast gate relays and detects tool power state via current sensors.

- **Mega (reference):** `DustCollection_v3/DustCollection_v3.ino` (v6.3, 2025-07-25) — kept as-is
- **Giga (active):** `DustCollection_Giga/DustCollection_Giga.ino` — migration target

## Hardware — Mega 2560 (existing)

| Item | Detail |
|------|--------|
| Display | 20×4 I2C LCD at address `0x27` |
| Current sensors | 5× CT coils on analog pins A0–A4, calibration `111.1`, sampled via EmonLib `calcIrms(1480)` |
| Blast gate relays | Pins 27, 35, 39, 43, 47 — LOW = open, HIGH = closed |
| DC relay | Pin 51 — LOW = on, HIGH = off (active-low relay board) |
| Manual buttons | Interrupt pins 19, 18, 2, 3, 4 (INPUT_PULLUP, FALLING edge) |
| IR receiver | Pin 7, NEC protocol codes hard-coded in `translateIR()` |

## Target Hardware — Giga R1 WiFi

| Component | Detail |
|-----------|--------|
| MCU | STM32H747XI (Cortex-M7 + M4), 3.3 V I/O |
| Display | Giga Display Shield — 800×480 RGB, GT911 capacitive touch |
| Current sensors | Same CT coils; EmonLib needs ADC resolution fix (see migration notes) |
| Connectivity | u-blox ANNA-B112 — WiFi **and** BLE built in; no extra hardware needed for BT remote |

## Planned Features (Roadmap)

### In Scope

**1. Giga Display GUI** *(replaces physical buttons)*
- Touchscreen buttons on the 800×480 display to open/close each blast gate and toggle DC on/off
- Physical wiring for the 5 manual buttons and ISRs can be removed
- IR remote code can also be removed once GUI is fully functional
- Libraries needed: `Arduino_GigaDisplay_GFX`, `Arduino_GigaDisplayTouch`

**2. Web UI — sensor threshold tuning**
- Serve a config page over WiFi (Giga has built-in WiFi via ANNA-B112)
- Allow per-tool threshold adjustment without recompiling
- Values must persist across reboots — store in Giga's onboard flash (use `FlashIAPBlockDevice` or `KVStore` from mbed) or on SD card via the Display Shield's SD slot
- Libraries needed: `WiFi` (mbed built-in), `Arduino_MbedOS_FlashIAP` or similar for persistence

**3. Energy data collection — adaptive triggering**
- Log per-tool current readings over time to establish a per-tool idle baseline
- Detect tool-on as a **delta above baseline** rather than a fixed absolute threshold
- Benefits: handles sensor drift, different tool loads, and triggers DC sooner
- Storage: rolling buffer in RAM during session; optionally log to SD card for offline analysis
- Implementation target: replace `TOOL_THRESHOLDS[]` with a `ToolProfile` struct that holds baseline, delta threshold, and last-N samples

### Future / Deferred

**4. Bluetooth remote** *(deferred)*
- Hardware: repurposed **cheap yellow display (CYD)** — confirmed via board
  silkscreen: **ESP32-32E** (plain ESP32-WROOM-32E, not S3), 3.5" 320×480
  ST7796, resistive touch (xpt2046), BLE 4.2 + WiFi — already running
  ESPHome for other features. Replaces the previously-planned dedicated
  ESP32-S3-WROOM-1 module. (Note: the eBay/Amazon listing text said
  "ESP32-S3... capacitive touch" — that was wrong; trust the board's own
  silkscreen over listing copy.)
- **Architecture (2026-08-23, role swap from original plan): Giga = BLE
  peripheral (advertises), CYD = BLE central (connects/writes).** Original
  plan had it the other way; flipped after debugging — see "Debugging
  history" below for why.
- Firmware split:
  - Giga side: `ArduinoBLE` peripheral mode in `DustCollection_Giga.ino` —
    advertises as local name `"DustCollection"`, one writable/notifiable
    characteristic (uint8 gate index 0–3, sentinel 255 = idle).
    `handleBle()` polls `gateCmdChar.written()` each loop and feeds the
    value into the existing `handleButtonPress(value)` — same path the
    touchscreen uses.
  - CYD side: ESPHome `esp32_ble_tracker` + `ble_client` (connects by MAC
    address, `auto_connect: true`), tool buttons' `on_click` call
    `ble_client.ble_write` with the gate index. See
    `BLE_Remote_CYD/dust-collection-remote.yaml`.
  - CYD side is a full replacement config, not a merge: the board's
    previous job (man-cave-mini-dash — outdoor/shop temp, A/C control, 3D
    print status) is dropped entirely.
- Gotcha: `ble_client`'s `mac_address` is hardcoded to the Giga's BLE
  address (captured from its own Serial log). If the Giga's BLE address
  ever changes (re-flash of a different ArduinoBLE version, hardware swap),
  this needs updating on the CYD side too.
- **Debugging history:** original plan (CYD peripheral, Giga central) got
  as far as Giga finding the CYD's advertisement and matching its service
  UUID, but `peripheral.connect()` failed every attempt. `BLE.debug(Serial)`
  HCI trace showed the connect command was accepted by the controller, but
  the link-layer connection never completed — timed out and self-cancelled
  every time. Ruled out: WiFi/BLE radio contention (tested both sides off),
  module firmware version (already at latest for this board — the
  "firmware 3.0.0+" fix that applies to other Nina-based boards doesn't
  apply here), BLE address type mismatch, signal strength. Found a matching
  unresolved bug report:
  [ArduinoBLE#329](https://github.com/arduino-libraries/ArduinoBLE/issues/329)
  (same symptom, closed "not planned", no fix). Confirmed Giga's BLE
  peripheral mode and hardware are fine (connected cleanly to a phone's
  nRF Connect app), so central duty moved to the CYD instead — ESP32's
  central stack is far more battle-tested for this direction.

**5. LD2410C presence detection on the CYD remote** *(tabled)*
- Goal: mmWave presence sensor auto-wakes/sleeps the CYD backlight (layer
  with the current touch/route-driven brightness — 40% idle, 100% on
  touch, back to 40% when a gate toggles off)
- Approach: ESPHome native `ld2410` component — UART (TX/RX, 5V, 256000
  baud), exposes a `has_target` binary_sensor; `on_press`/`on_release` drive
  `light.turn_on`/`light.turn_off` on `backlight`, same shape as existing
  `on_touch` wake logic
- Blocked on: which GPIOs are free on the board (only 14/13/12/15/2/33/36/27
  are claimed so far — need 2 more for UART), 5V availability on the
  exposed header, and whether touch-wake stays as a secondary trigger or
  presence fully replaces it

## Open Tasks (as of 2026-08-23)

- **Fill in real per-channel `EMON_CAL[]` values.** Currently all 5 entries
  are still the Mega's shared `111.1` placeholder. Run
  `DustCollection_Giga/Calibration/Calibration.ino` (clamp meter against raw
  `ICAL=1` readings) and paste results into `DustCollection_Giga.ino`.
- **Hardware bring-up test** of the just-committed startup grace period
  (`STARTUP_GRACE_MS`) and trigger debounce (`TRIGGER_DEBOUNCE_SAMPLES`) —
  written and wired but not yet run on real hardware.
- ~~BLE pair end-to-end test~~ — **confirmed working (2026-08-23)** after
  the role swap (Giga = peripheral, CYD = central via `ble_client`). See
  item 4 above for the full debugging trail.
- Roadmap item 5 (LD2410C presence detection on the CYD remote) remains
  tabled — see Planned Features above.

## Libraries

`Libraries/` holds local copies used on Mega:

| Library | Used for |
|---------|----------|
| `EmonLib` | Current RMS measurement (modified — check for Mega-specific ADC assumptions) |
| `IRremote` | IR remote blast gate control |
| `LiquidCrystal_I2C` | 20×4 LCD |
| `Elegoo_GFX_Library` | Leftover — not used in current sketch |
| `Elegoo_TFTLCD` | Leftover — not used in current sketch |
| `TouchScreen` | Leftover — Mega resistive touch, not used |
| `ezButton` | Not used in current sketch |
| `Rotary-master` | Not used in current sketch |

## Architecture

Single-file state machine in `DustCollection_v3.ino`:

```
STARTUP → MONITORING ⇄ TOOL_ACTIVATING → TOOL_RUNNING → TOOL_DEACTIVATING → MONITORING
                    ↕
              MANUAL_CONTROL
```

- **MONITORING**: polls `toolCurrents[]` against `TOOL_THRESHOLDS[]`; opens gate + activates DC when threshold crossed
- **MANUAL_CONTROL**: button/IR overrides gate states directly; exits when all gates closed
  - *Migration target*: physical buttons and IR replaced by display touch events; state machine logic stays the same
- `dustOn()` / `dustOff()` are the only functions that touch the DC relay pin

## Mega → Giga Migration Notes

- **Voltage**: Giga I/O is **3.3 V only**. Relay boards and CT sensors at 5 V need level shifting.
- **ADC resolution**: `analogRead()` returns 0–4095 (12-bit) on Giga vs 0–1023 on Mega. EmonLib's `calcIrms()` assumes 10-bit. Must recalibrate: `tools[i].current(pin, calibration)` and check `EmonLib.cpp` for hardcoded `1023`/`512` ADC references.
- **No PROGMEM**: Drop any `PROGMEM`/`pgm_read_*` if added later — ARM has flat memory.
- **Timers**: AVR timer registers (`TCCR*`, `OCR*`) won't compile. Use mbed or Arduino timer APIs.
- **Interrupts**: `digitalPinToInterrupt()` works on Giga, but all digital pins support interrupts — no need to restrict to specific pins.
- **Display**: Replace `LiquidCrystal_I2C` with `Arduino_GigaDisplay_GFX` + `Arduino_GigaDisplayTouch`. Screen is 800×480 — redesign layout.
- **IR**: IRremote library supports STM32/mbed — should compile, but verify timer usage.
- **Dual-core**: M7 runs `setup()`/`loop()`. M4 available via `RPC` for offloading sensor sampling.

## Build

Arduino IDE 2.x or arduino-cli. Board package: `Arduino Mbed OS Giga Boards`.

```bash
# Compile
arduino-cli compile --fqbn arduino:mbed_giga:giga DustCollection_v3/

# Upload (adjust port)
arduino-cli upload --fqbn arduino:mbed_giga:giga --port COM3 DustCollection_v3/
```
