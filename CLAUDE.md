# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Automated shop dust collection controller. Migrating from **Arduino Mega 2560** to **Arduino Giga R1 WiFi + Giga Display Shield**. Controls blast gate relays and detects tool power state via current sensors.

Current code: `DustCollection_v3/DustCollection_v3.ino` (v6.3, released 2025-07-25).

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
