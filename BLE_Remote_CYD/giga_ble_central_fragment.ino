// ── ArduinoBLE central fragment — Giga side of the CYD BLE remote ──────────
// NOT standalone. Splice into DustCollection_Giga.ino as noted at each block.
// Pairs with BLE_Remote_CYD/ble_server_fragment.yaml (CYD side).
//
// Reuses the existing handleButtonPress(idx) — same function the touchscreen
// calls — so BLE gate 0-3 presses get identical radio/toggle/state-machine
// behavior as a screen tap. No new gate logic needed.
//
// Requires: ArduinoBLE library (Library Manager), uses the Giga's built-in
// ANNA-B112 module — same radio as WiFi, so BLE central + WiFi coexist here
// same as noted for the CYD peripheral side.

// ── 1. Add near top, with other #includes ──────────────────────────────────
#include <ArduinoBLE.h>

// ── 2. Add near other top-level consts (must match ble_server_fragment.yaml
//       exactly, including case) ───────────────────────────────────────────
static const char* CYD_SERVICE_UUID    = "5f2d958b-1564-4547-9566-64862169a9e4";
static const char* GATE_CMD_CHAR_UUID  = "3c711568-4467-44b3-9f0f-a7efb402c8ba";
static const unsigned long BLE_RESCAN_INTERVAL = 5000; // ms between reconnect attempts

// ── 3. Add near other globals (with gateOpen[], lastManualIndex, etc.) ─────
BLEDevice        cydRemote;
BLECharacteristic gateCmdChar;
bool             bleConnected      = false;
unsigned long    lastBleScanAttempt = 0;

// ── 4. Add function prototypes near handleButtonPress(int) declaration ─────
void setupBle();
void handleBle();

// ── 5. Call once from setup(), after setupWifi()/loadSettings() ────────────
//     setupBle();

// ── 6. Call every loop() iteration, alongside handleTouch()/updateSensors():
//     handleBle();

// ── 7. Implementations — add wherever other helper functions live ──────────

void setupBle() {
    if (!BLE.begin()) {
        Serial.println("BLE.begin() failed — CYD remote will be unavailable");
        return;
    }
    BLE.scanForUuid(CYD_SERVICE_UUID);
    lastBleScanAttempt = millis();
}

void handleBle() {
    if (!bleConnected) {
        // Throttle scan-restart attempts rather than hammering BLE.available()
        BLEDevice peripheral = BLE.available();
        if (!peripheral) return;

        BLE.stopScan();

        bool ok = peripheral.connect() && peripheral.discoverAttributes();
        if (ok) {
            gateCmdChar = peripheral.characteristic(GATE_CMD_CHAR_UUID);
            ok = gateCmdChar && gateCmdChar.canSubscribe() && gateCmdChar.subscribe();
        }

        if (ok) {
            cydRemote     = peripheral;
            bleConnected  = true;
            Serial.println("CYD remote connected");
        } else {
            if (peripheral.connected()) peripheral.disconnect();
            BLE.scanForUuid(CYD_SERVICE_UUID); // retry
        }
        return;
    }

    // Connected: drop out and rescan if the link died
    if (!cydRemote.connected()) {
        bleConnected = false;
        Serial.println("CYD remote disconnected — rescanning");
        BLE.scanForUuid(CYD_SERVICE_UUID);
        return;
    }

    if (gateCmdChar.valueUpdated()) {
        uint8_t idx = 0;
        gateCmdChar.readValue(idx);
        if (idx <= 3) {              // 255 = idle sentinel from CYD, ignore
            handleButtonPress((int)idx);
        }
    }
}
