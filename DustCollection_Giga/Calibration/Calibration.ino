// Calibration.ino — CT sensor bench calibration for DustCollection_Giga
//
// Run this standalone (instead of the main sketch) with the Giga wired to its
// 4 active CT coils (A0=CNC Router, A1=Table Saw, A2=Sander, A3=Drill Press;
// A4/"Unused" is not wired and is skipped here).
//
// Procedure:
//   1. Open Serial Monitor at 115200. Each channel's raw Irms (computed with
//      ICAL=1, so it's proportional to actual amps but not yet scaled) prints
//      once a second.
//   2. Run a tool on one channel at a time, at steady-state load. Clamp a
//      reference meter around the same conductor and read actual RMS amps.
//   3. Type "<channel>,<actual amps>" and press Enter, e.g.  1,4.85
//      This records one calibration sample: CAL = actual / raw.
//   4. Repeat a few times per channel (different load points if possible) —
//      each submission is averaged into a running per-channel CAL estimate.
//   5. Type "s" any time to print the suggested EMON_CAL[] array to paste
//      into DustCollection_Giga.ino. Type "r,<channel>" to reset a channel's
//      average if a bad sample was recorded.

#include "EmonLib.h"

static const int NUM_CHANNELS = 4;
static const char* CHANNEL_NAMES[NUM_CHANNELS] = {
    "CNC Router", "Table Saw", "Sander", "Drill Press"
};
static const int CHANNEL_PINS[NUM_CHANNELS] = {A0, A1, A2, A3};

EnergyMonitor sensors[NUM_CHANNELS];

double calSum[NUM_CHANNELS]   = {};
int    calCount[NUM_CHANNELS] = {};
double lastRaw[NUM_CHANNELS]  = {};

void setup() {
    Serial.begin(115200);
    while (!Serial) {}
    analogReadResolution(ADC_BITS);

    for (int i = 0; i < NUM_CHANNELS; i++) {
        sensors[i].current(CHANNEL_PINS[i], 1.0); // ICAL=1 -> raw, uncalibrated reading
    }

    Serial.println("=== CT Sensor Calibration ===");
    Serial.println("Raw (uncalibrated, ICAL=1) Irms prints every ~1s per channel.");
    Serial.println("With a tool running steady-state, clamp a reference meter on the same wire,");
    Serial.println("then type: <channel 0-3>,<actual amps from clamp meter>  and press Enter.");
    Serial.println("Example: 1,4.85");
    Serial.println("Type 's' to print the suggested EMON_CAL[] array.");
    Serial.println("Type 'r,<channel>' to reset a channel's running average.");
    Serial.println();
}

void loop() {
    static unsigned long lastPrint = 0;

    for (int i = 0; i < NUM_CHANNELS; i++) {
        lastRaw[i] = sensors[i].calcIrms(1480);
    }

    if (millis() - lastPrint >= 1000) {
        lastPrint = millis();
        for (int i = 0; i < NUM_CHANNELS; i++) {
            Serial.print(CHANNEL_NAMES[i]);
            Serial.print(" (A"); Serial.print(i); Serial.print("): raw=");
            Serial.print(lastRaw[i], 4);
            if (calCount[i] > 0) {
                Serial.print("  suggestedCAL=");
                Serial.print(calSum[i] / calCount[i], 3);
                Serial.print(" (n="); Serial.print(calCount[i]); Serial.print(")");
            }
            Serial.println();
        }
        Serial.println("---");
    }

    handleSerialInput();
}

void handleSerialInput() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) return;

    if (line == "s") {
        printSummary();
        return;
    }

    int comma = line.indexOf(',');
    if (comma < 0) {
        Serial.println("Format: <channel>,<amps>   or  r,<channel>   or  s");
        return;
    }

    String first  = line.substring(0, comma);
    String second = line.substring(comma + 1);

    if (first == "r") {
        int ch = second.toInt();
        if (ch >= 0 && ch < NUM_CHANNELS) {
            calSum[ch] = 0;
            calCount[ch] = 0;
            Serial.print("Reset channel "); Serial.println(ch);
        }
        return;
    }

    int    ch         = first.toInt();
    double actualAmps = second.toFloat();
    if (ch < 0 || ch >= NUM_CHANNELS) {
        Serial.println("Channel must be 0-3");
        return;
    }
    if (lastRaw[ch] < 0.001) {
        Serial.println("Raw reading near zero -- is the tool actually running on this channel?");
        return;
    }

    double cal = actualAmps / lastRaw[ch];
    calSum[ch] += cal;
    calCount[ch]++;
    Serial.print("Channel "); Serial.print(ch); Serial.print(" (");
    Serial.print(CHANNEL_NAMES[ch]); Serial.print("): sample CAL=");
    Serial.print(cal, 3);
    Serial.print("  running avg="); Serial.println(calSum[ch] / calCount[ch], 3);
}

void printSummary() {
    Serial.println("=== Suggested EMON_CAL array (paste into DustCollection_Giga.ino) ===");
    Serial.print("static const double EMON_CAL[NUM_TOOLS] = {");
    for (int i = 0; i < NUM_CHANNELS; i++) {
        if (i > 0) Serial.print(", ");
        if (calCount[i] > 0) Serial.print(calSum[i] / calCount[i], 3);
        else                 Serial.print("111.1 /* no samples yet */");
    }
    Serial.println(", 111.1 /* Unused, not wired */};");
}
