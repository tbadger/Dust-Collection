// Dust Collection System — Arduino Giga R1 WiFi + Giga Display Shield
// Migrated from DustCollection_v3 (Mega 2560)
//
// What changed from Mega version:
//   - LiquidCrystal_I2C replaced by GigaDisplay_GFX (800×480 touchscreen)
//   - Physical buttons + ISRs removed; gate control is touch-driven
//   - IR remote removed
//   - analogReadResolution(12) added — EmonLib already handles 12-bit ARM ADC
//   - toolThresholds is now a mutable array (not const) for future Web UI tuning
//   - Drill Press button now properly toggles DC on AND off
//
// ── CALIBRATION NOTE ─────────────────────────────────────────────────────────
// CT sensor calibration (EMON_CAL = 111.1) was tuned for 5 V Mega supply.
// Giga runs at 3.3 V; EmonLib compensates via readVcc() returning 3300 for ARM,
// but actual measured Irms will differ from Mega. Re-tune EMON_CAL and
// toolThresholds after hardware bring-up.
//
// ── TOUCH COORDINATE NOTE ────────────────────────────────────────────────────
// GT911 may return raw (portrait) coordinates even when display is in landscape.
// If touches do not align with buttons, adjust mapTouch() at the bottom.

#include <Arduino_GigaDisplay_GFX.h>
#include <Arduino_GigaDisplayTouch.h>
#include "EmonLib.h"

// ── Hardware config ───────────────────────────────────────────────────────────
static const int NUM_TOOLS = 5;

static const char* TOOL_NAMES[NUM_TOOLS] = {
    "CNC Router", "Table Saw", "Sander", "Drill Press", "Unused"
};

// Blast gate relay pins — LOW = gate open, HIGH = gate closed
static const int BLAST_GATE_PINS[NUM_TOOLS] = {27, 35, 39, 43, 47};

// DC collector relay — LOW = on, HIGH = off (active-low relay board)
static const int DC_RELAY_PIN = 51;

// EmonLib CT calibration constant. See calibration note above.
static const double EMON_CAL = 111.1;

// Per-tool trigger thresholds in Amps.
// Not const — tunable at runtime via future Web UI.
double toolThresholds[NUM_TOOLS] = {160.0, 260.0, 240.0, 300.0, 300.0};

// ── Timing ────────────────────────────────────────────────────────────────────
static const unsigned long DISPLAY_INTERVAL         = 250;    // ms
static const unsigned long TOOL_TRANSITION_TIME     = 500;    // ms — gate-open to DC-on delay
static const unsigned long TOOL_OFF_TRANSITION_TIME = 15000;  // ms — DC-off delay after tool stops
static const unsigned long TOUCH_DEBOUNCE_MS        = 350;    // ms

// ── State machine ─────────────────────────────────────────────────────────────
enum SystemState {
    STARTUP,
    MONITORING,
    TOOL_ACTIVATING,
    TOOL_RUNNING,
    TOOL_DEACTIVATING,
    MANUAL_CONTROL
};

SystemState   currentState  = STARTUP;
unsigned long stateTimer    = 0;
unsigned long previousMillis = 0;

// ── Tool / gate state ─────────────────────────────────────────────────────────
bool   gateOpen[NUM_TOOLS]    = {false};
double toolCurrents[NUM_TOOLS] = {0};
int    lastManualIndex         = -1;

// ── EmonLib ───────────────────────────────────────────────────────────────────
EnergyMonitor sensors[NUM_TOOLS];

// ── Display & touch ───────────────────────────────────────────────────────────
GigaDisplay_GFX        tft;
Arduino_GigaDisplayTouch touch;

static const int SCREEN_W = 800;
static const int SCREEN_H = 480;

// RGB565 palette
static const uint16_t C_BG       = 0x1082;  // dark blue-gray background
static const uint16_t C_HEADER   = 0x000F;  // navy header
static const uint16_t C_WHITE    = 0xFFFF;
static const uint16_t C_BLACK    = 0x0000;
static const uint16_t C_GATE_ON  = 0x07E0;  // green — gate open
static const uint16_t C_GATE_OFF = 0x4228;  // mid-gray — gate closed
static const uint16_t C_DC_ON    = 0x07E0;  // green — DC running
static const uint16_t C_DC_OFF   = 0xF800;  // red — DC off
static const uint16_t C_ACTIVE   = 0xFD20;  // orange — current above threshold
static const uint16_t C_DISABLED = 0x2104;  // dark gray — unused tool button

struct Rect { int16_t x, y, w, h; };

// Gate toggle buttons — tools 0-3 drawn; tool 4 ("Unused") skipped
// Layout: 4 buttons across, y=120, h=140, 10px gaps and margins
static const Rect GATE_BTN[NUM_TOOLS] = {
    { 10,  120, 184, 140},  // CNC Router
    {204,  120, 184, 140},  // Table Saw
    {398,  120, 184, 140},  // Sander
    {592,  120, 184, 140},  // Drill Press (DC only — no gate)
    {  0,    0,   0,   0},  // Unused — zero-size, never drawn
};

// DC manual on/off button — centered, below gate buttons
static const Rect DC_BTN = {275, 385, 250, 80};

// ─────────────────────────────────────────────────────────────────────────────
// Forward declarations
// ─────────────────────────────────────────────────────────────────────────────
void drawFullUI();
void drawGateButton(int idx);
void drawDCButton();
void drawStatusBar();
void drawCurrentsRow();
void handleTouch();
void mapTouch(uint16_t rawX, uint16_t rawY, int& tx, int& ty);
void handleButtonPress(int idx);
void dustOn();
void dustOff();
bool areAllGatesClosed();
void updateSensors();
void handleStartupState();
void handleMonitoringState();
void handleToolActivatingState(unsigned long now);
void handleToolRunningState();
void handleToolDeactivatingState(unsigned long now);

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // 12-bit ADC required for EmonLib on ARM (EmonLib.h defines ADC_BITS = 12 for __arm__)
    analogReadResolution(ADC_BITS);

    // Blast gates — start closed
    for (int i = 0; i < NUM_TOOLS; i++) {
        pinMode(BLAST_GATE_PINS[i], OUTPUT);
        digitalWrite(BLAST_GATE_PINS[i], HIGH);
    }

    // DC relay — start off
    pinMode(DC_RELAY_PIN, OUTPUT);
    digitalWrite(DC_RELAY_PIN, HIGH);

    // EmonLib current sensors on A0–A4
    for (int i = 0; i < NUM_TOOLS; i++) {
        sensors[i].current(A0 + i, EMON_CAL);
    }

    // Display
    tft.begin();
    tft.setRotation(1);  // landscape 800×480
    drawFullUI();

    // Touch
    touch.begin();
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();

    handleTouch();
    updateSensors();

    // Periodic refresh of status bar and current readings
    if (now - previousMillis >= DISPLAY_INTERVAL) {
        drawStatusBar();
        drawCurrentsRow();
        previousMillis = now;
    }

    switch (currentState) {
        case STARTUP:            handleStartupState();               break;
        case MONITORING:         handleMonitoringState();            break;
        case TOOL_ACTIVATING:    handleToolActivatingState(now);     break;
        case TOOL_RUNNING:       handleToolRunningState();           break;
        case TOOL_DEACTIVATING:  handleToolDeactivatingState(now);   break;
        case MANUAL_CONTROL:     break;  // exits via touch events only
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Display
// ─────────────────────────────────────────────────────────────────────────────
void drawFullUI() {
    tft.fillScreen(C_BG);

    // Header bar
    tft.fillRect(0, 0, SCREEN_W, 55, C_HEADER);
    tft.setTextColor(C_WHITE);
    tft.setTextSize(3);
    tft.setCursor(10, 13);
    tft.print("Dust Collection Control");

    // Section label
    tft.setTextSize(2);
    tft.setTextColor(C_WHITE);
    tft.setCursor(10, 100);
    tft.print("Gates:");
    tft.setCursor(10, 270);
    tft.print("Currents:");
    tft.setCursor(10, 365);
    tft.print("Dust Collector:");

    for (int i = 0; i < NUM_TOOLS - 1; i++) drawGateButton(i);
    drawDCButton();
    drawStatusBar();
    drawCurrentsRow();
}

void drawGateButton(int idx) {
    const Rect& r = GATE_BTN[idx];

    bool isDrillPress = (idx == 3);
    bool dcOn = (digitalRead(DC_RELAY_PIN) == LOW);

    // For Drill Press: button indicates DC state (no gate to open)
    bool isActive = isDrillPress ? (dcOn && lastManualIndex == 3)
                                 : gateOpen[idx];

    uint16_t bg = (idx >= NUM_TOOLS - 1) ? C_DISABLED
                : isActive               ? C_GATE_ON
                                         : C_GATE_OFF;

    tft.fillRoundRect(r.x, r.y, r.w, r.h, 10, bg);
    tft.drawRoundRect(r.x, r.y, r.w, r.h, 10, C_WHITE);

    // Tool name (size 2, top of button)
    tft.setTextSize(2);
    tft.setTextColor(C_WHITE);
    tft.setCursor(r.x + 8, r.y + 16);
    tft.print(TOOL_NAMES[idx]);

    // State label (size 3, center of button)
    tft.setTextSize(3);
    tft.setCursor(r.x + 20, r.y + 75);
    if (isDrillPress) {
        tft.print(isActive ? "DC ON " : "DC OFF");
    } else {
        tft.print(isActive ? "OPEN" : "SHUT");
    }
}

void drawDCButton() {
    const Rect& r = DC_BTN;
    bool dcOn = (digitalRead(DC_RELAY_PIN) == LOW);

    tft.fillRoundRect(r.x, r.y, r.w, r.h, 12, dcOn ? C_DC_ON : C_DC_OFF);
    tft.drawRoundRect(r.x, r.y, r.w, r.h, 12, C_WHITE);

    tft.setTextColor(C_WHITE);
    tft.setTextSize(3);
    tft.setCursor(r.x + r.w / 2 - 42, r.y + 22);
    tft.print(dcOn ? " ON " : " OFF");
}

void drawStatusBar() {
    tft.fillRect(0, 60, SCREEN_W - 200, 35, C_BG);
    tft.setTextSize(2);
    tft.setTextColor(C_WHITE);
    tft.setCursor(10, 68);

    switch (currentState) {
        case STARTUP:           tft.print("Starting up...     "); break;
        case MONITORING:        tft.print("Monitoring         "); break;
        case TOOL_ACTIVATING:   tft.print("Tool detected...   "); break;
        case TOOL_RUNNING:      tft.print("Tool running       "); break;
        case TOOL_DEACTIVATING: tft.print("Shutting down...   "); break;
        case MANUAL_CONTROL:    tft.print("Manual control     "); break;
    }
}

void drawCurrentsRow() {
    // Clear row
    tft.fillRect(0, 295, SCREEN_W, 55, C_BG);
    tft.setTextSize(2);

    for (int i = 0; i < NUM_TOOLS; i++) {
        bool aboveThreshold = toolCurrents[i] > toolThresholds[i];
        tft.setTextColor(aboveThreshold ? C_ACTIVE : C_WHITE);
        tft.setCursor(10 + i * 156, 305);
        tft.print("T");
        tft.print(i + 1);
        tft.print(":");
        tft.print(toolCurrents[i], 1);
        tft.print("A");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Touch input
// ─────────────────────────────────────────────────────────────────────────────
static bool          touchActive = false;
static unsigned long lastTouchMs = 0;

// Remap GT911 raw coordinates to display pixel coordinates.
// With tft.setRotation(1) (landscape), raw portrait coords (x up to 480, y up to 800)
// need to be remapped. Adjust if touches are misaligned on hardware.
void mapTouch(uint16_t rawX, uint16_t rawY, int& tx, int& ty) {
    // Landscape: raw X → screen Y, raw Y → screen X (mirrored)
    tx = rawY;
    ty = rawX;
    // If still misaligned, try:
    //   tx = SCREEN_W - rawY;
    //   ty = rawX;
}

void handleTouch() {
    uint8_t    contacts;
    GDTpoint_t points[5];

    if (touch.getTouchPoints(contacts, points) && contacts > 0) {
        if (!touchActive && (millis() - lastTouchMs >= TOUCH_DEBOUNCE_MS)) {
            touchActive = true;
            lastTouchMs = millis();

            int tx, ty;
            mapTouch(points[0].x, points[0].y, tx, ty);

            // Gate buttons (tools 0-3)
            for (int i = 0; i < NUM_TOOLS - 1; i++) {
                const Rect& r = GATE_BTN[i];
                if (tx >= r.x && tx <= r.x + r.w &&
                    ty >= r.y && ty <= r.y + r.h) {
                    handleButtonPress(i);
                    return;
                }
            }

            // DC manual on/off button
            if (tx >= DC_BTN.x && tx <= DC_BTN.x + DC_BTN.w &&
                ty >= DC_BTN.y && ty <= DC_BTN.y + DC_BTN.h) {
                bool dcOn = (digitalRead(DC_RELAY_PIN) == LOW);
                if (dcOn) {
                    for (int i = 0; i < NUM_TOOLS; i++) {
                        gateOpen[i] = false;
                        digitalWrite(BLAST_GATE_PINS[i], HIGH);
                        drawGateButton(i < NUM_TOOLS - 1 ? i : i);
                    }
                    for (int i = 0; i < NUM_TOOLS - 1; i++) drawGateButton(i);
                    dustOff();
                    currentState   = MONITORING;
                    lastManualIndex = -1;
                } else {
                    dustOn();
                    currentState   = MANUAL_CONTROL;
                    lastManualIndex = -1;
                }
            }
        }
    } else {
        touchActive = false;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Gate and DC control
// ─────────────────────────────────────────────────────────────────────────────
void handleButtonPress(int idx) {
    if (idx == 3) {
        // Drill Press — toggle DC only; no physical gate
        bool dcOn = (digitalRead(DC_RELAY_PIN) == LOW);
        if (dcOn && lastManualIndex == 3) {
            dustOff();
            if (currentState == MANUAL_CONTROL) currentState = MONITORING;
        } else {
            dustOn();
            currentState = MANUAL_CONTROL;
        }
        lastManualIndex = idx;
        drawGateButton(idx);
        return;
    }

    // Toggle gate open/closed
    gateOpen[idx] = !gateOpen[idx];
    digitalWrite(BLAST_GATE_PINS[idx], gateOpen[idx] ? LOW : HIGH);

    if (gateOpen[idx]) {
        if (currentState != MANUAL_CONTROL) currentState = MANUAL_CONTROL;
        dustOn();
    } else if (areAllGatesClosed()) {
        dustOff();
        currentState = MONITORING;
    }

    lastManualIndex = idx;
    drawGateButton(idx);
}

bool areAllGatesClosed() {
    for (int i = 0; i < NUM_TOOLS; i++) {
        if (gateOpen[i]) return false;
    }
    return true;
}

void dustOn() {
    digitalWrite(DC_RELAY_PIN, LOW);
    drawDCButton();
}

void dustOff() {
    digitalWrite(DC_RELAY_PIN, HIGH);
    drawDCButton();
}

// ─────────────────────────────────────────────────────────────────────────────
// Sensor sampling
// ─────────────────────────────────────────────────────────────────────────────
void updateSensors() {
    for (int i = 0; i < NUM_TOOLS; i++) {
        toolCurrents[i] = sensors[i].calcIrms(1480);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// State machine handlers
// ─────────────────────────────────────────────────────────────────────────────
void handleStartupState() {
    static int ticks = 0;
    if (++ticks >= 5) {
        currentState = MONITORING;
        ticks = 0;
    }
}

void handleMonitoringState() {
    for (int i = 0; i < NUM_TOOLS; i++) {
        if (toolCurrents[i] > toolThresholds[i]) {
            gateOpen[i] = true;
            digitalWrite(BLAST_GATE_PINS[i], LOW);
            drawGateButton(i);
            currentState = TOOL_ACTIVATING;
            stateTimer   = millis();
            break;
        }
    }
}

void handleToolActivatingState(unsigned long now) {
    if (now - stateTimer >= TOOL_TRANSITION_TIME) {
        dustOn();
        currentState = TOOL_RUNNING;
    }
}

void handleToolRunningState() {
    for (int i = 0; i < NUM_TOOLS; i++) {
        if (toolCurrents[i] > (toolThresholds[i] * 0.8)) return;
    }
    // All tools below 80% threshold — start shutdown countdown
    currentState = TOOL_DEACTIVATING;
    stateTimer   = millis();
}

void handleToolDeactivatingState(unsigned long now) {
    // Re-check: any tool came back on?
    for (int i = 0; i < NUM_TOOLS; i++) {
        if (toolCurrents[i] > (toolThresholds[i] * 0.8)) {
            currentState = TOOL_RUNNING;
            return;
        }
    }

    if (now - stateTimer >= TOOL_OFF_TRANSITION_TIME) {
        dustOff();
        for (int i = 0; i < NUM_TOOLS; i++) {
            gateOpen[i] = false;
            digitalWrite(BLAST_GATE_PINS[i], HIGH);
        }
        for (int i = 0; i < NUM_TOOLS - 1; i++) drawGateButton(i);
        currentState = MONITORING;
    }
}
