// GigaHardwareTest.ino
// Run this BEFORE the main sketch to verify:
//   1. Display initializes and draws correctly
//   2. Touch coordinates map to screen pixels (calibrate mapTouch if needed)
//   3. Gate/relay pins are free (not driven by display shield)
//
// Open Serial Monitor at 115200 to see pin states and raw touch coords.

#include <Arduino_GigaDisplay_GFX.h>
#include <Arduino_GigaDisplayTouch.h>

GigaDisplay_GFX        tft;
Arduino_GigaDisplayTouch touch;

// Pins to verify — must read HIGH (floating/pullup) if shield does not use them
const int CHECK_PINS[] = {27, 35, 39, 51};
const char* CHECK_NAMES[] = {"D27 (Gate 1)", "D35 (Gate 2)", "D39 (Gate 3)", "D51 (DC relay)"};

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1500);  // give Serial monitor time to connect

    // Configure pins as inputs with pullup to detect if shield drives them
    for (int i = 0; i < 4; i++) {
        pinMode(CHECK_PINS[i], INPUT_PULLUP);
    }

    Serial.println("=== Giga Hardware Test ===");
    Serial.println();
    Serial.println("--- Pin check (expect HIGH if pin is free) ---");
    for (int i = 0; i < 4; i++) {
        Serial.print(CHECK_NAMES[i]);
        Serial.print(" = ");
        Serial.println(digitalRead(CHECK_PINS[i]) == HIGH ? "HIGH (free)" : "LOW  *** CONFLICT — pick a different pin ***");
    }
    Serial.println();

    // Display init
    Serial.print("Starting display... ");
    tft.begin();
    tft.setRotation(1);  // landscape 800x480
    Serial.println("OK");

    // Draw test pattern
    tft.fillScreen(0x0000);  // black

    // Color bars to confirm display orientation and colors
    tft.fillRect(0,   0, 200, 120, 0xF800);  // red    — top-left
    tft.fillRect(200, 0, 200, 120, 0x07E0);  // green
    tft.fillRect(400, 0, 200, 120, 0x001F);  // blue
    tft.fillRect(600, 0, 200, 120, 0xFFE0);  // yellow — top-right

    // Border to confirm full 800x480 extent
    tft.drawRect(0, 0, 800, 480, 0xFFFF);

    // Labels
    tft.setTextColor(0xFFFF);
    tft.setTextSize(3);
    tft.setCursor(10, 140);
    tft.print("Giga Display Test");

    tft.setTextSize(2);
    tft.setCursor(10, 200);
    tft.print("Width: 800  Height: 480  Rotation: 1");

    tft.setCursor(10, 240);
    tft.print("Touch anywhere — raw coords printed to Serial");

    tft.setCursor(10, 280);
    tft.print("Top-left RED, top-right YELLOW = correct orientation");

    // Corner markers
    tft.fillCircle(10,  10,  8, 0xFFFF);  // top-left
    tft.fillCircle(790, 10,  8, 0xFFFF);  // top-right
    tft.fillCircle(10,  470, 8, 0xFFFF);  // bottom-left
    tft.fillCircle(790, 470, 8, 0xFFFF);  // bottom-right

    // Touch init
    Serial.print("Starting touch... ");
    touch.begin();
    Serial.println("OK");
    Serial.println();
    Serial.println("Touch screen now. Check that printed (tx,ty) match where you touched.");
    Serial.println("If coordinates are rotated/mirrored, adjust mapTouch() in main sketch.");
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    uint8_t    contacts;
    GDTpoint_t points[5];

    contacts = touch.getTouchPoints(points);
    if (contacts > 0) {
        uint16_t rawX = points[0].x;
        uint16_t rawY = points[0].y;

        // Same mapping as main sketch: raw → landscape
        int tx = (int)rawY;
        int ty = 479 - (int)rawX;

        Serial.print("raw(");
        Serial.print(rawX);
        Serial.print(",");
        Serial.print(rawY);
        Serial.print(")  mapped(");
        Serial.print(tx);
        Serial.print(",");
        Serial.print(ty);
        Serial.println(")");

        // Draw dot at mapped position
        tft.fillCircle(tx, ty, 6, 0xFD20);  // orange dot

        // Show coordinates on screen
        tft.fillRect(10, 350, 500, 30, 0x0000);
        tft.setTextSize(2);
        tft.setTextColor(0xFFFF);
        tft.setCursor(10, 355);
        tft.print("raw(");
        tft.print(rawX);
        tft.print(",");
        tft.print(rawY);
        tft.print(") -> mapped(");
        tft.print(tx);
        tft.print(",");
        tft.print(ty);
        tft.print(")");

        delay(100);
    }
}
