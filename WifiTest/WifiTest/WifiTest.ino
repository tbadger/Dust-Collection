#include <WiFi.h>
void setup() {
    Serial.begin(115200);
    delay(2000);
    // scan first to init module
    int n = WiFi.scanNetworks();
    Serial.print("Networks found: "); Serial.println(n);
    Serial.print("MAC after scan: "); Serial.println(WiFi.macAddress());
    // now try connect
    WiFi.begin("BadGuest");
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
        delay(500); Serial.print(".");
    }
    Serial.println();
    Serial.print("MAC after begin: "); Serial.println(WiFi.macAddress());
    Serial.print("Status: "); Serial.println(WiFi.status());
    if (WiFi.status() == WL_CONNECTED)
        Serial.println(WiFi.localIP());
}
void loop() {}