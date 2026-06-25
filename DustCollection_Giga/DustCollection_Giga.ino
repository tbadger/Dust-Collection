// Dust Collection System — Arduino Giga R1 WiFi + Giga Display Shield
// Migrated from DustCollection_v3 (Mega 2560)
//
// What changed from Mega version:
//   - LiquidCrystal_I2C replaced by GigaDisplay_GFX (800×480 touchscreen)
//   - Physical buttons + ISRs removed; gate control is touch-driven
//   - IR remote removed
//   - analogReadResolution(12) added — EmonLib already handles 12-bit ARM ADC
//   - toolThresholds is now mutable, configurable via Web UI
//   - shutoffDelayMs replaces the old TOOL_OFF_TRANSITION_TIME const
//   - WiFi web server added for monitoring graphs and config
//
// ── CALIBRATION NOTE ─────────────────────────────────────────────────────────
// CT sensor calibration (EMON_CAL = 111.1) was tuned for 5 V Mega supply.
// Giga runs at 3.3 V; EmonLib compensates via readVcc() returning 3300 for ARM,
// but actual measured Irms will differ from Mega. Re-tune EMON_CAL and
// toolThresholds after hardware bring-up.
//
// ── WEB UI ───────────────────────────────────────────────────────────────────
// Set WIFI_SSID / WIFI_PASS below. When connected, the display header shows the
// IP address. Open http://<IP>/ in any browser to view graphs and edit settings.
// Settings persist across reboots via mbed KVStore (onboard flash).

#include <Arduino_GigaDisplay_GFX.h>
#include <Arduino_GigaDisplayTouch.h>
#include <WiFi.h>
#include "EmonLib.h"
#include "kvstore_global_api.h"

// ── WiFi credentials ─────────────────────────────────────────────────────────
// Edit arduino_secrets.h — never paste real credentials here
#include "arduino_secrets.h"
#define WIFI_SSID  SECRET_SSID
#define WIFI_PASS  SECRET_PASS

// ── Hardware config ───────────────────────────────────────────────────────────
static const int NUM_TOOLS = 5;
static const int NUM_GATES = 3;

static const char* TOOL_NAMES[NUM_TOOLS] = {
    "CNC Router", "Table Saw", "Sander", "Drill Press", "Unused"
};

static const int BLAST_GATE_PINS[NUM_TOOLS] = {
    27,   // CNC Router
    35,   // Table Saw
    39,   // Sander
    -1,   // Drill Press — no gate, DC only
    -1,   // Unused
};
static const int DC_RELAY_PIN = 51;
static const double EMON_CAL  = 111.1;

double toolThresholds[NUM_TOOLS] = {160.0, 260.0, 240.0, 300.0, 300.0};

// ── Timing ────────────────────────────────────────────────────────────────────
static const unsigned long DISPLAY_INTERVAL     = 250;   // ms
static const unsigned long TOOL_TRANSITION_TIME = 500;   // ms gate-open to DC-on
static const unsigned long TOUCH_DEBOUNCE_MS    = 80;    // ms
unsigned long              shutoffDelayMs        = 15000; // ms; configurable via web

// ── State machine ─────────────────────────────────────────────────────────────
enum SystemState {
    STARTUP, MONITORING, TOOL_ACTIVATING, TOOL_RUNNING, TOOL_DEACTIVATING, MANUAL_CONTROL
};
SystemState   currentState   = STARTUP;
unsigned long stateTimer     = 0;
unsigned long previousMillis = 0;

// ── Tool / gate state ─────────────────────────────────────────────────────────
bool   gateOpen[NUM_TOOLS]     = {};
double toolCurrents[NUM_TOOLS] = {};
int    lastManualIndex         = -1;

// ── EmonLib ───────────────────────────────────────────────────────────────────
EnergyMonitor sensors[NUM_TOOLS];

// ── Current history (ring buffer, 1 sample/sec, 120 sec per tool) ─────────────
#define HISTORY_SAMPLES 120
static float         currentHistory[NUM_TOOLS][HISTORY_SAMPLES] = {};
static int           historyHead  = 0;
static unsigned long lastHistoryMs = 0;

// ── Analytics / adaptive triggering ──────────────────────────────────────────
// toolThresholds[] is now a DELTA above idle baseline, not an absolute value.
// Requires hardware calibration — defaults are placeholders until sensors wired.
struct ToolProfile {
    float         baseline;     // EMA idle current (A)
    float         peakCurrent;  // peak since last activation
    float         timeToPeak;   // seconds from detect to peak
    unsigned long activationMs; // millis() at detection
};
static ToolProfile tools[NUM_TOOLS] = {};

// ── WiFi + web server ─────────────────────────────────────────────────────────
static WiFiServer webServer(80);
static bool       wifiReady = false;

// ── Display & touch ───────────────────────────────────────────────────────────
GigaDisplay_GFX          tft;
Arduino_GigaDisplayTouch touch;

static const int SCREEN_W = 800;
static const int SCREEN_H = 480;

static const uint16_t C_BG       = 0x1082;
static const uint16_t C_HEADER   = 0x000F;
static const uint16_t C_WHITE    = 0xFFFF;
static const uint16_t C_GATE_ON  = 0x0c47;
static const uint16_t C_GATE_OFF = 0x4228;
static const uint16_t C_DC_ON    = 0x0c47;
static const uint16_t C_DC_OFF   = 0xdb64;
static const uint16_t C_ACTIVE   = 0xFD20;
static const uint16_t C_DISABLED = 0x2104;
static const uint16_t C_STATUS   = 0x8db9;
static const uint16_t C_CURRENT  = 0x4cfc;

struct Rect { int16_t x, y, w, h; };

static const Rect GATE_BTN[NUM_TOOLS] = {
    { 14, 120, 184, 140},
    {210, 120, 184, 140},
    {406, 120, 184, 140},
    {602, 120, 184, 140},
    {  0,   0,   0,   0},
};
static const Rect DC_BTN = {275, 385, 250, 80};

// ── Flash persistence ─────────────────────────────────────────────────────────
static const uint32_t SETTINGS_MAGIC = 0xDC001001;
struct Settings {
    uint32_t magic;
    double   thresholds[NUM_TOOLS];
    uint32_t shutoffMs;
};

// ─────────────────────────────────────────────────────────────────────────────
// Forward declarations
// ─────────────────────────────────────────────────────────────────────────────
void drawFullUI();
void drawGateButton(int idx);
void drawDCButton();
void drawStatusBar();
void drawCurrentsRow();
void drawWifiStatus();
void handleTouch();
void mapTouch(uint16_t rawX, uint16_t rawY, int& tx, int& ty);
void handleButtonPress(int idx);
void dustOn();
void dustOff();
bool areAllGatesClosed();
void updateSensors();
void updateHistory();
void setupWifi();
void handleWebClient();
void serveIndexPage(WiFiClient& client);
void serveDataJson(WiFiClient& client);
void serveStatusJson(WiFiClient& client);
void handleSavePost(WiFiClient& client, const String& body);
void handleStartupState();
void handleMonitoringState();
void handleToolActivatingState(unsigned long now);
void handleToolRunningState();
void handleToolDeactivatingState(unsigned long now);
void loadSettings();
void saveSettings();

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    analogReadResolution(ADC_BITS);

    for (int i = 0; i < NUM_TOOLS; i++) {
        if (BLAST_GATE_PINS[i] >= 0) {
            pinMode(BLAST_GATE_PINS[i], OUTPUT);
            digitalWrite(BLAST_GATE_PINS[i], HIGH);
        }
    }
    pinMode(DC_RELAY_PIN, OUTPUT);
    digitalWrite(DC_RELAY_PIN, HIGH);

    for (int i = 0; i < NUM_TOOLS; i++) {
        sensors[i].current(A0 + i, EMON_CAL);
    }

    setupWifi();
    loadSettings();
    
    tft.begin();
    tft.setRotation(1);
    drawFullUI();
    touch.begin();

   
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();

    handleTouch();
    updateSensors();
    updateHistory();
    if (wifiReady) handleWebClient();

    if (now - previousMillis >= DISPLAY_INTERVAL) {
        drawStatusBar();
        drawCurrentsRow();
        previousMillis = now;
    }

    switch (currentState) {
        case STARTUP:            handleStartupState();              break;
        case MONITORING:         handleMonitoringState();           break;
        case TOOL_ACTIVATING:    handleToolActivatingState(now);    break;
        case TOOL_RUNNING:       handleToolRunningState();          break;
        case TOOL_DEACTIVATING:  handleToolDeactivatingState(now);  break;
        case MANUAL_CONTROL:     break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Flash persistence
// ─────────────────────────────────────────────────────────────────────────────
void loadSettings() {
    Settings s;
    size_t actual = 0;
    int rc = kv_get("/kv/dc_settings", &s, sizeof(s), &actual);
    if (rc == MBED_SUCCESS && actual == sizeof(s) && s.magic == SETTINGS_MAGIC) {
        for (int i = 0; i < NUM_TOOLS; i++) toolThresholds[i] = s.thresholds[i];
        shutoffDelayMs = s.shutoffMs;
        Serial.println("Settings loaded from flash.");
    } else {
        Serial.println("No saved settings — using defaults.");
    }
}

void saveSettings() {
    Settings s;
    s.magic = SETTINGS_MAGIC;
    for (int i = 0; i < NUM_TOOLS; i++) s.thresholds[i] = toolThresholds[i];
    s.shutoffMs = (uint32_t)shutoffDelayMs;
    int rc = kv_set("/kv/dc_settings", &s, sizeof(s), 0);
    if (rc != MBED_SUCCESS) {
        Serial.print("KVStore save failed: "); Serial.println(rc);
    } else {
        Serial.println("Settings saved to flash.");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// WiFi
// ─────────────────────────────────────────────────────────────────────────────
void setupWifi() {
    if (WiFi.status() == WL_NO_MODULE) {
        Serial.println("WiFi module not found — skipping WiFi.");
        return;
    }
    Serial.print("WiFi firmware: "); Serial.println(WiFi.firmwareVersion());

    delay(2000); // allow CYW4343W module to settle before begin()
    Serial.print("Connecting to "); Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long wifiDeadline = millis() + 30000UL; // 30s max, single begin()
    while (WiFi.status() != WL_CONNECTED && millis() < wifiDeadline) {
        delay(500);
        Serial.print(WiFi.status()); Serial.print(" ");
    }
    Serial.println();

    Serial.print("MAC: "); Serial.println(WiFi.macAddress());
    wifiReady = (WiFi.status() == WL_CONNECTED);
    if (wifiReady) {
        IPAddress ip = WiFi.localIP();
        Serial.print("WiFi connected. IP: "); Serial.println(ip);
        Serial.print("Web UI: http://"); Serial.print(ip); Serial.println("/");
        webServer.begin();
    } else {
        Serial.println("WiFi FAILED. Check credentials or move closer to AP.");
    }
}

void drawWifiStatus() {
    tft.fillRect(470, 4, 326, 47, C_HEADER);
    tft.setTextColor(C_STATUS);
    if (wifiReady) {
        IPAddress ip = WiFi.localIP();
        char buf[20];
        snprintf(buf, sizeof(buf), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
        tft.setTextSize(2);
        tft.setCursor(475, 15);
        tft.print("WiFi: ");
        tft.setTextSize(3);
        tft.setCursor(545, 15);
        tft.print(buf);
    } else {
        tft.setTextSize(2);
        tft.setCursor(475, 15);
        tft.print("WiFi: --");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Web server
// ─────────────────────────────────────────────────────────────────────────────
void handleWebClient() {
    WiFiClient client = webServer.accept();
    if (!client) return;

    unsigned long timeout = millis() + 300;
    while (!client.available() && millis() < timeout) { delay(1); }
    if (!client.available()) { client.stop(); return; }

    String requestLine = client.readStringUntil('\n');
    requestLine.trim();
    bool isPost = requestLine.startsWith("POST");

    int s1 = requestLine.indexOf(' ');
    int s2 = requestLine.indexOf(' ', s1 + 1);
    String url = (s1 >= 0 && s2 > s1) ? requestLine.substring(s1 + 1, s2) : "/";

    int contentLen = 0;
    while (client.connected()) {
        String hdr = client.readStringUntil('\n');
        hdr.trim();
        if (hdr.startsWith("Content-Length:")) contentLen = hdr.substring(15).toInt();
        if (hdr.isEmpty()) break;
    }

    if (url == "/") {
        serveIndexPage(client);
    } else if (url == "/api/data") {
        serveDataJson(client);
    } else if (url == "/api/status") {
        serveStatusJson(client);
    } else if (url == "/api/save" && isPost && contentLen > 0) {
        String body = "";
        unsigned long bt = millis() + 100;
        while ((int)body.length() < contentLen && millis() < bt) {
            if (client.available()) body += (char)client.read();
        }
        handleSavePost(client, body);
    } else {
        client.print("HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
    }

    client.flush();
    delay(1);
    client.stop();
}

void serveIndexPage(WiFiClient& client) {
    client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n");

    // Head + CSS
    client.print("<!DOCTYPE html><html><head>"
        "<meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Dust Collection</title>"
        "<style>"
        "*{box-sizing:border-box;margin:0;padding:0}"
        "body{background:#0d1117;color:#e6edf3;font-family:system-ui,sans-serif;padding:12px}"
        "h2{color:#58a6ff;margin-bottom:10px}"
        "h3{color:#79c0ff;font-size:1em;margin:6px 0}"
        ".card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:12px;margin:8px 0}"
        ".row{display:flex;flex-wrap:wrap;gap:8px}"
        ".tc{background:#21262d;border-radius:6px;padding:8px;min-width:130px;border:1px solid #30363d}"
        ".tc.ov{border-color:#f85149}"
        ".tn{font-size:.75em;color:#8b949e}"
        ".val{font-size:1.6em;font-weight:bold}"
        ".pk{font-size:.8em;color:#d29922;margin-top:2px}"
        "canvas{width:100%;height:200px;display:block;background:#0d1117}"
        "label{font-size:.8em;color:#8b949e;display:block;margin-bottom:3px}"
        "input[type=number]{background:#21262d;color:#e6edf3;border:1px solid #30363d;"
        "padding:5px 8px;border-radius:4px;width:90px}"
        "button{background:#238636;color:#fff;border:none;padding:8px 18px;"
        "border-radius:6px;cursor:pointer;margin-top:12px;font-size:.95em}"
        "button:hover{background:#2ea043}"
        "#msg{margin-left:10px;color:#3fb950;font-size:.9em}"
        ".row.form{align-items:flex-end;margin-top:8px;gap:16px}"
        "</style></head><body>");

    // HTML structure
    client.print("<h2>&#9889; Dust Collection System</h2>"
        "<div class='card'><h3>Live Readings</h3>"
        "<div class='row' id='live'><em style='color:#8b949e'>Loading...</em></div></div>"
        "<div class='card'><h3>Current History (2 min) &mdash; dashed = threshold</h3>"
        "<canvas id='c'></canvas></div>"
        "<div class='card'><h3>Settings</h3>"
        "<form id='f'>"
        "<div class='row form' id='tf'></div>"
        "<div style='margin-top:10px'>"
        "<label>Shutoff delay (seconds after tool turns off)</label>"
        "<input type='number' id='sd' min='1' max='300' step='1'>"
        "</div>"
        "<button type='submit'>Save Settings</button>"
        "<span id='msg'></span>"
        "</form></div>");

    // JavaScript — drawChart
    client.print("<script>\n"
        "var N=['CNC Router','Table Saw','Sander','Drill Press','Unused'];\n"
        "var C=['#f85149','#58a6ff','#3fb950','#d29922','#a371f7'];\n"
        "var ready=false;\n"
        "function drawChart(h,thr){\n"
        "  var cv=document.getElementById('c');\n"
        "  cv.width=cv.offsetWidth; cv.height=200;\n"
        "  var W=cv.width,H=cv.height,ctx=cv.getContext('2d');\n"
        "  ctx.fillStyle='#0d1117'; ctx.fillRect(0,0,W,H);\n"
        "  var n=h[0]?h[0].length:0; if(n<2)return;\n"
        "  var mx=0;\n"
        "  for(var i=0;i<h.length;i++) for(var j=0;j<h[i].length;j++) if(h[i][j]>mx) mx=h[i][j];\n"
        "  for(var i=0;i<thr.length;i++) if(thr[i]>mx) mx=thr[i];\n"
        "  if(!mx)return;\n"
        "  function py(v){return H-12-(v/mx)*(H-24);}\n"
        "  // grid\n"
        "  for(var f=1;f<=4;f++){\n"
        "    var y=py(mx*f/4);\n"
        "    ctx.strokeStyle='#30363d'; ctx.lineWidth=1;\n"
        "    ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke();\n"
        "    ctx.fillStyle='#8b949e'; ctx.font='10px system-ui';\n"
        "    ctx.fillText((mx*f/4).toFixed(0)+'A',2,y-2);\n"
        "  }\n");

    client.print(
        "  // threshold dashed lines\n"
        "  for(var i=0;i<thr.length;i++){\n"
        "    if(thr[i]<=0) continue;\n"
        "    ctx.strokeStyle=C[i]+'88'; ctx.lineWidth=1;\n"
        "    ctx.setLineDash([5,4]);\n"
        "    var y=py(thr[i]);\n"
        "    ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke();\n"
        "    ctx.setLineDash([]);\n"
        "  }\n"
        "  // history lines\n"
        "  for(var i=0;i<h.length;i++){\n"
        "    var hasData=false;\n"
        "    for(var j=0;j<h[i].length;j++) if(h[i][j]>0){hasData=true;break;}\n"
        "    if(!hasData) continue;\n"
        "    ctx.strokeStyle=C[i]; ctx.lineWidth=2;\n"
        "    ctx.beginPath();\n"
        "    for(var x=0;x<h[i].length;x++){\n"
        "      var px=(x/(n-1))*W, py2=py(h[i][x]);\n"
        "      if(x===0) ctx.moveTo(px,py2); else ctx.lineTo(px,py2);\n"
        "    }\n"
        "    ctx.stroke();\n"
        "  }\n"
        "  // legend\n"
        "  for(var i=0;i<N.length;i++){\n"
        "    ctx.fillStyle=C[i]; ctx.fillRect(10+i*110,4,12,12);\n"
        "    ctx.fillStyle='#e6edf3'; ctx.font='11px system-ui';\n"
        "    ctx.fillText(N[i],26+i*110,14);\n"
        "  }\n"
        "}\n");

    // JavaScript — live update
    client.print(
        "function updateLive(d){\n"
        "  var html='';\n"
        "  for(var i=0;i<d.tools.length;i++){\n"
        "    var t=d.tools[i];\n"
        "    var ov=t.current>t.threshold;\n"
        "    html+='<div class=\"tc'+(ov?' ov':'')+'\">';\n"
        "    html+='<div class=\"tn\">'+N[i]+'</div>';\n"
        "    html+='<div class=\"val\" style=\"color:'+(ov?'#f85149':'#58a6ff')+'\">'+t.current.toFixed(1)+'A</div>';\n"
        "    if(t.peak>0) html+='<div class=\"pk\">Peak '+t.peak.toFixed(1)+'A @ '+t.peakTime.toFixed(1)+'s</div>';\n"
        "    html+='<div class=\"pk\" style=\"color:#58a6ff\">Base '+t.baseline.toFixed(1)+'A</div>';\n"
        "    html+='</div>';\n"
        "  }\n"
        "  document.getElementById('live').innerHTML=html;\n"
        "  if(!ready){\n"
        "    ready=true;\n"
        "    var tf='';\n"
        "    for(var i=0;i<d.tools.length;i++){\n"
        "      tf+='<div><label>'+N[i]+' delta above idle (A)</label>';\n"
        "      tf+='<input type=\"number\" id=\"t'+i+'\" value=\"'+d.tools[i].threshold.toFixed(1)+'\" min=\"0\" max=\"500\" step=\"1\"></div>';\n"
        "    }\n"
        "    document.getElementById('tf').innerHTML=tf;\n"
        "    document.getElementById('sd').value=d.shutoffSecs;\n"
        "  }\n"
        "  var thresholds=[];\n"
        "  for(var i=0;i<d.tools.length;i++) thresholds.push(d.tools[i].threshold);\n"
        "  drawChart(d.history,thresholds);\n"
        "}\n");

    // JavaScript — poll + form submit
    client.print(
        "function poll(){\n"
        "  fetch('/api/data')\n"
        "    .then(function(r){return r.json();})\n"
        "    .then(updateLive)\n"
        "    .catch(function(){});\n"
        "}\n"
        "document.getElementById('f').addEventListener('submit',function(e){\n"
        "  e.preventDefault();\n"
        "  var v=[];\n"
        "  for(var i=0;i<5;i++){\n"
        "    var el=document.getElementById('t'+i);\n"
        "    v.push(el?parseFloat(el.value):0);\n"
        "  }\n"
        "  v.push(parseInt(document.getElementById('sd').value));\n"
        "  fetch('/api/save',{method:'POST',body:v.join(',')})\n"
        "    .then(function(){\n"
        "      var m=document.getElementById('msg');\n"
        "      m.textContent='Saved!';\n"
        "      setTimeout(function(){m.textContent='';},3000);\n"
        "      ready=false;\n"
        "    })\n"
        "    .catch(function(){document.getElementById('msg').textContent='Error';});\n"
        "});\n"
        "poll(); setInterval(poll,2000);\n"
        "</script></body></html>\n");
}

void serveDataJson(WiFiClient& client) {
    client.print("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
                 "Access-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n");
    client.print("{\"tools\":[");
    for (int i = 0; i < NUM_TOOLS; i++) {
        if (i > 0) client.print(",");
        client.print("{\"current\":");    client.print(toolCurrents[i], 2);
        client.print(",\"threshold\":");  client.print(toolThresholds[i], 1);
        client.print(",\"baseline\":");   client.print(tools[i].baseline, 2);
        client.print(",\"peak\":");       client.print(tools[i].peakCurrent, 2);
        client.print(",\"peakTime\":");   client.print(tools[i].timeToPeak, 2);
        client.print("}");
    }
    client.print("],\"history\":[");
    for (int i = 0; i < NUM_TOOLS; i++) {
        if (i > 0) client.print(",");
        client.print("[");
        for (int s = 0; s < HISTORY_SAMPLES; s++) {
            if (s > 0) client.print(",");
            client.print(currentHistory[i][(historyHead + s) % HISTORY_SAMPLES], 1);
        }
        client.print("]");
    }
    client.print("],\"shutoffSecs\":");
    client.print(shutoffDelayMs / 1000UL);
    client.print("}");
}

void serveStatusJson(WiFiClient& client) {
    static const char* STATE_NAMES[] = {
        "STARTUP","MONITORING","TOOL_ACTIVATING","TOOL_RUNNING","TOOL_DEACTIVATING","MANUAL_CONTROL"
    };
    client.print("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
                 "Access-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n");
    client.print("{\"state\":\""); client.print(STATE_NAMES[currentState]); client.print("\"");
    client.print(",\"shutoffSecs\":"); client.print(shutoffDelayMs / 1000UL);
    client.print(",\"tools\":[");
    for (int i = 0; i < NUM_TOOLS; i++) {
        if (i > 0) client.print(",");
        client.print("{\"name\":\"");    client.print(TOOL_NAMES[i]);
        client.print("\",\"current\":"); client.print(toolCurrents[i], 2);
        client.print(",\"threshold\":"); client.print(toolThresholds[i], 1);
        client.print(",\"baseline\":"); client.print(tools[i].baseline, 2);
        client.print(",\"gateOpen\":"); client.print(gateOpen[i] ? "true" : "false");
        client.print("}");
    }
    client.print("]}");
}

void handleSavePost(WiFiClient& client, const String& body) {
    // body: "thr0,thr1,thr2,thr3,thr4,shutoff_seconds"
    int field = 0, start = 0;
    for (int k = 0; k <= (int)body.length() && field < 6; k++) {
        if (k == (int)body.length() || body[k] == ',') {
            String token = body.substring(start, k);
            if (field < 5) toolThresholds[field] = token.toFloat();
            else           shutoffDelayMs = (unsigned long)token.toInt() * 1000UL;
            field++;
            start = k + 1;
        }
    }
    saveSettings();
    client.print("HTTP/1.1 200 OK\r\nContent-Length: 2\r\nConnection: close\r\n\r\nOK");
}

// ─────────────────────────────────────────────────────────────────────────────
// Display
// ─────────────────────────────────────────────────────────────────────────────
void drawFullUI() {
    tft.fillScreen(C_BG);
    tft.fillRect(0, 0, SCREEN_W, 55, C_HEADER);
    tft.setTextColor(C_WHITE);
    tft.setTextSize(3);
    tft.setCursor(12, 15);
    tft.print("Dust Collection Control");

    tft.setTextSize(2);
    tft.setCursor(12, 280);
    tft.print("Currents:");

    for (int i = 0; i < NUM_TOOLS - 1; i++) drawGateButton(i);
    drawDCButton();
    drawStatusBar();
    drawCurrentsRow();
    drawWifiStatus();
}

void drawGateButton(int idx) {
    const Rect& r = GATE_BTN[idx];
    bool isDrillPress = (idx == 3);
    bool dcOn = (digitalRead(DC_RELAY_PIN) == LOW);
    bool isActive = isDrillPress ? (dcOn && lastManualIndex == 3) : gateOpen[idx];
    uint16_t bg = (idx >= NUM_TOOLS - 1) ? C_DISABLED : isActive ? C_GATE_ON : C_GATE_OFF;

    tft.fillRoundRect(r.x, r.y, r.w, r.h, 10, bg);
    tft.drawRoundRect(r.x, r.y, r.w, r.h, 10, C_WHITE);
    tft.setTextSize(2);
    tft.setTextColor(C_WHITE);
    tft.setCursor(r.x + 16, r.y + 16);
    tft.print(TOOL_NAMES[idx]);
    tft.setTextSize(3);
    tft.setCursor(r.x + 20, r.y + 75);
    if (isDrillPress) tft.print(isActive ? "DC ON " : "DC OFF");
    else              tft.print(isActive ? "OPEN" : "SHUT");
}

void drawDCButton() {
    const Rect& r = DC_BTN;
    bool dcOn = (digitalRead(DC_RELAY_PIN) == LOW);
    tft.fillRoundRect(r.x, r.y, r.w, r.h, 12, dcOn ? C_DC_ON : C_DC_OFF);
    tft.drawRoundRect(r.x, r.y, r.w, r.h, 12, C_WHITE);
    tft.setTextColor(C_WHITE);
    tft.setTextSize(3);
    tft.setCursor(r.x + r.w / 2 - (dcOn ? 45 : 54), r.y + 28);
    tft.print(dcOn ? "DC ON" : "DC OFF");
}

void drawStatusBar() {
    tft.fillRect(0, 60, 470, 35, C_BG);
    tft.setTextSize(2);
    tft.setTextColor(C_STATUS);
    tft.setCursor(14, 72);
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
    tft.fillRect(0, 295, SCREEN_W, 55, C_BG);
    tft.setTextSize(4);
    for (int i = 0; i < 3; i++) {
        bool over = toolCurrents[i] > toolThresholds[i];
        tft.setTextColor(over ? C_ACTIVE : C_CURRENT);
        tft.setCursor(10 + i * 270, 315);
        tft.print("T"); tft.print(i + 1); tft.print(":");
        tft.print(toolCurrents[i], 1); tft.print("A");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Touch
// ─────────────────────────────────────────────────────────────────────────────
static bool          touchActive = false;
static unsigned long lastTouchMs = 0;

void mapTouch(uint16_t rawX, uint16_t rawY, int& tx, int& ty) {
    tx = (int)rawY;
    ty = (SCREEN_H - 1) - (int)rawX;
}

void handleTouch() {
    uint8_t    contacts;
    GDTpoint_t points[5];

    contacts = touch.getTouchPoints(points);
    if (contacts > 0) {
        if (!touchActive && (millis() - lastTouchMs >= TOUCH_DEBOUNCE_MS)) {
            touchActive = true;
            lastTouchMs = millis();

            int tx, ty;
            mapTouch(points[0].x, points[0].y, tx, ty);

            for (int i = 0; i < NUM_TOOLS - 1; i++) {
                const Rect& r = GATE_BTN[i];
                if (tx >= r.x && tx <= r.x + r.w &&
                    ty >= r.y && ty <= r.y + r.h) {
                    handleButtonPress(i);
                    return;
                }
            }

            if (tx >= DC_BTN.x && tx <= DC_BTN.x + DC_BTN.w &&
                ty >= DC_BTN.y && ty <= DC_BTN.y + DC_BTN.h) {
                bool dcOn = (digitalRead(DC_RELAY_PIN) == LOW);
                if (dcOn) {
                    for (int i = 0; i < NUM_TOOLS; i++) {
                        gateOpen[i] = false;
                        if (BLAST_GATE_PINS[i] >= 0) digitalWrite(BLAST_GATE_PINS[i], HIGH);
                    }
                    for (int i = 0; i < NUM_TOOLS - 1; i++) drawGateButton(i);
                    dustOff();
                    currentState    = MONITORING;
                    lastManualIndex = -1;
                } else {
                    dustOn();
                    currentState    = MANUAL_CONTROL;
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
    // Was this button already active?
    bool wasActive = (idx == 3)
        ? ((digitalRead(DC_RELAY_PIN) == LOW) && (lastManualIndex == 3))
        : gateOpen[idx];

    // Radio: close all gates atomically (DC relay unchanged here)
    for (int i = 0; i < NUM_TOOLS; i++) {
        gateOpen[i] = false;
        if (BLAST_GATE_PINS[i] >= 0) digitalWrite(BLAST_GATE_PINS[i], HIGH);
    }

    if (wasActive) {
        // Tapping active button turns everything off
        dustOff();
        currentState    = MONITORING;
        lastManualIndex = -1;
    } else {
        // Activate selected gate (idx 3 = Drill Press has no gate pin, DC only)
        if (BLAST_GATE_PINS[idx] >= 0) {
            gateOpen[idx] = true;
            digitalWrite(BLAST_GATE_PINS[idx], LOW);
        }
        dustOn();
        currentState    = MANUAL_CONTROL;
        lastManualIndex = idx;
    }

    for (int i = 0; i < NUM_TOOLS - 1; i++) drawGateButton(i);
}

bool areAllGatesClosed() {
    for (int i = 0; i < NUM_TOOLS; i++) if (gateOpen[i]) return false;
    return true;
}

void dustOn()  { digitalWrite(DC_RELAY_PIN, LOW);  drawDCButton(); }
void dustOff() { digitalWrite(DC_RELAY_PIN, HIGH); drawDCButton(); }

// ─────────────────────────────────────────────────────────────────────────────
// Sensor sampling + history
// ─────────────────────────────────────────────────────────────────────────────
void updateSensors() {
    static unsigned long lastSensorMs = 0;
    static int           sensorIndex  = 0;
    if (millis() - lastSensorMs < 100) return;
    lastSensorMs = millis();
    toolCurrents[sensorIndex] = sensors[sensorIndex].calcIrms(100);
    sensorIndex = (sensorIndex + 1) % NUM_TOOLS;
}

void updateHistory() {
    unsigned long now = millis();
    if (now - lastHistoryMs < 1000) return;
    lastHistoryMs = now;
    for (int i = 0; i < NUM_TOOLS; i++) {
        currentHistory[i][historyHead] = (float)toolCurrents[i];
    }
    historyHead = (historyHead + 1) % HISTORY_SAMPLES;
}

// ─────────────────────────────────────────────────────────────────────────────
// State machine handlers
// ─────────────────────────────────────────────────────────────────────────────
void handleStartupState() {
    // Pre-warm baseline with fast EMA (α=0.1) before entering MONITORING.
    // 30 samples at 100ms each = 3s → baseline reaches ~96% of true idle.
    // Prevents delta detection firing against a near-zero baseline on first entry.
    static unsigned long startMs = millis();
    for (int i = 0; i < NUM_TOOLS; i++) {
        tools[i].baseline = tools[i].baseline * 0.9f + (float)toolCurrents[i] * 0.1f;
    }
    if (millis() - startMs >= 3000UL) {
        currentState = MONITORING;
    }
}

void handleMonitoringState() {
    for (int i = 0; i < NUM_TOOLS; i++) {
        tools[i].baseline = tools[i].baseline * 0.995f + (float)toolCurrents[i] * 0.005f;

        if (toolCurrents[i] > tools[i].baseline + toolThresholds[i]) {
            gateOpen[i] = true;
            if (BLAST_GATE_PINS[i] >= 0) digitalWrite(BLAST_GATE_PINS[i], LOW);
            if (i < NUM_TOOLS - 1) drawGateButton(i);

            tools[i].activationMs = millis();
            tools[i].peakCurrent  = (float)toolCurrents[i];
            tools[i].timeToPeak   = 0.0f;

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
        if (tools[i].activationMs > 0 && (float)toolCurrents[i] > tools[i].peakCurrent) {
            tools[i].peakCurrent = (float)toolCurrents[i];
            tools[i].timeToPeak  = (millis() - tools[i].activationMs) / 1000.0f;
        }
    }

    // Tool off when current drops within 50% of delta threshold above baseline
    for (int i = 0; i < NUM_TOOLS; i++) {
        if (toolCurrents[i] > tools[i].baseline + (toolThresholds[i] * 0.5)) return;
    }
    currentState = TOOL_DEACTIVATING;
    stateTimer   = millis();
}

void handleToolDeactivatingState(unsigned long now) {
    for (int i = 0; i < NUM_TOOLS; i++) {
        if (toolCurrents[i] > tools[i].baseline + (toolThresholds[i] * 0.5)) {
            currentState = TOOL_RUNNING;
            return;
        }
    }
    if (now - stateTimer >= shutoffDelayMs) {
        dustOff();
        for (int i = 0; i < NUM_TOOLS; i++) {
            gateOpen[i] = false;
            if (BLAST_GATE_PINS[i] >= 0) digitalWrite(BLAST_GATE_PINS[i], HIGH);
            tools[i].activationMs = 0;
        }
        for (int i = 0; i < NUM_TOOLS - 1; i++) drawGateButton(i);
        currentState = MONITORING;
    }
}
