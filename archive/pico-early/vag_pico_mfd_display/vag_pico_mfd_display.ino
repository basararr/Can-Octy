#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <mcp2515.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// ==============================================================================
// VAG PICO W - MFD DISPLAY & SD LOGGER
// Architecture: Raspberry Pi Pico 2 W (RP2350) - Arduino Earlephilhower Core
// Dual Core: Core 1 (CAN Bus SPI0), Core 0 (WiFi, WebSockets, SD Logging SPI1)
// ==============================================================================

// --- HARDWARE PINS ---
// CAN BUS (SPI0)
#define PIN_CAN_MISO 16
#define PIN_CAN_CS   17
#define PIN_CAN_SCK  18
#define PIN_CAN_MOSI 19
#define PIN_CAN_INT  20

// SD CARD (SPI1)
#define PIN_SD_MISO  12
#define PIN_SD_CS    13
#define PIN_SD_SCK   14
#define PIN_SD_MOSI  15

// --- NETWORK CONFIG ---
#include "secrets.h"

WebServer server(80);
WebSocketsServer webSocket(81);

// --- SHARED DATA (CORE 1 -> CORE 0) ---
volatile float g_boost = 0.0f;
volatile float g_coolant = 0.0f;
volatile float g_oil = 0.0f;
volatile float g_egtPre = 0.0f;
volatile float g_egtPost = 0.0f;
volatile float g_soot = 0.0f;
volatile float g_dp = 0.0f;
volatile uint32_t g_km = 0;
volatile float g_battery = 0.0f;
volatile float g_rpm = 0.0f;

volatile unsigned long lastCanRxTime = 0;
volatile bool isSleeping = false; // POWER SAVING FLAG

// --- SD LOGGING STATE ---
File logFile;
bool isLogging = false;
unsigned long lastLogWrite = 0;
int logFileIndex = 0;

// --- HTML / CSS / JS PAYLOAD ---
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Gökay VAG MFD</title>
    <style>
        :root {
            --bg-color: #121212;
            --card-bg: #1e1e1e;
            --text-main: #ffffff;
            --text-dim: #aaaaaa;
            --accent-warn: #ff3d00;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            background-color: var(--bg-color);
            color: var(--text-main);
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
            overflow-x: hidden;
        }
        header {
            background-color: #000;
            padding: 15px 20px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 2px solid #333;
        }
        header h1 { font-size: 1.2rem; font-weight: 700; margin:0;}
        .status-dot {
            height: 12px; width: 12px;
            background-color: var(--accent-warn);
            border-radius: 50%;
            box-shadow: 0 0 8px var(--accent-warn);
        }
        .status-dot.connected { background-color: #00e676; box-shadow: 0 0 8px #00e676; }
        .container { padding: 15px; }
        .boost-card {
            background-color: var(--card-bg);
            border-radius: 12px;
            padding: 20px;
            margin-bottom: 15px;
            text-align: center;
            box-shadow: 0 4px 10px rgba(0,0,0,0.5);
        }
        .boost-card h2 { font-size: 1rem; color: var(--text-dim); text-transform: uppercase; margin-bottom: 5px;}
        .boost-val { font-size: 4rem; font-weight: 800; margin-bottom: 15px; font-variant-numeric: tabular-nums; }
        .boost-val span { font-size: 1.5rem; font-weight: normal; color: var(--text-dim); }
        .bar-container {
            width: 100%; height: 28px;
            background-color: #333;
            border-radius: 14px;
            position: relative;
            overflow: hidden;
        }
        .bar-zero-line { position: absolute; left: 25%; top: 0; bottom: 0; width: 2px; background-color: #fff; z-index: 10;}
        .bar-fill-boost { position: absolute; left: 25%; top: 0; bottom: 0; background: linear-gradient(90deg, #00b4db, #0083b0); width: 0%; transition: width 0.05s linear; }
        .bar-fill-vac { position: absolute; right: 75%; top: 0; bottom: 0; background: #ff9800; width: 0%; transition: width 0.05s linear; }
        .grid-2col { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; }
        .data-card {
            background-color: var(--card-bg);
            border-radius: 12px;
            padding: 15px;
            box-shadow: 0 4px 10px rgba(0,0,0,0.5);
            display: flex; flex-direction: column; justify-content: space-between;
        }
        .data-card .label { font-size: 0.85rem; color: var(--text-dim); text-transform: uppercase; margin-bottom: 8px;}
        .data-card .value { font-size: 1.8rem; font-weight: 700; font-variant-numeric: tabular-nums; }
        .data-card .value span { font-size: 1rem; font-weight: normal; color: var(--text-dim); }
        .warn { color: var(--accent-warn); }
    </style>
</head>
<body>
    <header>
        <h1>◆ Gökay VAG MFD</h1>
        <div>
            <span id="log-status" style="margin-right:10px; font-size:0.9rem; color:#aaa;">SD: BEKLIYOR</span>
            <div id="conn-status" class="status-dot"></div>
        </div>
    </header>
    <div class="container">
        <div class="boost-card">
            <h2>Turbo Basıncı</h2>
            <div id="boost-val" class="boost-val">0.00 <span>bar</span></div>
            <div class="bar-container">
                <div class="bar-zero-line"></div>
                <div id="bar-boost" class="bar-fill-boost"></div>
                <div id="bar-vac" class="bar-fill-vac"></div>
            </div>
        </div>
        <div class="grid-2col">
            <div class="data-card"><div class="label">Su Sıcaklığı</div><div id="coolant-val" class="value">--<span>°C</span></div></div>
            <div class="data-card"><div class="label">Yağ Sıcaklığı</div><div id="oil-val" class="value">--<span>°C</span></div></div>
            <div class="data-card"><div class="label">EGT Ön-DPF</div><div id="egt-pre-val" class="value">--<span>°C</span></div></div>
            <div class="data-card"><div class="label">EGT Son-DPF</div><div id="egt-post-val" class="value">--<span>°C</span></div></div>
            <div class="data-card"><div class="label">Kurum Miktarı</div><div id="soot-val" class="value">--<span>g</span></div></div>
            <div class="data-card"><div class="label">Fark Basıncı</div><div id="dp-val" class="value">--<span>mb</span></div></div>
            <div class="data-card"><div class="label">Motor Devri</div><div id="rpm-val" class="value">--<span>rpm</span></div></div>
            <div class="data-card"><div class="label">Akü Voltajı</div><div id="bat-val" class="value">--<span>V</span></div></div>
        </div>
    </div>
    <script>
        let ws;
        const sDot = document.getElementById('conn-status');
        const lStatus = document.getElementById('log-status');
        const ids = ['boost-val','bar-boost','bar-vac','coolant-val','oil-val','egt-pre-val','egt-post-val','soot-val','dp-val','rpm-val','bat-val'];
        const el = {}; ids.forEach(id => el[id] = document.getElementById(id));
        function connect() {
            ws = new WebSocket('ws://' + window.location.hostname + ':81/');
            ws.onopen = () => sDot.classList.add('connected');
            ws.onclose = () => { sDot.classList.remove('connected'); setTimeout(connect, 1000); };
            ws.onmessage = (e) => {
                try {
                    const d = JSON.parse(e.data);
                    lStatus.innerText = d.log ? "SD: KAYDEDILIYOR" : "SD: BEKLIYOR";
                    lStatus.style.color = d.log ? "#ff3d00" : "#aaa";
                    const b = parseFloat(d.b);
                    el['boost-val'].innerHTML = (b > 0 ? "+" : "") + b.toFixed(2) + ' <span>bar</span>';
                    if (b >= 0) {
                        let w = (b / 1.8) * 75; el['bar-boost'].style.width = Math.min(w, 75) + '%'; el['bar-vac'].style.width = '0%';
                    } else {
                        let w = (Math.abs(b) / 0.6) * 25; el['bar-vac'].style.width = Math.min(w, 25) + '%'; el['bar-boost'].style.width = '0%';
                    }
                    el['coolant-val'].innerHTML = Math.round(d.c) + '<span>°C</span>';
                    el['oil-val'].innerHTML = Math.round(d.o) + '<span>°C</span>';
                    el['egt-pre-val'].innerHTML = Math.round(d.ep) + '<span>°C</span>';
                    el['egt-post-val'].innerHTML = Math.round(d.es) + '<span>°C</span>';
                    el['soot-val'].innerHTML = d.s.toFixed(2) + '<span>g</span>';
                    el['dp-val'].innerHTML = d.dp.toFixed(1) + '<span>mb</span>';
                    el['rpm-val'].innerHTML = Math.round(d.rpm) + '<span>rpm</span>';
                    el['bat-val'].innerHTML = d.bat.toFixed(1) + '<span>V</span>';
                } catch(err) {}
            };
        }
        connect();
    </script>
</body>
</html>
)rawliteral";

// ==============================================================================
// CORE 1: CAN BUS & REAL-TIME POLLING
// ==============================================================================
MCP2515 can(PIN_CAN_CS);

// Pre-defined UDS DIDs (Same as Arduino Nano but faster polling)
const uint16_t DIDs[] = {0xF405, 0x11BE, 0x181A, 0x105E, 0x114F, 0x14F5, 0x11B2, 0x10F9, 0xF40C /* RPM */};
const int NUM_DIDS = 9;

void setup1() {
  // Setup SPI0 for CAN
  SPI.setMISO(PIN_CAN_MISO);
  SPI.setCS(PIN_CAN_CS);
  SPI.setSCK(PIN_CAN_SCK);
  SPI.setMOSI(PIN_CAN_MOSI);
  SPI.begin();
  
  can.reset();
  can.setBitrate(CAN_500KBPS, MCP_8MHZ);
  can.setNormalMode();
}

void loop1() {
  if (isSleeping) {
    // POWER SAVING MODE: Stop polling. Just listen passively for any CAN traffic (Car waking up).
    struct can_frame rx;
    if (can.readMessage(&rx) == MCP2515::ERROR_OK) {
      // Traffic detected! The car woke up. Reboot the Pico to start fresh.
      rp2040.reboot();
    }
    delay(100); // Check 10 times a second to save power
    return; // Skip normal polling
  }

  struct can_frame frame;
  struct can_frame rx;
  
  for(int i = 0; i < NUM_DIDS; i++) {
    uint16_t did = DIDs[i];
    
    // Clear RX Buffer
    while(can.readMessage(&rx) == MCP2515::ERROR_OK) {}
    
    frame.can_id = 0x7E0;
    frame.can_dlc = 8;
    frame.data[0] = 0x03;
    frame.data[1] = 0x22;
    frame.data[2] = (did >> 8) & 0xFF;
    frame.data[3] = did & 0xFF;
    for(int j=4; j<8; j++) frame.data[j] = 0x55;
    
    can.sendMessage(&frame);
    
    unsigned long startWait = millis();
    bool received = false;
    while(millis() - startWait < 50) { // 50ms timeout per request
      if (can.readMessage(&rx) == MCP2515::ERROR_OK && rx.can_id == 0x7E8) {
        if (rx.data[1] == 0x62 && rx.data[2] == frame.data[2] && rx.data[3] == frame.data[3]) {
          // Parse values
          uint32_t raw = 0;
          if (did == 0x1156 || did == 0x115E || did == 0x16A9 || did == 0x1347) {
            raw = (rx.data[4] << 24) | (rx.data[5] << 16) | (rx.data[6] << 8) | rx.data[7];
          } else {
            raw = (rx.data[4] << 8) | rx.data[5];
          }
          
          switch(did) {
            case 0x181A: g_boost = (raw * 0.25f - 1000.0f) / 1000.0f; break; // Boost Rel
            case 0xF405: g_coolant = (float)(rx.data[4] - 40); break; // Coolant OBD
            case 0x11BE: g_oil = raw / 10.0f - 273.1f; break; // Oil
            case 0x11B2: g_egtPre = raw / 10.0f - 273.1f; break;
            case 0x10F9: g_egtPost = raw / 10.0f - 273.1f; break;
            case 0x114F: g_soot = raw / 100.0f; break;
            case 0x14F5: g_dp = raw / 10.0f; break;
            case 0x105E: g_battery = raw / 50.0f; break;
            case 0xF40C: g_rpm = raw / 4.0f; break; // Engine RPM OBD
          }
          lastCanRxTime = millis();
          received = true;
          break;
        }
      }
    }
  }
}

// ==============================================================================
// CORE 0: NETWORKING & SD LOGGING
// ==============================================================================
void handleRoot() { server.send(200, "text/html", INDEX_HTML); }

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Setup SPI1 for SD Card
  SPI1.setMISO(PIN_SD_MISO);
  SPI1.setCS(PIN_SD_CS);
  SPI1.setSCK(PIN_SD_SCK);
  SPI1.setMOSI(PIN_SD_MOSI);
  
  if (SD.begin(PIN_SD_CS, SPI1)) {
    Serial.println("SD Card Initialized.");
  } else {
    Serial.println("SD Card Failed.");
  }
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  server.on("/", handleRoot);
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

unsigned long lastWsUpdate = 0;

void loop() {
  if (isSleeping) {
    delay(1000); // Core 0 sleeps, waiting for Core 1 to reboot the device.
    return;
  }

  server.handleClient();
  webSocket.loop();
  
  unsigned long now = millis();
  
  // --- DEEP SLEEP CHECK (5 Minutes of Inactivity) ---
  if (lastCanRxTime > 0 && (now - lastCanRxTime > 300000)) { // 300,000 ms = 5 mins
    isSleeping = true;
    if (isLogging) {
      logFile.close();
      isLogging = false;
    }
    webSocket.close();
    server.close();
    WiFi.mode(WIFI_OFF);
    Serial.println("ENTERING DEEP SLEEP. WiFi OFF.");
    return;
  }
  
  // --- SD LOGGING LOGIC (Start on CAN Rx, Stop on Timeout) ---
  bool hasCanActivity = (now - lastCanRxTime < 2000);
  
  if (hasCanActivity && !isLogging) {
    // Start Logging
    char filename[32];
    sprintf(filename, "/log_%d.csv", logFileIndex++);
    while (SD.exists(filename)) {
      sprintf(filename, "/log_%d.csv", logFileIndex++);
    }
    logFile = SD.open(filename, FILE_WRITE);
    if (logFile) {
      logFile.println("TimeMs,Boost_bar,Coolant_C,Oil_C,EGTPre_C,EGTPost_C,Soot_g,dP_mb,RPM,Battery_V");
      isLogging = true;
      Serial.print("Started logging to: ");
      Serial.println(filename);
    }
  } else if (!hasCanActivity && isLogging && (now - lastCanRxTime > 3000)) {
    // Stop Logging after 3s of no CAN activity
    logFile.close();
    isLogging = false;
    Serial.println("Stopped logging due to CAN timeout.");
  }
  
  // Log Data every 100ms (10Hz) if logging is active
  if (isLogging && (now - lastLogWrite >= 100)) {
    lastLogWrite = now;
    if (logFile) {
      logFile.printf("%lu,%.2f,%.0f,%.0f,%.0f,%.0f,%.2f,%.1f,%.0f,%.2f\n",
        now, g_boost, g_coolant, g_oil, g_egtPre, g_egtPost, g_soot, g_dp, g_rpm, g_battery);
      logFile.flush();
    }
  }
  
  // --- WEBSOCKETS BROADCAST (60Hz) ---
  if (now - lastWsUpdate >= 16) {
    lastWsUpdate = now;
    char jsonBuf[128];
    snprintf(jsonBuf, sizeof(jsonBuf), 
      "{\"b\":%.2f,\"c\":%.0f,\"o\":%.0f,\"ep\":%.0f,\"es\":%.0f,\"s\":%.2f,\"dp\":%.1f,\"rpm\":%.0f,\"bat\":%.1f,\"log\":%s}",
      g_boost, g_coolant, g_oil, g_egtPre, g_egtPost, g_soot, g_dp, g_rpm, g_battery, isLogging ? "true" : "false"
    );
    webSocket.broadcastTXT(jsonBuf);
  }
}
