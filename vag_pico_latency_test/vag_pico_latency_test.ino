#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <math.h>

// ==============================================================================
// VAG PICO W - UI LATENCY TEST (HEADLESS DASHBOARD)
// Architecture: Raspberry Pi Pico 2 W (RP2350) - Arduino Earlephilhower Core
// Goal: 60FPS WebSockets JSON Telemetry to iPhone SE 2022 over Wi-Fi AP
// ==============================================================================

// --- NETWORK CONFIG ---
#include "secrets.h"

WebServer server(80);
WebSocketsServer webSocket(81);

// --- SHARED DATA (CORE 1 -> CORE 0) ---
// volatile is used but we'll also rely on natural 32-bit alignment atomic reads
volatile float mockBoost = -0.5f;
volatile float mockCoolant = 90.0f;
volatile float mockOil = 95.0f;
volatile float mockEgtPre = 280.0f;
volatile float mockEgtPost = 310.0f;
volatile float mockSoot = 14.5f;
volatile float mockDp = 3.4f;

// --- HTML / CSS / JS PAYLOAD (SERVED TO IPHONE) ---
// This is the complete Web App served from the Pico's memory.
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Gökay MFD</title>
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
            -webkit-font-smoothing: antialiased;
        }
        
        /* Header */
        header {
            background-color: #000;
            padding: 15px 20px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 2px solid #333;
        }
        header h1 { font-size: 1.2rem; font-weight: 700; letter-spacing: 1px; color: #fff; margin:0;}
        .status-dot {
            height: 12px; width: 12px;
            background-color: var(--accent-warn);
            border-radius: 50%;
            display: inline-block;
            box-shadow: 0 0 8px var(--accent-warn);
            transition: background-color 0.3s;
        }
        .status-dot.connected {
            background-color: #00e676;
            box-shadow: 0 0 8px #00e676;
        }
        
        /* Main Container */
        .container { padding: 15px; }

        /* Boost Section (Full Width Top) */
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
        .bar-zero-line {
            position: absolute; left: 25%; top: 0; bottom: 0; width: 2px;
            background-color: #fff; z-index: 10;
        }
        .bar-fill-boost {
            position: absolute; left: 25%; top: 0; bottom: 0; width: 75%;
            background: #ffffff;
            transform-origin: left center; transform: scaleX(0); 
            will-change: transform; 
        }
        .bar-fill-vac {
            position: absolute; right: 75%; top: 0; bottom: 0; width: 25%;
            background: #ffffff;
            transform-origin: right center; transform: scaleX(0); 
            will-change: transform; 
        }

        /* 2-Column Grid */
        .grid-2col {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
        }
        .data-card {
            background-color: var(--card-bg);
            border-radius: 12px;
            padding: 15px;
            box-shadow: 0 4px 10px rgba(0,0,0,0.5);
            display: flex;
            flex-direction: column;
            justify-content: space-between;
        }
        .data-card .label { font-size: 0.85rem; color: var(--text-dim); text-transform: uppercase; margin-bottom: 8px;}
        .data-card .value { font-size: 1.8rem; font-weight: 700; font-variant-numeric: tabular-nums; }
        .data-card .value span { font-size: 1rem; font-weight: normal; color: var(--text-dim); }
        
        .warn { color: var(--accent-warn); }
    </style>
</head>
<body>
    <header>
        <h1>◆ Gökay MFD</h1>
        <div id="conn-status" class="status-dot"></div>
    </header>
    
    <div class="container">
        <!-- TOP: BOOST -->
        <div class="boost-card">
            <h2>Turbo Basıncı</h2>
            <div id="boost-val" class="boost-val">+0.00 <span>bar</span></div>
            <div class="bar-container">
                <div class="bar-zero-line"></div>
                <div id="bar-boost" class="bar-fill-boost"></div>
                <div id="bar-vac" class="bar-fill-vac"></div>
            </div>
        </div>

        <!-- GRID -->
        <div class="grid-2col">
            <div class="data-card">
                <div class="label">Su Sıcaklığı</div>
                <div id="coolant-val" class="value">--<span>°C</span></div>
            </div>
            <div class="data-card">
                <div class="label">Yağ Sıcaklığı</div>
                <div id="oil-val" class="value">--<span>°C</span></div>
            </div>
            
            <div class="data-card">
                <div class="label">EGT Ön-DPF</div>
                <div id="egt-pre-val" class="value">--<span>°C</span></div>
            </div>
            <div class="data-card">
                <div class="label">EGT Son-DPF</div>
                <div id="egt-post-val" class="value">--<span>°C</span></div>
            </div>
            
            <div class="data-card">
                <div class="label">Kurum Miktarı</div>
                <div id="soot-val" class="value">--<span>g</span></div>
            </div>
            <div class="data-card">
                <div class="label">Fark Basıncı</div>
                <div id="dp-val" class="value">--<span>mb</span></div>
            </div>
        </div>
    </div>

    <script>

        // --- RENDER LOOP (BUTTERY SMOOTH GAUGE) ---
        let targetBoost = 0;
        let currentBoost = 0;
        const elBarBoost = document.getElementById('bar-boost');
        const elBarVac = document.getElementById('bar-vac');

        function renderLoop() {
            // Lerp (Linear Interpolation) - Moves currentBoost 15% towards targetBoost every frame
            currentBoost += (targetBoost - currentBoost) * 0.15;
            
            if(currentBoost >= 0) {
                let scale = currentBoost / 1.8;
                if (scale > 1.0) scale = 1.0;
                elBarBoost.style.transform = 'scaleX(' + scale + ')';
                elBarVac.style.transform = 'scaleX(0)';
            } else {
                let scale = Math.abs(currentBoost) / 0.6;
                if (scale > 1.0) scale = 1.0;
                elBarVac.style.transform = 'scaleX(' + scale + ')';
                elBarBoost.style.transform = 'scaleX(0)';
            }
            
            requestAnimationFrame(renderLoop);
        }
        // Start rendering loop based on screen refresh rate (60/120Hz)
        requestAnimationFrame(renderLoop);

        // --- WEBSOCKETS ---
        let ws;
        const statusDot = document.getElementById('conn-status');
        
        const elBoost = document.getElementById('boost-val');
        const elCoolant = document.getElementById('coolant-val');
        const elOil = document.getElementById('oil-val');
        const elEgtPre = document.getElementById('egt-pre-val');
        const elEgtPost = document.getElementById('egt-post-val');
        const elSoot = document.getElementById('soot-val');
        const elDp = document.getElementById('dp-val');

        function connectWebSocket() {
            ws = new WebSocket('ws://' + window.location.hostname + ':81/');
            ws.onopen = () => { statusDot.classList.add('connected'); };
            ws.onclose = () => { 
                statusDot.classList.remove('connected'); 
                setTimeout(connectWebSocket, 1000); 
            };
            
            ws.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    
                    const boost = parseFloat(data.b);
                    elBoost.innerHTML = (boost > 0 ? "+" : "") + boost.toFixed(2) + ' <span>bar</span>';
                    
                    // Update target for the render loop instead of setting DOM directly
                    targetBoost = boost;

                    elCoolant.innerHTML = Math.round(data.c) + '<span>°C</span>';
                    elOil.innerHTML = Math.round(data.o) + '<span>°C</span>';
                    elEgtPre.innerHTML = Math.round(data.ep) + '<span>°C</span>';
                    elEgtPost.innerHTML = Math.round(data.es) + '<span>°C</span>';
                    
                    elSoot.innerHTML = data.s.toFixed(2) + '<span>g</span>';
                    elDp.innerHTML = data.dp.toFixed(1) + '<span>mb</span>';

                    elCoolant.className = (data.c > 110) ? "value warn" : "value";
                    elOil.className = (data.o > 130) ? "value warn" : "value";

                } catch(e) {}
            };
        }
        connectWebSocket();
    </script>
</body>
</html>
)rawliteral";

// ==============================================================================
// CORE 1: REAL-TIME MOCK DATA GENERATION
// ==============================================================================
void setup1() {
  // Setup logic for Core 1 (nothing for mock test)
}

void loop1() {
  unsigned long ms = millis();
  
  // Sine wave for boost (-0.6 to +1.8 bar)
  float sine = sin(ms / 477.0f);
  mockBoost = (sine * 1.2f) + 0.6f;
  
  mockCoolant = 90.0f + sin(ms / 2000.0f) * 5.0f;
  mockOil = 95.0f + cos(ms / 3000.0f) * 10.0f;
  mockEgtPre = 280.0f + sin(ms / 500.0f) * 30.0f;
  mockEgtPost = mockEgtPre + 40.0f;
  mockSoot = 14.5f + (ms % 10000) / 10000.0f;
  mockDp = 3.4f + sin(ms / 300.0f) * 1.5f;
  
  delay(1); 
}

// ==============================================================================
// CORE 0: NETWORKING & WEBSOCKETS
// ==============================================================================

void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %s\n", num, ip.toString().c_str());
      }
      break;
    case WStype_TEXT:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Starting VAG MFD UI Latency Test...");
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  
  server.on("/", handleRoot);
  server.begin();
  
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

unsigned long lastUpdate = 0;

void loop() {
  server.handleClient();
  webSocket.loop();
  
  unsigned long now = millis();
  // Limit websocket transmission to 30Hz (33ms) to avoid filling Pico W TCP buffer
  if (now - lastUpdate > 33) {
    lastUpdate = now;
    
    char jsonBuf[128];
    snprintf(jsonBuf, sizeof(jsonBuf), 
      "{\"b\":%.2f,\"c\":%.0f,\"o\":%.0f,\"ep\":%.0f,\"es\":%.0f,\"s\":%.2f,\"dp\":%.1f}",
      mockBoost, mockCoolant, mockOil, mockEgtPre, mockEgtPost, mockSoot, mockDp
    );
    webSocket.broadcastTXT(jsonBuf);
  }
}
