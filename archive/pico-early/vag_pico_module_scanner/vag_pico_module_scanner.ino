#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// ==============================================================================
// VAG PICO W - MODULE SCANNER (WEB UI)
// ==============================================================================

#define PIN_CAN_MISO 16
#define PIN_CAN_CS   17
#define PIN_CAN_SCK  18
#define PIN_CAN_MOSI 19

#include "secrets.h"

WebServer server(80);
WebSocketsServer webSocket(81);
MCP2515 can(PIN_CAN_CS);

bool triggerScan = false;

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Gateway Tarayıcı</title>
    <style>
        body { background: #121212; color: #fff; font-family: sans-serif; padding: 20px; }
        button { padding: 15px 30px; font-size: 18px; background: #00e676; border: none; border-radius: 8px; cursor: pointer; color: #000; font-weight: bold; width: 100%; margin-bottom: 20px; }
        #term { background: #000; color: #0f0; padding: 15px; border-radius: 8px; font-family: monospace; height: 300px; overflow-y: auto; white-space: pre-wrap; font-size: 14px; }
    </style>
</head>
<body>
    <h2>VAG Module Scanner (Web)</h2>
    <button onclick="start()">Taramayı Başlat (0x710)</button>
    <div id="term">Bağlantı bekleniyor...\n</div>
    <script>
        let ws = new WebSocket('ws://' + window.location.hostname + ':81/');
        const term = document.getElementById('term');
        
        ws.onopen = () => term.innerText += "Hazır. Taramak için butona basın.\n";
        ws.onmessage = (e) => { term.innerText += e.data + "\n"; term.scrollTop = term.scrollHeight; };
        
        function start() {
            if (ws.readyState === WebSocket.OPEN) {
                ws.send("START");
                term.innerText += "\n--- TARAMA BAŞLATILDI ---\n";
            }
        }
    </script>
</body>
</html>
)rawliteral";

void handleRoot() { server.send(200, "text/html", INDEX_HTML); }

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT && length > 0) {
    if (strncmp((char*)payload, "START", 5) == 0) {
      triggerScan = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  SPI.setMISO(PIN_CAN_MISO);
  SPI.setCS(PIN_CAN_CS);
  SPI.setSCK(PIN_CAN_SCK);
  SPI.setMOSI(PIN_CAN_MOSI);
  SPI.begin();
  
  can.reset();
  can.setBitrate(CAN_500KBPS, MCP_8MHZ);
  can.setNormalMode();
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  server.on("/", handleRoot);
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  server.handleClient();
  webSocket.loop();
  
  if (triggerScan) {
    triggerScan = false;
    
    struct can_frame frame;
    struct can_frame rx;
    
    // Clear buffer
    while(can.readMessage(&rx) == MCP2515::ERROR_OK) {}
    
    webSocket.broadcastTXT("[+] Opening Extended Session (0x10 0x03)...");
    frame.can_id = 0x710;
    frame.can_dlc = 8;
    frame.data[0] = 0x02; frame.data[1] = 0x10; frame.data[2] = 0x03;
    for(int i=3; i<8; i++) frame.data[i] = 0x55;
    can.sendMessage(&frame);
    delay(50);
    
    webSocket.broadcastTXT("[+] Requesting Installation List (0x04A0)...");
    frame.data[0] = 0x03; frame.data[1] = 0x22; frame.data[2] = 0x04; frame.data[3] = 0xA0;
    can.sendMessage(&frame);
    
    unsigned long startWait = millis();
    bool found = false;
    
    while(millis() - startWait < 500) {
      if (can.readMessage(&rx) == MCP2515::ERROR_OK && rx.can_id == 0x77A) {
        found = true;
        char buf[64];
        sprintf(buf, "[RAW 0x77A] %02X %02X %02X %02X %02X %02X %02X %02X", 
          rx.data[0], rx.data[1], rx.data[2], rx.data[3], rx.data[4], rx.data[5], rx.data[6], rx.data[7]);
        webSocket.broadcastTXT(buf);
      }
    }
    
    if (!found) {
      webSocket.broadcastTXT("[-] HATA: Gateway'den cevap alınamadı. Kontak açık mı?");
    } else {
      webSocket.broadcastTXT("[+] Tarama tamamlandı.");
    }
  }
}
