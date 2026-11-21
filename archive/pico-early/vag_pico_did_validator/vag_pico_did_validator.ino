#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// ==============================================================================
// VAG PICO W - DID VALIDATOR (WEB UI)
// ==============================================================================

#define PIN_CAN_MISO 16
#define PIN_CAN_CS   17
#define PIN_CAN_SCK  18
#define PIN_CAN_MOSI 19

#include "secrets.h"

WebServer server(80);
WebSocketsServer webSocket(81);
MCP2515 can(PIN_CAN_CS);

bool triggerVal = false;

const uint16_t TEST_DIDS[] = {
  0xF405, 0x11BE, 0x181A, 0x105E, 0x114F, 0x14F5, 0x11B2, 0x10F9, 0x1628, 0x11B6, 0x1347, 0x16A9
};
const int NUM_DIDS = sizeof(TEST_DIDS)/sizeof(uint16_t);

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>DID Doğrulayıcı</title>
    <style>
        body { background: #121212; color: #fff; font-family: sans-serif; padding: 20px; }
        button { padding: 15px 30px; font-size: 18px; background: #29b6f6; border: none; border-radius: 8px; cursor: pointer; color: #000; font-weight: bold; width: 100%; margin-bottom: 20px; }
        #term { background: #000; color: #0f0; padding: 15px; border-radius: 8px; font-family: monospace; height: 350px; overflow-y: auto; white-space: pre-wrap; font-size: 14px; }
    </style>
</head>
<body>
    <h2>VAG DID Validator (Web)</h2>
    <button onclick="start()">Doğrulamayı Başlat (0x7E0)</button>
    <div id="term">Bağlantı bekleniyor...\n</div>
    <script>
        let ws = new WebSocket('ws://' + window.location.hostname + ':81/');
        const term = document.getElementById('term');
        
        ws.onopen = () => term.innerText += "Hazır. Test etmek için butona basın.\n";
        ws.onmessage = (e) => { term.innerText += e.data + "\n"; term.scrollTop = term.scrollHeight; };
        
        function start() {
            if (ws.readyState === WebSocket.OPEN) {
                ws.send("START");
                term.innerText += "\n--- DOĞRULAMA BAŞLATILDI ---\n";
            }
        }
    </script>
</body>
</html>
)rawliteral";

void handleRoot() { server.send(200, "text/html", INDEX_HTML); }

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT && length > 0) {
    if (strncmp((char*)payload, "START", 5) == 0) triggerVal = true;
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
  
  if (triggerVal) {
    triggerVal = false;
    
    struct can_frame frame;
    struct can_frame rx;
    int successCount = 0;
    
    char buf[128];
    sprintf(buf, "[+] %d adet DID ECU (0x7E0) uzerinde test ediliyor...", NUM_DIDS);
    webSocket.broadcastTXT(buf);
    
    for(int i = 0; i < NUM_DIDS; i++) {
      uint16_t did = TEST_DIDS[i];
      
      while(can.readMessage(&rx) == MCP2515::ERROR_OK) {}
      
      frame.can_id = 0x7E0;
      frame.can_dlc = 8;
      frame.data[0] = 0x03; frame.data[1] = 0x22; frame.data[2] = (did >> 8) & 0xFF; frame.data[3] = did & 0xFF;
      for(int j=4; j<8; j++) frame.data[j] = 0x55;
      
      can.sendMessage(&frame);
      
      unsigned long startWait = millis();
      bool found = false;
      
      while(millis() - startWait < 100) {
        if (can.readMessage(&rx) == MCP2515::ERROR_OK && rx.can_id == 0x7E8) {
          if (rx.data[1] == 0x62 && rx.data[2] == frame.data[2] && rx.data[3] == frame.data[3]) {
            found = true; successCount++;
            sprintf(buf, "[OK] DID: 0x%04X | RAW: %02X%02X%02X%02X", did, rx.data[4], rx.data[5], rx.data[6], rx.data[7]);
            webSocket.broadcastTXT(buf);
            break;
          } else if (rx.data[1] == 0x7F && rx.data[2] == 0x22) {
            found = true;
            sprintf(buf, "[ERR] DID: 0x%04X | Negative Code: %02X", did, rx.data[3]);
            webSocket.broadcastTXT(buf);
            break;
          }
        }
      }
      
      if (!found) {
        sprintf(buf, "[TIMEOUT] DID: 0x%04X | No response", did);
        webSocket.broadcastTXT(buf);
      }
      delay(50);
    }
    
    sprintf(buf, "--- TEST BITTI: %d/%d BASARILI ---", successCount, NUM_DIDS);
    webSocket.broadcastTXT(buf);
  }
}
