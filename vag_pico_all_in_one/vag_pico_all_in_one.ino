
// Arduino Core'un USB Host modunu açması için ZORUNLU makrolar
// Bunlar Arduino.h'den ÖNCE tanımlanmak ZORUNDA!
#define USE_TINYUSB_HOST
#define CFG_TUH_MSC 1

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <pico/mutex.h>
#include <queue>
#include <ArduinoOTA.h>

#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"
#include <SdFat_Adafruit_Fork.h>
#include <SdFat.h> // LÜTFEN KÜTÜPHANE YÖNETİCİSİNDEN "SdFat - Adafruit Fork" KURUN
// ==============================================================================
// VAG PICO W - ALL-IN-ONE (MFD + SCANNER + VALIDATOR + PIO-USB LOGGING)
// Architecture: Raspberry Pi Pico 2 W (RP2350)
// ==============================================================================

#define PIN_CAN_MISO 16
#define PIN_CAN_CS   17
#define PIN_CAN_SCK  18
#define PIN_CAN_MOSI 19

// PIO USB Host Pins (D- is automatically DP + 1)
#define PIN_USB_HOST_DP 14 
// D+ = GPIO 14
// D- = GPIO 15

#include "secrets.h"

WebServer server(80);
WebSocketsServer webSocket(81);

enum SystemMode { MODE_MFD = 0, MODE_SCAN = 1, MODE_VALIDATE = 2 };
volatile SystemMode currentMode = MODE_MFD;

volatile bool triggerScan = false;
volatile bool triggerVal = false;
volatile bool isSleeping = false;
volatile bool isUpdating = false;

// --- SHARED MFD DATA (CORE 1 -> CORE 0) ---
const uint16_t MFD_DIDs[24] = {
  0xF405, 0x11BE, 0x181A, 0x105E, 0x100C, 0x114F, 0x114E, 0x14F5, 
  0x178C, 0x1156, 0x115E, 0x11F6, 0x11B2, 0x10F9, 0x1628, 0x163A, 
  0x163B, 0x167D, 0x1314, 0x105C, 0x1116, 0x16A9, 0x11B6, 0x1347
};

const uint8_t MFD_DID_LENS[24] = {
  1, 2, 2, 2, 2, 2, 2, 2, 
  2, 4, 4, 2, 2, 2, 2, 2, 
  2, 2, 2, 2, 2, 4, 2, 4
};

volatile float g_did_values[24] = {0};
volatile bool g_did_updated[24] = {false};
volatile unsigned long lastCanRxTime = 0;

// --- USB HOST GLOBALS ---
Adafruit_USBH_Host USBHost;
Adafruit_USBH_MSC_BlockDevice MSC_Host;
FatVolume usbFS;
File32 logFile;

bool isLogging = false;
unsigned long lastLogWrite = 0;

std::queue<String> logQueue;
mutex_t logMutex;

void pushLog(String msg) {
  mutex_enter_blocking(&logMutex);
  if(logQueue.size() < 50) logQueue.push(msg);
  mutex_exit(&logMutex);
}

// ==============================================================================
// CORE 1: CAN BUS ENGINE
// ==============================================================================
MCP2515* can;

void poll_DID(uint16_t did, int index) {
    struct can_frame rx;
    int fCount = 0;
    while (can->readMessage(&rx) == MCP2515::ERROR_OK && fCount++ < 10) {} 
    
    struct can_frame frame;
    frame.can_id = 0x7E0; frame.can_dlc = 8;
    frame.data[0] = 0x03; 
    frame.data[1] = 0x22; 
    frame.data[2] = (did >> 8) & 0xFF; 
    frame.data[3] = did & 0xFF;
    for(int j=4; j<8; j++) frame.data[j] = 0x55;
    
    can->sendMessage(&frame);
    
    unsigned long startWait = millis();
    bool isFirstFrame = false;
    uint8_t dataBuf[8] = {0};
    uint8_t byteLen = MFD_DID_LENS[index];
    bool success = false;
    uint32_t rawVal = 0;
    
    while(millis() - startWait < 150) { 
        if (can->readMessage(&rx) == MCP2515::ERROR_OK && rx.can_id == 0x7E8) {
            uint8_t pciType = rx.data[0] >> 4;
            
            if (pciType == 0) { // Single Frame
                if (rx.data[1] == 0x62 && rx.data[2] == ((did >> 8) & 0xFF) && rx.data[3] == (did & 0xFF)) {
                    uint32_t val = 0;
                    for (uint8_t i = 0; i < byteLen && i < 4; i++) {
                        val = (val << 8) | rx.data[4 + i];
                    }
                    rawVal = val;
                    success = true;
                    break;
                }
            }
            else if (pciType == 1) { // First Frame
                if (rx.data[2] == 0x62 && rx.data[3] == ((did >> 8) & 0xFF) && rx.data[4] == (did & 0xFF)) {
                    isFirstFrame = true;
                    dataBuf[0] = rx.data[5];
                    dataBuf[1] = rx.data[6];
                    dataBuf[2] = rx.data[7];
                    
                    struct can_frame fc;
                    fc.can_id = 0x7E0;
                    fc.can_dlc = 8;
                    fc.data[0] = 0x30;
                    fc.data[1] = 0x00;
                    fc.data[2] = 0x00;
                    for(int k=3; k<8; k++) fc.data[k]=0x55;
                    can->sendMessage(&fc);
                }
            }
            else if (pciType == 2 && isFirstFrame) { // Consecutive Frame
                dataBuf[3] = rx.data[1];
                dataBuf[4] = rx.data[2];
                dataBuf[5] = rx.data[3];
                dataBuf[6] = rx.data[4];
                
                uint32_t val = 0;
                if (byteLen == 4) {
                    val = ((uint32_t)dataBuf[0] << 24) | ((uint32_t)dataBuf[1] << 16) | ((uint32_t)dataBuf[2] << 8) | (uint32_t)dataBuf[3];
                } else if (byteLen == 2) {
                    val = ((uint32_t)dataBuf[0] << 8) | dataBuf[1];
                } else if (byteLen == 1) {
                    val = dataBuf[0];
                }
                rawVal = val;
                success = true;
                break;
            }
        }
    }
    
    if (success) {
        float val = 0.0f;
        uint32_t raw = rawVal;
        switch(did) {
           case 0xF405: val = raw - 40.0f; break; 
           case 0x11BE: 
           case 0x11B2:
           case 0x10F9:
           case 0x1628: val = (raw / 10.0f) - 273.1f; break; 
           case 0x181A: val = ((raw * 0.25f) - 1000.0f) / 1000.0f; break; 
           case 0x105E: val = raw / 50.0f; break; 
           case 0x100C:
           case 0x114F:
           case 0x114E:
           case 0x11F6:
           case 0x163A:
           case 0x163B:
           case 0x167D:
           case 0x1314:
           case 0x1116: val = raw / 100.0f; break;
           case 0x14F5:
           case 0x178C: val = raw / 10.0f; break;
           case 0x1156: val = raw / 1000.0f; break; 
           case 0x115E:
           case 0x1347: val = raw / 60.0f; break; 
           case 0x105C: val = raw / 81.92f; break;
           case 0x16A9: val = (float)raw; break; 
           case 0x11B6: val = (float)raw; break; 
        }
        g_did_values[index] = val;
        g_did_updated[index] = true;
        lastCanRxTime = millis();
    }
}

void setup1() {
  delay(1000); 
  can = new MCP2515(PIN_CAN_CS);
  SPI.setRX(PIN_CAN_MISO); 
  SPI.setSCK(PIN_CAN_SCK); 
  SPI.setTX(PIN_CAN_MOSI);
  SPI.begin();
  can->reset();
  can->setBitrate(CAN_500KBPS, MCP_8MHZ);
  can->setNormalMode();
}

int rr_index = 0;

void loop1() {
  if (isUpdating) { delay(100); return; }

  if (isSleeping) {
    struct can_frame rx;
    if (can->readMessage(&rx) == MCP2515::ERROR_OK) rp2040.reboot();
    delay(100);
    return;
  }

  if (currentMode == MODE_MFD) {
    poll_DID(0x181A, 2);
    if (rr_index == 2) rr_index++; 
    if (rr_index >= 24) rr_index = 0;
    poll_DID(MFD_DIDs[rr_index], rr_index);
    rr_index++;
    
  } else if (currentMode == MODE_SCAN) {
    if (triggerScan) {
      triggerScan = false;
      pushLog("\n--- SCANNING GATEWAY (0x710) ---");
      
      struct can_frame rx, frame;
      int fCount = 0;
      while(can->readMessage(&rx) == MCP2515::ERROR_OK && fCount++ < 10) {}
      
      frame.can_id = 0x710; frame.can_dlc = 8;
      frame.data[0]=0x02; frame.data[1]=0x10; frame.data[2]=0x03;
      for(int i=3; i<8; i++) frame.data[i] = 0x55;
      can->sendMessage(&frame);
      delay(50);
      
      pushLog("[+] Req 0x04A0...");
      frame.data[0]=0x03; frame.data[1]=0x22; frame.data[2]=0x04; frame.data[3]=0xA0;
      can->sendMessage(&frame);
      
      unsigned long st = millis();
      bool found = false;
      while(millis() - st < 500) {
        if(can->readMessage(&rx) == MCP2515::ERROR_OK && rx.can_id == 0x77A) {
          found = true;
          char buf[64];
          sprintf(buf, "[RAW 77A] %02X %02X %02X %02X %02X %02X %02X %02X", rx.data[0], rx.data[1], rx.data[2], rx.data[3], rx.data[4], rx.data[5], rx.data[6], rx.data[7]);
          pushLog(String(buf));
        }
      }
      if(!found) pushLog("[-] No response from 0x77A");
      pushLog("--- DONE ---");
      currentMode = MODE_MFD;
    }
  } else if (currentMode == MODE_VALIDATE) {
    if (triggerVal) {
      triggerVal = false;
      pushLog("\n--- VALIDATING 24 DIDs ---");
      struct can_frame rx, frame;
      int sCount = 0;
      
      for(int i=0; i<24; i++) {
        uint16_t did = MFD_DIDs[i];
        int fCount2 = 0;
        while(can->readMessage(&rx) == MCP2515::ERROR_OK && fCount2++ < 10) {}
        
        frame.can_id = 0x7E0; frame.can_dlc = 8;
        frame.data[0]=0x03; frame.data[1]=0x22; frame.data[2]=(did>>8)&0xFF; frame.data[3]=did&0xFF;
        for(int j=4; j<8; j++) frame.data[j] = 0x55;
        can->sendMessage(&frame);
        
        unsigned long st = millis();
        bool found = false;
        while(millis() - st < 150) {
          if (can->readMessage(&rx) == MCP2515::ERROR_OK && rx.can_id == 0x7E8) {
            uint8_t pci = rx.data[0] >> 4;
            if (pci == 0 && rx.data[1] == 0x62 && rx.data[2] == frame.data[2] && rx.data[3] == frame.data[3]) {
                found = true; sCount++;
                char buf[64]; sprintf(buf, "[OK-SF] 0x%04X", did); pushLog(String(buf));
                break;
            } else if (pci == 1 && rx.data[2] == 0x62 && rx.data[3] == frame.data[2] && rx.data[4] == frame.data[3]) {
                found = true; sCount++;
                char buf[64]; sprintf(buf, "[OK-FF] 0x%04X", did); pushLog(String(buf));
                break;
            }
          }
        }
        if(!found) {
          char buf[64];
          sprintf(buf, "[FAIL] 0x%04X Timeout", did);
          pushLog(String(buf));
        }
        delay(10); 
      }
      char endBuf[64];
      sprintf(endBuf, "--- VALIDATION: %d/24 OK ---", sCount);
      pushLog(String(endBuf));
      currentMode = MODE_MFD;
    }
  }
}

// --- HTML / CSS / JS PAYLOAD ---
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Gökay VAG All-In-One</title>
    <style>
        :root { --bg: #000; --accent: #fff; --gray: #333; --text: #fff; }
        body { margin: 0; padding: 10px; background: var(--bg); color: var(--text); font-family: -apple-system, sans-serif; user-select: none; }
        .header { display: flex; justify-content: space-between; font-size: 14px; color: #888; margin-bottom: 10px; position: relative;}
        .edit-btn { background: #333; color: white; border: none; padding: 5px 15px; border-radius: 15px; cursor: pointer; }
        #nav_menu { display:none; position:absolute; right:0; top:35px; background:#222; padding:5px; border-radius:10px; z-index:200; border:1px solid #444; width: 150px; box-shadow: 0 4px 15px rgba(0,0,0,0.8);}
        .nav-item { padding: 15px; font-size: 14px; color: #fff; border-bottom: 1px solid #333; cursor: pointer; text-align: center;}
        .nav-item:last-child { border-bottom: none; }
        .nav-item:active { background: #444; border-radius: 5px;}
        .boost-container { background: #111; border-radius: 15px; padding: 20px; text-align: center; margin-bottom: 15px; border: 1px solid #222; }
        .boost-val { font-size: 64px; font-weight: 800; line-height: 1; margin: 5px 0;}
        .boost-lbl { font-size: 16px; color: #888; text-transform: uppercase; letter-spacing: 2px; }
        .bar-bg { background: #222; height: 12px; border-radius: 6px; margin-top: 15px; overflow: hidden; }
        .bar-fill { background: var(--accent); height: 100%; width: 0%; transform-origin: left; will-change: transform; }
        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
        .slot { background: #111; border-radius: 12px; padding: 15px; display: flex; flex-direction: column; justify-content: center; position: relative; border: 1px solid #222; min-height: 80px;}
        .slot-val { font-size: 28px; font-weight: 700; }
        .slot-lbl { font-size: 12px; color: #888; margin-bottom: 5px;}
        .dpf-mini-bg { background: #333; height: 8px; border-radius: 4px; width: 100%; margin-top: 5px; overflow: hidden; }
        .dpf-mini-fill { background: #fff; height: 100%; width: 0%; transition: width 0.3s; }
        #modal { display: none; position: fixed; top: 0; left: 0; width: 100%; height: 100%; background: rgba(0,0,0,0.9); z-index: 100; padding: 20px; box-sizing: border-box; overflow-y: auto;}
        .modal-content { background: #111; padding: 20px; border-radius: 15px; margin-bottom: 50px;}
        .modal-title { font-size: 20px; font-weight: bold; margin-bottom: 20px; text-align: center;}
        .slot-select { margin-bottom: 15px; }
        .slot-select label { display: block; margin-bottom: 5px; color: #888;}
        select { width: 100%; padding: 10px; background: #222; color: white; border: 1px solid #444; border-radius: 8px; font-size: 16px;}
        .save-btn { background: white; color: black; width: 100%; padding: 15px; border: none; border-radius: 10px; font-size: 18px; font-weight: bold; margin-top: 20px;}
        .view { display: none; }
        .view.active { display: block; }
        .term { background: #111; padding: 15px; border-radius: 10px; font-family: monospace; font-size: 12px; color: #0f0; height: 60vh; overflow-y: scroll; white-space: pre-wrap; border: 1px solid #333;}
        .action-btn { background: #222; color: white; border: 1px solid #444; padding: 15px; border-radius: 10px; width: 100%; font-size: 16px; margin-bottom: 15px; cursor: pointer; }
        .action-btn:active { background: #444; }
    </style>
</head>
<body>

<div class="header">
    <div id="status">KAPALI</div>
    <div id="usb_status" style="color: gray; font-weight: bold;">USB: YOK</div>
    <div style="display:flex; gap:10px;">
        <button id="btn_edit" class="edit-btn" onclick="openSettings()">Düzenle</button>
        <button class="edit-btn" onclick="toggleMenu()">☰ Menü</button>
    </div>
    <div id="nav_menu">
        <div class="nav-item" onclick="switchTab('mfd')">Dashboard</div>
        <div class="nav-item" onclick="switchTab('scanner')">Scanner</div>
        <div class="nav-item" onclick="switchTab('validator')">Validator</div>
    </div>
</div>

<div id="view_mfd" class="view active">
    <div class="boost-container">
        <div class="boost-lbl">Turbo Basıncı</div>
        <div class="boost-val" id="val_boost">0.00</div>
        <div class="bar-bg"><div class="bar-fill" id="bar_boost"></div></div>
    </div>
    <div class="grid" id="main_grid"></div>
</div>

<div id="view_scanner" class="view">
    <button class="action-btn" onclick="sendCmd('CMD:START_SCAN')">Gateway Tara (0x710)</button>
    <div class="term" id="term_scan">Loglar bekleniyor...\n</div>
</div>

<div id="view_validator" class="view">
    <button class="action-btn" onclick="sendCmd('CMD:START_VAL')">24 DID'i Doğrula (0x7E0)</button>
    <div class="term" id="term_val">Loglar bekleniyor...\n</div>
</div>

<div id="modal">
    <div class="modal-content">
        <div class="modal-title">Ekranı Özelleştir</div>
        <div id="modal_slots"></div>
        <button class="save-btn" onclick="saveSettings()">Kaydet ve Çık</button>
    </div>
</div>

<script>
const DICT = {
    "F405": { name: "Soğutma Suyu", unit: "°C", fmt: v=>v.toFixed(0) },
    "11BE": { name: "Motor Yağı", unit: "°C", fmt: v=>v.toFixed(0) },
    "105E": { name: "Akü Voltajı", unit: "V", fmt: v=>v.toFixed(1) },
    "100C": { name: "Yakıt", unit: "L", fmt: v=>v.toFixed(1) },
    "114F": { name: "Soot Calc", unit: "g", fmt: v=>v.toFixed(2) },
    "114E": { name: "Soot Meas", unit: "g", fmt: v=>v.toFixed(2) },
    "14F5": { name: "DPF dP", unit: "hPa", fmt: v=>v.toFixed(1) },
    "178C": { name: "Oil Ash", unit: "g", fmt: v=>v.toFixed(1) },
    "1156": { name: "Mesafe / Regen", unit: "km", fmt: v=>v.toFixed(0) },
    "115E": { name: "Zaman / Regen", unit: "dk", fmt: v=>v.toFixed(0) },
    "11F6": { name: "DPF Sınır", unit: "g", fmt: v=>v.toFixed(2) },
    "11B2": { name: "EGT 1 (Pre)", unit: "°C", fmt: v=>v.toFixed(0) },
    "10F9": { name: "EGT 2 (Post)", unit: "°C", fmt: v=>v.toFixed(0) },
    "1628": { name: "İntake Temp", unit: "°C", fmt: v=>v.toFixed(0) },
    "163A": { name: "Ana Enj.", unit: "mg", fmt: v=>v.toFixed(2) },
    "163B": { name: "Post Enj 2", unit: "mg", fmt: v=>v.toFixed(2) },
    "167D": { name: "Post Enj 3", unit: "mg", fmt: v=>v.toFixed(2) },
    "1314": { name: "LP-EGR", unit: "%", fmt: v=>v.toFixed(1) },
    "105C": { name: "Şarj Yükü", unit: "%", fmt: v=>v.toFixed(0) },
    "1116": { name: "Radyatör Fanı", unit: "%", fmt: v=>v.toFixed(0) },
    "16A9": { name: "Odometer", unit: "km", fmt: v=>v.toFixed(0) },
    "11B6": { name: "Regen Sayısı", unit: "", fmt: v=>v.toFixed(0) },
    "1347": { name: "Regen Süresi", unit: "dk", fmt: v=>v.toFixed(0) },
    "DPF_PERC": { name: "DPF Doluluk", unit: "%", fmt: v=>v.toFixed(1) }
};

const defaultLayout = ["F405", "11BE", "105E", "DPF_PERC", "11B2", "10F9", "114F", "11F6"];
let currentLayout = JSON.parse(localStorage.getItem("mfd_layout")) || defaultLayout;
let currentTab = "mfd";

function toggleMenu() {
    let m = document.getElementById('nav_menu');
    m.style.display = (m.style.display === 'none' || m.style.display === '') ? 'block' : 'none';
}

function switchTab(tab) {
    document.querySelectorAll('.view').forEach(v => v.classList.remove('active'));
    document.getElementById('view_' + tab).classList.add('active');
    
    document.getElementById('btn_edit').style.display = (tab === 'mfd') ? 'block' : 'none';
    document.getElementById('nav_menu').style.display = 'none';
    currentTab = tab;
    
    if(tab === 'mfd') sendCmd("MODE:0");
    if(tab === 'scanner') sendCmd("MODE:1");
    if(tab === 'validator') sendCmd("MODE:2");
}

function sendCmd(cmd) {
    if(ws && ws.readyState === WebSocket.OPEN) ws.send(cmd);
}

function renderGrid() {
    let html = "";
    for(let i=0; i<8; i++) {
        let key = currentLayout[i];
        let info = DICT[key] || {name: "--", unit: "", fmt: v=>"--"};
        html += `<div class="slot" id="slot_${i}">
            <div class="slot-lbl">${info.name}</div>
            <div class="slot-val" id="val_${i}">-- ${info.unit}</div>
            ${key === 'DPF_PERC' ? `<div class="dpf-mini-bg"><div class="dpf-mini-fill" id="dpf_bar"></div></div>` : ''}
        </div>`;
    }
    document.getElementById("main_grid").innerHTML = html;
}

function openSettings() {
    let html = "";
    for(let i=0; i<8; i++) {
        html += `<div class="slot-select">
            <label>Kutu ${i+1}</label>
            <select id="sel_${i}">`;
        for (const [k, v] of Object.entries(DICT)) {
            html += `<option value="${k}" ${currentLayout[i] === k ? "selected" : ""}>${v.name}</option>`;
        }
        html += `</select></div>`;
    }
    document.getElementById("modal_slots").innerHTML = html;
    document.getElementById("modal").style.display = "block";
}

function saveSettings() {
    for(let i=0; i<8; i++) {
        currentLayout[i] = document.getElementById(`sel_${i}`).value;
    }
    localStorage.setItem("mfd_layout", JSON.stringify(currentLayout));
    document.getElementById("modal").style.display = "none";
    renderGrid();
}

renderGrid();

let ws;
let targetBoost = 0, currentBoost = 0;
let lastData = {}; 

function connectWS() {
    ws = new WebSocket("ws://" + location.hostname + ":81");
    ws.onopen = () => document.getElementById("status").innerText = "BAĞLI";
    ws.onclose = () => { document.getElementById("status").innerText = "KOPUK"; setTimeout(connectWS, 1000); };
    ws.onmessage = (e) => {
        if(e.data.startsWith("{")) {
            if(currentTab !== 'mfd') return;
            try {
                let j = JSON.parse(e.data);
                if(j.L !== undefined) {
                    let u = document.getElementById("usb_status");
                    if(u) {
                        u.innerText = j.L ? "USB: KAYIT(REC)" : "USB: YOK";
                        u.style.color = j.L ? "#ff4444" : "gray";
                    }
                }
                if(j.B !== undefined) {
                    targetBoost = j.B;
                    document.getElementById("val_boost").innerText = targetBoost.toFixed(2) + " BAR";
                }
                if(j.D !== undefined) {
                    lastData = j.D; 
                    if(j.DPF !== undefined) lastData["DPF_PERC"] = j.DPF;
                    
                    for(let i=0; i<8; i++) {
                        let key = currentLayout[i];
                        if(lastData[key] !== undefined) {
                            let info = DICT[key];
                            document.getElementById(`val_${i}`).innerText = info.fmt(lastData[key]) + " " + info.unit;
                            
                            if(key === "DPF_PERC") {
                                let pct = Math.max(0, Math.min(100, lastData[key]));
                                let el = document.getElementById("dpf_bar");
                                if(el) {
                                    el.style.width = pct + "%";
                                    el.style.background = pct > 90 ? "red" : (pct > 70 ? "orange" : "white");
                                }
                            }
                        } else {
                            document.getElementById(`val_${i}`).innerText = "-- " + (DICT[key] ? DICT[key].unit : "");
                        }
                    }
                }
            } catch(err) {}
        } else {
            // Text Log (Scanner/Validator)
            if(currentTab === 'scanner') {
                let ts = document.getElementById("term_scan");
                ts.innerText += e.data + "\n";
                ts.scrollTop = ts.scrollHeight;
            } else if(currentTab === 'validator') {
                let tv = document.getElementById("term_val");
                tv.innerText += e.data + "\n";
                tv.scrollTop = tv.scrollHeight;
            }
        }
    };
}

function anim() {
    currentBoost += (targetBoost - currentBoost) * 0.2;
    let pct = Math.max(0, Math.min(100, (currentBoost / 3.0) * 100)); 
    document.getElementById("bar_boost").style.transform = `scaleX(${pct / 100})`;
    requestAnimationFrame(anim);
}

connectWS();
requestAnimationFrame(anim);
</script>
</body>
</html>
)rawliteral";

// ==============================================================================
// CORE 0: WEBSERVER, WEBSOCKETS, PIO-USB HOST LOGGING
// ==============================================================================
void handleRoot() { server.send(200, "text/html", INDEX_HTML); }

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT && length > 0) {
    String msg = (char*)payload;
    if (msg.startsWith("MODE:")) {
      int m = msg.substring(5).toInt();
      if(m >= 0 && m <= 2) currentMode = (SystemMode)m;
    }
    else if (msg == "CMD:START_SCAN") triggerScan = true;
    else if (msg == "CMD:START_VAL") triggerVal = true;
  }
}

// USB Host Mount Callback
void tuh_msc_mount_cb(uint8_t dev_addr) {
  pushLog("[+] USB Bellek TESPIT EDILDI (dev_addr: " + String(dev_addr) + ")");
  if (usbFS.begin(&MSC_Host)) {
    pushLog("[+] USB FS Bağlandı. Dosya oluşturuluyor...");
    for (int i = 1; i <= 999; i++) {
      char fName[32];
      sprintf(fName, "log_%03d.csv", i);
      if (!usbFS.exists(fName)) {
        logFile = usbFS.open(fName, O_WRITE | O_CREAT);
        if (logFile) {
          isLogging = true;
          logFile.println("TimeMs,Coolant,Oil,Boost,Batt,Fuel,SootCalc,SootMeas,dP,Ash,Dist,Time,Limit,EGT_Pre,EGT_Post,Intake,MainInj,Post2,Post3,EGR,GenLoad,Fan1,Odo,RegCnt,RegDur,DPF_Perc");
          logFile.sync();
          pushLog("[+] Log basladi: " + String(fName));
        }
        break;
      }
    }
  }
}

// USB Host Unmount Callback
void tuh_msc_umount_cb(uint8_t dev_addr) {
  pushLog("[-] USB Bellek CIKARILDI!");
  isLogging = false;
  if(logFile) logFile.close();
}

void setup() {
  Serial.begin(115200);
  mutex_init(&logMutex);
  
  // PIO-USB Initialization
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIN_USB_HOST_DP; // D+ is GPIO 14, D- is GPIO 15
  USBHost.configure_pio_usb(1, &pio_cfg);
  
  USBHost.begin(1);
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  
  server.on("/", handleRoot);
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  ArduinoOTA.onStart([]() { isUpdating = true; });
  ArduinoOTA.begin();
}

unsigned long lastWsUpdate = 0;

void loop() {
  // MUST RUN USB HOST TASK FREQUENTLY
  USBHost.task();
  
  if (isSleeping) {
    delay(1000);
    return;
  }

  ArduinoOTA.handle();
  server.handleClient();
  webSocket.loop();
  
  mutex_enter_blocking(&logMutex);
  while(!logQueue.empty()) {
    String msg = logQueue.front();
    logQueue.pop();
    mutex_exit(&logMutex);
    webSocket.broadcastTXT(msg);
    mutex_enter_blocking(&logMutex);
  }
  mutex_exit(&logMutex);
  
  unsigned long now = millis();
  
  if (lastCanRxTime > 0 && (now - lastCanRxTime > 300000)) {
    isSleeping = true;
    if(logFile) logFile.close();
    webSocket.close();
    server.close();
    WiFi.mode(WIFI_OFF);
    return;
  }
  
  // WebSockets (30 Hz)
  if (now - lastWsUpdate >= 33 && currentMode == MODE_MFD) {
    lastWsUpdate = now;
    
    String json = "{";
    json += "\"L\":" + String(isLogging ? 1 : 0) + ",";
    if (g_did_updated[2]) {
        json += "\"B\":" + String(g_did_values[2], 2) + ",";
    }
    
    if (g_did_updated[5] && g_did_updated[11] && g_did_values[11] > 0.0f) {
        float limit = g_did_values[11];
        float dpf = (g_did_values[5] / limit) * 100.0f;
        json += "\"DPF\":" + String(dpf, 1) + ",";
    }
    
    json += "\"D\":{";
    bool first = true;
    for(int i=0; i<24; i++) {
        if(g_did_updated[i]) {
            if(!first) json += ",";
            json += "\"" + String(MFD_DIDs[i], HEX) + "\":" + String(g_did_values[i], 2);
            first = false;
        }
    }
    json += "}}";
    
    webSocket.broadcastTXT(json);
  }
  
  // USB Flash Drive Logging (300 ms / ~3.3 Hz)
  if (isLogging && (now - lastLogWrite >= 300) && currentMode == MODE_MFD) {
    lastLogWrite = now;
    
    String csv = String(now) + ",";
    
    for(int i=0; i<24; i++) {
        if (g_did_updated[i]) {
            csv += String(g_did_values[i], 2);
        }
        csv += ",";
    }
    
    float dpf = 0.0f;
    if (g_did_updated[5] && g_did_updated[11] && g_did_values[11] > 0.0f) {
        dpf = (g_did_values[5] / g_did_values[11]) * 100.0f;
    }
    csv += String(dpf, 1);
    
    logFile.println(csv);
    logFile.sync();
  }
}
