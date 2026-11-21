// ==========================================================================
// VAG CAN GATEWAY SCANNER - ISO-TP FLOW CONTROL (UDS)
// Arduino Nano + MCP2515 (8 MHz, 500 kbps) + SSD1306 128x64 SPI
// ==========================================================================

#include <SPI.h>
#include <mcp2515.h>
#include <U8g2lib.h>

// ---------- PIN TANIMLARI ----------
#define PIN_CAN_CS    10
#define PIN_CAN_INT    2
#define PIN_OLED_CS    9
#define PIN_OLED_DC    8
#define PIN_OLED_RES   7

// ---------- UDS PROTOKOL AYARLARI ----------
#define GW_TX_ID      0x710   // Gateway İletişim ID
#define GW_RX_ID      0x77A   // Gateway Yanıt ID
#define TIMEOUT_MS    800

MCP2515 can(PIN_CAN_CS);
U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, PIN_OLED_CS, PIN_OLED_DC, PIN_OLED_RES);

// Paylaşımlı tampon (Payload)
#define MAX_PAYLOAD 256
uint8_t rxBuffer[MAX_PAYLOAD];
uint16_t rxLen = 0;

void clearCanBuffer() {
  struct can_frame dummy;
  while (can.readMessage(&dummy) == MCP2515::ERROR_OK);
}

// --------------------------------------------------------------------------
// ISO-TP FLOW CONTROL DESTEKLİ UDS YANIT BEKLEYİCİ
// --------------------------------------------------------------------------
bool waitUDSResponse(uint16_t reqId, uint16_t resId, uint8_t expectedSvc, uint16_t expectedDid, uint8_t* outBuf, uint16_t maxLen, uint16_t* outLen, bool* negOut) {
  unsigned long t0 = millis();
  struct can_frame f;
  uint16_t currentLen = 0;
  uint16_t totalLen = 0;
  uint8_t expectedSeq = 1;
  
  while ((millis() - t0) < TIMEOUT_MS) {
    if (can.readMessage(&f) != MCP2515::ERROR_OK) continue;
    if (f.can_id != resId) continue;

    uint8_t pciType = f.data[0] & 0xF0;

    // --- Single Frame (SF) ---
    if (pciType == 0x00) {
      uint8_t sfLen = f.data[0] & 0x0F;
      // Negatif Yanıt Kontrolü (0x7F)
      if (f.data[1] == 0x7F && f.data[2] == (expectedSvc - 0x40)) { 
        *negOut = true; 
        return true;
      }
      // Pozitif Yanıt
      if (f.data[1] == expectedSvc) {
        if (expectedDid > 0) { // DID bekleniyorsa (Service 0x22)
          uint16_t resDid = ((uint16_t)f.data[2] << 8) | f.data[3];
          if (resDid == expectedDid) {
            *negOut = false;
            currentLen = sfLen - 3; // Servis ve DID byte'larını çıkar
            if (currentLen > maxLen) currentLen = maxLen;
            for (uint16_t i = 0; i < currentLen; i++) outBuf[i] = f.data[4 + i];
            *outLen = currentLen;
            return true;
          }
        } else { // Sadece servis yanıtı bekleniyorsa (Service 0x10)
          *negOut = false;
          currentLen = sfLen - 1;
          if (currentLen > maxLen) currentLen = maxLen;
          for (uint16_t i = 0; i < currentLen; i++) outBuf[i] = f.data[2 + i];
          *outLen = currentLen;
          return true;
        }
      }
    }
    
    // --- First Frame (FF) ---
    else if (pciType == 0x10) {
      totalLen = (((uint16_t)(f.data[0] & 0x0F)) << 8) | f.data[1];
      
      if (f.data[2] == expectedSvc) {
        uint16_t resDid = ((uint16_t)f.data[3] << 8) | f.data[4];
        if (resDid == expectedDid) {
          *negOut = false;
          // FF içindeki ilk verileri kopyala (DID sonrası 3 byte)
          for (uint8_t i = 0; i < 3; i++) {
            if (currentLen < maxLen) outBuf[currentLen++] = f.data[5 + i];
          }
          
          // Flow Control (FC) Gönder - Göndermeye Devam Et (Block=0, STmin=0)
          struct can_frame fc;
          fc.can_id = reqId;
          fc.can_dlc = 8;
          fc.data[0] = 0x30; // Clear To Send
          fc.data[1] = 0x00; // Block Size = 0
          fc.data[2] = 0x00; // STmin = 0
          for(int i = 3; i < 8; i++) fc.data[i] = 0x55; // Padding
          can.sendMessage(&fc);
          
          t0 = millis(); // Timeout sıfırla
        }
      }
    }
    
    // --- Consecutive Frame (CF) ---
    else if (pciType == 0x20) {
      uint8_t seq = f.data[0] & 0x0F;
      if (seq == expectedSeq) {
        expectedSeq = (expectedSeq + 1) & 0x0F;
        
        for (uint8_t i = 1; i < 8; i++) {
          if (currentLen < (totalLen - 3) && currentLen < maxLen) {
            outBuf[currentLen++] = f.data[i];
          }
        }
        
        if (currentLen >= (totalLen - 3) || currentLen >= maxLen) {
          *outLen = currentLen;
          return true; // Tüm veri alındı
        }
        t0 = millis(); // Bir sonraki CF için timeout sıfırla
      }
    }
  }
  return false;
}

// UDS Diagnostic Session (0x10) Başlatma
bool startExtendedSession(uint16_t txId, uint16_t rxId) {
  clearCanBuffer();
  struct can_frame f;
  f.can_id = txId;
  f.can_dlc = 8;
  f.data[0] = 0x02; // SF, 2 bytes
  f.data[1] = 0x10; // Diagnostic Session Control
  f.data[2] = 0x03; // Extended Diagnostic Session
  for(int i=3; i<8; i++) f.data[i] = 0x55;
  can.sendMessage(&f);

  bool neg = false;
  uint16_t len = 0;
  // 0x10 servisine 0x50 olarak yanıt gelir
  return waitUDSResponse(txId, rxId, 0x50, 0x0000, rxBuffer, MAX_PAYLOAD, &len, &neg) && !neg;
}

// UDS Read Data By Identifier (0x22) Sorgusu
bool readDID(uint16_t txId, uint16_t rxId, uint16_t did) {
  clearCanBuffer();
  struct can_frame f;
  f.can_id = txId;
  f.can_dlc = 8;
  f.data[0] = 0x03; // SF, 3 bytes
  f.data[1] = 0x22; // Read Data By Identifier
  f.data[2] = (did >> 8) & 0xFF;
  f.data[3] = did & 0xFF;
  for(int i=4; i<8; i++) f.data[i] = 0x55;
  can.sendMessage(&f);

  bool neg = false;
  rxLen = 0;
  // 0x22 servisine 0x62 olarak yanıt gelir
  return waitUDSResponse(txId, rxId, 0x62, did, rxBuffer, MAX_PAYLOAD, &rxLen, &neg) && !neg;
}

// Modül isimleri haritalaması (Örnek VAG Adresleri)
const char* getModuleName(uint8_t modId) {
  switch(modId) {
    case 0x01: return "Engine (ECU)";
    case 0x02: return "Auto Trans";
    case 0x03: return "ABS Brakes";
    case 0x08: return "Auto HVAC";
    case 0x09: return "Cent. Elect";
    case 0x15: return "Airbags";
    case 0x17: return "Instruments";
    case 0x19: return "CAN Gateway";
    case 0x44: return "Steering";
    case 0x5F: return "Information";
    default: return "Unknown";
  }
}

// --------------------------------------------------------------------------
// EKRAN VE ANA AKIŞ
// --------------------------------------------------------------------------
void setup() {
  u8g2.begin();
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.drawStr(10, 20, "GW Scanner v1");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(10, 40, "Baglaniliyor...");
  } while(u8g2.nextPage());

  can.reset();
  can.setBitrate(CAN_500KBPS, MCP_8MHZ);
  can.setNormalMode();
  delay(1000);

  // 1. Adım: Extended Session Aç
  if (!startExtendedSession(GW_TX_ID, GW_RX_ID)) {
    u8g2.firstPage();
    do {
      u8g2.drawStr(10, 30, "Session Error!");
    } while(u8g2.nextPage());
    while(1);
  }

  // 2. Adım: Installation List (0x04A0 veya 0x0500) Oku
  // NOT: MQB'de genelde 0x04A0, 0x04A1 vs kullanılır. Denemek gerek.
  if (readDID(GW_TX_ID, GW_RX_ID, 0x04A0) || readDID(GW_TX_ID, GW_RX_ID, 0x0500)) {
    // Okunan payload içerisinde modül numaraları (hex) bulunur
  } else {
    u8g2.firstPage();
    do {
      u8g2.drawStr(10, 30, "List Read Fail!");
    } while(u8g2.nextPage());
    while(1);
  }
}

void loop() {
  // rxBuffer içerisindeki veriyi sayfa sayfa ekranda göster
  // Her sayfa 4 modül göstersin
  uint8_t modulesFound = rxLen;
  uint8_t totalPages = (modulesFound + 3) / 4;

  for (uint8_t page = 0; page < totalPages; page++) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x10_tf);
      char header[24];
      sprintf(header, "Moduller: %d", modulesFound);
      u8g2.drawStr(0, 10, header);
      u8g2.drawHLine(0, 12, 128);

      for (uint8_t i = 0; i < 4; i++) {
        uint8_t idx = page * 4 + i;
        if (idx < modulesFound) {
          uint8_t modHex = rxBuffer[idx];
          char buf[32];
          sprintf(buf, "%02X - %s", modHex, getModuleName(modHex));
          u8g2.drawStr(0, 26 + (i * 12), buf);
        }
      }
    } while(u8g2.nextPage());
    delay(3000);
  }
}
