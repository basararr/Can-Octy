// ==========================================================================
// VAG BEAUTIFUL MFD - ISO-TP FLOW CONTROL (UDS)
// Arduino Nano + MCP2515 (8 MHz, 500 kbps) + SSD1306 128x64 SPI
// ==========================================================================

#include <SPI.h>
#include <mcp2515.h>
#include <U8g2lib.h>
#include <stdlib.h>

// ---------- PIN TANIMLARI ----------
#define PIN_CAN_CS    10
#define PIN_CAN_INT    2
#define PIN_OLED_CS    9
#define PIN_OLED_DC    8
#define PIN_OLED_RES   7

// ---------- UDS PROTOKOL AYARLARI ----------
#define ECU_TX_ID     0x7E0
#define ECU_RX_ID     0x7E8
#define TIMEOUT_MS    300

MCP2515 can(PIN_CAN_CS);
U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, PIN_OLED_CS, PIN_OLED_DC, PIN_OLED_RES);

#define MAX_PAYLOAD 32
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

    if (pciType == 0x00) { // SF
      uint8_t sfLen = f.data[0] & 0x0F;
      if (f.data[1] == 0x7F && f.data[2] == (expectedSvc - 0x40)) { *negOut = true; return true; }
      if (f.data[1] == expectedSvc) {
        uint16_t resDid = ((uint16_t)f.data[2] << 8) | f.data[3];
        if (resDid == expectedDid) {
          *negOut = false;
          currentLen = sfLen - 3;
          if (currentLen > maxLen) currentLen = maxLen;
          for (uint16_t i = 0; i < currentLen; i++) outBuf[i] = f.data[4 + i];
          *outLen = currentLen;
          return true;
        }
      }
    }
    else if (pciType == 0x10) { // FF
      totalLen = (((uint16_t)(f.data[0] & 0x0F)) << 8) | f.data[1];
      if (f.data[2] == expectedSvc) {
        uint16_t resDid = ((uint16_t)f.data[3] << 8) | f.data[4];
        if (resDid == expectedDid) {
          *negOut = false;
          for (uint8_t i = 0; i < 3; i++) {
            if (currentLen < maxLen) outBuf[currentLen++] = f.data[5 + i];
          }
          struct can_frame fc;
          fc.can_id = reqId; fc.can_dlc = 8;
          fc.data[0] = 0x30; fc.data[1] = 0x00; fc.data[2] = 0x00;
          for(int i = 3; i < 8; i++) fc.data[i] = 0x55;
          can.sendMessage(&fc);
          t0 = millis();
        }
      }
    }
    else if (pciType == 0x20) { // CF
      uint8_t seq = f.data[0] & 0x0F;
      if (seq == expectedSeq) {
        expectedSeq = (expectedSeq + 1) & 0x0F;
        for (uint8_t i = 1; i < 8; i++) {
          if (currentLen < (totalLen - 3) && currentLen < maxLen) outBuf[currentLen++] = f.data[i];
        }
        if (currentLen >= (totalLen - 3) || currentLen >= maxLen) {
          *outLen = currentLen; return true;
        }
        t0 = millis();
      }
    }
  }
  return false;
}

bool readDID(uint16_t did, uint32_t* outVal) {
  clearCanBuffer();
  struct can_frame f;
  f.can_id = ECU_TX_ID;
  f.can_dlc = 8;
  f.data[0] = 0x03; // SF, 3 bytes
  f.data[1] = 0x22;
  f.data[2] = (did >> 8) & 0xFF;
  f.data[3] = did & 0xFF;
  for(int i=4; i<8; i++) f.data[i] = 0x55;
  can.sendMessage(&f);

  bool neg = false;
  rxLen = 0;
  if (waitUDSResponse(ECU_TX_ID, ECU_RX_ID, 0x62, did, rxBuffer, MAX_PAYLOAD, &rxLen, &neg) && !neg) {
    if (rxLen == 1) *outVal = rxBuffer[0];
    else if (rxLen == 2) *outVal = ((uint16_t)rxBuffer[0] << 8) | rxBuffer[1];
    else if (rxLen >= 4) *outVal = ((uint32_t)rxBuffer[0] << 24) | ((uint32_t)rxBuffer[1] << 16) | ((uint32_t)rxBuffer[2] << 8) | rxBuffer[3];
    return true;
  }
  return false;
}

// --------------------------------------------------------------------------
// MFD ÇİZİM VE DEĞİŞKENLER
// --------------------------------------------------------------------------
float valRPM = 0, valSpeed = 0, valBoost = 0, valCoolant = 0;
char strBuf[16];

void updateScreen() {
  u8g2.firstPage();
  do {
    // Arayüzü dörde bölüyoruz:
    // Sol Üst: Devir, Sağ Üst: Hız
    // Sol Alt: Boost, Sağ Alt: Su Sıcaklığı
    
    // Ayırıcı çizgiler
    u8g2.drawVLine(64, 0, 64);
    u8g2.drawHLine(0, 32, 128);

    // --- SOL ÜST: RPM ---
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(2, 8, "RPM");
    u8g2.setFont(u8g2_font_7x14B_tf);
    dtostrf(valRPM, 1, 0, strBuf);
    u8g2.drawStr(2, 26, strBuf);

    // --- SAĞ ÜST: HIZ ---
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(68, 8, "SPEED km/h");
    u8g2.setFont(u8g2_font_7x14B_tf);
    dtostrf(valSpeed, 1, 0, strBuf);
    u8g2.drawStr(68, 26, strBuf);

    // --- SOL ALT: BOOST ---
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(2, 40, "BOOST Bar");
    u8g2.setFont(u8g2_font_7x14B_tf);
    dtostrf(valBoost, 1, 2, strBuf);
    u8g2.drawStr(2, 58, strBuf);

    // --- SAĞ ALT: COOLANT ---
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(68, 40, "COOLANT C");
    u8g2.setFont(u8g2_font_7x14B_tf);
    dtostrf(valCoolant, 1, 0, strBuf);
    u8g2.drawStr(68, 58, strBuf);

  } while(u8g2.nextPage());
}

void setup() {
  u8g2.begin();
  can.reset();
  can.setBitrate(CAN_500KBPS, MCP_8MHZ);
  can.setNormalMode();
}

void loop() {
  uint32_t raw;

  // DIDs for demonstration (These might need adjustment based on exact CLHA ECU formulas)
  // Devir (RPM) DID: 0xF40C (Genelde VAG standart)
  if (readDID(0xF40C, &raw)) valRPM = raw / 4.0; // Standart OBD RPM formülü

  // Hız (Speed) DID: 0xF40D
  if (readDID(0xF40D, &raw)) valSpeed = raw; 

  // Boost (MAP) DID: 0x181A (Kullanıcının gönderdiği koddan)
  if (readDID(0x181A, &raw)) valBoost = (raw * 0.25f) / 1000.0f;

  // Coolant DID: 0xF405 (Kullanıcının gönderdiği koddan)
  if (readDID(0xF405, &raw)) valCoolant = (float)((uint8_t)raw) - 40.0f;

  updateScreen();
  delay(100); // 10 FPS tazeleme hızı
}
