// ==========================================================================
// VAG EA288 (1.6 TDI - CLHA) UDS DID ADRES DOĞRULAYICI - v6 (ISO-TP MASTER)
// Arduino Nano + MCP2515 (8 MHz kristal, 500 kbps) + SSD1306 128x64 SPI
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

// ---------- CAN / UDS PROTOKOL AYARLARI ----------
#define CAN_TX_ID     0x7E0
#define CAN_RX_ID     0x7E8
#define SCAN_TIMEOUT_MS  400
#define DISPLAY_DELAY_MS 3000

// ==========================================================================
// FORMÜL TİPLERİ VE DID TABLOSU (PROGMEM)
// ==========================================================================
enum FormulaType : uint8_t {
  FMT_DIV100,       // raw / 100
  FMT_DIV10,        // raw / 10
  FMT_DIV10_M2731,  // raw / 10 - 273.1 (Kelvin -> Celsius)
  FMT_BYTE_M40,     // A - 40 (1-bayt Standart OBD Coolant)
  FMT_BOOST_MBAR,   // (raw * 0.25) / 1000 (bar - MAP Mutlak Basınç)
  FMT_DIV50,        // raw / 50 (Akü Voltajı)
  FMT_U32_M_TO_KM,  // 4-bayt metre -> km (raw32 / 1000)
  FMT_U32_S_TO_MIN, // 4-bayt saniye -> dakika (raw32 / 60)
  FMT_DIV_81_92,    // raw / 81.92 (Alternatör Yükü %)
  FMT_RAW           // Ham tamsayı / Bitwise durum
};

struct DID_Entry {
  uint16_t    did;
  char        label[13];
  char        unit[7];
  FormulaType fmt;
  uint8_t     byteLen;
};

// Yeni Eklenenler dahil olmak üzere DID listesi
static const DID_Entry DID_TABLE[] PROGMEM = {
  { 0x114F, "Soot Calc",   "g",     FMT_DIV100,       2 },
  { 0x114E, "Soot Meas",   "g",     FMT_DIV100,       2 },
  { 0x14F5, "DPF dP",      "mbar",  FMT_DIV10,        2 },
  { 0x178C, "Oil Ash",     "g",     FMT_DIV10,        2 },
  { 0x11B2, "EGT Pre-DPF", "C",     FMT_DIV10_M2731,  2 },
  { 0x10F9, "EGT PostDPF", "C",     FMT_DIV10_M2731,  2 },
  { 0x11BE, "Oil Temp",    "C",     FMT_DIV10_M2731,  2 },
  { 0x1628, "Intake Temp", "C",     FMT_DIV10_M2731,  2 },
  { 0xF405, "Coolant",     "C",     FMT_BYTE_M40,     1 },
  { 0x1156, "Dist/Regen",  "km",    FMT_U32_M_TO_KM,  4 },
  { 0x115E, "Time/Regen",  "min",   FMT_U32_S_TO_MIN, 4 },
  { 0x1347, "Regen Dur",   "min",   FMT_U32_S_TO_MIN, 4 },
  { 0x11B6, "Inj Allowed", "adet",  FMT_RAW,          2 },
  { 0x16A9, "Odometer",    "km",    FMT_U32_M_TO_KM,  4 },
  { 0x181A, "Boost",       "bar",   FMT_BOOST_MBAR,   2 },
  { 0x163A, "Main Inj",    "mg/st", FMT_DIV100,       2 },
  { 0x1314, "LP-EGR",      "%",     FMT_DIV100,       2 },
  { 0x105C, "Gen Load",    "%",     FMT_DIV_81_92,    2 },
  { 0x105E, "Battery",     "V",     FMT_DIV50,        2 },
  { 0x100C, "Fuel Level",  "L",     FMT_DIV100,       2 }
};

#define DID_COUNT ((uint8_t)(sizeof(DID_TABLE) / sizeof(DID_Entry)))

struct DID_Result {
  uint32_t raw;
  uint8_t  flags;
};
DID_Result results[DID_COUNT];
char g_buf[24];

MCP2515 can(PIN_CAN_CS);
U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, PIN_OLED_CS, PIN_OLED_DC, PIN_OLED_RES);

#define MAX_PAYLOAD 32
uint8_t rxBuffer[MAX_PAYLOAD];
uint16_t rxLen = 0;

static void readEntry(uint8_t idx, DID_Entry* e) { memcpy_P(e, &DID_TABLE[idx], sizeof(DID_Entry)); }

static float applyFormula(uint32_t raw, FormulaType fmt) {
  switch (fmt) {
    case FMT_DIV100:       return raw / 100.0f;
    case FMT_DIV10:        return raw / 10.0f;
    case FMT_DIV10_M2731:  return (raw / 10.0f) - 273.1f;
    case FMT_BYTE_M40:     return (float)((uint8_t)raw) - 40.0f;
    case FMT_BOOST_MBAR:   return (raw * 0.25f) / 1000.0f;
    case FMT_DIV50:        return raw / 50.0f;
    case FMT_U32_M_TO_KM:  return raw / 1000.0f;
    case FMT_U32_S_TO_MIN: return raw / 60.0f;
    case FMT_DIV_81_92:    return raw / 81.92f;
    default:               return (float)raw;
  }
}

void clearCanBuffer() { struct can_frame dummy; while (can.readMessage(&dummy) == MCP2515::ERROR_OK); }

// ISO-TP Destekli waitUDSResponse
bool waitUDSResponse(uint16_t reqId, uint16_t resId, uint8_t expectedSvc, uint16_t expectedDid, uint8_t* outBuf, uint16_t maxLen, uint16_t* outLen, bool* negOut) {
  unsigned long t0 = millis();
  struct can_frame f;
  uint16_t currentLen = 0, totalLen = 0;
  uint8_t expectedSeq = 1;
  while ((millis() - t0) < SCAN_TIMEOUT_MS) {
    if (can.readMessage(&f) != MCP2515::ERROR_OK) continue;
    if (f.can_id != resId) continue;
    uint8_t pciType = f.data[0] & 0xF0;

    if (pciType == 0x00) { // SF
      uint8_t sfLen = f.data[0] & 0x0F;
      if (f.data[1] == 0x7F && f.data[2] == (expectedSvc - 0x40)) { *negOut = true; return true; }
      if (f.data[1] == expectedSvc) {
        uint16_t resDid = ((uint16_t)f.data[2] << 8) | f.data[3];
        if (resDid == expectedDid) {
          *negOut = false; currentLen = sfLen - 3;
          if (currentLen > maxLen) currentLen = maxLen;
          for (uint16_t i = 0; i < currentLen; i++) outBuf[i] = f.data[4 + i];
          *outLen = currentLen; return true;
        }
      }
    }
    else if (pciType == 0x10) { // FF
      totalLen = (((uint16_t)(f.data[0] & 0x0F)) << 8) | f.data[1];
      if (f.data[2] == expectedSvc) {
        uint16_t resDid = ((uint16_t)f.data[3] << 8) | f.data[4];
        if (resDid == expectedDid) {
          *negOut = false;
          for (uint8_t i = 0; i < 3; i++) { if (currentLen < maxLen) outBuf[currentLen++] = f.data[5 + i]; }
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
        if (currentLen >= (totalLen - 3) || currentLen >= maxLen) { *outLen = currentLen; return true; }
        t0 = millis();
      }
    }
  }
  return false;
}

void sendUDS(uint16_t did) {
  clearCanBuffer();
  struct can_frame f;
  f.can_id = CAN_TX_ID; f.can_dlc = 8;
  f.data[0] = 0x03; f.data[1] = 0x22;
  f.data[2] = (did >> 8) & 0xFF; f.data[3] = did & 0xFF;
  for (int i=4; i<8; i++) f.data[i] = 0x55;
  can.sendMessage(&f);
}

void drawWelcome() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.drawStr(10, 22, "EA288 1.6TDI");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(8,  36, "ISO-TP Master v6");
    u8g2.drawStr(14, 50, "Baslatiliyor..");
  } while (u8g2.nextPage());
}

void drawScanScreen(uint8_t idx) {
  DID_Entry e; readEntry(idx, &e);
  uint8_t pct = (uint8_t)((uint32_t)idx * 100 / DID_COUNT);
  uint8_t filled = (uint8_t)((uint32_t)pct * 124 / 100);

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "EA288 UDS TARAMA");
    snprintf(g_buf, sizeof(g_buf), "0x%04X  %u/%u", e.did, idx + 1, DID_COUNT);
    u8g2.drawStr(0, 24, g_buf);
    u8g2.drawStr(0, 36, e.label);
    u8g2.drawFrame(0, 44, 128, 10);
    if (filled > 0) u8g2.drawBox(2, 46, filled, 6);
  } while (u8g2.nextPage());
}

void drawResult(uint8_t idx) {
  DID_Entry e; readEntry(idx, &e);
  DID_Result r = results[idx];
  bool ok = r.flags & 0x01;
  bool neg = r.flags & 0x02;

  char valStr[24];
  if (!ok) strcpy(valStr, "NO RESP");
  else if (neg) strcpy(valStr, "NEG 7F");
  else {
    if (e.fmt == FMT_RAW) snprintf(valStr, sizeof(valStr), "%lu %s", r.raw, e.unit);
    else {
      float v = applyFormula(r.raw, e.fmt);
      char fBuf[12];
      dtostrf(v, 1, 2, fBuf);
      snprintf(valStr, sizeof(valStr), "%s %s", fBuf, e.unit);
    }
  }

  uint8_t strW = u8g2.getStrWidth(valStr);
  uint8_t xPos = (strW < 128) ? (128 - strW) / 2 : 0;

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tf);
    snprintf(g_buf, sizeof(g_buf), "%u/%u  0x%04X", idx + 1, DID_COUNT, e.did);
    u8g2.drawStr(0, 10, g_buf);
    u8g2.setFont(u8g2_font_7x14_tf);
    u8g2.drawStr(0, 28, e.label);
    u8g2.setFont(u8g2_font_9x18B_tf);
    u8g2.drawStr(xPos, 50, valStr);
  } while (u8g2.nextPage());
}

void setup() {
  u8g2.begin();
  drawWelcome(); delay(1500);

  can.reset();
  can.setBitrate(CAN_500KBPS, MCP_8MHZ);
  can.setNormalMode();

  for (uint8_t i = 0; i < DID_COUNT; i++) {
    DID_Entry e; readEntry(i, &e);
    drawScanScreen(i);
    sendUDS(e.did);

    bool neg = false;
    rxLen = 0;
    bool got = waitUDSResponse(CAN_TX_ID, CAN_RX_ID, 0x62, e.did, rxBuffer, MAX_PAYLOAD, &rxLen, &neg);

    uint32_t val = 0;
    if (got && !neg) {
      if (rxLen == 1) val = rxBuffer[0];
      else if (rxLen == 2) val = ((uint16_t)rxBuffer[0] << 8) | rxBuffer[1];
      else if (rxLen >= 4) val = ((uint32_t)rxBuffer[0] << 24) | ((uint32_t)rxBuffer[1] << 16) | ((uint32_t)rxBuffer[2] << 8) | rxBuffer[3];
    }

    results[i].raw = (got && !neg) ? val : 0;
    results[i].flags = (got ? 0x01 : 0x00) | (neg ? 0x02 : 0x00);
    delay(40);
  }
}

void loop() {
  for (uint8_t i = 0; i < DID_COUNT; i++) {
    drawResult(i);
    delay(DISPLAY_DELAY_MS);
  }
}
