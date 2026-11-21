// =============================================================================
// VAG EA288 1.6 TDI (CLHA) REAL-TIME MFD DISPLAY
// Version: v1.3.0 | Platform: VAG MQB (Skoda / VW / Seat / Audi)
// Hardware: Arduino Nano (ATmega328P) + MCP2515 (8MHz/500kbps) + SSD1306 OLED
// =============================================================================

#include <SPI.h>
#include <mcp2515.h>
#include <U8g2lib.h>
#include <stdlib.h>

#define SKETCH_VERSION "v1.3"

// === HARDWARE PINS ===
#define PIN_CAN_CS  10
#define PIN_CAN_INT 2
#define PIN_OLED_CS 9
#define PIN_OLED_DC 8
#define PIN_OLED_RES 7

// === OLED & CAN GLOBALS ===
U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, PIN_OLED_CS, PIN_OLED_DC, PIN_OLED_RES);
MCP2515 can(PIN_CAN_CS);

// === FORMULA DEFINITIONS ===
enum FormulaType : uint8_t {
  FMT_DIV100,       // raw / 100
  FMT_DIV10,        // raw / 10
  FMT_DIV10_M2731,  // raw / 10 - 273.1 (Kelvin -> Celsius)
  FMT_BYTE_M40,     // (uint8_t)raw - 40 (OBD Coolant)
  FMT_BOOST_REL,    // (raw * 0.25 - 1000) / 1000.0 (Relatif bar - Vakum/Basınç)
  FMT_DIV50,        // raw / 50 (Battery Voltage)
  FMT_U32_M_TO_KM,  // 4-byte meters -> km (raw / 1000.0)
  FMT_U32_S_TO_MIN, // 4-byte seconds -> minutes (raw / 60.0)
  FMT_DIV_81_92,    // raw / 81.92 (Alternator Load %)
  FMT_RAW           // Raw integer value
};

// === DID ENTRY STRUCTURE ===
struct DID_Def {
  uint16_t    did;
  FormulaType fmt;
  uint8_t     byteLen;  // 1, 2, or 4
};

// === MASTER DID LIST (PROGMEM - All 26 DIDs preserved) ===
#define TOTAL_DIDS 26
const DID_Def master_dids[TOTAL_DIDS] PROGMEM = {
  {0xF405, FMT_BYTE_M40,     1}, // 0:  Coolant (Page 2)
  {0x11BE, FMT_DIV10_M2731,  2}, // 1:  Oil Temp (Page 2)
  {0x181A, FMT_BOOST_REL,    2}, // 2:  Boost Relatif (Page 0)
  {0x105E, FMT_DIV50,        2}, // 3:  Battery (Top Bar on all pages)
  {0x100C, FMT_DIV100,       2}, // 4:  Fuel (Page 4)
  {0x114F, FMT_DIV100,       2}, // 5:  Soot Calc (Page 1)
  {0x114E, FMT_DIV100,       2}, // 6:  Soot Meas (Preserved in master list)
  {0x14F5, FMT_DIV10,        2}, // 7:  DPF dP (Page 1)
  {0x178C, FMT_DIV10,        2}, // 8:  Oil Ash (Page 1)
  {0x1156, FMT_U32_M_TO_KM,  4}, // 9:  Dist/Regen (Page 1)
  {0x115E, FMT_U32_S_TO_MIN, 4}, // 10: Time/Regen (Page 1)
  {0x11F6, FMT_DIV100,       2}, // 11: Limit Field (Page 1)
  {0x11B2, FMT_DIV10_M2731,  2}, // 12: EGT Pre (Page 2)
  {0x10F9, FMT_DIV10_M2731,  2}, // 13: EGT Post (Page 2)
  {0x1628, FMT_DIV10_M2731,  2}, // 14: Intake Temp (Page 4)
  {0x163A, FMT_DIV100,       2}, // 15: Main Inj (Page 3)
  {0x163B, FMT_DIV100,       2}, // 16: Post Inj 2 (Page 3)
  {0x167D, FMT_DIV100,       2}, // 17: Post Inj 3 (Page 3)
  {0x1314, FMT_DIV100,       2}, // 18: LP-EGR (Page 3)
  {0x105C, FMT_DIV_81_92,    2}, // 19: Gen Load (Page 3)
  {0x1116, FMT_DIV100,       2}, // 20: Fan 1 (Page 3)
  {0x1114, FMT_DIV100,       2}, // 21: Fan 2 (Preserved in master list)
  {0x16A9, FMT_RAW,          4}, // 22: Odometer km (Page 4)
  {0x1189, FMT_RAW,          1}, // 23: Aux Heat (Preserved in master list)
  {0x11B6, FMT_RAW,          2}, // 24: Regen Count (Page 4)
  {0x1347, FMT_U32_S_TO_MIN, 4}  // 25: Regen Dur (Page 4)
};

// === PAGE DEFINITIONS (PROGMEM - Optimized Query Sets) ===
const uint8_t page0_dids[] PROGMEM = {2};                           // Page 0 (Boost only)
const uint8_t page1_dids[] PROGMEM = {5, 7, 8, 9, 10, 11};         // Page 1 (DPF: Soot, dP, Ash, Dist, Time, Limit)
const uint8_t page2_dids[] PROGMEM = {0, 1, 12, 13};               // Page 2 (Temp: Coolant, Oil, EGT Pre, EGT Post)
const uint8_t page3_dids[] PROGMEM = {15, 16, 17, 18, 19, 20};     // Page 3 (Motor: Main, Post2, Post3, EGR, Alt, Fan)
const uint8_t page4_dids[] PROGMEM = {22, 4, 14, 24, 25};          // Page 4 (Sistem: KM, Fuel, Intake, Regen#, RegenDur)

const uint8_t* const page_did_ptrs[] PROGMEM = {page0_dids, page1_dids, page2_dids, page3_dids, page4_dids};
const uint8_t page_did_counts[] PROGMEM = {1, 6, 4, 6, 5};

// === PER-PAGE DISPLAY DURATIONS (Page 0 = 20s, Pages 1-4 = 5s) ===
const unsigned long PAGE_DURATIONS[5] = {20000, 5000, 5000, 5000, 5000};

// === RUNTIME VALUES ===
static uint32_t didValues[TOTAL_DIDS];
static uint8_t  didStatus[TOTAL_DIDS];  // 0=no data, 1=ok, 2=error
static char g_buf[24];

// === UDS FUNCTIONS ===
static void clearCanBuffer() {
  struct can_frame rx;
  while (can.readMessage(&rx) == MCP2515::ERROR_OK) {}
}

static void sendUDS(uint16_t did) {
  struct can_frame frame;
  frame.can_id = 0x7E0;
  frame.can_dlc = 8;
  frame.data[0] = 0x03; // PCI: Single frame, 3 bytes
  frame.data[1] = 0x22; // Service 0x22 (ReadDataByIdentifier)
  frame.data[2] = (did >> 8) & 0xFF;
  frame.data[3] = did & 0xFF;
  frame.data[4] = 0x55; // Padding
  frame.data[5] = 0x55;
  frame.data[6] = 0x55;
  frame.data[7] = 0x55;
  can.sendMessage(&frame);
}

static bool waitUDSResponse(uint16_t did, uint8_t byteLen, uint32_t* outVal) {
  unsigned long startT = millis();
  struct can_frame rx;
  bool isFirstFrame = false;
  uint8_t dataBuf[8] = {0}; 
  
  while (millis() - startT < 150) {
    if (can.readMessage(&rx) == MCP2515::ERROR_OK && rx.can_id == 0x7E8) {
      uint8_t pciType = rx.data[0] >> 4;
      
      if (pciType == 0) { // Single Frame
        if (rx.data[1] == 0x62 && rx.data[2] == ((did >> 8) & 0xFF) && rx.data[3] == (did & 0xFF)) {
           uint32_t val = 0;
           for (uint8_t i = 0; i < byteLen && i < 4; i++) {
             val = (val << 8) | rx.data[4 + i];
           }
           *outVal = val;
           return true;
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
          can.sendMessage(&fc);
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
        }
        *outVal = val;
        return true;
      }
    }
  }
  return false;
}

static void queryDID(uint8_t idx) {
  if (idx >= TOTAL_DIDS) return;
  uint16_t did;
  uint8_t len;
  
  did = pgm_read_word(&master_dids[idx].did);
  len = pgm_read_byte(&master_dids[idx].byteLen);
  
  clearCanBuffer();
  sendUDS(did);
  uint32_t val = 0;
  if (waitUDSResponse(did, len, &val)) {
    didValues[idx] = val;
    didStatus[idx] = 1;
  } else {
    didStatus[idx] = 2; // Error/Timeout
  }
}

static float getFormattedFloat(uint8_t idx) {
  if (idx >= TOTAL_DIDS || didStatus[idx] != 1) return 0.0;
  uint32_t raw = didValues[idx];
  FormulaType fmt = (FormulaType)pgm_read_byte(&master_dids[idx].fmt);
  
  switch(fmt) {
    case FMT_DIV100:       return raw / 100.0f;
    case FMT_DIV10:        return raw / 10.0f;
    case FMT_DIV10_M2731:  return (raw / 10.0f) - 273.1f;
    case FMT_BYTE_M40:     return (float)((uint8_t)raw) - 40.0f;
    case FMT_BOOST_REL:    return ((float)raw * 0.25f - 1000.0f) / 1000.0f;
    case FMT_DIV50:        return raw / 50.0f;
    case FMT_U32_M_TO_KM:  return raw / 1000.0f;
    case FMT_U32_S_TO_MIN: return raw / 60.0f;
    case FMT_DIV_81_92:    return raw / 81.92f;
    case FMT_RAW:          return (float)raw;
    default:               return 0.0f;
  }
}

static void queryPageDIDs(uint8_t page) {
  // Always query Battery (DID 3) for the top status bar
  queryDID(3);
  
  uint8_t count = pgm_read_byte(&page_did_counts[page]);
  const uint8_t* ptr = (const uint8_t*)pgm_read_word(&page_did_ptrs[page]);
  
  for (uint8_t i = 0; i < count; i++) {
    uint8_t idx = pgm_read_byte(&ptr[i]);
    if (idx != 3) {
      queryDID(idx);
    }
  }
}

// === DRAWING HELPERS ===
void drawInvertText(u8g2_uint_t x, u8g2_uint_t y, const char* text, bool invert) {
  if (invert) {
    u8g2_uint_t w = u8g2.getStrWidth(text) + 2;
    int8_t a = u8g2.getAscent();
    int8_t d = u8g2.getDescent();
    int8_t h = a - d + 2;
    u8g2.setDrawColor(1);
    u8g2.drawBox(x - 1, y - a - 1, w, h);
    u8g2.setDrawColor(0);
  }
  u8g2.drawStr(x, y, text);
  u8g2.setDrawColor(1); // Restore normal color
}

void drawStatusBar(const char* title) {
  u8g2.setFont(u8g2_font_5x7_tr);
  
  // Left: Page Title
  u8g2.drawStr(0, 7, title);
  
  // Right: Battery Voltage (DID 3)
  if (didStatus[3] == 1) {
    float b = getFormattedFloat(3);
    dtostrf(b, 4, 1, g_buf);
    strcat(g_buf, "V");
    bool invert = (b < 11.5f);
    uint8_t strW = u8g2.getStrWidth(g_buf);
    drawInvertText(128 - strW, 7, g_buf, invert);
  } else {
    u8g2.drawStr(98, 7, "--.-V");
  }
  
  u8g2.drawLine(0, 9, 128, 9);
}

// === 10-SECOND PEAK BOOST TRACKING ===
static float peakBoost = -1.0f;
static unsigned long peakTimestamp = 0;

static void updatePeakBoost(float cur) {
  unsigned long now = millis();
  if (cur >= peakBoost || peakBoost < -0.9f) {
    peakBoost = cur;
    peakTimestamp = now;
  } else if (now - peakTimestamp >= 10000) { // 10s hold window expired
    peakBoost = cur;
    peakTimestamp = now;
  }
}

// === PAGE 0: BOOST & 10s PEAK GAUGE (20s) ===
void drawPage0() {
  drawStatusBar("\x04 BOOST");
  
  if (didStatus[2] == 1) {
    float boost = getFormattedFloat(2);
    updatePeakBoost(boost);
    
    // Main Current Boost Value (Large Font)
    char numBuf[10];
    dtostrf(boost, 4, 2, numBuf);
    if (boost > 0.005f) {
      snprintf(g_buf, sizeof(g_buf), "+%s bar", numBuf);
    } else {
      snprintf(g_buf, sizeof(g_buf), "%s bar", numBuf);
    }
    u8g2.setFont(u8g2_font_9x18B_tr);
    uint8_t strW = u8g2.getStrWidth(g_buf);
    u8g2.drawStr((128 - strW) / 2, 27, g_buf);
    
    // Bi-directional Boost / Vacuum Bar Gauge
    // Zero marker line at x=34 (Vacuum left up to -0.6 bar, Boost right up to +1.8 bar)
    u8g2.drawFrame(2, 32, 124, 8);
    u8g2.drawLine(34, 32, 34, 39); // Zero marker line
    if (boost > 0.0f) {
      int barW = (int)((boost / 1.8f) * 91.0f);
      if (barW > 91) barW = 91;
      u8g2.drawBox(35, 32, barW, 8);
    } else if (boost < 0.0f) {
      int vacW = (int)(((-boost) / 0.6f) * 31.0f);
      if (vacW > 31) vacW = 31;
      u8g2.drawBox(34 - vacW, 32, vacW, 8);
    }
    
    // 10-Second Peak Boost Display
    char peakBuf[10];
    dtostrf(peakBoost, 4, 2, peakBuf);
    if (peakBoost > 0.005f) {
      snprintf(g_buf, sizeof(g_buf), "10s PIK: +%s bar", peakBuf);
    } else {
      snprintf(g_buf, sizeof(g_buf), "10s PIK: %s bar", peakBuf);
    }
    u8g2.setFont(u8g2_font_6x10_tr);
    strW = u8g2.getStrWidth(g_buf);
    u8g2.drawStr((128 - strW) / 2, 56, g_buf);
  } else {
    u8g2.setFont(u8g2_font_9x18B_tr);
    u8g2.drawStr(24, 30, "-.-- bar");
  }
}

// === PAGE 1: DPF STATUS (5s) ===
void drawPage1() {
  drawStatusBar("\x04 DPF");
  
  float sootCalc = 0.0f;
  float limit = 0.0f;
  bool hasSoot = (didStatus[5] == 1);
  bool hasLimit = (didStatus[11] == 1);
  
  if (hasSoot) sootCalc = getFormattedFloat(5);
  if (hasLimit) limit = getFormattedFloat(11);
  
  // 1. Top: Centered Soot amount vs Limit (2 decimals)
  char sBuf[8], lBuf[8];
  if (hasSoot) dtostrf(sootCalc, 4, 2, sBuf); else strcpy(sBuf, "--.--");
  if (hasLimit && limit > 0.0f) dtostrf(limit, 4, 2, lBuf); else strcpy(lBuf, "--.--");
  snprintf(g_buf, sizeof(g_buf), "%sg / %sg", sBuf, lBuf);
  u8g2.setFont(u8g2_font_6x10_tr);
  uint8_t strW = u8g2.getStrWidth(g_buf);
  u8g2.drawStr((128 - strW) / 2, 21, g_buf);
  
  // 2. Soot Progress Bar
  int pct = 0;
  if (hasSoot && hasLimit && limit > 0.0f) {
    pct = (int)((sootCalc / limit) * 100);
  }
  int barW = (pct * 124) / 100;
  if (barW > 124) barW = 124;
  if (barW < 0) barW = 0;
  u8g2.drawFrame(2, 25, 124, 7);
  u8g2.drawBox(2, 25, barW, 7);
  
  // 3. Middle: Left = Oil Ash (Kul), Right = dP (Fark Basinci)
  u8g2.setFont(u8g2_font_6x10_tr);
  if (didStatus[8] == 1) {
    float ash = getFormattedFloat(8);
    dtostrf(ash, 4, 2, sBuf);
    snprintf(g_buf, sizeof(g_buf), "Kul:%sg", sBuf);
  } else {
    strcpy(g_buf, "Kul:--.-g");
  }
  u8g2.drawStr(2, 45, g_buf);
  
  if (didStatus[7] == 1) {
    float dP = getFormattedFloat(7);
    dtostrf(dP, 3, 1, sBuf);
    snprintf(g_buf, sizeof(g_buf), "dP:%smb", sBuf);
  } else {
    strcpy(g_buf, "dP:--mb");
  }
  strW = u8g2.getStrWidth(g_buf);
  u8g2.drawStr(126 - strW, 45, g_buf);
  
  // 4. Bottom: Left = KM since regen, Right = Minutes since regen
  u8g2.setFont(u8g2_font_5x7_tr);
  if (didStatus[9] == 1) {
    int dist = (int)getFormattedFloat(9);
    snprintf(g_buf, sizeof(g_buf), "Regen: %dkm", dist);
  } else {
    strcpy(g_buf, "Regen: --km");
  }
  u8g2.drawStr(2, 60, g_buf);
  
  if (didStatus[10] == 1) {
    int timeM = (int)getFormattedFloat(10);
    snprintf(g_buf, sizeof(g_buf), "%d dk once", timeM);
  } else {
    strcpy(g_buf, "-- dk");
  }
  strW = u8g2.getStrWidth(g_buf);
  u8g2.drawStr(126 - strW, 60, g_buf);
}

// === PAGE 2: TEMPERATURES 2x2 GRID (5s) ===
void drawPage2() {
  drawStatusBar("\x04 SICAKLIK");
  
  // Top Row (Y=26): Yağ (DID 1) & Su (DID 0)
  u8g2.setFont(u8g2_font_7x14_tr);
  
  // Sol Üst: Yağ
  if (didStatus[1] == 1) {
    float oil = getFormattedFloat(1);
    snprintf(g_buf, sizeof(g_buf), "Yag:%d\xb0" "C", (int)oil);
    bool invert = (oil > 130.0f);
    drawInvertText(2, 26, g_buf, invert);
  } else {
    u8g2.drawStr(2, 26, "Yag:--\xb0" "C");
  }
  
  // Sağ Üst: Su
  if (didStatus[0] == 1) {
    float coolant = getFormattedFloat(0);
    snprintf(g_buf, sizeof(g_buf), "Su:%d\xb0" "C", (int)coolant);
    bool invert = (coolant > 110.0f);
    uint8_t strW = u8g2.getStrWidth(g_buf);
    drawInvertText(126 - strW, 26, g_buf, invert);
  } else {
    u8g2.drawStr(74, 26, "Su:--\xb0" "C");
  }
  
  // Divider line
  u8g2.drawLine(0, 33, 128, 33);
  
  // Bottom Row (Y=54): Ön DPF (DID 12) & Son DPF (DID 13) - Large & Bold!
  u8g2.setFont(u8g2_font_7x14B_tr);
  
  // Sol Alt: Ön DPF
  if (didStatus[12] == 1) {
    float egtPre = getFormattedFloat(12);
    snprintf(g_buf, sizeof(g_buf), "On:%d\xb0" "C", (int)egtPre);
    bool invert = (egtPre > 650.0f);
    drawInvertText(2, 53, g_buf, invert);
  } else {
    u8g2.drawStr(2, 53, "On:--\xb0" "C");
  }
  
  // Sağ Alt: Son DPF
  if (didStatus[13] == 1) {
    float egtPost = getFormattedFloat(13);
    snprintf(g_buf, sizeof(g_buf), "Son:%d\xb0" "C", (int)egtPost);
    bool invert = (egtPost > 650.0f);
    uint8_t strW = u8g2.getStrWidth(g_buf);
    drawInvertText(126 - strW, 53, g_buf, invert);
  } else {
    u8g2.drawStr(66, 53, "Son:--\xb0" "C");
  }
}

// === PAGE 3: MOTOR 3x2 GRID (5s) ===
void drawPage3() {
  drawStatusBar("\x04 MOTOR");
  u8g2.setFont(u8g2_font_6x10_tr);
  
  char valBuf[8];
  
  // Satır 1: Sol = Ana Enj (DID 15), Sağ = EGR (DID 18)
  if (didStatus[15] == 1) {
    dtostrf(getFormattedFloat(15), 4, 1, valBuf);
    snprintf(g_buf, sizeof(g_buf), "Ana:%smg", valBuf);
  } else strcpy(g_buf, "Ana:--mg");
  u8g2.drawStr(2, 23, g_buf);
  
  if (didStatus[18] == 1) {
    dtostrf(getFormattedFloat(18), 3, 1, valBuf);
    snprintf(g_buf, sizeof(g_buf), "EGR:%s%%", valBuf);
  } else strcpy(g_buf, "EGR:--%");
  u8g2.drawStr(70, 23, g_buf);
  
  // Satır 2: Sol = Post2 (DID 16), Sağ = Post3 (DID 17)
  if (didStatus[16] == 1) {
    dtostrf(getFormattedFloat(16), 4, 2, valBuf);
    snprintf(g_buf, sizeof(g_buf), "P2:%smg", valBuf);
  } else strcpy(g_buf, "P2:--mg");
  u8g2.drawStr(2, 40, g_buf);
  
  if (didStatus[17] == 1) {
    dtostrf(getFormattedFloat(17), 4, 2, valBuf);
    snprintf(g_buf, sizeof(g_buf), "P3:%smg", valBuf);
  } else strcpy(g_buf, "P3:--mg");
  u8g2.drawStr(70, 40, g_buf);
  
  // Satır 3: Sol = Alternatör (DID 19), Sağ = Fan Hızı (DID 20)
  if (didStatus[19] == 1) {
    dtostrf(getFormattedFloat(19), 3, 1, valBuf);
    snprintf(g_buf, sizeof(g_buf), "Alt:%s%%", valBuf);
  } else strcpy(g_buf, "Alt:--%");
  u8g2.drawStr(2, 57, g_buf);
  
  if (didStatus[20] == 1) {
    int fan = (int)getFormattedFloat(20);
    snprintf(g_buf, sizeof(g_buf), "Fan:%d%%", fan);
  } else strcpy(g_buf, "Fan:--%");
  u8g2.drawStr(70, 57, g_buf);
}

// === PAGE 4: SYSTEM (5s) ===
void drawPage4() {
  drawStatusBar("\x04 SISTEM");
  
  // 1. Üst Orta: KM (DID 22)
  u8g2.setFont(u8g2_font_7x14B_tr);
  if (didStatus[22] == 1) {
    uint32_t km = didValues[22];
    snprintf(g_buf, sizeof(g_buf), "%lu km", km);
  } else {
    strcpy(g_buf, "-- km");
  }
  uint8_t strW = u8g2.getStrWidth(g_buf);
  u8g2.drawStr((128 - strW) / 2, 24, g_buf);
  
  u8g2.drawLine(0, 27, 128, 27);
  
  // 2. Orta Satır (Y=42): Sol = Yakıt (DID 4), Sağ = Emme Sıcaklığı (DID 14)
  u8g2.setFont(u8g2_font_6x10_tr);
  char valBuf[8];
  if (didStatus[4] == 1) {
    dtostrf(getFormattedFloat(4), 4, 1, valBuf);
    snprintf(g_buf, sizeof(g_buf), "Yakit:%sL", valBuf);
  } else {
    strcpy(g_buf, "Yakit:--L");
  }
  u8g2.drawStr(2, 42, g_buf);
  
  if (didStatus[14] == 1) {
    float intake = getFormattedFloat(14);
    snprintf(g_buf, sizeof(g_buf), "Emme:%d\xb0" "C", (int)intake);
  } else {
    strcpy(g_buf, "Emme:--\xb0" "C");
  }
  strW = u8g2.getStrWidth(g_buf);
  u8g2.drawStr(126 - strW, 42, g_buf);
  
  // 3. Alt Satır (Y=58): Sol = Regen Sayacı (DID 24), Sağ = Regen Süresi (DID 25)
  if (didStatus[24] == 1) {
    int rCnt = (int)getFormattedFloat(24);
    snprintf(g_buf, sizeof(g_buf), "Regen:#%d", rCnt);
  } else {
    strcpy(g_buf, "Regen:#--");
  }
  u8g2.drawStr(2, 58, g_buf);
  
  if (didStatus[25] == 1) {
    int rDur = (int)getFormattedFloat(25);
    snprintf(g_buf, sizeof(g_buf), "Sur:%ddk", rDur);
  } else {
    strcpy(g_buf, "Sur:--dk");
  }
  strW = u8g2.getStrWidth(g_buf);
  u8g2.drawStr(126 - strW, 58, g_buf);
}

void drawWelcome() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawFrame(0, 0, 128, 64);
    u8g2.drawStr(24, 15, "EA288 1.6 TDI");
    u8g2.drawStr(16, 28, "MFD Display " SKETCH_VERSION);
    u8g2.drawStr(20, 45, "Baslatiliyor...");
    u8g2.drawLine(0, 52, 128, 52);
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(10, 61, "500kbps   0x7E0");
  } while (u8g2.nextPage());
}

// === MAIN LOGIC ===
void setup() {
  u8g2.begin();
  u8g2.setContrast(255);
  
  drawWelcome();
  delay(1500);
  
  can.reset();
  can.setBitrate(CAN_500KBPS, MCP_8MHZ);
  can.setNormalMode();
}

void loop() {
  static uint8_t currentPage = 0;
  static unsigned long lastPageTime = 0;
  
  // Custom display duration per page (Page 0 = 20s, Pages 1-4 = 5s)
  if (millis() - lastPageTime >= PAGE_DURATIONS[currentPage]) {
    currentPage = (currentPage + 1) % 5;
    lastPageTime = millis();
  }
  
  queryPageDIDs(currentPage);
  
  u8g2.firstPage();
  do {
    switch (currentPage) {
      case 0: drawPage0(); break;
      case 1: drawPage1(); break;
      case 2: drawPage2(); break;
      case 3: drawPage3(); break;
      case 4: drawPage4(); break;
    }
  } while (u8g2.nextPage());
}
