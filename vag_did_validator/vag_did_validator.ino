// =============================================================================
// VAG EA288 UDS DID ADDRESS VALIDATOR (MULTI-MODULE DIAGNOSTIC TOOL)
// Version: v6.1.0 | Platform: VAG MQB (Skoda / VW / Seat / Audi)
// Hardware: Arduino Nano (ATmega328P) + MCP2515 (8MHz/500kbps) + SSD1306 OLED
// =============================================================================

#include <SPI.h>
#include <mcp2515.h>
#include <U8g2lib.h>
#include <stdlib.h>

#define SKETCH_VERSION "v6.1"

// --- Hardware Pin Definitions ---
#define PIN_CAN_CS  10
#define PIN_CAN_INT 2
#define PIN_OLED_CS 9
#define PIN_OLED_DC 8
#define PIN_OLED_RES 7

// --- Display & CAN Instances ---
U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, PIN_OLED_CS, PIN_OLED_DC, PIN_OLED_RES);
MCP2515 can(PIN_CAN_CS);

// --- Global Buffer ---
static char g_buf[24];

// --- Module Definitions ---
struct ModuleDef {
  uint16_t txId;
  uint16_t rxId;
  char     name[9];   // 8 chars + null
  uint8_t  didStart;  // Start index in master DID table
  uint8_t  didCount;  // Number of DIDs for this module
};

static const ModuleDef MODULES[] PROGMEM = {
  {0x7E0, 0x7E8, "ECU"     , 0,  28},  // Motor ECU
  {0x7E1, 0x7E9, "TCU"     , 28,  4},  // Şanzıman
  {0x714, 0x77E, "KOMBI"   , 32,  4},  // Gösterge
  {0x710, 0x77A, "GATEWAY" , 36,  4},  // Ağ Geçidi
};
#define MODULE_COUNT 4

// --- DID Definitions ---
enum FormulaType : uint8_t {
  FMT_DIV100,       // raw / 100
  FMT_DIV10,        // raw / 10
  FMT_DIV10_M2731,  // raw / 10 - 273.1 (Kelvin -> Celsius)
  FMT_BYTE_M40,     // (uint8_t)raw - 40
  FMT_BOOST_MBAR,   // (raw * 0.25) / 1000 (bar)
  FMT_DIV50,        // raw / 50
  FMT_U32_M_TO_KM,  // 4-byte metres -> km
  FMT_U32_S_TO_MIN, // 4-byte seconds -> minutes
  FMT_DIV_81_92,    // raw / 81.92
  FMT_RAW           // Raw integer
};

struct DID_Entry {
  uint16_t    did;
  char        label[13]; // Max 12 chars + null
  char        unit[7];   // Max 6 chars + null
  FormulaType fmt;
  uint8_t     byteLen;   // 1, 2, or 4
};

static const DID_Entry DID_TABLE[] PROGMEM = {
  // ===== ECU DIDs (Index 0-27) =====
  // --- DPF ve Egzoz Emisyon ---
  { 0x114F, "Soot Calc",   "g",     FMT_DIV100,       2 },
  { 0x114E, "Soot Meas",   "g",     FMT_DIV100,       2 },
  { 0x14F5, "DPF dP",      "mbar",  FMT_DIV10,        2 },
  { 0x178C, "Oil Ash",     "g",     FMT_DIV10,        2 },
  // --- Sıcaklıklar ---
  { 0x11B2, "EGT Pre-DPF", "C",     FMT_DIV10_M2731,  2 },
  { 0x10F9, "EGT PostDPF", "C",     FMT_DIV10_M2731,  2 },
  { 0x11BE, "Oil Temp",    "C",     FMT_DIV10_M2731,  2 },
  { 0x1628, "Intake Temp", "C",     FMT_DIV10_M2731,  2 },
  { 0xF405, "Coolant",     "C",     FMT_BYTE_M40,     1 },
  // --- Rejenerasyon İstatistikleri ---
  { 0x1156, "Dist/Regen",  "km",    FMT_U32_M_TO_KM,  4 },
  { 0x115E, "Time/Regen",  "min",   FMT_U32_S_TO_MIN, 4 },
  { 0x1347, "Regen Dur",   "min",   FMT_U32_S_TO_MIN, 4 },
  { 0x11B6, "Inj Allowed", "adet",  FMT_RAW,          2 },
  // --- Motor / Sistem / Yakıt ---
  { 0x16A9, "Odometer",    "km",    FMT_RAW,          4 },
  { 0x181A, "Boost",       "bar",   FMT_BOOST_MBAR,   2 },
  { 0x163A, "Main Inj",    "mg/st", FMT_DIV100,       2 },
  { 0x163B, "Post Inj 2",  "mg/st", FMT_DIV100,       2 },
  { 0x167D, "Post Inj 3",  "mg/st", FMT_DIV100,       2 },
  { 0x1314, "LP-EGR",      "%",     FMT_DIV100,       2 },
  { 0x105C, "Gen Load",    "%",     FMT_DIV_81_92,    2 },
  { 0x105E, "Battery",     "V",     FMT_DIV50,        2 },
  { 0x100C, "Fuel Level",  "L",     FMT_DIV100,       2 },
  { 0x1116, "Fan 1",       "%",     FMT_DIV100,       2 },
  { 0x1114, "Fan 2",       "%",     FMT_DIV100,       2 },
  { 0x1189, "Aux Heater",  "st",    FMT_RAW,          1 },
  // --- Fabrika DPF Limitleri ---
  { 0x11F6, "Limit Field", "g",     FMT_DIV100,       2 },
  { 0x12CB, "Limit Svc",   "g",     FMT_DIV100,       2 },
  { 0x1793, "Limit Ash",   "g",     FMT_DIV100,       2 },

  // ===== TCU DIDs (Index 28-31) =====
  { 0xF187, "Part Num",    "",      FMT_RAW,          2 },
  { 0xF189, "SW Ver",      "",      FMT_RAW,          2 },
  { 0xF191, "HW Ver",      "",      FMT_RAW,          2 },
  { 0xF405, "Trans Temp",  "C",     FMT_BYTE_M40,     1 },

  // ===== KOMBI DIDs (Index 32-35) =====
  { 0xF187, "Part Num",    "",      FMT_RAW,          2 },
  { 0xF189, "SW Ver",      "",      FMT_RAW,          2 },
  { 0xF191, "HW Ver",      "",      FMT_RAW,          2 },
  { 0x2173, "VIN",         "",      FMT_RAW,          2 },

  // ===== GATEWAY DIDs (Index 36-39) =====
  { 0xF187, "Part Num",    "",      FMT_RAW,          2 },
  { 0xF189, "SW Ver",      "",      FMT_RAW,          2 },
  { 0xF191, "HW Ver",      "",      FMT_RAW,          2 },
  { 0x04A0, "Install List","",      FMT_RAW,          2 },
};
#define TOTAL_DID_COUNT (sizeof(DID_TABLE) / sizeof(DID_Entry))

// --- Results Storage ---
struct DID_Result {
  uint32_t raw;    // 32-bit raw data
  uint8_t  flags;  // bit0: response received, bit1: negative response
};
DID_Result results[TOTAL_DID_COUNT];

// --- Module Statistics ---
struct ModuleStats {
  uint8_t ok;
  uint8_t neg;
  uint8_t timeout;
};
ModuleStats moduleStats[MODULE_COUNT];

// --- Calculation Helper ---
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

// --- UDS CAN Functions ---
static void clearCAN() {
  struct can_frame f;
  while (can.readMessage(&f) == MCP2515::ERROR_OK) {}
}

static void sendUDS(uint16_t txId, uint16_t did) {
  struct can_frame f;
  f.can_id  = txId;
  f.can_dlc = 8;
  f.data[0] = 0x03;
  f.data[1] = 0x22; // ReadDataByIdentifier
  f.data[2] = (uint8_t)(did >> 8);
  f.data[3] = (uint8_t)(did & 0xFF);
  f.data[4] = f.data[5] = f.data[6] = f.data[7] = 0x55; // Padding
  can.sendMessage(&f);
}

// Wait for response and extract raw value
static bool waitUDSResponse(uint16_t rxId, uint16_t did, uint8_t byteLen, uint32_t* outVal, bool* negOut) {
  struct can_frame f;
  uint32_t startMs = millis();
  *negOut = false;
  *outVal = 0;

  while (millis() - startMs < 300) {
    if (can.readMessage(&f) == MCP2515::ERROR_OK) {
      if (f.can_id == rxId) {
        uint8_t pci = f.data[0] >> 4;
        
        // Single Frame
        if (pci == 0) {
          if (f.data[1] == 0x7F && f.data[2] == 0x22) {
            *negOut = true;
            return true;
          }
          if (f.data[1] == 0x62 && f.data[2] == (did >> 8) && f.data[3] == (did & 0xFF)) {
            if (byteLen == 1) *outVal = f.data[4];
            else if (byteLen == 2) *outVal = ((uint32_t)f.data[4] << 8) | f.data[5];
            else if (byteLen == 4) *outVal = ((uint32_t)f.data[4] << 24) | ((uint32_t)f.data[5] << 16) | ((uint32_t)f.data[6] << 8) | f.data[7];
            return true;
          }
        }
        // First Frame (Iso-TP)
        else if (pci == 1) {
          if (f.data[2] == 0x62 && f.data[3] == (did >> 8) && f.data[4] == (did & 0xFF)) {
            // For this validator, we mostly extract the first few bytes from the First Frame
            // This is sufficient for identification DIDs up to 3 bytes
            if (byteLen == 1) *outVal = f.data[5];
            else if (byteLen == 2) *outVal = ((uint32_t)f.data[5] << 8) | f.data[6];
            return true; 
          }
        }
      }
    }
  }
  return false; // Timeout
}

// --- UI Display Functions ---
static void showWelcome() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_7x14B_tr);
    u8g2.drawStr(5, 12, "EA288 1.6TDI");
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(5, 26, "DID Validator " SKETCH_VERSION);
    u8g2.drawStr(5, 38, "Multi-Modul Tarama");
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(5, 48, "Baslatiliyor..");
    u8g2.drawHLine(0, 52, 128);
    u8g2.drawStr(5, 62, "4 Modul  500kbps");
  } while (u8g2.nextPage());
  delay(2000);
}

static void showScanProgress(const ModuleDef* mod, const DID_Entry* did, uint8_t curr, uint8_t total) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tr);
    snprintf(g_buf, sizeof(g_buf), "%s TARA  %d/%d", mod->name, curr + 1, total);
    u8g2.drawStr(0, 10, g_buf);
    u8g2.drawHLine(0, 14, 128);
    
    snprintf(g_buf, sizeof(g_buf), "0x%04X  %s", did->did, did->label);
    u8g2.drawStr(0, 26, g_buf);
    u8g2.drawStr(0, 38, "Sorguluyor...");
    
    // Progress bar
    uint8_t pct = ((uint16_t)(curr + 1) * 100) / total;
    uint8_t barW = (uint8_t)((uint16_t)pct * 100 / 100);
    if (barW > 100) barW = 100;
    u8g2.drawFrame(0, 42, 102, 8);
    u8g2.drawBox(1, 43, barW, 6);
    snprintf(g_buf, sizeof(g_buf), "%d%%", pct);
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(106, 49, g_buf);
    
    snprintf(g_buf, sizeof(g_buf), "TX:0x%03X RX:0x%03X", mod->txId, mod->rxId);
    u8g2.drawStr(0, 62, g_buf);
  } while (u8g2.nextPage());
}

static void showModuleSummary(const ModuleDef* mod, const ModuleStats* stats) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_7x14B_tr);
    snprintf(g_buf, sizeof(g_buf), "%s TAMAM", mod->name);
    u8g2.drawStr(0, 12, g_buf);
    u8g2.drawHLine(0, 16, 128);
    
    u8g2.setFont(u8g2_font_7x14_tr);
    snprintf(g_buf, sizeof(g_buf), "Toplam  : %d", mod->didCount);
    u8g2.drawStr(0, 30, g_buf);
    snprintf(g_buf, sizeof(g_buf), "Basarili: %d", stats->ok);
    u8g2.drawStr(0, 42, g_buf);
    snprintf(g_buf, sizeof(g_buf), "NegResp : %d", stats->neg);
    u8g2.drawStr(0, 54, g_buf);
    snprintf(g_buf, sizeof(g_buf), "Timeout : %d", stats->timeout);
    u8g2.drawStr(0, 64, g_buf);
  } while (u8g2.nextPage());
  delay(2000);
}

static void showGrandSummary() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_7x14B_tr);
    u8g2.drawStr(0, 12, "TUM MODULLER TAMAM");
    u8g2.drawHLine(0, 16, 128);
    
    u8g2.setFont(u8g2_font_6x10_tr);
    uint8_t y = 28;
    for (uint8_t m = 0; m < MODULE_COUNT; m++) {
      ModuleDef mod;
      memcpy_P(&mod, &MODULES[m], sizeof(ModuleDef));
      snprintf(g_buf, sizeof(g_buf), "%-8s:%2d/%2d [OK]", mod.name, moduleStats[m].ok, mod.didCount);
      u8g2.drawStr(0, y, g_buf);
      y += 12;
    }
  } while (u8g2.nextPage());
  delay(4000);
}

static void drawResult(uint8_t globalIdx, const ModuleDef* mod, uint8_t idxInModule) {
  DID_Entry did;
  memcpy_P(&did, &DID_TABLE[globalIdx], sizeof(DID_Entry));
  DID_Result res = results[globalIdx];

  u8g2.firstPage();
  do {
    // Header
    u8g2.setFont(u8g2_font_6x10_tr);
    snprintf(g_buf, sizeof(g_buf), "%s %d/%d 0x%04X", mod->name, idxInModule + 1, mod->didCount, did.did);
    u8g2.drawStr(0, 10, g_buf);
    u8g2.drawHLine(0, 14, 128);
    
    // Label
    u8g2.setFont(u8g2_font_7x14_tr);
    u8g2.drawStr(0, 28, did.label);
    
    // Result Status / Value
    if (!(res.flags & 0x01)) {
      u8g2.drawStr(20, 46, "TIMEOUT");
    } else if (res.flags & 0x02) {
      u8g2.drawStr(20, 46, "NEG RESP");
    } else {
      u8g2.setFont(u8g2_font_9x18B_tr);
      if (did.fmt == FMT_RAW) {
        snprintf(g_buf, sizeof(g_buf), "%lu %s", res.raw, did.unit);
      } else {
        float val = applyFormula(res.raw, did.fmt);
        dtostrf(val, 6, 2, g_buf);
        uint8_t len = strlen(g_buf);
        snprintf(g_buf + len, sizeof(g_buf) - len, " %s", did.unit);
      }
      // Center roughly
      uint8_t w = u8g2.getStrWidth(g_buf);
      u8g2.drawStr((128 - w) / 2, 46, g_buf);
    }
    
    // Footer
    u8g2.drawHLine(0, 52, 128);
    u8g2.setFont(u8g2_font_5x7_tr);
    const char* st = (res.flags & 0x01) ? ((res.flags & 0x02) ? "[NEG]" : "[OK]") : "[T/O]";
    snprintf(g_buf, sizeof(g_buf), "RAW:0x%04lX %s", res.raw, st);
    u8g2.drawStr(0, 62, g_buf);
    
  } while (u8g2.nextPage());
}

// --- Setup & Main Logic ---
void setup() {
  u8g2.begin();
  SPI.begin();
  can.reset();
  can.setBitrate(CAN_500KBPS, MCP_8MHZ);
  can.setNormalMode();

  showWelcome();

  // Scan all modules
  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    ModuleDef mod;
    memcpy_P(&mod, &MODULES[m], sizeof(ModuleDef));
    
    moduleStats[m].ok = 0;
    moduleStats[m].neg = 0;
    moduleStats[m].timeout = 0;

    for (uint8_t i = 0; i < mod.didCount; i++) {
      uint8_t globalIdx = mod.didStart + i;
      DID_Entry did;
      memcpy_P(&did, &DID_TABLE[globalIdx], sizeof(DID_Entry));
      
      showScanProgress(&mod, &did, i, mod.didCount);
      
      clearCAN();
      sendUDS(mod.txId, did.did);
      
      uint32_t val;
      bool neg;
      bool recv = waitUDSResponse(mod.rxId, did.did, did.byteLen, &val, &neg);
      
      results[globalIdx].raw = val;
      results[globalIdx].flags = 0;
      if (recv) {
        results[globalIdx].flags |= 0x01;
        if (neg) {
          results[globalIdx].flags |= 0x02;
          moduleStats[m].neg++;
        } else {
          moduleStats[m].ok++;
        }
      } else {
        moduleStats[m].timeout++;
      }
      
      delay(40); // 40ms rest
    }
    showModuleSummary(&mod, &moduleStats[m]);
    delay(100);
  }
  
  showGrandSummary();
}

void loop() {
  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    ModuleDef mod;
    memcpy_P(&mod, &MODULES[m], sizeof(ModuleDef));
    for (uint8_t i = 0; i < mod.didCount; i++) {
      uint8_t globalIdx = mod.didStart + i;
      drawResult(globalIdx, &mod, i);
      delay(3000); // 3 seconds per result
    }
  }
}
