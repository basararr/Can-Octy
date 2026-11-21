// ==========================================================================
// VAG EA288 (1.6 TDI - CLHA) UDS DID ADRES DOĞRULAYICI - v5 (MASTER)
// Arduino Nano + MCP2515 (8 MHz kristal, 500 kbps) + SSD1306 128x64 SPI
// Kütüphaneler: autowp/mcp2515  |  olikraus/U8g2
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
#define SVC_READ_DID  0x22
#define SVC_READ_RESP 0x62
#define SVC_NEG_RESP  0x7F

#define SCAN_TIMEOUT_MS  300
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
  char        label[13]; // Maks 12 karakter + null
  char        unit[7];   // Maks 6 karakter + null
  FormulaType fmt;
  uint8_t     byteLen;   // 1, 2 veya 4 bayt
};

// 28 ADET MASTER DID LİSTESİ (SRAM Tasarrufu için Flash/PROGMEM'de saklanır)
static const DID_Entry DID_TABLE[] PROGMEM = {
  // --- DPF VE EGZOZ EMİSYON ---
  { 0x114F, "Soot Calc",   "g",     FMT_DIV100,       2 }, // Hesaplanan Kurum
  { 0x114E, "Soot Meas",   "g",     FMT_DIV100,       2 }, // Ölçülen Kurum
  { 0x14F5, "DPF dP",      "mbar",  FMT_DIV10,        2 }, // DPF Fark Basıncı
  { 0x178C, "Oil Ash",     "g",     FMT_DIV10,        2 }, // Biriken Yağ Külü [YENİ]

  // --- SICAKLIKLAR ---
  { 0x11B2, "EGT Pre-DPF", "C",     FMT_DIV10_M2731,  2 }, // Turbo Çıkış / DPF Giriş Sıcaklığı
  { 0x10F9, "EGT PostDPF", "C",     FMT_DIV10_M2731,  2 }, // DPF Çıkış Sıcaklığı
  { 0x11BE, "Oil Temp",    "C",     FMT_DIV10_M2731,  2 }, // Motor Yağ Sıcaklığı
  { 0x1628, "Intake Temp", "C",     FMT_DIV10_M2731,  2 }, // Emme Manifoldu Sıcaklığı
  { 0xF405, "Coolant",     "C",     FMT_BYTE_M40,     1 }, // Motor Suyu Sıcaklığı

  // --- REJENERASYON İSTATİSTİKLERİ ---
  { 0x1156, "Dist/Regen",  "km",    FMT_U32_M_TO_KM,  4 }, // Son Regenden Beri Mesafe (32-bit)
  { 0x115E, "Time/Regen",  "min",   FMT_U32_S_TO_MIN, 4 }, // Son Regenden Beri Süre (32-bit)
  { 0x1347, "Regen Dur",   "min",   FMT_U32_S_TO_MIN, 4 }, // Aktif Regen Çalışma Süresi (32-bit)
  { 0x11B6, "Inj Allowed", "adet",  FMT_RAW,          2 }, // Regen Döngü Sayacı

  // --- MOTOR / SİSTEM / YAKIT ---
  { 0x16A9, "Odometer",    "km",    FMT_U32_M_TO_KM,  4 }, // ECU Gerçek Kilometresi (32-bit) [YENİ]
  { 0x181A, "Boost",       "bar",   FMT_BOOST_MBAR,   2 }, // Manifold Mutlak Basıncı (MAP)
  { 0x163A, "Main Inj",    "mg/st", FMT_DIV100,       2 }, // Ana Enjeksiyon Miktarı
  { 0x163B, "Post Inj 2",  "mg/st", FMT_DIV100,       2 }, // Geç Enjeksiyon 2 Miktarı
  { 0x167D, "Post Inj 3",  "mg/st", FMT_DIV100,       2 }, // Geç Enjeksiyon 3 Miktarı
  { 0x1314, "LP-EGR",      "%",     FMT_DIV100,       2 }, // Düşük Basınç EGR Valf Açıklığı
  { 0x105C, "Gen Load",    "%",     FMT_DIV_81_92,    2 }, // Alternatör Yükü [YENİ]
  { 0x105E, "Battery",     "V",     FMT_DIV50,        2 }, // Akü Voltajı
  { 0x100C, "Fuel Level",  "L",     FMT_DIV100,       2 }, // Depodaki Yakıt Miktarı
  { 0x1116, "Fan 1",       "%",     FMT_DIV100,       2 }, // Radyatör Fanı 1 PWM
  { 0x1114, "Fan 2",       "%",     FMT_DIV100,       2 }, // Radyatör Fanı 2 PWM
  { 0x1189, "Aux Heater",  "st",    FMT_RAW,          1 }, // Elektrikli Ek Isıtıcı Röleleri [YENİ]

  // --- FABRİKA DPF LİMİTLERİ ---
  { 0x11F6, "Limit Field", "g",     FMT_DIV100,       2 }, // Otomatik Yolda Regen Sınırı (22.77g)
  { 0x12CB, "Limit Svc",   "g",     FMT_DIV100,       2 }, // Manuel Servis Regen Sınırı (40.00g)
  { 0x1793, "Limit Ash",   "g",     FMT_DIV100,       2 }, // DPF Ömür Kül Sınırı (80.00g)
};

#define DID_COUNT ((uint8_t)(sizeof(DID_TABLE) / sizeof(DID_Entry)))

// ==========================================================================
// SONUÇ DEPOLAMA (SRAM TASARRUFLU)
// ==========================================================================
struct DID_Result {
  uint32_t raw;   // 32-bit genişletilmiş ham veri
  uint8_t  flags; // bit0: yanıt alındı, bit1: negatif yanıt (NRC)
};
DID_Result results[DID_COUNT];

static char g_buf[24];

// ==========================================================================
// NESNELER
// ==========================================================================
MCP2515 can(PIN_CAN_CS);

// Sadece 128 bayt SRAM harcayan Page Buffer modu
U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI
    u8g2(U8G2_R0, PIN_OLED_CS, PIN_OLED_DC, PIN_OLED_RES);

// ==========================================================================
// YARDIMCI VE DÖNÜŞÜM FONKSİYONLARI
// ==========================================================================
static void readEntry(uint8_t idx, DID_Entry* e) {
  memcpy_P(e, &DID_TABLE[idx], sizeof(DID_Entry));
}

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

// MCP2515 RX Tamponunu temizleme
static void clearCanBuffer() {
  struct can_frame dummy;
  while (can.readMessage(&dummy) == MCP2515::ERROR_OK);
}

// UDS 0x22 DID Sorgusu Gönderme
static void sendUDS(uint16_t did) {
  struct can_frame f;
  f.can_id  = CAN_TX_ID;
  f.can_dlc = 8;
  f.data[0] = 0x03;          // ISO-TP Single Frame (3 bayt veri)
  f.data[1] = SVC_READ_DID;  // Service 0x22
  f.data[2] = (uint8_t)(did >> 8);
  f.data[3] = (uint8_t)(did & 0xFF);
  f.data[4] = f.data[5] = f.data[6] = f.data[7] = 0x55; // VAG UDS Dolgu
  can.sendMessage(&f);
}

// UDS Yanıtını Bekleme ve Bayt Uzunluğuna Göre Çözümleme
static bool waitUDSResponse(uint16_t did, uint8_t byteLen, uint32_t* outVal, bool* negOut) {
  unsigned long t0 = millis();
  struct can_frame f;
  
  while ((millis() - t0) < SCAN_TIMEOUT_MS) {
    if (can.readMessage(&f) != MCP2515::ERROR_OK) continue;
    if (f.can_id != CAN_RX_ID) continue; // Sadece motor ECU'sunu dinle

    uint8_t pciType = f.data[0] & 0xF0;

    // 1. DURUM: Tekli Paket (Single Frame)
    if (pciType == 0x00) {
      // Negatif Yanıt Kontrolü (0x7F 0x22)
      if (f.data[1] == SVC_NEG_RESP && f.data[2] == SVC_READ_DID) {
        *negOut = true; 
        return true;
      }
      // Pozitif Yanıt Kontrolü (0x62 DID_H DID_L)
      if (f.data[1] == SVC_READ_RESP) {
        uint16_t rd = ((uint16_t)f.data[2] << 8) | f.data[3];
        if (rd == did) {
          *negOut = false;
          if (byteLen == 1) {
            *outVal = f.data[4];
          } else if (byteLen == 2) {
            *outVal = ((uint16_t)f.data[4] << 8) | f.data[5];
          } else if (byteLen >= 4) {
            *outVal = ((uint32_t)f.data[4] << 24) | ((uint32_t)f.data[5] << 16) |
                      ((uint32_t)f.data[6] << 8)  | f.data[7];
          }
          return true;
        }
      }
    }
    // 2. DURUM: Çoklu Paket Başlığı (First Frame)
    else if (pciType == 0x10) {
      if (f.data[2] == SVC_READ_RESP) {
        uint16_t rd = ((uint16_t)f.data[3] << 8) | f.data[4];
        if (rd == did) {
          *negOut = false;
          if (byteLen == 1) {
            *outVal = f.data[5];
          } else if (byteLen == 2) {
            *outVal = ((uint16_t)f.data[5] << 8) | f.data[6];
          } else if (byteLen >= 4) {
            *outVal = ((uint32_t)f.data[5] << 24) | ((uint32_t)f.data[6] << 16) |
                      ((uint32_t)f.data[7] << 8);
          }
          return true;
        }
      }
    }
  }
  return false;
}

// ==========================================================================
// EKRAN ÇİZİM FONKSİYONLARI
// ==========================================================================
void drawWelcome() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.drawStr(10, 22, "EA288 1.6TDI");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(8,  36, "UDS Master v5");
    u8g2.drawStr(14, 50, "Baslatiliyor..");
    u8g2.drawHLine(0, 54, 128);
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, 63, "500kbps 0x7E0>0x7E8");
  } while (u8g2.nextPage());
}

void drawScanScreen(uint8_t idx) {
  DID_Entry e;
  readEntry(idx, &e);
  uint8_t pct    = (uint8_t)((uint32_t)idx * 100 / DID_COUNT);
  uint8_t filled = (uint8_t)((uint32_t)pct  * 124 / 100);

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "EA288 UDS TARAMA");
    u8g2.drawHLine(0, 12, 128);
    snprintf(g_buf, sizeof(g_buf), "0x%04X  %u/%u", e.did, (uint8_t)(idx + 1), DID_COUNT);
    u8g2.drawStr(0, 24, g_buf);
    u8g2.drawStr(0, 36, e.label);
    
    u8g2.drawFrame(0, 44, 128, 10);
    if (filled > 0) u8g2.drawBox(2, 46, filled, 6);
    
    snprintf(g_buf, sizeof(g_buf), "%u%%", pct);
    u8g2.drawStr(56, 63, g_buf);
  } while (u8g2.nextPage());
}

void drawResult(uint8_t idx) {
  DID_Entry  e;
  readEntry(idx, &e);
  DID_Result r   = results[idx];
  bool ok  = r.flags & 0x01;
  bool neg = r.flags & 0x02;

  char valStr[24];

  if (!ok) {
    strcpy(valStr, "NO RESP");
  } else if (neg) {
    strcpy(valStr, "NEG 7F");
  } else {
    if (e.fmt == FMT_RAW) {
      snprintf(valStr, sizeof(valStr), "%lu %s", r.raw, e.unit);
    } else {
      float v = applyFormula(r.raw, e.fmt);
      char fBuf[12];
      dtostrf(v, 1, 2, fBuf); // AVR float dönüşümü
      snprintf(valStr, sizeof(valStr), "%s %s", fBuf, e.unit);
    }
  }

  u8g2.setFont(u8g2_font_9x18B_tf);
  uint8_t strW = u8g2.getStrWidth(valStr);
  uint8_t xPos = (strW < 128) ? (128 - strW) / 2 : 0;

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tf);
    snprintf(g_buf, sizeof(g_buf), "%u/%u  0x%04X", (uint8_t)(idx + 1), DID_COUNT, e.did);
    u8g2.drawStr(0, 10, g_buf);
    u8g2.drawHLine(0, 12, 128);
    
    u8g2.setFont(u8g2_font_7x14_tf);
    u8g2.drawStr(0, 28, e.label);
    
    u8g2.setFont(u8g2_font_9x18B_tf);
    u8g2.drawStr(xPos, 50, valStr);
    
    u8g2.setFont(u8g2_font_5x7_tf);
    if (!ok) {
      u8g2.drawStr(0, 63, "TIMEOUT");   u8g2.drawStr(100, 63, "[ERR]");
    } else if (neg) {
      u8g2.drawStr(0, 63, "NEG YANIT"); u8g2.drawStr(100, 63, "[ERR]");
    } else {
      u8g2.drawStr(0, 63, "BASARILI");  u8g2.drawStr(104, 63, "[OK]");
    }
  } while (u8g2.nextPage());
}

void drawSummary() {
  uint8_t okCnt = 0, negCnt = 0, toCnt = 0;
  for (uint8_t i = 0; i < DID_COUNT; i++) {
    uint8_t f = results[i].flags;
    if      (!(f & 0x01)) toCnt++;
    else if   (f & 0x02)  negCnt++;
    else                   okCnt++;
  }
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.drawStr(20, 14, "TARAMA TAMAM");
    u8g2.drawHLine(0, 16, 128);
    
    u8g2.setFont(u8g2_font_7x14_tf);
    snprintf(g_buf, sizeof(g_buf), "Toplam  : %u", DID_COUNT);
    u8g2.drawStr(0, 32, g_buf);
    snprintf(g_buf, sizeof(g_buf), "Basarili: %u", okCnt);
    u8g2.drawStr(0, 46, g_buf);
    snprintf(g_buf, sizeof(g_buf), "Hata/TO : %u/%u", negCnt, toCnt);
    u8g2.drawStr(0, 60, g_buf);
  } while (u8g2.nextPage());
  delay(3000);
}

// ==========================================================================
// SETUP
// ==========================================================================
void setup() {
  u8g2.begin();
  u8g2.setContrast(200);
  drawWelcome();
  delay(1500);

  // MCP2515 Başlatma (8 MHz kristal, 500 kbps)
  can.reset();
  can.setBitrate(CAN_500KBPS, MCP_8MHZ);
  can.setNormalMode();

  // ------ 28 ADET DID TARAMA DÖNGÜSÜ ------
  for (uint8_t i = 0; i < DID_COUNT; i++) {
    DID_Entry e;
    readEntry(i, &e);
    drawScanScreen(i);
    
    clearCanBuffer(); // Eski paket artıklarını temizle
    sendUDS(e.did);   // UDS 0x22 sorgusu gönder

    uint32_t val = 0;
    bool     neg = false;
    bool     got = waitUDSResponse(e.did, e.byteLen, &val, &neg);

    results[i].raw   = (got && !neg) ? val : 0;
    results[i].flags = (got ? 0x01 : 0x00) | (neg ? 0x02 : 0x00);
    delay(40); // ECU dinlenme süresi
  }

  drawSummary();
}

// ==========================================================================
// LOOP — Sonuçları 3'er saniye arayla ekranda döngüsel göster
// ==========================================================================
void loop() {
  for (uint8_t i = 0; i < DID_COUNT; i++) {
    drawResult(i);
    delay(DISPLAY_DELAY_MS);
  }
}