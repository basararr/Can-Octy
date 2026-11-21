// =============================================================================
// Skoda_MFD_v3.ino — Skoda Octavia 1.6 TDI EA288 / DQ200 OLED MFD
// Donanim : Arduino Nano ATmega328P
//           MCP2515 CAN (CS:D10, INT:D2, 8MHz, 500kbps)
//           SSD1306 OLED 128x64 SPI (CS:D9, DC:D8, RST:D7)
// Platform : Arduino IDE + MCP_CAN lib + U8g2 lib
//
// Sürüm Notları (v3.0):
//   - Tüm doğrulanmış DID adresleri uygulandı
//   - U8g2 1-sayfa buffer modu (128 byte RAM)
//   - Donanımsal INT pini (D2) aktif
//   - 4 kademeli OLED burn-in koruması
//   - Peak boost hold 10sn
//   - DPF REGEN dinamik uyarı kutusu
//   - Açılış DPF sağlık ara ekranı (5sn)
//   - Kontak kapanınca otomatik OLED uyku
// =============================================================================

#include <SPI.h>
#include <mcp_can.h>
#include <U8g2lib.h>

// ─── PIN TANIMLARI ─────────────────────────────────────────────────────────────
#define PIN_CAN_CS    10
#define PIN_CAN_INT    2
#define PIN_OLED_CS    9
#define PIN_OLED_DC    8
#define PIN_OLED_RST   7

// ─── CAN KİMLİKLERİ ─────────────────────────────────────────────────────────────
#define ECU_TX_ID     0x7E0
#define ECU_RX_ID     0x7E8
#define TCU_TX_ID     0x7E1
#define TCU_RX_ID     0x7E9
#define OBD_BCAST     0x7DF

// ─── UDS / OBD ─────────────────────────────────────────────────────────────────
#define UDS_READ_DID  0x22
#define OBD_MODE_01   0x01
#define OBD_MODE_03   0x03

// ─── DOĞRULANMIŞ DID ADRESLERİ (Gerçek Araç Taraması) ─────────────────────────
#define DID_DPF_DIFF_PRESS  0x1153   // DPF fark basıncı hPa (NOT soot!)
#define DID_DPF_EGT         0x1156   // DPF egzoz sıcaklığı (*0.01 °C)
#define DID_DPF_SOOT        0x115A   // DPF kurum kütlesi (*1.0 g)
#define DID_DPF_ASH         0x115B   // DPF kül kütlesi (*0.5 g)
#define DID_LAST_REGEN      0x11E1   // Son rejenerasyon mesafesi (/10 km)
#define DID_OIL_TEMP        0x2024   // Motor yağ sıcaklığı (raw-40 °C)
#define DID_DSG_TEMP        0x040F   // DSG mekatronik sıcaklığı (*0.25 °C)
#define PID_BOOST           0x0B     // OBD Mode 01: Manifold basıncı kPa
#define PID_COOLANT         0x05     // OBD Mode 01: Soğutma suyu °C (raw-40)
#define PID_DTC_COUNT       0xFF     // OBD Mode 03: DTC sayısı (buf[2])

// ─── ZAMANLAMA ──────────────────────────────────────────────────────────────────
#define TIMEOUT_MS          80
#define BOOST_INTERVAL      25    // ms — Turbo basıncı okuma sıklığı
#define SLOW_INTERVAL       500   // ms — Yavaş sensörler
#define SLEEP_AFTER_MS      25000 // ms — CAN sessizliği sonrası OLED uyku
#define PIXEL_SHIFT_MS      120000 // ms — Burn-in shift periyodu (2 dakika)
#define PEAK_HOLD_MS        10000  // ms — Peak boost görüntüleme süresi
#define SPLASH_DURATION_MS  5000   // ms — Açılış DPF ekranı süresi

// ─── GLOBAL DEĞİŞKENLER ─────────────────────────────────────────────────────────
MCP_CAN CAN(PIN_CAN_CS);
U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0,
  PIN_OLED_CS, PIN_OLED_DC, PIN_OLED_RST);

// Sensör verileri
float   turboBar       = 0.0;
float   peakBar        = 0.0;
int16_t coolantC       = 0;
float   egtC           = 0.0;
float   dsgTempC       = 0.0;
float   oilTempC       = 0.0;
float   dpfSootG       = 0.0;
float   dpfAshG        = 0.0;
float   dpfDiffPress   = 0.0;
uint16_t lastRegenKm   = 0;
uint8_t  dtcCount      = 0;
bool     dpfRegenActive = false;
bool     oilTempValid   = false;

// Zamanlama
uint32_t lastBoostMs    = 0;
uint32_t lastSlowMs     = 0;
uint32_t lastCanMs      = 0;
uint32_t peakHoldStart  = 0;
uint32_t lastPixelShift = 0;
bool     oledAsleep     = false;
bool     splashDone     = false;
uint32_t startupMs      = 0;

// Piksel kaydırma (burn-in)
int8_t shiftX = 0;
int8_t shiftY = 0;
const int8_t shiftPattern[4][2] = {{0,0},{1,0},{1,1},{0,1}};
uint8_t shiftIdx = 0;

// Yavaş sensör sıralayıcı
uint8_t slowCycle = 0;

// ─── FONKSIYON BİLDİRİMLERİ ─────────────────────────────────────────────────────
bool sendOBD(uint8_t mode, uint8_t pid, uint8_t *buf, uint8_t &len);
bool sendUDS(uint32_t txID, uint32_t rxID, uint16_t did, uint8_t *buf, uint8_t &len);
bool waitCAN(uint32_t rxID, uint8_t *buf, uint8_t &len);
void sendFC(uint32_t txID);
void drawMainScreen();
void drawSplashScreen();
void drawBoostArc(float bar, float peak);
void drawDPFBox();
void wakeSPIBus(bool oled);

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  // OLED de-ghosting: tam beyaz süpürme (150ms)
  u8g2.begin();
  u8g2.setContrast(170);
  u8g2.firstPage();
  do {
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, 0, 128, 64);
  } while (u8g2.nextPage());
  delay(150);

  // CAN başlat
  byte initResult;
  for (uint8_t i = 0; i < 5; i++) {
    initResult = CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
    if (initResult == CAN_OK) break;
    delay(300);
  }
  CAN.setMode(MCP_NORMAL);
  pinMode(PIN_CAN_INT, INPUT);

  startupMs = millis();
  lastCanMs = millis();
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  uint32_t now = millis();

  // ── Açılış splash ekranı (5 saniye) ─────────────────────────────────────────
  if (!splashDone) {
    if (now - startupMs < SPLASH_DURATION_MS) {
      drawSplashScreen();
      return;
    } else {
      splashDone = true;
    }
  }

  // ── CAN sessizliği → OLED uyku ───────────────────────────────────────────────
  if (now - lastCanMs > SLEEP_AFTER_MS) {
    if (!oledAsleep) {
      u8g2.setPowerSave(1);
      oledAsleep = true;
    }
    return;
  } else if (oledAsleep) {
    u8g2.setPowerSave(0);
    oledAsleep = false;
  }

  // ── Turbo basıncı (25ms) ─────────────────────────────────────────────────────
  if (now - lastBoostMs >= BOOST_INTERVAL) {
    lastBoostMs = now;
    uint8_t buf[8]; uint8_t len = 0;
    if (sendOBD(OBD_MODE_01, PID_BOOST, buf, len) && len >= 4) {
      lastCanMs = now;
      float kpa = (float)buf[3];        // buf[0]=len, buf[1]=41, buf[2]=PID, buf[3]=value
      turboBar  = (kpa - 100.0) * 0.01; // 100 kPa = 0 bar göreceli
      if (turboBar < -0.1) turboBar = 0.0;
      if (turboBar > peakBar) {
        peakBar       = turboBar;
        peakHoldStart = now;
      }
      if (now - peakHoldStart > PEAK_HOLD_MS) peakBar = turboBar;
    }
  }

  // ── Yavaş sensörler (500ms, sırayla) ────────────────────────────────────────
  if (now - lastSlowMs >= SLOW_INTERVAL) {
    lastSlowMs = now;
    uint8_t buf[16]; uint8_t len = 0;

    switch (slowCycle) {
      case 0: // Soğutma suyu sıcaklığı (OBD)
        if (sendOBD(OBD_MODE_01, PID_COOLANT, buf, len) && len >= 4) {
          lastCanMs = now;
          coolantC  = (int16_t)buf[3] - 40;
        }
        break;

      case 1: // DPF egzoz sıcaklığı (UDS 0x1156)
        if (sendUDS(ECU_TX_ID, ECU_RX_ID, DID_DPF_EGT, buf, len) && len >= 4) {
          lastCanMs = now;
          uint16_t raw = ((uint16_t)buf[3] << 8) | buf[4];
          egtC = raw * 0.01;
        }
        break;

      case 2: // DSG mekatronik sıcaklığı (UDS TCU 0x040F)
        if (sendUDS(TCU_TX_ID, TCU_RX_ID, DID_DSG_TEMP, buf, len) && len >= 4) {
          lastCanMs = now;
          uint16_t raw = ((uint16_t)buf[3] << 8) | buf[4];
          dsgTempC = raw * 0.25;
        }
        break;

      case 3: // Motor yağ sıcaklığı (UDS 0x2024)
        if (sendUDS(ECU_TX_ID, ECU_RX_ID, DID_OIL_TEMP, buf, len) && len >= 4) {
          lastCanMs = now;
          int16_t raw = (int16_t)(((uint16_t)buf[3] << 8) | buf[4]);
          oilTempC    = raw - 40;
          oilTempValid = (oilTempC >= 50);
        }
        break;

      case 4: // DPF kurum kütlesi (UDS 0x115A)
        if (sendUDS(ECU_TX_ID, ECU_RX_ID, DID_DPF_SOOT, buf, len) && len >= 4) {
          lastCanMs = now;
          uint16_t raw = ((uint16_t)buf[3] << 8) | buf[4];
          dpfSootG = (float)raw;
        }
        break;

      case 5: // DPF kül kütlesi (UDS 0x115B)
        if (sendUDS(ECU_TX_ID, ECU_RX_ID, DID_DPF_ASH, buf, len) && len >= 4) {
          lastCanMs = now;
          uint16_t raw = ((uint16_t)buf[3] << 8) | buf[4];
          dpfAshG = raw * 0.5;
        }
        break;

      case 6: // DPF fark basıncı (UDS 0x1153)
        if (sendUDS(ECU_TX_ID, ECU_RX_ID, DID_DPF_DIFF_PRESS, buf, len) && len >= 4) {
          lastCanMs = now;
          uint16_t raw = ((uint16_t)buf[3] << 8) | buf[4];
          dpfDiffPress = raw * 0.01; // hPa
          // Regen tespiti: yüksek fark basıncı + yüksek EGT
          dpfRegenActive = (egtC > 400.0 && dpfDiffPress < 1.0);
        }
        break;

      case 7: // Son rejenerasyon mesafesi (UDS 0x11E1)
        if (sendUDS(ECU_TX_ID, ECU_RX_ID, DID_LAST_REGEN, buf, len) && len >= 4) {
          lastCanMs = now;
          uint16_t raw = ((uint16_t)buf[3] << 8) | buf[4];
          lastRegenKm  = raw / 10;
        }
        break;

      case 8: // DTC sayısı (OBD Mode 03)
        if (sendOBD(OBD_MODE_03, 0x00, buf, len) && len >= 3) {
          lastCanMs  = now;
          dtcCount   = buf[2];
        }
        break;
    }

    slowCycle++;
    if (slowCycle > 8) slowCycle = 0;
  }

  // ── Burn-in kaydırma (2 dakikada bir) ───────────────────────────────────────
  if (now - lastPixelShift > PIXEL_SHIFT_MS) {
    lastPixelShift = now;
    shiftIdx = (shiftIdx + 1) % 4;
    shiftX   = shiftPattern[shiftIdx][0];
    shiftY   = shiftPattern[shiftIdx][1];
  }

  // ── Ekran çiz ────────────────────────────────────────────────────────────────
  drawMainScreen();
}

// =============================================================================
// ANA EKRAN (U8g2 1-sayfa buffer modu)
// =============================================================================
void drawMainScreen() {
  u8g2.firstPage();
  do {
    int8_t ox = shiftX;
    int8_t oy = shiftY;

    // ── Sol Kolon: Turbo göstergesi ──────────────────────────────────────────
    drawBoostArc(turboBar, peakBar);

    // ── Turbo sayısal değer ─────────────────────────────────────────────────
    char boostStr[8];
    dtostrf(turboBar, 4, 2, boostStr);
    u8g2.setFont(u8g2_font_logisoso16_tn); // Büyük rakam fontu
    u8g2.drawStr(ox + 2, oy + 46, boostStr);

    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(ox + 2, oy + 55, "bar TURBO");

    // Peak hold göstergesi
    if (peakBar > 0.05) {
      char peakStr[10];
      dtostrf(peakBar, 4, 2, peakStr);
      u8g2.drawStr(ox + 2, oy + 63, peakStr);
      u8g2.drawStr(ox + 28, oy + 63, "pk");
    }

    // ── Orta dikey ayraç ────────────────────────────────────────────────────
    u8g2.drawVLine(ox + 55, oy + 0, 64);

    // ── Sağ Üst: Sıcaklıklar ────────────────────────────────────────────────
    u8g2.setFont(u8g2_font_5x7_tr);

    // Soğutma suyu
    char cwStr[8]; itoa(coolantC, cwStr, 10);
    u8g2.drawStr(ox + 58, oy + 8,  "SU:");
    u8g2.drawStr(ox + 80, oy + 8,  cwStr);
    u8g2.drawStr(ox + 105, oy + 8, "C");

    // DSG sıcaklığı
    char dsgStr[8]; dtostrf(dsgTempC, 4, 1, dsgStr);
    u8g2.drawStr(ox + 58, oy + 17, "DSG:");
    u8g2.drawStr(ox + 80, oy + 17, dsgStr);
    u8g2.drawStr(ox + 110, oy + 17, "C");

    // EGT
    char egtStr[8]; dtostrf(egtC, 4, 1, egtStr);
    u8g2.drawStr(ox + 58, oy + 26, "EGT:");
    u8g2.drawStr(ox + 80, oy + 26, egtStr);
    u8g2.drawStr(ox + 110, oy + 26, "C");

    // Yağ sıcaklığı
    u8g2.drawStr(ox + 58, oy + 35, "YAG:");
    if (oilTempValid) {
      char oilStr[8]; dtostrf(oilTempC, 4, 1, oilStr);
      u8g2.drawStr(ox + 80, oy + 35, oilStr);
      u8g2.drawStr(ox + 110, oy + 35, "C");
    } else {
      u8g2.drawStr(ox + 80, oy + 35, "--- C");
    }

    // ── Sağ Alt: DPF Kutusu ──────────────────────────────────────────────────
    drawDPFBox();

    // ── Sağ Alt: DTC Uyarısı ────────────────────────────────────────────────
    if (dtcCount > 0) {
      u8g2.setFont(u8g2_font_5x7_tr);
      char dtcStr[12]; snprintf(dtcStr, sizeof(dtcStr), "!DTC:%d", dtcCount);
      u8g2.setDrawColor(1);
      u8g2.drawBox(ox + 58, oy + 56, 70, 8);
      u8g2.setDrawColor(0);
      u8g2.drawStr(ox + 60, oy + 63, dtcStr);
      u8g2.setDrawColor(1);
    }

  } while (u8g2.nextPage());
}

// =============================================================================
// TURBO YEDEK ÇUBUĞU (Arc gauge)
// =============================================================================
void drawBoostArc(float bar, float peak) {
  // 0.0 → 2.0 bar aralığı
  float clampedBar  = constrain(bar,  0.0, 2.0);
  float clampedPeak = constrain(peak, 0.0, 2.0);

  // Yatay progress bar (basit ama okunabilir)
  int bx = 2, by = 2, bw = 50, bh = 14;
  int fillW = (int)((clampedBar / 2.0) * bw);
  int peakX = bx + (int)((clampedPeak / 2.0) * bw);

  // Çerçeve
  u8g2.drawFrame(bx, by, bw, bh);
  // Dolu kısım
  u8g2.drawBox(bx + 1, by + 1, fillW, bh - 2);
  // Peak çizgisi
  if (clampedPeak > 0.05) {
    u8g2.drawVLine(peakX, by, bh);
  }

  // Ölçek çizgileri: 0.5, 1.0, 1.5 bar
  for (float v = 0.5; v < 2.0; v += 0.5) {
    int xPos = bx + (int)((v / 2.0) * bw);
    u8g2.drawVLine(xPos, by + bh, 3);
  }
}

// =============================================================================
// DPF KUTUSU
// =============================================================================
void drawDPFBox() {
  int bx = 58, by = 44, bw = 68, bh = 10;

  if (dpfRegenActive) {
    // REGEN aktif: tam beyaz (inverted) kutu — dikkat çekici
    u8g2.setDrawColor(1);
    u8g2.drawBox(bx, by, bw, bh);
    u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(bx + 2, by + 8, "REGEN AKTIF!");
    u8g2.setDrawColor(1);
  } else {
    // Normal: kurum g + kul g progress
    u8g2.drawFrame(bx, by, bw, bh);
    // Kurum yüzdesi (max ~30g)
    float sootPct = constrain(dpfSootG / 30.0, 0, 1.0);
    u8g2.drawBox(bx + 1, by + 1, (int)(sootPct * (bw - 2)), bh - 2);

    u8g2.setFont(u8g2_font_4x6_tr);
    char dpfStr[16]; snprintf(dpfStr, sizeof(dpfStr), "S:%.0fg A:%.0fg", dpfSootG, dpfAshG);
    // Yazi rengi: doluluk >%60 ise ters
    if (sootPct > 0.6) {
      u8g2.setDrawColor(0);
      u8g2.drawStr(bx + 2, by + 8, dpfStr);
      u8g2.setDrawColor(1);
    } else {
      u8g2.drawStr(bx + 2, by + 8, dpfStr);
    }
  }
}

// =============================================================================
// AÇILIŞ SPLASH EKRANI (DPF Saglik)
// =============================================================================
void drawSplashScreen() {
  // Animasyonlu ilerleme: 5 saniye
  uint32_t elapsed  = millis() - startupMs;
  uint8_t  progress = (uint8_t)((elapsed * 128) / SPLASH_DURATION_MS);

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(2, 8, "SKODA MFD v3.0");
    u8g2.drawStr(2, 17, "DPF SAGLIK:");

    // Kurum
    char sootStr[16]; snprintf(sootStr, sizeof(sootStr), "Kurum: %.0f g", dpfSootG);
    u8g2.drawStr(2, 27, sootStr);

    // Kul
    char ashStr[16]; snprintf(ashStr, sizeof(ashStr), "Kul  : %.1f g", dpfAshG);
    u8g2.drawStr(2, 36, ashStr);

    // Son regen
    char regenStr[20]; snprintf(regenStr, sizeof(regenStr), "Regen: %u km once", lastRegenKm);
    u8g2.drawStr(2, 45, regenStr);

    // DTC
    char dtcStr[12]; snprintf(dtcStr, sizeof(dtcStr), "DTC  : %d", dtcCount);
    u8g2.drawStr(2, 54, dtcStr);

    // Progress bar
    u8g2.drawFrame(2, 57, 124, 6);
    u8g2.drawBox(3, 58, progress, 4);

  } while (u8g2.nextPage());
}

// =============================================================================
// OBD MODE 01 / 03 İSTEĞİ
// =============================================================================
bool sendOBD(uint8_t mode, uint8_t pid, uint8_t *buf, uint8_t &len) {
  uint8_t msg[8] = {0x02, mode, pid, 0x00, 0x00, 0x00, 0x00, 0x00};
  if (mode == OBD_MODE_03) msg[0] = 0x01; // Mode 03 tek bayt
  if (CAN.sendMsgBuf(OBD_BCAST, 0, 8, msg) != CAN_OK) return false;
  return waitCAN(ECU_RX_ID, buf, len);
}

// =============================================================================
// UDS ReadDataByIdentifier (0x22) — Single + Multi Frame
// =============================================================================
bool sendUDS(uint32_t txID, uint32_t rxID, uint16_t did, uint8_t *buf, uint8_t &len) {
  uint8_t msg[8] = {
    0x03, UDS_READ_DID,
    (uint8_t)(did >> 8), (uint8_t)(did & 0xFF),
    0x00, 0x00, 0x00, 0x00
  };
  if (CAN.sendMsgBuf(txID, 0, 8, msg) != CAN_OK) return false;

  uint8_t raw[8] = {0}; uint8_t rawLen = 0;
  if (!waitCAN(rxID, raw, rawLen)) return false;

  uint8_t pci = (raw[0] & 0xF0) >> 4;

  if (pci == 0x0) {
    if (raw[1] == 0x7F) return false;
    len = min((uint8_t)(raw[0] & 0x0F), (uint8_t)7);
    memcpy(buf, raw, len + 1);
    return true;
  }

  if (pci == 0x1) {
    if (raw[2] == 0x7F) return false;
    uint8_t fc[8] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    CAN.sendMsgBuf(txID, 0, 8, fc);
    delay(5);
    uint8_t cf[8] = {0}; uint8_t cfLen = 0;
    if (!waitCAN(rxID, cf, cfLen)) return false;
    // İlk FF verisi + CF verisi birleştir (max 16 byte buf)
    memcpy(buf, raw, 8);
    memcpy(buf + 8, cf + 1, min(cfLen - 1, (uint8_t)7));
    len = 15;
    return true;
  }
  return false;
}

// =============================================================================
// CAN YANIT BEKLE (INT pini öncelikli)
// =============================================================================
bool waitCAN(uint32_t rxID, uint8_t *buf, uint8_t &len) {
  uint32_t t0 = millis();
  while (millis() - t0 < TIMEOUT_MS) {
    if (digitalRead(PIN_CAN_INT) == LOW || CAN.checkReceive() == CAN_MSGAVAIL) {
      long unsigned int id; uint8_t l; uint8_t tmp[8];
      if (CAN.readMsgBuf(&id, &l, tmp) == CAN_OK && id == rxID) {
        memcpy(buf, tmp, l);
        len = l;
        return true;
      }
    }
  }
  return false;
}
