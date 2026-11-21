/* ==========================================================================
 * Proje: Skoda Octavia 1.6 TDI (EA288 MQB) - ÖZEL TURBO BOOST GÖSTERGESİ (v1.2)
 * Donanım: Arduino Nano + MCP2515 (8MHz) + SSD1306 SPI OLED (128x64)
 * 
 * Yenilik:
 *  - [Mükemmel Hizalama] BAR ve Peak göstergesi 8 piksel daha sağa kaydırıldı
 * ========================================================================== */

#include <SPI.h>
#include <mcp_can.h>
#include <U8g2lib.h>

#define PIN_CAN_CS    10
#define PIN_CAN_INT    2
#define PIN_OLED_CS    9
#define PIN_OLED_DC    8
#define PIN_OLED_RES   7

U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, PIN_OLED_CS, PIN_OLED_DC, PIN_OLED_RES);
MCP_CAN CAN0(PIN_CAN_CS);

#define ECU_REQ_ID          0x7E0
#define ECU_RESP_ID         0x7E8

// Canlı Turbo Değişkenleri
float    val_turbo_bar      = 0.0;
float    peak_turbo_bar     = 0.0;
uint32_t last_peak_time_ms  = 0;
const uint32_t PEAK_HOLD_MS = 10000; // 10 Saniye Peak Tutma

// Sistem & Zamanlayıcılar
bool     can_online         = false;
uint32_t last_query_ms      = 0;
uint32_t last_resp_ms       = 0;

// Burn-In Koruma Değişkenleri
int8_t   shift_x            = 0;
int8_t   shift_y            = 0;
uint32_t last_shift_ms      = 0;
bool     oled_is_sleeping   = false;

void initSPILock() {
  pinMode(PIN_CAN_CS, OUTPUT);
  pinMode(PIN_OLED_CS, OUTPUT);
  pinMode(PIN_CAN_INT, INPUT_PULLUP);
  digitalWrite(PIN_CAN_CS, HIGH);
  digitalWrite(PIN_OLED_CS, HIGH);
}

void sendTurboRequest() {
  uint8_t tx[8] = {0x02, 0x01, 0x0B, 0x55, 0x55, 0x55, 0x55, 0x55};
  digitalWrite(PIN_OLED_CS, HIGH);
  CAN0.sendMsgBuf(ECU_REQ_ID, 0, 8, tx);
}

void readTurboResponse() {
  while (!digitalRead(PIN_CAN_INT) || CAN0.checkReceive() == CAN_MSGAVAIL) {
    uint32_t rx_id; uint8_t len; uint8_t buf[8];
    digitalWrite(PIN_OLED_CS, HIGH);
    CAN0.readMsgBuf(&rx_id, &len, buf);

    if (rx_id == ECU_RESP_ID && buf[1] == 0x41 && buf[2] == 0x0B) {
      last_resp_ms = millis();
      can_online = true;

      if (oled_is_sleeping) {
        u8g2.setPowerSave(0);
        oled_is_sleeping = false;
      }

      uint8_t kpa = buf[3];
      val_turbo_bar = (kpa > 100) ? (float)(kpa - 100) * 0.01 : 0.0;

      if (val_turbo_bar > peak_turbo_bar) {
        peak_turbo_bar = val_turbo_bar;
        last_peak_time_ms = millis();
      }
    }
  }

  if (millis() - last_peak_time_ms >= PEAK_HOLD_MS) {
    peak_turbo_bar = val_turbo_bar;
  }
}

void runTurboScheduler() {
  uint32_t now = millis();
  if (now - last_query_ms >= 20) {
    last_query_ms = now;
    sendTurboRequest();
  }

  if (now - last_resp_ms > 2000) can_online = false;

  if (!can_online && (now - last_resp_ms > 25000) && !oled_is_sleeping) {
    u8g2.setPowerSave(1);
    oled_is_sleeping = true;
  }
}

void updateBurnInProtection() {
  uint32_t now = millis();
  if (now - last_shift_ms >= 120000) {
    last_shift_ms = now;
    static uint8_t orbit_step = 0;
    orbit_step = (orbit_step + 1) % 4;

    switch (orbit_step) {
      case 0: shift_x = 0; shift_y = 0; break;
      case 1: shift_x = 1; shift_y = 0; break;
      case 2: shift_x = 1; shift_y = 1; break;
      case 3: shift_x = 0; shift_y = 1; break;
    }
  }
}

void runPixelRefreshSweep() {
  u8g2.firstPage();
  do {
    u8g2.drawBox(0, 0, 128, 64);
  } while (u8g2.nextPage());
  delay(150);
}

void drawSplashScreen() {
  u8g2.firstPage();
  do {
    u8g2.drawRFrame(0, 0, 128, 64, 4);
    u8g2.setFont(u8g2_font_helvB10_tf);
    u8g2.drawStr(14, 22, "SKODA AUTO");
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(16, 38, "OCTAVIA BOOST");
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(24, 52, "VENT GAUGE v1.2");
  } while (u8g2.nextPage());
  delay(1200);

  runPixelRefreshSweep();
}

void renderTurboDisplay() {
  if (oled_is_sleeping) return;

  digitalWrite(PIN_CAN_CS, HIGH);
  u8g2.firstPage();
  do {
    // 1. ÜST BAŞLIK & CAN DURUMU
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(2 + shift_x, 9 + shift_y, "OCTAVIA BOOST");

    if (can_online) {
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.drawStr(88 + shift_x, 8 + shift_y, "CAN: OK");
      u8g2.drawDisc(124 + shift_x, 5 + shift_y, 2);
    } else {
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.drawStr(84 + shift_x, 8 + shift_y, "NO COMM");
      u8g2.drawFrame(122 + shift_x, 3 + shift_y, 5, 5);
    }
    u8g2.drawHLine(0, 12 + shift_y, 128);

    // 2. ORTA ALAN: BÜYÜK DİJİTAL TURBO BASINCI
    u8g2.setFont(u8g2_font_inb21_mr);
    u8g2.setCursor(1 + shift_x, 38 + shift_y);
    u8g2.print(val_turbo_bar, 2);

    // SAĞ ALAN: DAHA DA SAĞA ALINMIŞ BİRİM VE PEAK BİLGİSİ
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.drawStr(84 + shift_x, 24 + shift_y, "BAR");

    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(80 + shift_x, 38 + shift_y, "P:");
    u8g2.setCursor(93 + shift_x, 38 + shift_y);
    u8g2.print(peak_turbo_bar, 2);

    // 3. ALT ALAN: DİNAMİK GAUGE ÇUBUĞU (0.0 - 2.0 BAR)
    const uint8_t bar_x = 4 + shift_x;
    const uint8_t bar_y = 44 + shift_y;
    const uint8_t bar_w = 120;
    const uint8_t bar_h = 10;

    // Dış Çerçeve
    u8g2.drawFrame(bar_x, bar_y, bar_w, bar_h);

    // Basınç Dolgusu
    int16_t fill_w = (int16_t)((val_turbo_bar / 2.0) * (bar_w - 4));
    if (fill_w > (bar_w - 4)) fill_w = (bar_w - 4);
    if (fill_w < 0) fill_w = 0;

    if (fill_w > 0) {
      u8g2.drawBox(bar_x + 2, bar_y + 2, fill_w, bar_h - 4);
    }

    // Peak İbre Çizgisi (Tepeyi işaretler)
    int16_t peak_x = (int16_t)((peak_turbo_bar / 2.0) * (bar_w - 4));
    if (peak_x > (bar_w - 4)) peak_x = (bar_w - 4);
    if (peak_x > 0) {
      u8g2.drawVLine(bar_x + 2 + peak_x, bar_y - 2, bar_h + 4);
    }

    // 4. KILAVUZ YAZILARI (0.0, 1.0, 2.0)
    u8g2.setFont(u8g2_font_4x6_tf);
    u8g2.drawStr(bar_x, 62 + shift_y, "0.0");
    u8g2.drawVLine(bar_x + (bar_w / 2), bar_y + bar_h, 2);
    u8g2.drawStr(bar_x + (bar_w / 2) - 4, 62 + shift_y, "1.0");
    u8g2.drawStr(bar_x + bar_w - 9, 62 + shift_y, "2.0");

  } while (u8g2.nextPage());
}

void setup() {
  initSPILock();
  
  u8g2.begin();
  u8g2.setContrast(170);

  drawSplashScreen();

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN0.setMode(MCP_NORMAL);
    can_online = true;
  }
}

void loop() {
  updateBurnInProtection();
  runTurboScheduler();
  readTurboResponse();
  renderTurboDisplay();
}