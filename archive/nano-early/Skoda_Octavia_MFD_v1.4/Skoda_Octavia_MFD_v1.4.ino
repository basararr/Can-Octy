/* ==========================================================================
 * Proje: Skoda Octavia 1.6 TDI OEM Vent-Pod OLED MFD (v1.4 - Kararlı Üretim Sürümü)
 * Donanım: 1. Arduino Nano + MCP2515 (8MHz) + SSD1306 SPI OLED (7 Pin)
 * 
 * Özellikler & Yenilikler:
 *  - [Arayüz] Sadeleştirilmiş OEM Üst Çubuk (Sadece "OCTAVIA" & CAN Durumu)
 *  - [Sağlık] 5 Saniyelik DPF Kül (Ash), Son Regen Mesafesi & DTC Ara Ekranı
 *  - [Güvenlik] Rejenerasyon Süresince Sürekli Ters Renk (Dolu Beyaz Kutu) Uyarısı
 *  - [Performans] 25ms Turbo Öncelikli (Interleaving) Süper Akıcı Gösterge Barı
 *  - [Ekran Koruması] 4 Kademeli Burn-In Koruması (Orbiting, Sweep, Sleep, Contrast)
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
#define TCM_REQ_ID          0x7E1
#define TCM_RESP_ID         0x7E9

#define DID_DPF_ASH_MASS    0x202E  // Kül Kütlesi (g)
#define DID_DPF_LAST_REGEN  0x2035  // Son Rejenerasyon Mesafesi (km)
#define DID_DPF_SOOT_CALC   0x2029  // Kurum Kütlesi (g)
#define DID_EGT_PRE_DPF     0x2045  // Egzoz Gazı Sıcaklığı (°C)
#define DID_TURBO_BOOST     0x115B  // Turbo Basıncı (mbar)
#define DID_OIL_TEMP        0x2024  // Yağ Sıcaklığı (°C)
#define DID_DSG_TEMP        0x1000  // DSG Sıcaklığı (°C)

// Canlı Sensör Değişkenleri
float    val_turbo_bar      = 0.0;
float    val_dpf_soot_g     = 0.0;
int16_t  val_oil_temp_c     = 0;
int16_t  val_dsg_temp_c     = 0;
int16_t  val_egt_temp_c     = 0;
bool     val_regen_active   = false;
float    peak_turbo_bar     = 0.0;

// Ara Ekran İçin Başlangıç Verileri
float    boot_ash_g         = 0.0;
uint16_t boot_last_regen_km = 0;
uint8_t  boot_dtc_count     = 0;

// Sistem Zamanlayıcıları
bool     can_online         = false;
uint8_t  scheduler_step     = 0;
uint32_t last_query_ms      = 0;
uint32_t last_resp_ms       = 0;
uint32_t last_log_ms        = 0;

// Burn-In Koruma Değişkenleri
int8_t   shift_x            = 0;
int8_t   shift_y            = 0;
uint32_t last_shift_ms      = 0;
bool     oled_is_sleeping   = false;

void initSPILock() {
  pinMode(PIN_CAN_CS, OUTPUT);
  pinMode(PIN_OLED_CS, OUTPUT);
  digitalWrite(PIN_CAN_CS, HIGH);
  digitalWrite(PIN_OLED_CS, HIGH);
}

void sendUDSRequest(uint32_t can_id, uint16_t did) {
  uint8_t tx[8] = {0x03, 0x22, (uint8_t)(did >> 8), (uint8_t)(did & 0xFF), 0x55, 0x55, 0x55, 0x55};
  digitalWrite(PIN_OLED_CS, HIGH);
  CAN0.sendMsgBuf(can_id, 0, 8, tx);
}

uint16_t fetchStartupUDS(uint32_t req_id, uint32_t resp_id, uint16_t did) {
  sendUDSRequest(req_id, did);
  uint32_t start_w = millis();
  while (millis() - start_w < 200) {
    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
      uint32_t rx_id; uint8_t len; uint8_t buf[8];
      digitalWrite(PIN_OLED_CS, HIGH);
      CAN0.readMsgBuf(&rx_id, &len, buf);
      if (rx_id == resp_id && buf[1] == 0x62 && (((uint16_t)buf[2] << 8) | buf[3]) == did) {
        return ((uint16_t)buf[4] << 8) | buf[5];
      }
    }
  }
  return 0;
}

uint8_t fetchStartupDTC(uint32_t req_id, uint32_t resp_id) {
  uint8_t tx[8] = {0x03, 0x19, 0x02, 0xFF, 0x55, 0x55, 0x55, 0x55};
  digitalWrite(PIN_OLED_CS, HIGH);
  CAN0.sendMsgBuf(req_id, 0, 8, tx);
  uint32_t start_w = millis();
  while (millis() - start_w < 200) {
    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
      uint32_t rx_id; uint8_t len; uint8_t buf[8];
      digitalWrite(PIN_OLED_CS, HIGH);
      CAN0.readMsgBuf(&rx_id, &len, buf);
      if (rx_id == resp_id && buf[1] == 0x59) {
        if ((buf[0] & 0xF0) == 0x00 && buf[0] <= 3) return 0;
        else return 1;
      }
    }
  }
  return 0;
}

void printLogHeader() {
  Serial.println(F("\nTIME_MS,TURBO_BAR,PEAK_BAR,DPF_SOOT_G,OIL_C,DSG_C,EGT_C,REGEN"));
}

void streamDataLog() {
  uint32_t now = millis();
  if (now - last_log_ms >= 250) {
    last_log_ms = now;
    if (can_online) {
      Serial.print(now); Serial.print(',');
      Serial.print(val_turbo_bar, 2); Serial.print(',');
      Serial.print(peak_turbo_bar, 2); Serial.print(',');
      Serial.print(val_dpf_soot_g, 1); Serial.print(',');
      Serial.print(val_oil_temp_c); Serial.print(',');
      Serial.print(val_dsg_temp_c); Serial.print(',');
      Serial.print(val_egt_temp_c); Serial.print(',');
      Serial.println(val_regen_active ? 1 : 0);
    }
  }
}

void processIncomingCAN() {
  while (CAN0.checkReceive() == CAN_MSGAVAIL) {
    uint32_t rx_id; uint8_t len; uint8_t buf[8];
    digitalWrite(PIN_OLED_CS, HIGH);
    CAN0.readMsgBuf(&rx_id, &len, buf);

    if (rx_id == ECU_RESP_ID) {
      last_resp_ms = millis();
      can_online = true;

      if (oled_is_sleeping) {
        u8g2.setPowerSave(0);
        oled_is_sleeping = false;
      }

      if (buf[1] == 0x62) {
        uint16_t did = ((uint16_t)buf[2] << 8) | buf[3];

        if (did == DID_TURBO_BOOST) {
          uint16_t mbar = ((uint16_t)buf[4] << 8) | buf[5];
          val_turbo_bar = (mbar > 1000) ? (float)(mbar - 1013) / 1000.0 : 0.0;
          if (val_turbo_bar > peak_turbo_bar) peak_turbo_bar = val_turbo_bar;
        }
        else if (did == DID_DPF_SOOT_CALC) {
          val_dpf_soot_g = (((uint16_t)buf[4] << 8) | buf[5]) * 0.01;
        }
        else if (did == DID_EGT_PRE_DPF) {
          val_egt_temp_c = (int16_t)((((uint16_t)buf[4] << 8) | buf[5]) * 0.1) - 40;
        }
        else if (did == DID_OIL_TEMP) {
          val_oil_temp_c = (int16_t)buf[4] - 40;
        }
      }
    }
    else if (rx_id == TCM_RESP_ID) {
      last_resp_ms = millis();
      if (buf[1] == 0x62 && (((uint16_t)buf[2] << 8) | buf[3]) == DID_DSG_TEMP) {
        val_dsg_temp_c = (int16_t)buf[4] - 40;
      }
    }
  }

  val_regen_active = (val_dpf_soot_g >= 24.0 || val_egt_temp_c >= 500);
}

void runDiagnosticsScheduler() {
  uint32_t now = millis();
  if (now - last_query_ms >= 25) {
    last_query_ms = now;

    switch (scheduler_step) {
      case 0: sendUDSRequest(ECU_REQ_ID, DID_TURBO_BOOST);   break;
      case 1: sendUDSRequest(ECU_REQ_ID, DID_DPF_SOOT_CALC); break;
      case 2: sendUDSRequest(ECU_REQ_ID, DID_TURBO_BOOST);   break;
      case 3: sendUDSRequest(ECU_REQ_ID, DID_EGT_PRE_DPF);   break;
      case 4: sendUDSRequest(ECU_REQ_ID, DID_TURBO_BOOST);   break;
      case 5: sendUDSRequest(ECU_REQ_ID, DID_OIL_TEMP);      break;
      case 6: sendUDSRequest(ECU_REQ_ID, DID_TURBO_BOOST);   break;
      case 7: sendUDSRequest(TCM_REQ_ID, DID_DSG_TEMP);      break;
    }
    scheduler_step = (scheduler_step + 1) % 8;
  }

  if (now - last_resp_ms > 2500) can_online = false;

  // 25 sn CAN sinyali yoksa ekranı kapat
  if (!can_online && (now - last_resp_ms > 25000) && !oled_is_sleeping) {
    u8g2.setPowerSave(1);
    oled_is_sleeping = true;
  }
}

void updateBurnInProtection() {
  uint32_t now = millis();
  if (now - last_shift_ms >= 120000) { // 2 dakikada bir 1 piksel kaydır
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
    u8g2.drawStr(12, 38, "OCTAVIA 1.6 TDI");
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(18, 52, "VENT MFD v1.4");
  } while (u8g2.nextPage());
  delay(1200);

  runPixelRefreshSweep();
}

void showIntermediateScreen() {
  boot_ash_g         = fetchStartupUDS(ECU_REQ_ID, ECU_RESP_ID, DID_DPF_ASH_MASS) * 0.01;
  boot_last_regen_km = fetchStartupUDS(ECU_REQ_ID, ECU_RESP_ID, DID_DPF_LAST_REGEN);
  val_dpf_soot_g     = fetchStartupUDS(ECU_REQ_ID, ECU_RESP_ID, DID_DPF_SOOT_CALC) * 0.01;
  boot_dtc_count     = fetchStartupDTC(ECU_REQ_ID, ECU_RESP_ID);

  uint32_t start_screen_ms = millis();
  const uint32_t total_display_ms = 5000;

  while (millis() - start_screen_ms < total_display_ms) {
    uint32_t elapsed = millis() - start_screen_ms;
    uint8_t progress_width = (uint8_t)((elapsed * 124) / total_display_ms);

    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.drawStr(2, 7, "SISTEM & DPF SAGLIK OZETI");
      u8g2.drawHLine(0, 9, 128);

      u8g2.drawStr(2, 19, "DPF KUL (ASH) :");
      u8g2.setCursor(72, 19);
      u8g2.print(boot_ash_g, 1);
      u8g2.drawStr(98, 19, "g /80g");

      u8g2.drawStr(2, 30, "SON REGEN     :");
      u8g2.setCursor(72, 30);
      u8g2.print(boot_last_regen_km);
      u8g2.drawStr(98, 30, "km");

      u8g2.drawStr(2, 41, "MEVCUT KURUM  :");
      u8g2.setCursor(72, 41);
      u8g2.print(val_dpf_soot_g, 1);
      u8g2.drawStr(98, 41, "g");

      u8g2.drawStr(2, 52, "ARIZA (DTC)   :");
      if (boot_dtc_count == 0) {
        u8g2.drawStr(72, 52, "0 HATA (OK)");
      } else {
        u8g2.drawStr(72, 52, "ARIZA VAR!");
      }

      u8g2.drawFrame(1, 57, 126, 6);
      if (progress_width > 0) {
        u8g2.drawBox(2, 58, progress_width, 4);
      }

    } while (u8g2.nextPage());
  }
}

void renderDashboard() {
  if (oled_is_sleeping) return;

  digitalWrite(PIN_CAN_CS, HIGH);
  u8g2.firstPage();
  do {
    // 1. ÜST BİLGİ ÇUBUĞU (Sadeleştirilmiş: Sadece "OCTAVIA")
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(2 + shift_x, 8 + shift_y, "OCTAVIA");

    if (can_online) {
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.drawStr(88 + shift_x, 7 + shift_y, "CAN: OK");
      u8g2.drawDisc(124 + shift_x, 4 + shift_y, 2);
    } else {
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.drawStr(84 + shift_x, 7 + shift_y, "NO COMM");
      u8g2.drawFrame(122 + shift_x, 2 + shift_y, 5, 5);
    }
    u8g2.drawHLine(0, 10 + shift_y, 128);

    // 2. SOL ÜST: TURBO BASINCI, PEAK & GAUGE BAR
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(2 + shift_x, 22 + shift_y, "T:");
    
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.setCursor(16 + shift_x, 23 + shift_y);
    u8g2.print(val_turbo_bar, 2);
    
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(48 + shift_x, 17 + shift_y, "P:");
    u8g2.setCursor(57 + shift_x, 17 + shift_y);
    u8g2.print(peak_turbo_bar, 2);
    u8g2.drawStr(48 + shift_x, 24 + shift_y, "bar");

    u8g2.drawFrame(2 + shift_x, 27 + shift_y, 78, 5);
    int8_t bar_w = (int8_t)((val_turbo_bar / 2.0) * 74.0);
    if (bar_w > 74) bar_w = 74;
    if (bar_w < 0)  bar_w = 0;
    if (bar_w > 0) u8g2.drawBox(4 + shift_x, 29 + shift_y, bar_w, 2);

    // 3. SAĞ ÜST: DPF KUTUSU (REJENERASYONDA SÜREKLİ DOLU BEYAZ)
    if (val_regen_active) {
      u8g2.drawBox(83 + shift_x, 11 + shift_y, 44, 24);
      u8g2.setDrawColor(0);
      
      u8g2.setFont(u8g2_font_6x12_tf);
      u8g2.drawStr(85 + shift_x, 21 + shift_y, "DPF:");
      u8g2.setFont(u8g2_font_7x14B_tf);
      u8g2.setCursor(85 + shift_x, 33 + shift_y);
      u8g2.print(val_dpf_soot_g, 1);
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.drawStr(116 + shift_x, 33 + shift_y, "g");
      
      u8g2.setDrawColor(1);
    } else {
      u8g2.setFont(u8g2_font_6x12_tf);
      u8g2.drawStr(85 + shift_x, 21 + shift_y, "DPF:");
      u8g2.setFont(u8g2_font_7x14B_tf);
      u8g2.setCursor(85 + shift_x, 33 + shift_y);
      u8g2.print(val_dpf_soot_g, 1);
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.drawStr(116 + shift_x, 33 + shift_y, "g");
    }

    u8g2.drawHLine(0, 36 + shift_y, 128);

    // 4. SOL ALT: YAĞ VE DSG SICAKLIKLARI
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(2 + shift_x, 48 + shift_y, "OIL:");
    u8g2.setCursor(28 + shift_x, 48 + shift_y);
    u8g2.print(val_oil_temp_c);
    u8g2.drawStr(48 + shift_x, 48 + shift_y, "\xb0""C");

    u8g2.drawStr(2 + shift_x, 60 + shift_y, "DSG:");
    u8g2.setCursor(28 + shift_x, 60 + shift_y);
    u8g2.print(val_dsg_temp_c);
    u8g2.drawStr(48 + shift_x, 60 + shift_y, "\xb0""C");

    // 5. SAĞ ALT: EGT & REJENERASYON (DOLU BEYAZ KUTU)
    u8g2.drawVLine(70 + shift_x, 37 + shift_y, 27);
    
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(74 + shift_x, 47 + shift_y, "EGT:");
    u8g2.setCursor(98 + shift_x, 47 + shift_y);
    u8g2.print(val_egt_temp_c);
    u8g2.drawStr(118 + shift_x, 47 + shift_y, "C");

    u8g2.drawStr(74 + shift_x, 59 + shift_y, "REGEN:");
    if (val_regen_active) {
      u8g2.drawBox(105 + shift_x, 51 + shift_y, 22, 11);
      u8g2.setDrawColor(0);
      u8g2.drawStr(107 + shift_x, 59 + shift_y, "ACT");
      u8g2.setDrawColor(1);
    } else {
      u8g2.drawStr(107 + shift_x, 59 + shift_y, "OFF");
    }

  } while (u8g2.nextPage());
}

void setup() {
  Serial.begin(115200);
  initSPILock();
  
  u8g2.begin();
  u8g2.setContrast(170);

  drawSplashScreen();

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN0.setMode(MCP_NORMAL);
    can_online = true;
  }

  showIntermediateScreen();
  printLogHeader();
}

void loop() {
  updateBurnInProtection();
  runDiagnosticsScheduler();
  processIncomingCAN();
  streamDataLog();
  renderDashboard();
}