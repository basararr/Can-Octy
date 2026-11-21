/* ==========================================================================
 * Proje: Skoda Octavia 1.6 TDI Yüksek Hızlı OLED MFD + Telemetri Loglayıcı (v1.2)
 * Donanım: 1. Arduino Nano + MCP2515 (8MHz) + SSD1306 SPI OLED
 * Özellikler: Turbo Öncelikli Hızlı Sorgulama, 1Hz REC Simgesi, Canlı EGT/Regen
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

#define DID_DPF_SOOT_CALC   0x2029
#define DID_EGT_PRE_DPF     0x2045
#define DID_TURBO_BOOST     0x115B
#define DID_OIL_TEMP        0x2024
#define DID_DSG_TEMP        0x1000

// Canlı Sensör Verileri
float    val_turbo_bar      = 0.0;
float    val_dpf_soot_g     = 0.0;
int16_t  val_oil_temp_c     = 0;
int16_t  val_dsg_temp_c     = 0;
int16_t  val_egt_temp_c     = 0;
bool     val_regen_active   = false;

float    peak_turbo_bar     = 0.0;
bool     can_online         = false;
uint8_t  scheduler_step     = 0;
uint32_t last_query_ms      = 0;
uint32_t last_resp_ms       = 0;
uint32_t last_log_ms        = 0;
uint32_t last_blink_ms      = 0;
bool     rec_blink_state    = true;

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

void printLogHeader() {
  Serial.println(F("\nTIME_MS,TURBO_BAR,PEAK_BAR,DPF_SOOT_G,OIL_C,DSG_C,EGT_C,REGEN"));
}

void streamDataLog() {
  uint32_t now = millis();
  if (now - last_log_ms >= 250) { // 250ms'de bir telemetri satırı
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

  // REC simgesinin yanıp sönme hızı: 1000ms (1 Hz - Çok daha sakin)
  if (now - last_blink_ms >= 800) {
    last_blink_ms = now;
    rec_blink_state = !rec_blink_state;
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

  // Rejenerasyon Belirleme (Kurum >= 24g VEYA EGT >= 500°C)
  val_regen_active = (val_dpf_soot_g >= 24.0 || val_egt_temp_c >= 500);
}

// Turbo Öncelikli Süper Hızlı Sorgulama Motoru (Her 25ms'de bir tetiklenir)
void runDiagnosticsScheduler() {
  uint32_t now = millis();
  if (now - last_query_ms >= 25) { // 25ms = Saniyede 40 sorgu!
    last_query_ms = now;

    // Turbo'ya %50 öncelik verilir (Her 2 adımda bir Turbo okunur)
    switch (scheduler_step) {
      case 0: sendUDSRequest(ECU_REQ_ID, DID_TURBO_BOOST);   break;
      case 1: sendUDSRequest(ECU_REQ_ID, DID_DPF_SOOT_CALC); break;
      case 2: sendUDSRequest(ECU_REQ_ID, DID_TURBO_BOOST);   break;
      case 3: sendUDSRequest(ECU_REQ_ID, DID_EGT_PRE_DPF);     break;
      case 4: sendUDSRequest(ECU_REQ_ID, DID_TURBO_BOOST);   break;
      case 5: sendUDSRequest(ECU_REQ_ID, DID_OIL_TEMP);      break;
      case 6: sendUDSRequest(ECU_REQ_ID, DID_TURBO_BOOST);   break;
      case 7: sendUDSRequest(TCM_REQ_ID, DID_DSG_TEMP);      break;
    }
    scheduler_step = (scheduler_step + 1) % 8;
  }

  if (now - last_resp_ms > 2500) can_online = false;
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
    u8g2.drawStr(20, 52, "FAST MFD v1.2");
  } while (u8g2.nextPage());
  delay(1400);
}

void renderDashboard() {
  digitalWrite(PIN_CAN_CS, HIGH);
  u8g2.firstPage();
  do {
    // 1. ÜST DURUM ÇUBUĞU
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(2, 7, "OCTAVIA");

    if (can_online && rec_blink_state) {
      u8g2.drawBox(48, 1, 20, 7);
      u8g2.setDrawColor(0);
      u8g2.drawStr(50, 7, "REC");
      u8g2.setDrawColor(1);
    } else if (can_online) {
      u8g2.drawFrame(48, 1, 20, 7);
      u8g2.drawStr(50, 7, "LOG");
    }

    if (can_online) {
      u8g2.drawStr(84, 7, "CAN: OK");
      u8g2.drawDisc(124, 4, 2);
    } else {
      u8g2.drawStr(80, 7, "NO COMM");
      u8g2.drawFrame(122, 2, 5, 5);
    }
    u8g2.drawHLine(0, 9, 128);

    // 2. TURBO BASINCI & GAUGE BAR
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(2, 21, "T:");
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.setCursor(16, 22);
    u8g2.print(val_turbo_bar, 2);
    
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(48, 16, "P:");
    u8g2.setCursor(57, 16);
    u8g2.print(peak_turbo_bar, 2);
    u8g2.drawStr(48, 23, "bar");

    u8g2.drawFrame(2, 26, 78, 5);
    int8_t bar_w = (int8_t)((val_turbo_bar / 2.0) * 74.0);
    if (bar_w > 74) bar_w = 74;
    if (bar_w < 0)  bar_w = 0;
    if (bar_w > 0) u8g2.drawBox(4, 28, bar_w, 2);

    // 3. DPF KURUM
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(88, 20, "DPF:");
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.setCursor(88, 33);
    u8g2.print(val_dpf_soot_g, 1);
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(118, 33, "g");

    if (val_dpf_soot_g >= 24.0) {
      u8g2.drawFrame(85, 11, 43, 24);
    }
    u8g2.drawHLine(0, 36, 128);

    // 4. YAĞ VE DSG SICAKLIĞI
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(2, 48, "OIL:");
    u8g2.setCursor(28, 48);
    u8g2.print(val_oil_temp_c);
    u8g2.drawStr(48, 48, "\xb0""C");

    u8g2.drawStr(2, 60, "DSG:");
    u8g2.setCursor(28, 60);
    u8g2.print(val_dsg_temp_c);
    u8g2.drawStr(48, 60, "\xb0""C");

    // 5. EGT & REJENERASYON BİLGİSİ
    u8g2.drawVLine(70, 37, 27);
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(75, 47, "EGT:");
    u8g2.setCursor(99, 47);
    u8g2.print(val_egt_temp_c);
    u8g2.drawStr(118, 47, "C");

    u8g2.drawStr(75, 59, "REGEN:");
    if (val_regen_active) {
      u8g2.drawStr(108, 59, "ACT");
      u8g2.drawFrame(106, 51, 22, 11); // Rejenerasyon açıkken etrafını kutu içine al
    } else {
      u8g2.drawStr(108, 59, "OFF");
    }
  } while (u8g2.nextPage());
}

void setup() {
  Serial.begin(115200);
  initSPILock();
  u8g2.begin();
  drawSplashScreen();

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN0.setMode(MCP_NORMAL);
    can_online = true;
  }
  printLogHeader();
}

void loop() {
  runDiagnosticsScheduler();
  processIncomingCAN();
  streamDataLog();
  renderDashboard();
}