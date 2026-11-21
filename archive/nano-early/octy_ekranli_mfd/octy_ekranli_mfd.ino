/* ==========================================================================
 * Proje: Skoda Octavia 1.6 TDI (CLHA - MQB - EDC17C64) OLED Gösterge + Telemetri Loglayıcı
 * Donanım: Arduino Nano + MCP2515 (8MHz) + SSD1306 0.96" SPI OLED (7 Pin)
 * Protokol: CAN-Bus 500kbps / UDS (Service 0x22) & OBD-II (Service 0x01)
 * Log Formatı: CSV (Zaman, Turbo, DPF, Yag, DSG, EGT, Rejenerasyon) @ 115200 Baud
 * ========================================================================== */

#include <SPI.h>
#include <mcp_can.h>
#include <U8g2lib.h>

// --------------------------------------------------------------------------
// PIN TANIMLAMALARI (Ortak SPI Mimarisi)
// --------------------------------------------------------------------------
#define PIN_CAN_CS    10  // MCP2515 Chip Select
#define PIN_CAN_INT    2  // MCP2515 Interrupt
#define PIN_OLED_CS    9  // OLED Chip Select
#define PIN_OLED_DC    8  // OLED Data / Command
#define PIN_OLED_RES   7  // OLED Reset

// U8g2 1-Page Buffer (Sadece 128 Byte RAM Tüketimi)
U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, PIN_OLED_CS, PIN_OLED_DC, PIN_OLED_RES);
MCP_CAN CAN0(PIN_CAN_CS);

// --------------------------------------------------------------------------
// CAN-BUS & UDS / OBD2 ADRESLERİ
// --------------------------------------------------------------------------
#define ECU_REQ_ID          0x7E0
#define ECU_RESP_ID         0x7E8
#define TCM_REQ_ID          0x7E1
#define TCM_RESP_ID         0x7E9

#define DID_DPF_SOOT_CALC   0x2029  // DPF Kurum Kütlesi (g)
#define DID_EGT_PRE_DPF     0x2045  // Egzoz Gazı Sıcaklığı (°C)
#define DID_TURBO_BOOST     0x115B  // Turbo Mutlak Basıncı (mbar)
#define DID_OIL_TEMP        0x2024  // Yağ Sıcaklığı (°C)
#define DID_DSG_TEMP        0x1000  // DSG Sıcaklığı (°C)

// --------------------------------------------------------------------------
// SENSÖR VE İSTATİSTİK DEĞİŞKENLERİ
// --------------------------------------------------------------------------
float    val_turbo_bar      = 0.0;
float    val_dpf_soot_g     = 0.0;
int16_t  val_oil_temp_c     = 0;
int16_t  val_dsg_temp_c     = 0;
int16_t  val_egt_temp_c     = 0;
bool     val_regen_active   = false;

// Sürüş Seansı Zirve (Peak / Max) Değerleri
float    peak_turbo_bar     = 0.0;
int16_t  peak_oil_temp_c    = 0;
int16_t  peak_egt_temp_c    = 0;

// Sistem & Zamanlayıcılar
bool     can_online         = false;
uint8_t  query_step         = 0;
uint32_t last_query_ms      = 0;
uint32_t last_resp_ms       = 0;
uint32_t last_log_ms        = 0;     // Log Frekans Zamanlayıcısı
bool     rec_blink_state    = false; // [REC] Işığı Yanıp Sönme Durumu

// ==========================================================================
// YARDIMCI FONKSİYONLAR: SPI & CAN & LOGLAMA
// ==========================================================================

void initSPILock() {
  pinMode(PIN_CAN_CS, OUTPUT);
  pinMode(PIN_OLED_CS, OUTPUT);
  digitalWrite(PIN_CAN_CS, HIGH);
  digitalWrite(PIN_OLED_CS, HIGH);
}

// UDS (Service 0x22) İsteği Gönder
void sendUDSRequest(uint32_t can_id, uint16_t did) {
  uint8_t tx_frame[8] = {
    0x03, 0x22, 
    (uint8_t)(did >> 8), 
    (uint8_t)(did & 0xFF), 
    0x55, 0x55, 0x55, 0x55
  };
  digitalWrite(PIN_OLED_CS, HIGH);
  CAN0.sendMsgBuf(can_id, 0, 8, tx_frame);
}

// CSV Log Başlığı (Cihaz Açılışında 1 Kez Basılır)
void printLogHeader() {
  Serial.println(F("\n================================================================================"));
  Serial.println(F("           SKODA OCTAVIA 1.6 TDI CAN-BUS CANLI TELEMETRI LOGU                   "));
  Serial.println(F("================================================================================"));
  Serial.println(F("TIME_MS,TURBO_BAR,PEAK_BAR,DPF_SOOT_G,OIL_TEMP_C,DSG_TEMP_C,EGT_C,REGEN_STATUS"));
}

// CSV Formatında Periyodik Canlı Veri Loglama (Zero RAM Allocation)
void streamDataLog() {
  uint32_t current_ms = millis();
  
  // Her 250 ms'de bir log satırı gönder (Saniyede 4 örnek)
  if (current_ms - last_log_ms >= 250) {
    last_log_ms = current_ms;
    rec_blink_state = !rec_blink_state; // Kayıt ışığını yak/söndür

    // Sadece CAN bağlantısı aktifse logla
    if (can_online) {
      Serial.print(current_ms);
      Serial.print(',');
      Serial.print(val_turbo_bar, 2);
      Serial.print(',');
      Serial.print(peak_turbo_bar, 2);
      Serial.print(',');
      Serial.print(val_dpf_soot_g, 1);
      Serial.print(',');
      Serial.print(val_oil_temp_c);
      Serial.print(',');
      Serial.print(val_dsg_temp_c);
      Serial.print(',');
      Serial.print(val_egt_temp_c);
      Serial.print(',');
      Serial.println(val_regen_active ? 1 : 0);
    }
  }
}

// CAN Gelen Mesaj İşleyici & Zirve (Peak) Hesaplayıcı
void processIncomingCAN() {
  while (CAN0.checkReceive() == CAN_MSGAVAIL) {
    uint32_t rx_id = 0;
    uint8_t len = 0;
    uint8_t buf[8];

    digitalWrite(PIN_OLED_CS, HIGH);
    CAN0.readMsgBuf(&rx_id, &len, buf);

    // Motor Beyni (0x7E8)
    if (rx_id == ECU_RESP_ID) {
      last_resp_ms = millis();
      can_online = true;

      if (buf[1] == 0x62) { // UDS Pozitif Yanıt
        uint16_t resp_did = ((uint16_t)buf[2] << 8) | buf[3];

        // 1. DPF Kurum Miktarı (0x2029)
        if (resp_did == DID_DPF_SOOT_CALC) {
          uint16_t raw_soot = ((uint16_t)buf[4] << 8) | buf[5];
          val_dpf_soot_g = raw_soot * 0.01;
        }
        // 2. Egzoz Gazı Sıcaklığı - EGT (0x2045)
        else if (resp_did == DID_EGT_PRE_DPF) {
          uint16_t raw_egt = ((uint16_t)buf[4] << 8) | buf[5];
          val_egt_temp_c = (int16_t)(raw_egt * 0.1) - 40;
          if (val_egt_temp_c > peak_egt_temp_c) peak_egt_temp_c = val_egt_temp_c;
        }
        // 3. Motor Yağ Sıcaklığı (0x2024)
        else if (resp_did == DID_OIL_TEMP) {
          val_oil_temp_c = (int16_t)buf[4] - 40;
          if (val_oil_temp_c > peak_oil_temp_c) peak_oil_temp_c = val_oil_temp_c;
        }
        // 4. Turbo Basıncı (0x115B)
        else if (resp_did == DID_TURBO_BOOST) {
          uint16_t raw_mbar = ((uint16_t)buf[4] << 8) | buf[5];
          if (raw_mbar > 1000) {
            val_turbo_bar = (float)(raw_mbar - 1013) / 1000.0;
          } else {
            val_turbo_bar = 0.0;
          }
          // Zirve Turbo Değerini Güncelle
          if (val_turbo_bar > peak_turbo_bar) {
            peak_turbo_bar = val_turbo_bar;
          }
        }
      }
    }
    // DSG Şanzıman Beyni (0x7E9)
    else if (rx_id == TCM_RESP_ID) {
      last_resp_ms = millis();
      if (buf[1] == 0x62 && (((uint16_t)buf[2] << 8) | buf[3]) == DID_DSG_TEMP) {
        val_dsg_temp_c = (int16_t)buf[4] - 40;
      }
    }
  }

  // Rejenerasyon Durumu Belirleme (Kurum > 22g veya EGT > 500°C)
  val_regen_active = (val_dpf_soot_g >= 22.0 || val_egt_temp_c >= 500);
}

// Sensör Sorgulama Motoru (75ms Sıralı Döngü)
void runDiagnosticsScheduler() {
  uint32_t current_ms = millis();
  
  if (current_ms - last_query_ms >= 75) {
    last_query_ms = current_ms;
    
    switch (query_step) {
      case 0: sendUDSRequest(ECU_REQ_ID, DID_TURBO_BOOST);   break;
      case 1: sendUDSRequest(ECU_REQ_ID, DID_DPF_SOOT_CALC); break;
      case 2: sendUDSRequest(ECU_REQ_ID, DID_OIL_TEMP);      break;
      case 3: sendUDSRequest(TCM_REQ_ID, DID_DSG_TEMP);      break;
    }
    query_step = (query_step + 1) % 4;
  }

  if (current_ms - last_resp_ms > 2500) {
    can_online = false;
  }
}

// ==========================================================================
// EKRAN VE GRAFİK FONKSİYONLARI (U8g2 Sayfa Modu)
// ==========================================================================

void drawSplashScreen() {
  u8g2.firstPage();
  do {
    u8g2.drawRFrame(0, 0, 128, 64, 4);
    u8g2.setFont(u8g2_font_helvB10_tf);
    u8g2.drawStr(14, 22, "SKODA AUTO");
    
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(12, 38, "OCTAVIA 1.6 TDI");
    
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(18, 52, "LOGGER & MFD v1.1");
  } while (u8g2.nextPage());
  delay(1600);
}

void renderDashboard() {
  digitalWrite(PIN_CAN_CS, HIGH); // CAN modülünü SPI'da izole et

  u8g2.firstPage();
  do {
    // -------------------------------------------------------------
    // 1. ÜST BİLGİ & DURUM ÇUBUĞU
    // -------------------------------------------------------------
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(2, 7, "OCTAVIA");

    // Canlı Loglama / REC Göstergesi (Yanıp Sönen Şık Kutu)
    if (can_online && rec_blink_state) {
      u8g2.drawBox(48, 1, 20, 7);
      u8g2.setDrawColor(0); // Beyaz zemin üstü siyah yazı
      u8g2.drawStr(50, 7, "REC");
      u8g2.setDrawColor(1); // Normal renge dön
    } else if (can_online) {
      u8g2.drawFrame(48, 1, 20, 7);
      u8g2.drawStr(50, 7, "LOG");
    }

    // CAN Bağlantı Durumu
    if (can_online) {
      u8g2.drawStr(84, 7, "CAN: OK");
      u8g2.drawDisc(124, 4, 2);
    } else {
      u8g2.drawStr(80, 7, "NO COMM");
      u8g2.drawFrame(122, 2, 5, 5);
    }
    
    u8g2.drawHLine(0, 9, 128);

    // -------------------------------------------------------------
    // 2. SOL ÜST: TURBO BASINCI, PEAK BOOST & GAUGE BAR
    // -------------------------------------------------------------
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(2, 21, "T:");
    
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.setCursor(16, 22);
    u8g2.print(val_turbo_bar, 2);
    
    // Zirve (Peak) Boost Göstergesi (Örn: P:1.45)
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(48, 16, "P:");
    u8g2.setCursor(57, 16);
    u8g2.print(peak_turbo_bar, 2);
    u8g2.drawStr(48, 23, "bar");

    // Turbo Dinamik Barı (0.0 - 2.0 bar)
    u8g2.drawFrame(2, 26, 78, 5);
    int8_t bar_w = (int8_t)((val_turbo_bar / 2.0) * 74.0);
    if (bar_w > 74) bar_w = 74;
    if (bar_w < 0)  bar_w = 0;
    if (bar_w > 0) {
      u8g2.drawBox(4, 28, bar_w, 2);
    }

    // -------------------------------------------------------------
    // 3. SAĞ ÜST: DPF KURUM MİKTARI
    // -------------------------------------------------------------
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(88, 20, "DPF:");
    
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.setCursor(88, 33);
    u8g2.print(val_dpf_soot_g, 1);
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(118, 33, "g");

    // 24g Üzerinde DPF Uyarı Çerçevesi
    if (val_dpf_soot_g >= 24.0) {
      u8g2.drawFrame(85, 11, 43, 24);
    }

    u8g2.drawHLine(0, 36, 128);

    // -------------------------------------------------------------
    // 4. SOL ALT: YAĞ VE DSG SICAKLIKLARI
    // -------------------------------------------------------------
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(2, 48, "OIL:");
    u8g2.setCursor(28, 48);
    u8g2.print(val_oil_temp_c);
    u8g2.drawStr(48, 48, "\xb0""C");

    u8g2.drawStr(2, 60, "DSG:");
    u8g2.setCursor(28, 60);
    u8g2.print(val_dsg_temp_c);
    u8g2.drawStr(48, 60, "\xb0""C");

    // -------------------------------------------------------------
    // 5. SAĞ ALT: EGT & REJENERASYON BİLGİSİ
    // -------------------------------------------------------------
    u8g2.drawVLine(70, 37, 27);
    
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(75, 47, "EGT:");
    u8g2.setCursor(99, 47);
    u8g2.print(val_egt_temp_c);
    u8g2.drawStr(118, 47, "C");

    u8g2.drawStr(75, 59, "REGEN:");
    if (val_regen_active) {
      u8g2.drawStr(108, 59, "ACT"); // Rejenerasyon Devrede
    } else {
      u8g2.drawStr(108, 59, "OFF");
    }

  } while (u8g2.nextPage());
}

// ==========================================================================
// ARDUINO SETUP & LOOP
// ==========================================================================
void setup() {
  // 1. Seri Port Loglama Hattını Başlat (115200 Baud)
  Serial.begin(115200);

  // 2. SPI Donanım Güvenliği
  initSPILock();
  
  // 3. OLED Başlatma ve Karşılama Ekranı
  u8g2.begin();
  drawSplashScreen();

  // 4. CAN-Bus Başlatma (500 kbps, 8MHz Kristal)
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN0.setMode(MCP_NORMAL);
    can_online = true;
  } else {
    can_online = false;
  }

  // 5. CSV Log Başlıklarını Seri Porta Yaz
  printLogHeader();
}

void loop() {
  // 1. Sıralı UDS İsteklerini Gönder (Non-blocking)
  runDiagnosticsScheduler();

  // 2. Gelen CAN Yanıtlarını İşle ve Peak Değerleri Güncelle
  processIncomingCAN();

  // 3. Telemetri Verilerini CSV Formatında Seri Porta Logla (250 ms'de bir)
  streamDataLog();

  // 4. OLED Göstergeyi Yenile
  renderDashboard();
}