/* ==========================================================================
 * Proje: Skoda Octavia 1.6 TDI (CLHA - MQB - EDC17C64) Havalandırma Izgarası OLED Gösterge
 * Donanım: Arduino Nano + MCP2515 (8MHz) + SSD1306 0.96" SPI OLED (7 Pin)
 * Protokol: CAN-Bus 500kbps / UDS (Service 0x22) & OBD-II (Service 0x01)
 * ========================================================================== */

#include <SPI.h>
#include <mcp_can.h>
#include <U8g2lib.h>

// --------------------------------------------------------------------------
// PIN TANIMLAMALARI (Ortak SPI Mimarisi)
// --------------------------------------------------------------------------
#define PIN_CAN_CS    10  // MCP2515 Chip Select
#define PIN_CAN_INT    2  // MCP2515 Interrupt (Opsiyonel donanım pini)
#define PIN_OLED_CS    9  // OLED Chip Select
#define PIN_OLED_DC    8  // OLED Data / Command
#define PIN_OLED_RES   7  // OLED Reset
// SCK  -> D13 (Arduino Donanımsal SPI Saati - Ortak)
// MOSI -> D11 (Arduino Donanımsal SPI Veri Çıkışı - Ortak)
// MISO -> D12 (MCP2515 SO - Sadece CAN okuma için)

// --------------------------------------------------------------------------
// NESNE TANIMLARI
// --------------------------------------------------------------------------
// U8g2 Page Buffer (128 Byte RAM Tüketimi) Donanımsal SPI Sürücüsü
U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, PIN_OLED_CS, PIN_OLED_DC, PIN_OLED_RES);
MCP_CAN CAN0(PIN_CAN_CS);

// --------------------------------------------------------------------------
// CAN-BUS & UDS / OBD2 TANIMLARI
// --------------------------------------------------------------------------
#define ECU_REQ_ID      0x7E0  // Motor Beyni (EDC17C64) İstek ID
#define ECU_RESP_ID     0x7E8  // Motor Beyni Cevap ID
#define TCM_REQ_ID      0x7E1  // Şanzıman Beyni (DSG DQ200) İstek ID
#define TCM_RESP_ID     0x7E9  // Şanzıman Beyni Cevap ID

// VAG UDS (0x22) Data Identifier (DID) Kodları
#define DID_DPF_SOOT_CALC   0x2029  // DPF Hesaplanmış Kurum Kütlesi (0.01 g veya 0.1 g)
#define DID_EGT_PRE_DPF     0x2045  // DPF Giriş Egzoz Gazı Sıcaklığı (°C)
#define DID_TURBO_BOOST     0x115B  // Turbo Mutlak Basıncı (hPa / mbar)
#define DID_OIL_TEMP        0x2024  // Motor Yağ Sıcaklığı (°C)
#define DID_DSG_TEMP        0x1000  // DSG Kavrama/Yağ Sıcaklığı (°C)

// --------------------------------------------------------------------------
// SİSTEM DEĞİŞKENLERİ
// --------------------------------------------------------------------------
float  val_turbo_bar    = 0.0;   // Göreli Turbo Basıncı (Bar)
float  val_dpf_soot_g   = 0.0;   // DPF Kurum Miktarı (Gram)
int16_t val_oil_temp_c  = 0;     // Yağ Sıcaklığı (°C)
int16_t val_dsg_temp_c  = 0;     // DSG Sıcaklığı (°C)
int16_t val_egt_temp_c  = 0;     // Egzoz Gazı Sıcaklığı (°C)

bool   can_online       = false; // CAN Hattı Bağlantı Durumu
uint8_t query_step      = 0;     // Sorgu Döngü Durumu
uint32_t last_query_ms  = 0;     // Sorgu Zamanlayıcısı
uint32_t last_resp_ms   = 0;     // Son Geçerli Cevap Zamanlayıcısı

// ==========================================================================
// YARDIMCI FONKSİYONLAR: SPI & CAN
// ==========================================================================

// SPI Çakışmasını önlemek için CS pinlerini güvene al
void initSPILock() {
  pinMode(PIN_CAN_CS, OUTPUT);
  pinMode(PIN_OLED_CS, OUTPUT);
  digitalWrite(PIN_CAN_CS, HIGH);
  digitalWrite(PIN_OLED_CS, HIGH);
}

// UDS (Service 0x22) İsteği Gönder (Single Frame ISO-TP)
void sendUDSRequest(uint32_t can_id, uint16_t did) {
  uint8_t tx_frame[8] = {
    0x03,                         // PCI: Single Frame, 3 Veri Byte'ı
    0x22,                         // Service: ReadDataByIdentifier
    (uint8_t)(did >> 8),          // DID High Byte
    (uint8_t)(did & 0xFF),        // DID Low Byte
    0x55, 0x55, 0x55, 0x55        // Standart VAG Padding
  };
  
  digitalWrite(PIN_OLED_CS, HIGH); // OLED'i SPI'da pasife çek
  CAN0.sendMsgBuf(can_id, 0, 8, tx_frame);
}

// Standart OBD-II (Service 0x01) İsteği Gönder
void sendOBDRequest(uint32_t can_id, uint8_t pid) {
  uint8_t tx_frame[8] = {
    0x02,                         // PCI: Single Frame, 2 Veri Byte'ı
    0x01,                         // Service: Show Current Data
    pid,                          // PID Kodu
    0x55, 0x55, 0x55, 0x55, 0x55  // Standart Padding
  };
  
  digitalWrite(PIN_OLED_CS, HIGH);
  CAN0.sendMsgBuf(can_id, 0, 8, tx_frame);
}

// Gelen CAN Mesajlarını Ayrıştır
void processIncomingCAN() {
  while (CAN0.checkReceive() == CAN_MSGAVAIL) {
    uint32_t rx_id = 0;
    uint8_t len = 0;
    uint8_t buf[8];

    digitalWrite(PIN_OLED_CS, HIGH);
    CAN0.readMsgBuf(&rx_id, &len, buf);

    // Motor Beyni (0x7E8) Cevapları
    if (rx_id == ECU_RESP_ID) {
      last_resp_ms = millis();
      can_online = true;

      // 1. UDS Service 0x22 Pozitif Cevabı (0x62)
      if (buf[1] == 0x62) {
        uint16_t resp_did = ((uint16_t)buf[2] << 8) | buf[3];

        // A) DPF Kurum Miktarı (0x2029)
        if (resp_did == DID_DPF_SOOT_CALC) {
          uint16_t raw_soot = ((uint16_t)buf[4] << 8) | buf[5];
          val_dpf_soot_g = raw_soot * 0.01; // 0.01g katsayısı
        }
        // B) Egzoz Gazı Sıcaklığı - EGT (0x2045)
        else if (resp_did == DID_EGT_PRE_DPF) {
          uint16_t raw_egt = ((uint16_t)buf[4] << 8) | buf[5];
          val_egt_temp_c = (int16_t)(raw_egt * 0.1) - 40;
        }
        // C) Motor Yağ Sıcaklığı (0x2024)
        else if (resp_did == DID_OIL_TEMP) {
          val_oil_temp_c = (int16_t)buf[4] - 40;
        }
        // D) Turbo Basıncı (UDS 0x115B)
        else if (resp_did == DID_TURBO_BOOST) {
          uint16_t raw_mbar = ((uint16_t)buf[4] << 8) | buf[5];
          // Atmosfer basıncı ~1013 mbar düşülerek göreli bar hesaplanır
          if (raw_mbar > 1000) {
            val_turbo_bar = (float)(raw_mbar - 1013) / 1000.0;
          } else {
            val_turbo_bar = 0.0;
          }
        }
      }
      // 2. Standart OBD-II Service 0x01 Pozitif Cevabı (0x41)
      else if (buf[1] == 0x41) {
        // PID 0x0B -> MAP / Manifold Basıncı (kPa)
        if (buf[2] == 0x0B) {
          int16_t kpa = buf[3];
          val_turbo_bar = (float)(kpa - 101) * 0.01;
          if (val_turbo_bar < 0) val_turbo_bar = 0.0;
        }
        // PID 0x5C -> Motor Yağ Sıcaklığı (°C)
        else if (buf[2] == 0x5C) {
          val_oil_temp_c = (int16_t)buf[3] - 40;
        }
      }
    }
    // DSG Şanzıman Beyni (0x7E9) Cevapları
    else if (rx_id == TCM_RESP_ID) {
      last_resp_ms = millis();
      if (buf[1] == 0x62 && (((uint16_t)buf[2] << 8) | buf[3]) == DID_DSG_TEMP) {
        val_dsg_temp_c = (int16_t)buf[4] - 40;
      }
    }
  }
}

// Belirli aralıklarla sensörleri sırayla sorgula (CAN-Bus Trafiğini Boğmaz)
void runDiagnosticsScheduler() {
  uint32_t current_ms = millis();
  
  // Her 75ms'de bir parametre sorgula (Tüm döngü ~300ms)
  if (current_ms - last_query_ms >= 75) {
    last_query_ms = current_ms;
    
    switch (query_step) {
      case 0:
        // Turbo Basıncı Sorgusu
        sendUDSRequest(ECU_REQ_ID, DID_TURBO_BOOST);
        break;
      case 1:
        // DPF Kurum Kütlesi Sorgusu
        sendUDSRequest(ECU_REQ_ID, DID_DPF_SOOT_CALC);
        break;
      case 2:
        // Motor Yağ Sıcaklığı Sorgusu
        sendUDSRequest(ECU_REQ_ID, DID_OIL_TEMP);
        break;
      case 3:
        // DSG Sıcaklığı veya EGT Sorgusu
        sendUDSRequest(TCM_REQ_ID, DID_DSG_TEMP);
        break;
    }
    
    query_step = (query_step + 1) % 4;
  }

  // 2.5 saniye boyunca cevap gelmezse bağlantı koptu/kontak kapalı say
  if (current_ms - last_resp_ms > 2500) {
    can_online = false;
  }
}

// ==========================================================================
// EKRAN VE GRAFİK FONKSİYONLARI (U8g2 Sayfa Tabanlı)
// ==========================================================================

// Karşılama (Açılış) Ekranı - Sadece setup'ta çalışır
void drawSplashScreen() {
  u8g2.firstPage();
  do {
    // Çerçeve
    u8g2.drawRFrame(0, 0, 128, 64, 4);
    
    // Logo & Karşılama Metinleri
    u8g2.setFont(u8g2_font_helvB10_tf);
    u8g2.drawStr(14, 22, "SKODA AUTO");
    
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(12, 38, "OCTAVIA 1.6 TDI");
    
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(24, 52, "MQB CAN-BUS v1.0");
  } while (u8g2.nextPage());
  
  delay(1600); // Karşılama logosunu kısa süre göster
}

// Ana Gösterge Ekranını Çiz
void renderDashboard() {
  digitalWrite(PIN_CAN_CS, HIGH); // CAN modülünü SPI'da izole et

  u8g2.firstPage();
  do {
    // 1. ÜST BAŞLIK / DURUM ÇUBUĞU
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(2, 7, "OCTAVIA 1.6 TDI");
    
    // CAN Durum Göstergesi (Sağ Üst)
    if (can_online) {
      u8g2.drawStr(88, 7, "CAN: OK");
      u8g2.drawDisc(124, 4, 2); // Küçük yeşil/aktif noktası
    } else {
      u8g2.drawStr(80, 7, "NO COMM");
      u8g2.drawFrame(122, 2, 5, 5);
    }
    
    u8g2.drawHLine(0, 9, 128); // Üst ayırıcı çizgi

    // 2. SOL ÜST: TURBO BASINCI & MİNİ GAUGE BAR
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(2, 22, "TURBO:");
    
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.setCursor(44, 23);
    u8g2.print(val_turbo_bar, 2);
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(74, 23, "bar");

    // Turbo Dinamik Bar Grafiği (0.0 - 2.0 bar aralığı, 44px genişlik)
    u8g2.drawFrame(2, 26, 78, 5);
    int8_t bar_w = (int8_t)((val_turbo_bar / 2.0) * 74.0);
    if (bar_w > 74) bar_w = 74;
    if (bar_w < 0)  bar_w = 0;
    if (bar_w > 0) {
      u8g2.drawBox(4, 28, bar_w, 2);
    }

    // 3. SAĞ ÜST: DPF KURUM MİKTARI
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(88, 20, "DPF:");
    
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.setCursor(88, 33);
    u8g2.print(val_dpf_soot_g, 1);
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(118, 33, "g");

    // DPF 24g üzerine çıkarsa Rejenerasyon Uyarı Simgesi
    if (val_dpf_soot_g >= 24.0) {
      u8g2.drawFrame(86, 11, 42, 24); // Uyarı çerçevesi
    }

    u8g2.drawHLine(0, 36, 128); // Orta ayırıcı çizgi

    // 4. SOL ALT: YAĞ VE DSG SICAKLIKLARI
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(2, 48, "OIL:");
    u8g2.setCursor(28, 48);
    u8g2.print(val_oil_temp_c);
    u8g2.drawStr(48, 48, "\xb0""C"); // Derece sembolü

    u8g2.drawStr(2, 60, "DSG:");
    u8g2.setCursor(28, 60);
    u8g2.print(val_dsg_temp_c);
    u8g2.drawStr(48, 60, "\xb0""C");

    // 5. SAĞ ALT: DPF DURUMU VEYA EGT BİLGİSİ
    u8g2.drawVLine(70, 37, 27); // Dikey ayırıcı
    
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(75, 47, "EGT:");
    u8g2.setCursor(99, 47);
    u8g2.print(val_egt_temp_c);
    u8g2.drawStr(118, 47, "C");

    // DPF Durum Özeti (Rejenerasyon Takibi)
    u8g2.drawStr(75, 59, "REGEN:");
    if (val_dpf_soot_g >= 22.0 || val_egt_temp_c > 500) {
      u8g2.drawStr(108, 59, "ACT"); // Rejenerasyon Aktif / Yüksek EGT
    } else {
      u8g2.drawStr(108, 59, "OFF");
    }

  } while (u8g2.nextPage());
}

// ==========================================================================
// ARDUINO SETUP & LOOP
// ==========================================================================
void setup() {
  // 1. SPI Donanımını ve CS Pinlerini Hazırla
  initSPILock();
  
  // 2. OLED Ekranı Başlat ve Açılış Ekranını Çiz
  u8g2.begin();
  drawSplashScreen();

  // 3. MCP2515 CAN Kontrolcüsünü Başlat (8MHz Kristal, 500kbps Baud Hızı)
  // Araç OBD soketi Pin 6-14 MQB platformunda standart 500kbps'dir.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN0.setMode(MCP_NORMAL); // Normal çalışma moduna al
    can_online = true;
  } else {
    can_online = false;
  }
}

void loop() {
  // 1. Sensör İsteklerini Zamanla Gönder (Non-blocking)
  runDiagnosticsScheduler();

  // 2. Gelen CAN-Bus Paketlerini Dinle ve Değerleri Güncelle
  processIncomingCAN();

  // 3. OLED Ekranı Yenile
  renderDashboard();
}