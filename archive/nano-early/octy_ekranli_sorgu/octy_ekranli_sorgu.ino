/* ==========================================================================
 * Proje: VAG MQB (Skoda Octavia) ECU & TCU Model/Parça No Tanımlama Aracı
 * Donanım: Arduino Nano + MCP2515 (8MHz) + SSD1306 0.96" SPI OLED
 * Amaç: ECU (0x7E0) ve TCU (0x7E1) modellerini, VAG parça numaralarını,
 *       yazılım versiyonlarını ve ASAM/ODX veri setlerini tespit etmek.
 * Çıktı: Hem Serial Monitör (115200 baud) hem de OLED Ekran.
 * ========================================================================== */

#include <SPI.h>
#include <mcp_can.h>
#include <U8g2lib.h>

// --------------------------------------------------------------------------
// PIN TANIMLAMALARI (Mevcut Devrenizle %100 Uyumlu)
// --------------------------------------------------------------------------
#define PIN_CAN_CS    10
#define PIN_CAN_INT    2
#define PIN_OLED_CS    9
#define PIN_OLED_DC    8
#define PIN_OLED_RES   7

// U8g2 Donanımsal SPI Sürücüsü (1-Page / Düşük RAM Modu)
U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, PIN_OLED_CS, PIN_OLED_DC, PIN_OLED_RES);
MCP_CAN CAN0(PIN_CAN_CS);

// --------------------------------------------------------------------------
// CAN & UDS TEŞHİS ADRESLERİ
// --------------------------------------------------------------------------
#define CAN_ID_ECU_REQ      0x7E0   // Motor Beyni İstek ID
#define CAN_ID_ECU_RESP     0x7E8   // Motor Beyni Cevap ID
#define CAN_ID_TCU_REQ      0x7E1   // Şanzıman Beyni (DSG) İstek ID
#define CAN_ID_TCU_RESP     0x7E9   // Şanzıman Beyni Cevap ID

// VAG UDS (Service 0x22) Standart Tanımlama DID Kodları (ISO 14229)
#define DID_VAG_PART_NO     0xF187  // VAG Yedek Parça Numarası (Örn: 04L 906 021 M)
#define DID_SW_VERSION      0xF189  // Yazılım Versiyon Numarası (Örn: 9970 / 1234)
#define DID_HW_PART_NO      0xF191  // Donanım Parça Numarası (Örn: 04L 907 445)
#define DID_SYSTEM_NAME     0xF197  // Sistem Adı / Motor Kodu (Örn: R4 1.6l TDI EDC17)
#define DID_ASAM_ODX_ID     0xF19E  // ASAM/ODX Teşhis Veri Seti Adı

// --------------------------------------------------------------------------
// TESPİT EDİLEN VERİLERİN TUTULDUĞU TAMPON BELLEKLER
// --------------------------------------------------------------------------
char vin_str[18]            = "Bilinmiyor";
char ecu_part_str[20]       = "Bulunamadi";
char ecu_sys_str[30]        = "Bulunamadi";
char ecu_sw_str[10]         = "---";
char ecu_asam_str[30]       = "---";

char tcu_part_str[20]       = "Bulunamadi";
char tcu_sys_str[30]        = "Bulunamadi";
char tcu_sw_str[10]         = "---";

bool ecu_found              = false;
bool tcu_found              = false;

// ==========================================================================
// SPI & ISO-TP MULTI-FRAME PROTOKOL MOTORU
// ==========================================================================

void initSPISafety() {
  pinMode(PIN_CAN_CS, OUTPUT);
  pinMode(PIN_OLED_CS, OUTPUT);
  digitalWrite(PIN_CAN_CS, HIGH);
  digitalWrite(PIN_OLED_CS, HIGH);
}

// ISO-TP Flow Control (Akış Kontrolü) Gönder
void sendFlowControl(uint32_t req_id) {
  uint8_t fc_frame[8] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  digitalWrite(PIN_OLED_CS, HIGH);
  CAN0.sendMsgBuf(req_id, 0, 8, fc_frame);
}

// UDS Servis 0x22 ile Metin Tabanlı DID Oku (Multi-Frame Desteğiyle)
bool readUDSString(uint32_t req_id, uint32_t resp_id, uint16_t did, char* out_buf, uint8_t max_len) {
  uint8_t tx_frame[8] = {
    0x03, 
    0x22, 
    (uint8_t)(did >> 8), 
    (uint8_t)(did & 0xFF), 
    0x55, 0x55, 0x55, 0x55
  };

  // 1. İsteği Gönder
  digitalWrite(PIN_OLED_CS, HIGH);
  if (CAN0.sendMsgBuf(req_id, 0, 8, tx_frame) != CAN_OK) {
    return false;
  }

  // 2. Cevabı Bekle (Zaman Aşımı: 600 ms)
  uint32_t start_time = millis();
  uint8_t raw_data[64];
  uint16_t total_expected = 0;
  uint16_t received_bytes = 0;
  bool is_multi_frame = false;

  while (millis() - start_time < 600) {
    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
      uint32_t rx_id = 0;
      uint8_t len = 0;
      uint8_t buf[8];
      
      digitalWrite(PIN_OLED_CS, HIGH);
      CAN0.readMsgBuf(&rx_id, &len, buf);

      if (rx_id == resp_id) {
        uint8_t pci_type = (buf[0] & 0xF0) >> 4;

        // A) Single Frame (Tek Paket: 0x0...)
        if (pci_type == 0x00) {
          uint8_t payload_len = buf[0] & 0x0F;
          if (buf[1] == 0x62) { // 0x22 Pozitif Cevap
            for (uint8_t i = 4; i <= payload_len && (i - 4) < max_len - 1; i++) {
              out_buf[i - 4] = (char)buf[i];
              out_buf[i - 3] = '\0';
            }
            return true;
          }
        }
        // B) First Frame (Çoklu Paket Başlangıcı: 0x1...)
        else if (pci_type == 0x01) {
          total_expected = (((uint16_t)(buf[0] & 0x0F)) << 8) | buf[1];
          is_multi_frame = true;

          // İlk paketteki verileri kopyala (buf[5] ve sonrası gerçek veri)
          for (uint8_t i = 5; i < 8; i++) {
            if (received_bytes < sizeof(raw_data)) {
              raw_data[received_bytes++] = buf[i];
            }
          }
          // ECU'ya "Kalan paketleri göndermeye devam et" (Flow Control) de
          sendFlowControl(req_id);
        }
        // C) Consecutive Frame (Ardışık Paketler: 0x2...)
        else if (pci_type == 0x02 && is_multi_frame) {
          for (uint8_t i = 1; i < 8; i++) {
            if (received_bytes < sizeof(raw_data) && received_bytes < total_expected) {
              raw_data[received_bytes++] = buf[i];
            }
          }

          // İstenen veriler tamamlandı mı?
          if (received_bytes >= (total_expected - 3)) { // 3 byte (0x62 DID_H DID_L) hariç
            uint8_t out_idx = 0;
            for (uint8_t i = 0; i < received_bytes && out_idx < max_len - 1; i++) {
              // Yazdırılabilir ASCII karakterleri filtrele
              if (raw_data[i] >= 32 && raw_data[i] <= 126) {
                out_buf[out_idx++] = (char)raw_data[i];
              }
            }
            out_buf[out_idx] = '\0';
            return true;
          }
        }
      }
    }
  }
  return false;
}

// Şasi Numarasını Oku (OBD2 Mode 09 PID 0x02)
bool readVIN() {
  uint8_t tx_frame[8] = {0x02, 0x09, 0x02, 0x55, 0x55, 0x55, 0x55, 0x55};
  digitalWrite(PIN_OLED_CS, HIGH);
  CAN0.sendMsgBuf(CAN_ID_ECU_REQ, 0, 8, tx_frame);

  uint32_t start_time = millis();
  uint8_t vin_bytes[20];
  uint8_t count = 0;
  bool mf = false;

  while (millis() - start_time < 600) {
    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
      uint32_t rx_id = 0;
      uint8_t len = 0;
      uint8_t buf[8];
      CAN0.readMsgBuf(&rx_id, &len, buf);

      if (rx_id == CAN_ID_ECU_RESP) {
        if ((buf[0] & 0xF0) == 0x10) { // First Frame
          mf = true;
          sendFlowControl(CAN_ID_ECU_REQ);
        } else if ((buf[0] & 0xF0) == 0x20 && mf) { // Consecutive Frames
          for (uint8_t i = 1; i < 8 && count < 17; i++) {
            if (buf[i] >= '0' && buf[i] <= 'Z') {
              vin_bytes[count++] = buf[i];
            }
          }
          if (count >= 17) {
            vin_bytes[17] = '\0';
            strncpy(vin_str, (char*)vin_bytes, 18);
            return true;
          }
        }
      }
    }
  }
  return false;
}

// ==========================================================================
// EKRAN VE RAPORLAMA FONKSİYONLARI
// ==========================================================================

void drawScanStatus(const char* msg) {
  digitalWrite(PIN_CAN_CS, HIGH);
  u8g2.firstPage();
  do {
    u8g2.drawRFrame(0, 0, 128, 64, 3);
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(16, 20, "VAG ECU SCANNER");
    u8g2.drawHLine(4, 25, 120);
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(8, 42, "Taranıyor:");
    u8g2.drawStr(8, 54, msg);
  } while (u8g2.nextPage());
}

void drawFinalScreen() {
  digitalWrite(PIN_CAN_CS, HIGH);
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(2, 7, "VAG TEHIS RAPORU");
    u8g2.drawHLine(0, 9, 128);

    // Motor Beyni Özeti
    u8g2.drawStr(2, 18, "ECU:");
    u8g2.setCursor(26, 18);
    u8g2.print(ecu_part_str);

    u8g2.setCursor(2, 28);
    u8g2.print(ecu_sys_str);

    // Şanzıman Beyni Özeti
    u8g2.drawHLine(0, 32, 128);
    u8g2.drawStr(2, 42, "TCU:");
    u8g2.setCursor(26, 42);
    u8g2.print(tcu_part_str);

    u8g2.setCursor(2, 52);
    u8g2.print(tcu_sys_str);

    u8g2.drawStr(2, 62, "Detay: Seri Port (115200)");
  } while (u8g2.nextPage());
}

// ==========================================================================
// ARDUINO SETUP & SCANNER DÖNGÜSÜ
// ==========================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000); // USB Seri Portu bekle

  Serial.println(F("\n======================================================="));
  Serial.println(F("    SKODA / VAG MQB ECU & TCU OTOMATIK TANILAMA ARACI   "));
  Serial.println(F("======================================================="));

  initSPISafety();
  u8g2.begin();

  // MCP2515 Başlat (500kbps, 8MHz Kristal)
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN0.setMode(MCP_NORMAL);
    Serial.println(F("[CAN-BUS] MCP2515 Basariyla Baglandi (500 kbps, 8MHz)."));
  } else {
    Serial.println(F("[HATA] MCP2515 Baslatilamadi! Baglantilari kontrol edin."));
    drawScanStatus("MCP2515 HATA!");
    while (1);
  }

  Serial.println(F("[BILGI] Kontagin ACIK konumda oldugundan emin olun..."));
  delay(1000);

  // 1. Şasi Numarasını (VIN) Oku
  drawScanStatus("Sasi No (VIN)...");
  if (readVIN()) {
    Serial.print(F("[VIN] Sasi Numarasi: "));
    Serial.println(vin_str);
  }

  // 2. MOTOR BEYNİ (ECU - 0x7E0) TARAMASI
  drawScanStatus("Motor Beyni (ECU)...");
  Serial.println(F("\n--- [1] MOTOR BEYNI (ECU: 0x7E0) BILGILERI ---"));
  
  if (readUDSString(CAN_ID_ECU_REQ, CAN_ID_ECU_RESP, DID_VAG_PART_NO, ecu_part_str, sizeof(ecu_part_str))) {
    ecu_found = true;
    Serial.print(F(" -> VAG Parca Numarasi : ")); Serial.println(ecu_part_str);
  }
  if (readUDSString(CAN_ID_ECU_REQ, CAN_ID_ECU_RESP, DID_SYSTEM_NAME, ecu_sys_str, sizeof(ecu_sys_str))) {
    Serial.print(F(" -> Sistem / Motor Kodu: ")); Serial.println(ecu_sys_str);
  }
  if (readUDSString(CAN_ID_ECU_REQ, CAN_ID_ECU_RESP, DID_SW_VERSION, ecu_sw_str, sizeof(ecu_sw_str))) {
    Serial.print(F(" -> Yazilim Versiyonu  : ")); Serial.println(ecu_sw_str);
  }
  if (readUDSString(CAN_ID_ECU_REQ, CAN_ID_ECU_RESP, DID_ASAM_ODX_ID, ecu_asam_str, sizeof(ecu_asam_str))) {
    Serial.print(F(" -> ASAM/ODX Dataset ID: ")); Serial.println(ecu_asam_str);
  }

  if (!ecu_found) {
    Serial.println(F(" -> [UYARI] Motor Beyninden (0x7E0) cevap alinamadi!"));
  }

  delay(300);

  // 3. ŞANZIMAN BEYNİ (TCU / DSG - 0x7E1) TARAMASI
  drawScanStatus("Sanziman (TCU/DSG)...");
  Serial.println(F("\n--- [2] SANZIMAN BEYNI (TCU: 0x7E1) BILGILERI ---"));
  
  if (readUDSString(CAN_ID_TCU_REQ, CAN_ID_TCU_RESP, DID_VAG_PART_NO, tcu_part_str, sizeof(tcu_part_str))) {
    tcu_found = true;
    Serial.print(F(" -> VAG Parca Numarasi : ")); Serial.println(tcu_part_str);
  }
  if (readUDSString(CAN_ID_TCU_REQ, CAN_ID_TCU_RESP, DID_SYSTEM_NAME, tcu_sys_str, sizeof(tcu_sys_str))) {
    Serial.print(F(" -> Sanziman Modeli    : ")); Serial.println(tcu_sys_str);
  }
  if (readUDSString(CAN_ID_TCU_REQ, CAN_ID_TCU_RESP, DID_SW_VERSION, tcu_sw_str, sizeof(tcu_sw_str))) {
    Serial.print(F(" -> Yazilim Versiyonu  : ")); Serial.println(tcu_sw_str);
  }

  if (!tcu_found) {
    Serial.println(F(" -> [BILGI] Sanziman beyni bulunamadi (Arac Manuel vites olabilir veya DSG farkli ID'de)."));
  }

  Serial.println(F("\n======================================================="));
  Serial.println(F("               TARAMA ISLEMI TAMAMLANDI               "));
  Serial.println(F("=======================================================\n"));

  // 4. Sonuçları OLED Ekrana Yansıt
  drawFinalScreen();
}

void loop() {
  // Tarama işlemi bitti, sonuçlar ekranda sabit kalır.
  delay(1000);
}