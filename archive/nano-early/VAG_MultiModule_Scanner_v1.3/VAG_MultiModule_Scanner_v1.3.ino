/* ==========================================================================
 * Proje: VAG MQB 5-Modül Teşhis + DPF Kül (Ash) & Kurum (Soot) Sağlık Analizi (v1.3)
 * Donanım: 1. Arduino Nano + MCP2515 (8MHz) + SSD1306 SPI OLED
 * Eklenen Teşhisler:
 *  - DPF Kül Kütlesi (Oil Ash Mass - DID 0x202E) [Sabit Filtre Ömrü]
 *  - DPF Hesaplanmış Kurum (Soot Calc - DID 0x2029)
 *  - DPF Ölçülmüş Kurum (Soot Meas - DID 0x202A)
 *  - Son Rejenerasyondan Beri Mesafe (DID 0x2035) [KM]
 * ========================================================================== */

#include <SPI.h>
#include <mcp_can.h>
#include <U8g2lib.h>

#define PIN_CAN_CS    10
#define PIN_OLED_CS    9
#define PIN_OLED_DC    8
#define PIN_OLED_RES   7

U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, PIN_OLED_CS, PIN_OLED_DC, PIN_OLED_RES);
MCP_CAN CAN0(PIN_CAN_CS);

// UDS DID Kodları
#define DID_VAG_PART_NO     0xF187
#define DID_SW_VERSION      0xF189
#define DID_SYSTEM_NAME     0xF197

#define DID_DPF_ASH_MASS    0x202E  // Kül Kütlesi (0.01 g veya 0.1 g)
#define DID_DPF_SOOT_CALC   0x2029  // Hesaplanmış Kurum (g)
#define DID_DPF_SOOT_MEAS   0x202A  // Ölçülmüş Kurum (g)
#define DID_DPF_LAST_REGEN  0x2035  // Son Rejenerasyon KM

struct ModuleTarget {
  uint32_t req_id;
  uint32_t resp_id;
  const char* name;
};

const ModuleTarget targets[5] = {
  {0x7E0, 0x7E8, "1. MOTOR (ECU)"},
  {0x7E1, 0x7E9, "2. SANZIMAN (TCU)"},
  {0x7E2, 0x7EA, "3. FREN (ABS/ESP)"},
  {0x714, 0x77E, "4. GOSTERGE (KOMBI)"},
  {0x710, 0x77A, "5. AG GECIDI (GATEWAY)"}
};

char temp_part[20];
char temp_sys[30];
char temp_sw[10];

// DPF Sağlık Değişkenleri
float dpf_ash_g      = 0.0;
float dpf_soot_calc  = 0.0;
float dpf_soot_meas  = 0.0;
uint16_t dpf_last_km = 0;

void initSPISafety() {
  pinMode(PIN_CAN_CS, OUTPUT);
  pinMode(PIN_OLED_CS, OUTPUT);
  digitalWrite(PIN_CAN_CS, HIGH);
  digitalWrite(PIN_OLED_CS, HIGH);
}

void sendFlowControl(uint32_t req_id) {
  uint8_t fc[8] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  digitalWrite(PIN_OLED_CS, HIGH);
  CAN0.sendMsgBuf(req_id, 0, 8, fc);
}

// UDS Metin Okuma (Parça No vb.)
bool readUDSString(uint32_t req_id, uint32_t resp_id, uint16_t did, char* out_buf, uint8_t max_len) {
  uint8_t tx[8] = {0x03, 0x22, (uint8_t)(did >> 8), (uint8_t)(did & 0xFF), 0x55, 0x55, 0x55, 0x55};
  digitalWrite(PIN_OLED_CS, HIGH);
  if (CAN0.sendMsgBuf(req_id, 0, 8, tx) != CAN_OK) return false;

  uint32_t start_time = millis();
  uint8_t raw[64]; uint16_t total = 0; uint16_t rcv = 0; bool mf = false;

  while (millis() - start_time < 400) {
    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
      uint32_t rx_id; uint8_t len; uint8_t buf[8];
      digitalWrite(PIN_OLED_CS, HIGH);
      CAN0.readMsgBuf(&rx_id, &len, buf);

      if (rx_id == resp_id) {
        uint8_t pci = (buf[0] & 0xF0) >> 4;
        if (pci == 0x00 && buf[1] == 0x62) {
          uint8_t plen = buf[0] & 0x0F;
          for (uint8_t i = 4; i <= plen && (i - 4) < max_len - 1; i++) {
            out_buf[i - 4] = (char)buf[i]; out_buf[i - 3] = '\0';
          }
          return true;
        } else if (pci == 0x01) {
          total = (((uint16_t)(buf[0] & 0x0F)) << 8) | buf[1];
          mf = true;
          for (uint8_t i = 5; i < 8; i++) if (rcv < sizeof(raw)) raw[rcv++] = buf[i];
          sendFlowControl(req_id);
        } else if (pci == 0x02 && mf) {
          for (uint8_t i = 1; i < 8; i++) if (rcv < sizeof(raw)) raw[rcv++] = buf[i];
          if (rcv >= (total - 3)) {
            uint8_t o_idx = 0;
            for (uint8_t i = 0; i < rcv && o_idx < max_len - 1; i++) {
              if (raw[i] >= 32 && raw[i] <= 126) out_buf[o_idx++] = (char)raw[i];
            }
            out_buf[o_idx] = '\0';
            return true;
          }
        }
      }
    }
  }
  return false;
}

// UDS Sayısal Değer Okuma (Sensör ve DPF Verileri)
uint16_t readUDSValue(uint32_t req_id, uint32_t resp_id, uint16_t did) {
  uint8_t tx[8] = {0x03, 0x22, (uint8_t)(did >> 8), (uint8_t)(did & 0xFF), 0x55, 0x55, 0x55, 0x55};
  digitalWrite(PIN_OLED_CS, HIGH);
  CAN0.sendMsgBuf(req_id, 0, 8, tx);

  uint32_t start_time = millis();
  while (millis() - start_time < 300) {
    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
      uint32_t rx_id; uint8_t len; uint8_t buf[8];
      digitalWrite(PIN_OLED_CS, HIGH);
      CAN0.readMsgBuf(&rx_id, &len, buf);

      if (rx_id == resp_id && buf[1] == 0x62) {
        if ((((uint16_t)buf[2] << 8) | buf[3]) == did) {
          return ((uint16_t)buf[4] << 8) | buf[5];
        }
      }
    }
  }
  return 0;
}

void drawScanStatus(const char* name) {
  digitalWrite(PIN_CAN_CS, HIGH);
  u8g2.firstPage();
  do {
    u8g2.drawRFrame(0, 0, 128, 64, 3);
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(12, 18, "VAG TEHIS ARACI");
    u8g2.drawHLine(4, 22, 120);
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(6, 38, "Taranan Modul:");
    u8g2.drawStr(6, 52, name);
  } while (u8g2.nextPage());
}

// DPF Sağlık Özeti Ekranı
void drawDPFHealthScreen() {
  digitalWrite(PIN_CAN_CS, HIGH);
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(2, 10, "DPF SAGLIK RAPORU");
    u8g2.drawHLine(0, 12, 128);

    // Kül Kütlesi (Oil Ash)
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(2, 23, "KUL (ASH)  :");
    u8g2.setCursor(68, 23);
    u8g2.print(dpf_ash_g, 1);
    u8g2.drawStr(98, 23, "g /80g");

    // Hesaplanmış Kurum
    u8g2.drawStr(2, 34, "HESAP KURUM:");
    u8g2.setCursor(68, 34);
    u8g2.print(dpf_soot_calc, 1);
    u8g2.drawStr(98, 34, "g");

    // Ölçülmüş Kurum
    u8g2.drawStr(2, 45, "OLCUM KURUM:");
    u8g2.setCursor(68, 45);
    u8g2.print(dpf_soot_meas, 1);
    u8g2.drawStr(98, 45, "g");

    // Son Rejenerasyon Mesafesi
    u8g2.drawStr(2, 56, "SON REGEN  :");
    u8g2.setCursor(68, 56);
    u8g2.print(dpf_last_km);
    u8g2.drawStr(98, 56, "km once");
    u8g2.drawHLine(0, 59, 128);

  } while (u8g2.nextPage());
}

void setup() {
  Serial.begin(115200);
  initSPISafety();
  u8g2.begin();

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN0.setMode(MCP_NORMAL);
    Serial.println(F("\n======================================================="));
    Serial.println(F("    SKODA OCTAVIA 1.6 TDI MODUL & DPF SAGLIK TEHISI    "));
    Serial.println(F("=======================================================\n"));
  }

  // 1. BEŞ ANA MODÜLÜ TARA
  for (uint8_t i = 0; i < 5; i++) {
    drawScanStatus(targets[i].name);
    Serial.println(targets[i].name);

    strcpy(temp_part, "Bulunamadi");
    strcpy(temp_sys, "---");
    strcpy(temp_sw, "---");

    bool ok = readUDSString(targets[i].req_id, targets[i].resp_id, DID_VAG_PART_NO, temp_part, sizeof(temp_part));
    readUDSString(targets[i].req_id, targets[i].resp_id, DID_SYSTEM_NAME, temp_sys, sizeof(temp_sys));
    readUDSString(targets[i].req_id, targets[i].resp_id, DID_SW_VERSION, temp_sw, sizeof(temp_sw));

    if (ok) {
      Serial.print(F("  -> VAG Parca No: ")); Serial.println(temp_part);
      Serial.print(F("  -> Sistem Tanimi: ")); Serial.println(temp_sys);
      Serial.print(F("  -> SW Versiyon  : ")); Serial.println(temp_sw);
    } else {
      Serial.println(F("  -> Modul Bulunamadi / Baglanti Yok"));
    }
    Serial.println(F("-------------------------------------------------------"));
    delay(150);
  }

  // 2. MOTOR BEYNİNDEN DPF SAĞLIK BİLGİLERİNİ ÇEK
  drawScanStatus("DPF Analiz Ediliyor...");
  
  dpf_ash_g     = readUDSValue(0x7E0, 0x7E8, DID_DPF_ASH_MASS) * 0.01;
  dpf_soot_calc = readUDSValue(0x7E0, 0x7E8, DID_DPF_SOOT_CALC) * 0.01;
  dpf_soot_meas = readUDSValue(0x7E0, 0x7E8, DID_DPF_SOOT_MEAS) * 0.01;
  dpf_last_km   = readUDSValue(0x7E0, 0x7E8, DID_DPF_LAST_REGEN);

  // Seri Porta Detaylı DPF Raporu Yaz
  Serial.println(F("\n======================================================="));
  Serial.println(F("             DPF DETAYLI SAGLIK ANALIZI                "));
  Serial.println(F("======================================================="));
  Serial.print(F(" -> DPF Kul Kutlesi (Oil Ash Mass)  : ")); Serial.print(dpf_ash_g, 2); Serial.println(F(" g (Limit: 80.00 g)"));
  Serial.print(F(" -> Filtre Kapatilma / Omur Orani   : %")); Serial.println((dpf_ash_g / 80.0) * 100.0, 1);
  Serial.print(F(" -> Hesaplanmis Kurum (Soot Calc)   : ")); Serial.print(dpf_soot_calc, 2); Serial.println(F(" g"));
  Serial.print(F(" -> Olculmus Kurum (Soot Measured)  : ")); Serial.print(dpf_soot_meas, 2); Serial.println(F(" g"));
  Serial.print(F(" -> Son Rejenerasyondan Beri Mesafe : ")); Serial.print(dpf_last_km); Serial.println(F(" km"));
  Serial.println(F("=======================================================\n"));

  // 3. Sonuçları OLED Ekrana Yansıt
  drawDPFHealthScreen();
}

void loop() {
  delay(1000);
}