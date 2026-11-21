/* ==========================================================================
 * Proje: Skoda Octavia 1.6 TDI (EDC17C64 & DQ200) CAN-Bus Simülatörü
 * Donanım: 2. Arduino Nano + MCP2515 (8MHz Kristal, CS: D10)
 * Amaç: Gerçek araç olmadan masaüstünde MFD Göstergeyi ve Scanner'ı test etmek.
 * ========================================================================== */

#include <SPI.h>
#include <mcp_can.h>

#define PIN_CAN_CS 10
MCP_CAN CAN0(PIN_CAN_CS);

// CAN ID Tanımları
#define ECU_REQ_ID      0x7E0
#define ECU_RESP_ID     0x7E8
#define TCM_REQ_ID      0x7E1
#define TCM_RESP_ID     0x7E9

// Simüle Edilen Dinamik Araç Değerleri
float   sim_turbo_bar   = 0.0;    // 0.0 - 1.55 bar arası salınır
float   sim_dpf_soot    = 14.2;   // DPF Kurum Miktarı (Gram)
int16_t sim_oil_temp    = 90;     // Yağ Sıcaklığı (°C)
int16_t sim_dsg_temp    = 82;     // DSG Sıcaklığı (°C)
int16_t sim_egt_temp    = 320;    // Egzoz Gazı Sıcaklığı (°C)

// Simülasyon Zamanlayıcıları
uint32_t last_sim_tick  = 0;
float    sim_angle      = 0.0;    // Gazlama animasyonu için sinüs açısı

// --------------------------------------------------------------------------
// MULTI-FRAME ISO-TP YANIT MOTORU (Scanner Testi İçin)
// --------------------------------------------------------------------------
void sendISOTPString(uint32_t resp_id, uint32_t req_id, uint16_t did, const char* str) {
  uint8_t str_len = strlen(str);
  uint16_t total_payload = 3 + str_len; // 0x62 + DID_H + DID_L + Metin

  if (total_payload <= 7) {
    // Single Frame (Tek Paket)
    uint8_t sf[8] = {
      (uint8_t)total_payload, 
      0x62, 
      (uint8_t)(did >> 8), 
      (uint8_t)(did & 0xFF), 
      0x55, 0x55, 0x55, 0x55
    };
    for (uint8_t i = 0; i < str_len; i++) sf[4 + i] = str[i];
    CAN0.sendMsgBuf(resp_id, 0, 8, sf);
  } 
  else {
    // 1. First Frame (İlk Paket)
    uint8_t ff[8] = {
      (uint8_t)(0x10 | ((total_payload >> 8) & 0x0F)),
      (uint8_t)(total_payload & 0xFF),
      0x62,
      (uint8_t)(did >> 8),
      (uint8_t)(did & 0xFF),
      (uint8_t)str[0],
      (uint8_t)str[1],
      (uint8_t)str[2]
    };
    CAN0.sendMsgBuf(resp_id, 0, 8, ff);

    // 2. Flow Control Bekle
    uint32_t start_w = millis();
    bool fc_received = false;
    while (millis() - start_w < 300) {
      if (CAN0.checkReceive() == CAN_MSGAVAIL) {
        uint32_t rx_id; uint8_t len; uint8_t buf[8];
        CAN0.readMsgBuf(&rx_id, &len, buf);
        if (rx_id == req_id && (buf[0] & 0xF0) == 0x30) {
          fc_received = true;
          break;
        }
      }
    }

    if (!fc_received) return;

    // 3. Consecutive Frames (Ardışık Paketleri Gönder)
    uint8_t bytes_sent = 3;
    uint8_t seq = 1;
    while (bytes_sent < str_len) {
      uint8_t cf[8] = {(uint8_t)(0x20 | (seq & 0x0F)), 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
      for (uint8_t i = 1; i < 8 && bytes_sent < str_len; i++) {
        cf[i] = str[bytes_sent++];
      }
      seq = (seq + 1) & 0x0F;
      CAN0.sendMsgBuf(resp_id, 0, 8, cf);
      delay(5);
    }
  }
}

// Şasi No (VIN) Simülasyonu (Mode 09 PID 0x02)
void sendSimulatedVIN() {
  const char* vin = "TMBJJ7NE0D0192834";
  uint8_t ff[8] = {0x10, 0x14, 0x49, 0x02, 0x01, (uint8_t)vin[0], (uint8_t)vin[1], (uint8_t)vin[2]};
  CAN0.sendMsgBuf(ECU_RESP_ID, 0, 8, ff);

  delay(20);
  uint8_t cf1[8] = {0x21, (uint8_t)vin[3], (uint8_t)vin[4], (uint8_t)vin[5], (uint8_t)vin[6], (uint8_t)vin[7], (uint8_t)vin[8], (uint8_t)vin[9]};
  CAN0.sendMsgBuf(ECU_RESP_ID, 0, 8, cf1);

  delay(10);
  uint8_t cf2[8] = {0x22, (uint8_t)vin[10], (uint8_t)vin[11], (uint8_t)vin[12], (uint8_t)vin[13], (uint8_t)vin[14], (uint8_t)vin[15], (uint8_t)vin[16]};
  CAN0.sendMsgBuf(ECU_RESP_ID, 0, 8, cf2);
}

// --------------------------------------------------------------------------
// GERÇEKÇİ SÜRÜŞ ANİMASYONU MOTORU
// --------------------------------------------------------------------------
void updateDynamicCarPhysics() {
  uint32_t now = millis();
  if (now - last_sim_tick >= 50) { // Her 50ms'de bir fizik güncelle
    last_sim_tick = now;

    // Turbo Basıncı: 0.0 - 1.55 bar arası gazlama eğrisi simülasyonu
    sim_angle += 0.04;
    if (sim_angle > 6.28) sim_angle = 0.0;
    float raw_sin = sin(sim_angle);
    
    if (raw_sin > 0) {
      sim_turbo_bar = raw_sin * 1.52; // Gazlama
      sim_egt_temp = 320 + (int16_t)(sim_turbo_bar * 160.0); // Turbo arttıkça egzoz ısınır
    } else {
      sim_turbo_bar = 0.05; // Rölanti / Süzülme
      sim_egt_temp = 270;
    }

    // DPF Kurumunu yavaşça artır (Her ~20 sn'de bir 0.1g artar)
    static uint32_t dpf_timer = 0;
    if (now - dpf_timer > 20000) {
      dpf_timer = now;
      sim_dpf_soot += 0.1;
      if (sim_dpf_soot > 26.0) sim_dpf_soot = 12.0; // Rejenerasyon sıfırlama testi
    }
  }
}

// --------------------------------------------------------------------------
// CAN-BUS TALEP İŞLEYİCİ
// --------------------------------------------------------------------------
void handleIncomingRequests() {
  if (CAN0.checkReceive() == CAN_MSGAVAIL) {
    uint32_t rx_id;
    uint8_t len;
    uint8_t buf[8];
    CAN0.readMsgBuf(&rx_id, &len, buf);

    // 1. MOTOR BEYNİNE (0x7E0) GELEN İSTEKLER
    if (rx_id == ECU_REQ_ID) {
      
      // A) UDS 0x22 Sorguları
      if (buf[1] == 0x22) {
        uint16_t req_did = ((uint16_t)buf[2] << 8) | buf[3];

        // Turbo Basıncı (DID 0x115B)
        if (req_did == 0x115B) {
          uint16_t mbar = 1013 + (uint16_t)(sim_turbo_bar * 1000.0);
          uint8_t resp[8] = {0x05, 0x62, 0x11, 0x5B, (uint8_t)(mbar >> 8), (uint8_t)(mbar & 0xFF), 0x55, 0x55};
          CAN0.sendMsgBuf(ECU_RESP_ID, 0, 8, resp);
        }
        // DPF Kurum Miktarı (DID 0x2029)
        else if (req_did == 0x2029) {
          uint16_t raw_soot = (uint16_t)(sim_dpf_soot * 100.0); // 0.01g katsayısı
          uint8_t resp[8] = {0x05, 0x62, 0x20, 0x29, (uint8_t)(raw_soot >> 8), (uint8_t)(raw_soot & 0xFF), 0x55, 0x55};
          CAN0.sendMsgBuf(ECU_RESP_ID, 0, 8, resp);
        }
        // Motor Yağ Sıcaklığı (DID 0x2024)
        else if (req_did == 0x2024) {
          uint8_t resp[8] = {0x04, 0x62, 0x20, 0x24, (uint8_t)(sim_oil_temp + 40), 0x55, 0x55, 0x55};
          CAN0.sendMsgBuf(ECU_RESP_ID, 0, 8, resp);
        }
        // Egzoz Gazı Sıcaklığı - EGT (DID 0x2045)
        else if (req_did == 0x2045) {
          uint16_t raw_egt = (uint16_t)((sim_egt_temp + 40) * 10);
          uint8_t resp[8] = {0x05, 0x62, 0x20, 0x45, (uint8_t)(raw_egt >> 8), (uint8_t)(raw_egt & 0xFF), 0x55, 0x55};
          CAN0.sendMsgBuf(ECU_RESP_ID, 0, 8, resp);
        }
        // --- SCANNER KODU İÇİN TANIMLAMALAR ---
        else if (req_did == 0xF187) {
          sendISOTPString(ECU_RESP_ID, ECU_REQ_ID, 0xF187, "04L906021M "); // VAG Parça No
        }
        else if (req_did == 0xF197) {
          sendISOTPString(ECU_RESP_ID, ECU_REQ_ID, 0xF197, "R4 1.6l TDI EDC17C64"); // Sistem Adı
        }
        else if (req_did == 0xF189) {
          sendISOTPString(ECU_RESP_ID, ECU_REQ_ID, 0xF189, "9970"); // Yazılım Versiyonu
        }
        else if (req_did == 0xF19E) {
          sendISOTPString(ECU_RESP_ID, ECU_REQ_ID, 0xF19E, "EV_ECM16TDI02104L906021M"); // ASAM
        }
      }
      // B) OBD2 Mode 09 (VIN İstekleri)
      else if (buf[1] == 0x09 && buf[2] == 0x02) {
        sendSimulatedVIN();
      }
    }
    // 2. ŞANZIMAN BEYNİNE (TCU: 0x7E1) GELEN İSTEKLER
    else if (rx_id == TCM_REQ_ID) {
      if (buf[1] == 0x22) {
        uint16_t req_did = ((uint16_t)buf[2] << 8) | buf[3];

        // DSG Sıcaklığı (DID 0x1000)
        if (req_did == 0x1000) {
          uint8_t resp[8] = {0x04, 0x62, 0x10, 0x00, (uint8_t)(sim_dsg_temp + 40), 0x55, 0x55, 0x55};
          CAN0.sendMsgBuf(TCM_RESP_ID, 0, 8, resp);
        }
        // Scanner Tanımları (DSG)
        else if (req_did == 0xF187) {
          sendISOTPString(TCM_RESP_ID, TCM_REQ_ID, 0xF187, "0CW300047C");
        }
        else if (req_did == 0xF197) {
          sendISOTPString(TCM_RESP_ID, TCM_REQ_ID, 0xF197, "GSG DQ200-MQB");
        }
        else if (req_did == 0xF189) {
          sendISOTPString(TCM_RESP_ID, TCM_REQ_ID, 0xF189, "6502");
        }
      }
    }
  }
}

// --------------------------------------------------------------------------
// SETUP & LOOP
// --------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("[SIMULATOR] Skoda Octavia 1.6 TDI ECU/TCU Emulator Baslatiliyor..."));

  pinMode(PIN_CAN_CS, OUTPUT);
  digitalWrite(PIN_CAN_CS, HIGH);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN0.setMode(MCP_NORMAL);
    Serial.println(F("[SIMULATOR] MCP2515 Hazir (500 kbps, 8MHz). Istekler Dinleniyor!"));
  } else {
    Serial.println(F("[HATA] Simülatör MCP2515 Baslatilamadi! Kablolari kontrol edin."));
    while (1);
  }
}

void loop() {
  // 1. Dinamik Fizik Motorunu Çalıştır (Gazlama, Isınma vb.)
  updateDynamicCarPhysics();

  // 2. 1. Nano'dan Gelen CAN İsteklerine Anında Cevap Ver
  handleIncomingRequests();
}