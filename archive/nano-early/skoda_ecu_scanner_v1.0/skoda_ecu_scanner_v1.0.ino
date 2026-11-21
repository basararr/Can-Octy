// ============================================================
//  SKODA MQB — ECU / TCU CAN-Bus TARAMA ARACI
//  Sürüm  : V1.0
//  Amaç   : OBD2 hattındaki tüm ECU/TCU'ları tespit etmek,
//            UDS yanıt adreslerini ve ECU Kimlik bilgilerini
//            (Part Number, SW Version) okumak.
//
//  Çıktı  : Serial Monitor (115200 baud) + OLED ekran
//
//  Donanım: Ana projeyle aynı (MCP2515 + SSD1306 OLED)
//    SCK   -> D13 | MOSI -> D11 | MISO -> D12
//    CAN CS -> D10 | OLED CS -> D9 | DC -> D8 | RES -> D7
//
//  Kullanım:
//    1. Bu kodu yükleyin, aracın kontağını açın.
//    2. Serial Monitörü 115200 baud ile açın.
//    3. Tarama otomatik başlar (~60 saniye sürebilir).
//    4. Çıktıyı not alın -> ana koda uygulayın.
//
//  Tarama Yöntemi:
//    ISO 15765-4 OBD2 adres aralığı: 0x700 – 0x7FF
//    Her adrese UDS 0x10 (DiagnosticSessionControl: Default)
//    ve UDS 0x22 0xF1 0x8C (ECU Serial Number) gönderilir.
//    Yanıt veren adresler tespit edilir ve kimlik bilgisi sorgulanır.
// ============================================================

#include <SPI.h>
#include <mcp_can.h>
#include <U8g2lib.h>

// ─── Pin Tanımları ────────────────────────────────────────────
#define CAN_CS_PIN    10
#define OLED_CS_PIN    9
#define OLED_DC_PIN    8
#define OLED_RES_PIN   7

// ─── Tarama Parametreleri ─────────────────────────────────────
// OBD2 fiziksel istek adresleri: 0x700 – 0x77F (yaygın ECU aralığı)
#define SCAN_ADDR_START   0x700
#define SCAN_ADDR_END     0x77F
// Her ECU için bekleme süresi (ms) — çok kısa olursa yanıtlar kaçar
#define SCAN_TIMEOUT_MS   80
// Yanıt adresi = İstek adresi + 0x08 (ISO 15765-4 kuralı)
#define RESP_OFFSET       0x08

// ─── Bilinen VAG MQB ECU Adresi Haritası ─────────────────────
// Bu tablo yanıt adresine göre ECU adını basar
struct KnownECU {
  uint16_t resp_id;
  const char *name;    // PROGMEM'e taşımak için const char*
};

// En yaygın VAG MQB ECU yanıt adresleri
// (Istek=Yanıt-0x08, örn: ECM ister 0x7E0, yanıtlar 0x7E8)
static const KnownECU known_ecus[] PROGMEM = {
  {0x7E8, "ECM  (Motor Beyni)"},
  {0x7E9, "TCU  (Sanziman/DSG)"},
  {0x7EA, "TCU2 (DSG Alt Modul)"},
  {0x7EB, "HCM  (Hibrit/Elektrik)"},
  {0x7EC, "ABS  (Fren/ESP Modulu)"},
  {0x7ED, "SRS  (Hava Yastigi)"},
  {0x7EE, "BCM  (Karoser Modulu)"},
  {0x7EF, "TPMS (Lastik Basinc)"},
  {0x77E, "HVAC (Klima Modulu)"},
  {0x77D, "IPC  (Kombi Gostergesi)"},
  {0x77C, "EPS  (Elektrik Direksiyon)"},
  {0x77B, "PDC  (Park Sensoru)"},
  {0x77A, "CAM  (Kamera/LDW)"},
  {0x779, "GW   (Gateway Modulu)"},
  {0x778, "ACM  (Ses Sistemi)"},
  {0x76D, "TCM  (Telematik/GSM)"},
  {0x760, "OBD  (Tani Modulu)"},
};
#define KNOWN_ECU_COUNT  (sizeof(known_ecus) / sizeof(known_ecus[0]))

// ─── Nesne Tanımları ──────────────────────────────────────────
MCP_CAN CAN(CAN_CS_PIN);

U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(
  U8G2_R0, OLED_CS_PIN, OLED_DC_PIN, OLED_RES_PIN
);

// ─── Tarama Sonuç Dizisi ──────────────────────────────────────
// Maksimum 20 ECU tutabiliriz (RAM kısıtı)
#define MAX_FOUND  20
struct FoundECU {
  uint16_t req_id;
  uint16_t resp_id;
  uint8_t  ecu_data[6];   // Kimlik verisi (ham)
  uint8_t  data_len;
  bool     id_ok;          // Kimlik başarıyla alındı mı?
};

static FoundECU found_list[MAX_FOUND];
static uint8_t found_count = 0;

// ─── CAN Rx Tamponu ───────────────────────────────────────────
static uint8_t  rx_buf[8];
static uint8_t  rx_len;
static uint32_t rx_id;

// ============================================================
//  YARDIMCI FONKSİYONLAR
// ============================================================

// Bilinen ECU tablosunda isim ara (PROGMEM'den oku)
const char* get_ecu_name(uint16_t resp_id) {
  for (uint8_t i = 0; i < KNOWN_ECU_COUNT; i++) {
    uint16_t tid = pgm_read_word(&known_ecus[i].resp_id);
    if (tid == resp_id) {
      return (const char*)pgm_read_ptr(&known_ecus[i].name);
    }
  }
  return nullptr;
}

// CAN mesajı gönder (8 byte, ISO 15765-4 padding)
bool can_send(uint16_t id, uint8_t b0, uint8_t b1,
              uint8_t b2, uint8_t b3) {
  uint8_t msg[8] = {b0, b1, b2, b3, 0x00, 0x00, 0x00, 0x00};
  return (CAN.sendMsgBuf(id, 0, 8, msg) == CAN_OK);
}

// Belirtilen yanıt ID'sinden mesaj bekle
bool can_wait(uint16_t expected_resp_id, uint32_t timeout_ms) {
  uint32_t t = millis();
  while ((millis() - t) < timeout_ms) {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      if (CAN.readMsgBuf(&rx_id, &rx_len, rx_buf) == CAN_OK) {
        if ((uint16_t)rx_id == expected_resp_id) {
          return true;
        }
      }
    }
  }
  return false;
}

// Herhangi bir CAN mesajı bekle (broadcast tarama için)
bool can_wait_any(uint32_t timeout_ms) {
  uint32_t t = millis();
  while ((millis() - t) < timeout_ms) {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      if (CAN.readMsgBuf(&rx_id, &rx_len, rx_buf) == CAN_OK) {
        // UDS pozitif yanıt mı? (0x50 = 0x40|0x10, 0x62 = 0x40|0x22)
        if (rx_len >= 2 &&
            (rx_buf[1] == 0x50 || rx_buf[1] == 0x62 ||
             rx_buf[1] == 0x41)) {
          return true;
        }
      }
    }
  }
  return false;
}

// Negatif yanıt mı kontrol et
bool is_negative_resp(void) {
  return (rx_len >= 1 && rx_buf[1] == 0x7F);
}

// ============================================================
//  OLED EKRAN FONKSİYONLARI
// ============================================================

void oled_splash(void) {
  u8g2.firstPage();
  do {
    u8g2.drawRFrame(2, 2, 124, 60, 4);
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(12, 18);
    u8g2.print(F("ECU TARAMA ARACI"));
    u8g2.setCursor(22, 30);
    u8g2.print(F("MQB CAN-Bus"));
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.setCursor(8, 44);
    u8g2.print(F("Serial Monitor: 115200"));
    u8g2.setCursor(14, 55);
    u8g2.print(F("Kontak acin -> bas."));
  } while (u8g2.nextPage());
}

void oled_scanning(uint16_t current_addr, uint8_t progress_pct) {
  char buf[12];

  u8g2.firstPage();
  do {
    // Başlık
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawBox(0, 0, 128, 10);
    u8g2.setDrawColor(0);
    u8g2.setCursor(20, 8);
    u8g2.print(F("TARAMA YAPILIYOR..."));
    u8g2.setDrawColor(1);

    // Aktif adres
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(0, 24);
    u8g2.print(F("Adres: 0x"));
    // Hex yazdır
    if (current_addr < 0x1000) u8g2.print('0');
    if (current_addr < 0x100)  u8g2.print('0');
    u8g2.print(current_addr, HEX);

    // Bulunan ECU sayısı
    u8g2.setCursor(0, 36);
    u8g2.print(F("Bulunan: "));
    u8g2.print(found_count);
    u8g2.print(F(" ECU"));

    // Progress bar
    u8g2.drawFrame(0, 44, 128, 10);
    uint8_t bar_w = (uint8_t)(progress_pct * 126UL / 100);
    if (bar_w > 0) u8g2.drawBox(1, 45, bar_w, 8);

    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.setCursor(50, 63);
    u8g2.print(progress_pct);
    u8g2.print('%');

  } while (u8g2.nextPage());
}

void oled_result(uint8_t idx) {
  // Tek bir ECU sonucunu göster (döngüyle çağrılır)
  if (idx >= found_count) return;
  FoundECU &e = found_list[idx];

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawBox(0, 0, 128, 10);
    u8g2.setDrawColor(0);
    u8g2.setCursor(2, 8);
    u8g2.print(F("SONUC "));
    u8g2.print(idx + 1);
    u8g2.print('/');
    u8g2.print(found_count);
    u8g2.setDrawColor(1);

    // Yanıt adresi
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(0, 22);
    u8g2.print(F("RESP: 0x"));
    u8g2.print(e.resp_id, HEX);
    u8g2.print(F("  REQ: 0x"));
    u8g2.print(e.req_id, HEX);

    // Bilinen ECU adı
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.setCursor(0, 34);
    const char *name = get_ecu_name(e.resp_id);
    if (name) {
      u8g2.print(name);
    } else {
      u8g2.print(F("Bilinmeyen ECU"));
    }

    // Ham kimlik verisi (hex)
    if (e.id_ok && e.data_len > 0) {
      u8g2.setCursor(0, 46);
      u8g2.print(F("ID: "));
      for (uint8_t b = 0; b < e.data_len && b < 6; b++) {
        if (e.ecu_data[b] < 0x10) u8g2.print('0');
        u8g2.print(e.ecu_data[b], HEX);
        u8g2.print(' ');
      }
    }

    u8g2.drawHLine(0, 55, 128);
    u8g2.setCursor(2, 63);
    u8g2.print(F("3sn sonra devam..."));

  } while (u8g2.nextPage());
}

void oled_done(void) {
  u8g2.firstPage();
  do {
    u8g2.drawRFrame(2, 2, 124, 60, 4);
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(12, 18);
    u8g2.print(F("TARAMA TAMAM!"));
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.setCursor(4, 30);
    u8g2.print(found_count);
    u8g2.print(F(" ECU/TCU bulundu."));
    u8g2.setCursor(4, 42);
    u8g2.print(F("Serial Monitor'u"));
    u8g2.setCursor(4, 52);
    u8g2.print(F("kontrol edin."));
  } while (u8g2.nextPage());
}

// ============================================================
//  UDS SORGULAMA FONKSİYONLARI
// ============================================================

// UDS 0x10 01: Default Diagnostic Session — ECU'nun ayakta
// olup olmadığını test etmek için hafif sorgu
bool uds_ping(uint16_t req_id) {
  uint16_t resp_id = req_id + RESP_OFFSET;
  // [02][10][01] = DiagnosticSessionControl, defaultSession
  if (!can_send(req_id, 0x02, 0x10, 0x01, 0x00)) return false;
  return can_wait(resp_id, SCAN_TIMEOUT_MS);
}

// UDS 0x22 F1 8C: ECU Serial Number (Seri No)
bool uds_read_serial(uint16_t req_id) {
  uint16_t resp_id = req_id + RESP_OFFSET;
  // [03][22][F1][8C] = ReadDataByIdentifier, ECU Serial Number DID
  if (!can_send(req_id, 0x03, 0x22, 0xF1, 0x8C)) return false;
  return can_wait(resp_id, SCAN_TIMEOUT_MS);
}

// UDS 0x22 F1 87: Supplier ECU Software Part Number
bool uds_read_part_number(uint16_t req_id) {
  uint16_t resp_id = req_id + RESP_OFFSET;
  if (!can_send(req_id, 0x03, 0x22, 0xF1, 0x87)) return false;
  return can_wait(resp_id, SCAN_TIMEOUT_MS);
}

// UDS 0x22 F1 89: ECU Software Version Number
bool uds_read_sw_version(uint16_t req_id) {
  uint16_t resp_id = req_id + RESP_OFFSET;
  if (!can_send(req_id, 0x03, 0x22, 0xF1, 0x89)) return false;
  return can_wait(resp_id, SCAN_TIMEOUT_MS);
}

// ============================================================
//  SERİAL ÇIKTI FONKSİYONLARI
// ============================================================

void print_separator(void) {
  Serial.println(F("──────────────────────────────────────────────────"));
}

void print_hex_byte(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
  Serial.print(' ');
}

void print_hex_buf(uint8_t *buf, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) print_hex_byte(buf[i]);
}

void print_ascii_buf(uint8_t *buf, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    char c = (char)buf[i];
    Serial.print((c >= 0x20 && c <= 0x7E) ? c : '.');
  }
}

// Bir ECU'nun tüm kimlik bilgilerini Serial'a döktür
void serial_print_ecu(uint8_t idx) {
  FoundECU &e = found_list[idx];
  print_separator();
  Serial.print(F("[ECU #"));
  Serial.print(idx + 1);
  Serial.println(']');

  Serial.print(F("  Istek Adresi  (REQ): 0x"));
  Serial.println(e.req_id, HEX);
  Serial.print(F("  Yanit Adresi (RESP): 0x"));
  Serial.println(e.resp_id, HEX);

  const char *name = get_ecu_name(e.resp_id);
  Serial.print(F("  Bilinen Isim        : "));
  if (name) {
    Serial.println(name);
  } else {
    Serial.println(F("Bilinmeyen (yeni modul olabilir)"));
  }

  if (e.id_ok && e.data_len > 0) {
    Serial.print(F("  Kimlik Ham Verisi   : "));
    print_hex_buf(e.ecu_data, e.data_len);
    Serial.println();
    Serial.print(F("  ASCII Yorumu        : "));
    print_ascii_buf(e.ecu_data, e.data_len);
    Serial.println();
  }

  // Ana koda uygulama notu
  Serial.print(F("  >> ANA KOD ICIN: ECM_RESP_ID = 0x"));
  Serial.print(e.resp_id, HEX);
  Serial.print(F(" | ECM_PHYS_ID = 0x"));
  Serial.println(e.req_id, HEX);
}

// Tarama bitti özet
void serial_print_summary(void) {
  print_separator();
  Serial.println(F("TARAMA OZETI — ANA KODA KOPYALAYIN:"));
  print_separator();
  Serial.println(F("// skoda_mqb_display.ino icin adres sabitleri:"));

  bool ecm_found = false, tcu_found = false;

  for (uint8_t i = 0; i < found_count; i++) {
    FoundECU &e = found_list[i];
    const char *name = get_ecu_name(e.resp_id);

    Serial.print(F("#define "));

    if (e.resp_id == 0x7E8) {
      Serial.print(F("ECM_PHYS_ID   0x"));
      Serial.print(e.req_id, HEX);
      Serial.print(F("  // ECM_RESP_ID = 0x"));
      Serial.println(e.resp_id, HEX);
      ecm_found = true;
    } else if (e.resp_id == 0x7E9 || e.resp_id == 0x7EA) {
      Serial.print(F("TCU_PHYS_ID   0x"));
      Serial.print(e.req_id, HEX);
      Serial.print(F("  // TCU_RESP_ID = 0x"));
      Serial.println(e.resp_id, HEX);
      tcu_found = true;
    } else {
      Serial.print(F("ADDR_"));
      // resp_id hex
      Serial.print(e.resp_id, HEX);
      Serial.print(F("_PHYS  0x"));
      Serial.print(e.req_id, HEX);
      if (name) {
        Serial.print(F("  // "));
        Serial.println(name);
      } else {
        Serial.println(F("  // Bilinmeyen ECU"));
      }
    }
  }

  if (!ecm_found) {
    Serial.println(F("// UYARI: ECM (0x7E8) bulunamadi!"));
    Serial.println(F("//   -> Kontak acik mi? CAN-H/L dogru mu?"));
  }
  if (!tcu_found) {
    Serial.println(F("// UYARI: TCU (0x7E9/0x7EA) bulunamadi!"));
    Serial.println(F("//   -> DSG sanziman bu aracta olmayabilir."));
  }

  print_separator();
  Serial.println();
  Serial.println(F("Detayli bilgi icin yukaridaki sonuclara bakin."));
}

// ============================================================
//  ANA TARAMA FONKSİYONU
// ============================================================

void run_scan(void) {
  uint16_t total = SCAN_ADDR_END - SCAN_ADDR_START + 1;
  uint16_t scanned = 0;

  Serial.println(F("\n============================================"));
  Serial.println(F("  SKODA MQB ECU/TCU CAN-Bus TARAMA ARACI  "));
  Serial.println(F("============================================"));
  Serial.print(F("Tarama araligi: 0x"));
  Serial.print(SCAN_ADDR_START, HEX);
  Serial.print(F(" - 0x"));
  Serial.println(SCAN_ADDR_END, HEX);
  Serial.print(F("Zaman asimi   : "));
  Serial.print(SCAN_TIMEOUT_MS);
  Serial.println(F(" ms / adres"));
  print_separator();

  for (uint16_t addr = SCAN_ADDR_START;
       addr <= SCAN_ADDR_END && found_count < MAX_FOUND;
       addr++) {

    scanned++;
    uint8_t pct = (uint8_t)(scanned * 100UL / total);

    // Her 8 adrese bir OLED güncelle (çok sık çizim = yavaşlama)
    if ((scanned % 8) == 0) {
      oled_scanning(addr, pct);
    }

    // ── Adım 1: UDS 0x10 01 PING ─────────────────────────────
    if (!uds_ping(addr)) continue;  // Yanıt yok -> sonraki adres

    // Yanıt geldi! ECU bulundu.
    uint16_t resp_id = addr + RESP_OFFSET;

    Serial.print(F(">>> ECU BULUNDU! REQ=0x"));
    Serial.print(addr, HEX);
    Serial.print(F(" | RESP=0x"));
    Serial.print(resp_id, HEX);
    const char *name = get_ecu_name(resp_id);
    if (name) {
      Serial.print(F(" | "));
      Serial.print(name);
    }
    Serial.println();

    // Listeye ekle
    FoundECU &e = found_list[found_count];
    e.req_id   = addr;
    e.resp_id  = resp_id;
    e.id_ok    = false;
    e.data_len = 0;

    // ── Adım 2: ECU Part Number / Seri No oku ────────────────
    // Önce F1 87 (Part Number) dene
    if (uds_read_part_number(addr)) {
      if (!is_negative_resp() && rx_buf[1] == 0x62) {
        // Veri Byte4'ten başlar (PCI + SID + DID_H + DID_L = 4 byte)
        uint8_t di = 4;
        uint8_t dlen = (rx_buf[0] & 0x0F);  // PCI tek çerçeve uzunluğu
        if (dlen > 4) dlen -= 3;             // SID+DID hesap
        if (dlen > 6) dlen = 6;             // tampon sınırı

        for (uint8_t b = 0; b < dlen && (di + b) < rx_len; b++) {
          e.ecu_data[b] = rx_buf[di + b];
          e.data_len++;
        }
        e.id_ok = true;

        Serial.print(F("  Part No (F187): "));
        print_hex_buf(e.ecu_data, e.data_len);
        Serial.print(F("  \""));
        print_ascii_buf(e.ecu_data, e.data_len);
        Serial.println('"');
      }
    }

    // F187 yoksa F18C (Serial Number) dene
    if (!e.id_ok) {
      if (uds_read_serial(addr)) {
        if (!is_negative_resp() && rx_buf[1] == 0x62) {
          uint8_t di   = 4;
          uint8_t dlen = (rx_buf[0] & 0x0F);
          if (dlen > 4) dlen -= 3;
          if (dlen > 6) dlen = 6;

          for (uint8_t b = 0; b < dlen && (di + b) < rx_len; b++) {
            e.ecu_data[b] = rx_buf[di + b];
            e.data_len++;
          }
          e.id_ok = true;

          Serial.print(F("  Serial No (F18C): "));
          print_hex_buf(e.ecu_data, e.data_len);
          Serial.print(F("  \""));
          print_ascii_buf(e.ecu_data, e.data_len);
          Serial.println('"');
        }
      }
    }

    // SW Version (F189)
    if (uds_read_sw_version(addr)) {
      if (!is_negative_resp() && rx_buf[1] == 0x62) {
        Serial.print(F("  SW Version (F189): "));
        print_hex_buf(rx_buf + 4, min(rx_len - 4, 4));
        Serial.print(F("  \""));
        print_ascii_buf(rx_buf + 4, min(rx_len - 4, 4));
        Serial.println('"');
      }
    }

    found_count++;

    // OLED'de anlık sonuç göster
    oled_result(found_count - 1);
    delay(800);  // Kullanıcı okuyabilsin
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  // SPI CS pinleri yüksek (pasif) konuma
  pinMode(CAN_CS_PIN, OUTPUT);
  digitalWrite(CAN_CS_PIN, HIGH);
  pinMode(OLED_CS_PIN, OUTPUT);
  digitalWrite(OLED_CS_PIN, HIGH);

  // OLED başlat
  u8g2.begin();
  u8g2.setContrast(200);

  oled_splash();
  delay(2000);

  Serial.println(F("MCP2515 baslatiliyor..."));

  // CAN başlat (500kbps, 8MHz kristal)
  uint8_t retry = 0;
  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    retry++;
    Serial.print(F("CAN baslatma hatasi, deneme: "));
    Serial.println(retry);
    if (retry >= 5) {
      Serial.println(F("CAN BASLATILAMADI! Baglantilari kontrol et."));
      u8g2.firstPage();
      do {
        u8g2.setFont(u8g2_font_6x10_tr);
        u8g2.setCursor(5, 20);
        u8g2.print(F("CAN HATASI!"));
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.setCursor(5, 35);
        u8g2.print(F("MCP2515 yanit vermiyor"));
        u8g2.setCursor(5, 47);
        u8g2.print(F("SPI + 8MHz kristal"));
        u8g2.setCursor(5, 58);
        u8g2.print(F("kontrolu yapin."));
      } while (u8g2.nextPage());
      while (1) delay(1000);
    }
    delay(300);
  }

  // Tüm CAN mesajlarını al (filtre yok) — tarama için şart
  CAN.setMode(MCP_NORMAL);

  Serial.println(F("CAN OK. Tarama basliyor..."));

  // ── Ana Taramayı Çalıştır ─────────────────────────────────
  run_scan();

  // ── Sonuçları Serial'a Döktür ─────────────────────────────
  Serial.println();
  Serial.println(F("=== TUM ECU DETAYLARI ==="));
  for (uint8_t i = 0; i < found_count; i++) {
    serial_print_ecu(i);
  }

  serial_print_summary();

  oled_done();
}

// ============================================================
//  LOOP — Tarama bitti, sadece özet ekrana bas
// ============================================================
void loop() {
  // Sonuçları 3'er saniyede döngüyle göster
  for (uint8_t i = 0; i < found_count; i++) {
    oled_result(i);
    delay(3000);
  }

  // Hiç ECU bulunamadıysa uyarı göster
  if (found_count == 0) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x10_tr);
      u8g2.setCursor(5, 20);
      u8g2.print(F("ECU BULUNAMADI!"));
      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.setCursor(5, 34);
      u8g2.print(F("-> Kontak acik mi?"));
      u8g2.setCursor(5, 44);
      u8g2.print(F("-> CAN-H/L bağlı mı?"));
      u8g2.setCursor(5, 54);
      u8g2.print(F("-> 500kbps dogru mu?"));
    } while (u8g2.nextPage());
    delay(3000);
  }
}

// ============================================================
//  EOF - skoda_ecu_scanner.ino
//
//  Serial Monitor Çıktısı Örneği:
//  ────────────────────────────────────────────────────
//  >>> ECU BULUNDU! REQ=0x7E0 | RESP=0x7E8 | ECM (Motor Beyni)
//    Part No (F187): 04 L9 06 B  -> "04L906B..."
//    SW Version (F189): 01 02 03 04
//  >>> ECU BULUNDU! REQ=0x7E1 | RESP=0x7E9 | TCU (Sanziman/DSG)
//    Serial No (F18C): 31 32 33 34 35 36
//
//  TARAMA OZETI:
//  #define ECM_PHYS_ID   0x7E0  // ECM_RESP_ID = 0x7E8
//  #define TCU_PHYS_ID   0x7E1  // TCU_RESP_ID = 0x7E9
// ============================================================
