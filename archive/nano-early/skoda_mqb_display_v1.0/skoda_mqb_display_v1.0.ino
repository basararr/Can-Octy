// ============================================================
//  SKODA MQB OBD2 DISPLAY - V1.0
//  Araç  : 2013 Skoda Octavia 1.6 TDI (CLHA / EDC17C64)
//  Platform: MQB  |  CAN-Bus: ISO 15765-4 (11-bit, 500kbps)
//  Protokol: UDS Service 0x22 (ReadDataByIdentifier)
//
//  Donanım:
//    - Arduino Nano (ATmega328P, 5V)
//    - MCP2515 + TJA1050 (8 MHz kristal)
//    - 0.96" SPI OLED 128x64 SSD1306 (7-pin)
//    - LM2596 Buck Converter (OBD2 Pin16 -> 5V)
//
//  SPI Pin Haritası:
//    SCK  -> D13 (Ortak)
//    MOSI -> D11 (Ortak)
//    MISO -> D12
//    CAN CS  -> D10
//    OLED CS -> D9
//    OLED DC -> D8
//    OLED RES -> D7
//    CAN INT  -> D2 (şu an kullanılmıyor, ileride kesme için)
//
//  Kütüphaneler:
//    - mcp_can       (coryjfowler/MCP_CAN_lib)
//    - U8g2          (olikraus/u8g2)
//
//  UDS DID'leri (VAG MQB EDC17C64):
//    0xF407 -> DPF Kurum Kütlesi (g/L veya mg, ECU değeri)
//    0x0295 -> Turbo Manifold Basıncı (ham ADC -> kPa)
//    0xF4AE -> DSG / Şanzıman Yağ Sıcaklığı
//    0xF40C -> Motor Soğutma Suyu Sıcaklığı (kontrol amaçlı)
//
//  NOT: UDS 0x22, okuma servisidir; Security Access gerektirmez.
// ============================================================

#include <SPI.h>
#include <mcp_can.h>
#include <U8g2lib.h>

// ─── Derleme Zamanı Sabitleri ────────────────────────────────
#define CAN_CS_PIN    10
#define CAN_INT_PIN    2   // şu an kullanılmıyor
#define OLED_CS_PIN    9
#define OLED_DC_PIN    8
#define OLED_RES_PIN   7

// CAN Tanımlayıcıları (11-bit, ISO 15765-4 OBD2 standardı)
#define TESTER_ID      0x7DF   // Genel OBD2 yayın adresi (tüm ECU'lar)
#define ECM_RESP_ID    0x7E8   // ECM (EDC17C64) yanıt adresi
// İsteğe bağlı: belirli ECU için 0x7E0 (fiziksel adresleme)
#define ECM_PHYS_ID    0x7E0   // ECM fiziksel istek adresi (daha güvenilir)

// UDS Servis Kodları
#define UDS_READ_DATA  0x22    // ReadDataByIdentifier
#define UDS_RESP_OK    0x62    // Pozitif yanıt (0x40 | 0x22)
#define UDS_NEG_RESP   0x7F    // Negatif yanıt

// VAG MQB EDC17C64 Data Identifier'ları
// DPF Kurum Kütlesi: VAG-spesifik PID
#define DID_DPF_SOOT_HI  0xF4
#define DID_DPF_SOOT_LO  0x07

// Turbo / Emme Manifold Mutlak Basıncı
#define DID_TURBO_HI     0x02
#define DID_TURBO_LO     0x95

// DSG Şanzıman Yağ Sıcaklığı (DQ200/DQ250)
#define DID_DSG_TEMP_HI  0xF4
#define DID_DSG_TEMP_LO  0xAE

// Soğutma Suyu Sıcaklığı (referans / kontrol)
#define DID_COOLANT_HI   0xF4
#define DID_COOLANT_LO   0x0C

// Güncelleme periyotları (ms) - RAM tasarrufu için sabit
#define UPDATE_INTERVAL_FAST  500    // Turbo + DSG (500ms)
#define UPDATE_INTERVAL_SLOW  2000   // DPF (her 2 saniye)
#define CAN_TIMEOUT_MS        150    // CAN yanıt zaman aşımı

// ─── U8g2 OLED (SSD1306, Donanım SPI, Page Buffer Modu) ──────
// Page buffer modu: sadece 128 byte RAM kullanır!
U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(
  U8G2_R0,        // Döndürme yok
  OLED_CS_PIN,    // CS
  OLED_DC_PIN,    // DC
  OLED_RES_PIN    // RST
);

// ─── MCP2515 CAN Modülü ───────────────────────────────────────
MCP_CAN CAN(CAN_CS_PIN);

// ─── Veri Yapıları (struct ile RAM düzeni optimize edildi) ────
struct VehicleData {
  int16_t  dpf_soot_raw;    // DPF ham değer (ECU birimi: 0.01 g/L)
  uint16_t turbo_raw;       // Turbo ham ADC değeri (0-65535 -> kPa)
  int16_t  dsg_temp_raw;    // DSG sıcaklık ham değer (0.1°C çözünürlük)
  int16_t  coolant_raw;     // Soğutma suyu sıcaklığı ham değer
  bool     can_ok;          // CAN bağlantısı sağlandı mı?
  bool     ecu_resp;        // ECU en az bir kez yanıtladı mı?
  uint8_t  screen_page;     // Aktif ekran sayfası (0=Ana, 1=DPF, 2=DSG)
};

static VehicleData vd;  // global tek örnek, sıfır başlatmalı

// ─── Zamanlama Takibi ─────────────────────────────────────────
static uint32_t last_fast_update = 0;
static uint32_t last_slow_update = 0;
static uint32_t last_page_switch = 0;
#define PAGE_SWITCH_INTERVAL 4000  // Her 4 saniyede sayfa geçişi

// ─── CAN Mesaj Tamponu (Stack üzerinde - heap yok) ───────────
static uint8_t can_rx_buf[8];
static uint8_t can_rx_len = 0;
static uint32_t can_rx_id = 0;

// ============================================================
//  YARDIMCI FONKSIYONLAR: Değer Dönüşümleri
// ============================================================

// DPF kurum kütlesi: ECU ham değeri -> g/L (x0.01 çarpanı)
// Örnek: raw=450 -> 4.50 g/L
// print() ile float kullanmamak için tam ve ondalık ayrı hesaplanır
void dpf_to_str(int16_t raw, char *buf, uint8_t buflen) {
  if (raw < 0) raw = 0;
  int whole   = raw / 100;
  int decimal = raw % 100;
  // snprintf Flash-safe değil; itoa+manuel birleştirme kullanıyoruz
  uint8_t i = 0;
  itoa(whole, buf, 10);
  i = strlen(buf);
  if (i + 4 < buflen) {
    buf[i++] = '.';
    if (decimal < 10) buf[i++] = '0';
    itoa(decimal, buf + i, 10);
    i = strlen(buf);
    // Birim ekle
    if (i + 4 < buflen) {
      buf[i++] = ' ';
      buf[i++] = 'g';
      buf[i++] = '/';
      buf[i++] = 'L';
      buf[i]   = '\0';
    }
  }
}

// Turbo manifold basıncı: VAG raw -> kPa -> bar (görüntüleme için)
// VAG EDC17: 1 birim = 0.01 kPa (ya da ECU kalibrasyonuna göre farklı)
// Araca göre ayarlanmalıdır; referans olarak 0.01 kPa/bit kullanıldı.
// Bar = kPa / 100; boost = Bar - 1.013 (atmosfer)
void turbo_to_str(uint16_t raw, char *buf, uint8_t buflen) {
  // kPa cinsinden (ham * 0.01)
  uint16_t kpa      = raw / 100;           // tam kPa
  uint8_t  kpa_dec  = (raw % 100) / 10;   // 1 ondalık basamak
  uint8_t  i = 0;
  itoa(kpa, buf, 10);
  i = strlen(buf);
  if (i + 5 < buflen) {
    buf[i++] = '.';
    buf[i++] = '0' + kpa_dec;
    buf[i++] = ' ';
    buf[i++] = 'k';
    buf[i++] = 'P';
    buf[i++] = 'a';
    buf[i]   = '\0';
  }
}

// DSG/şanzıman sıcaklığı: raw * 0.1 -> °C
void dsg_temp_to_str(int16_t raw, char *buf, uint8_t buflen) {
  int whole   = raw / 10;
  int decimal = abs(raw % 10);
  uint8_t i = 0;
  itoa(whole, buf, 10);
  i = strlen(buf);
  if (i + 4 < buflen) {
    buf[i++] = '.';
    buf[i++] = '0' + decimal;
    buf[i++] = 0xB0; // ASCII derece sembolü (bazı fontlarda)
    buf[i++] = 'C';
    buf[i]   = '\0';
  }
}

// Soğutma suyu: raw - 40 offset (SAE J1979 standart offseti)
// VAG EDC17 için raw değer doğrudan °C olabilir, offset=0 veya 40
void coolant_to_str(int16_t raw, char *buf, uint8_t buflen) {
  int16_t celsius = raw - 40;  // Standart J1979 ofseti; ECU'ya göre ayarla
  uint8_t i = 0;
  itoa(celsius, buf, 10);
  i = strlen(buf);
  if (i + 3 < buflen) {
    buf[i++] = 0xB0;
    buf[i++] = 'C';
    buf[i]   = '\0';
  }
}

// ============================================================
//  CAN / UDS FONKSIYONLARI
// ============================================================

// UDS ReadDataByIdentifier (0x22) isteği gönder
// ISO 15765-4: 8 byte'lık tek çerçeve (Single Frame)
// Byte0: PCI uzunluk (0x03 = 3 byte veri)
// Byte1: Servis ID (0x22)
// Byte2-3: DID (High, Low)
// Byte4-7: 0x00 padding (ISO 15765-4 zorunluluğu)
bool uds_request(uint8_t did_hi, uint8_t did_lo) {
  uint8_t msg[8] = {
    0x03,         // Tek çerçeve, 3 byte veri
    UDS_READ_DATA,
    did_hi,
    did_lo,
    0x00, 0x00, 0x00, 0x00  // ISO 15765-4 padding
  };

  // ECM_PHYS_ID ile fiziksel adresleme (daha güvenilir, broadcast yerine)
  uint8_t ret = CAN.sendMsgBuf(ECM_PHYS_ID, 0, 8, msg);
  return (ret == CAN_OK);
}

// CAN veri yolu üzerinden yanıt bekle ve tamponla
// Dönüş: true = ECM yanıtladı, false = zaman aşımı
bool uds_wait_response(void) {
  uint32_t start = millis();
  while ((millis() - start) < CAN_TIMEOUT_MS) {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      uint8_t ret = CAN.readMsgBuf(&can_rx_id, &can_rx_len, can_rx_buf);
      if (ret == CAN_OK && can_rx_id == ECM_RESP_ID) {
        return true;
      }
    }
  }
  return false;  // zaman aşımı
}

// Yanıt çerçevesini çözümle ve veri baytlarını döndür
// ISO 15765-4 Tek Çerçeve yapısı:
//   Byte0: PCI (alt nibble = veri uzunluğu)
//   Byte1: Yanıt SID (0x62 = UDS OK)
//   Byte2-3: Gelen DID (echo)
//   Byte4...: Veri
// Dönüş: veri başlangıç indeksi (genellikle 4), 0 = hata
uint8_t parse_uds_response(uint8_t did_hi, uint8_t did_lo) {
  if (can_rx_len < 4) return 0;

  // Negatif yanıt kontrolü
  if (can_rx_buf[1] == UDS_NEG_RESP) return 0;

  // Pozitif yanıt ve doğru DID kontrolü
  if (can_rx_buf[1] == UDS_RESP_OK &&
      can_rx_buf[2] == did_hi &&
      can_rx_buf[3] == did_lo) {
    return 4;  // Veri Byte4'ten başlıyor
  }
  return 0;
}

// ─── Bireysel veri sorgulama fonksiyonları ───────────────────

void query_dpf(void) {
  if (!uds_request(DID_DPF_SOOT_HI, DID_DPF_SOOT_LO)) return;
  if (!uds_wait_response()) return;

  uint8_t di = parse_uds_response(DID_DPF_SOOT_HI, DID_DPF_SOOT_LO);
  if (di == 0) return;

  vd.ecu_resp = true;
  // Genellikle 2 byte (big-endian): Byte4 MSB, Byte5 LSB
  if (can_rx_len >= di + 2) {
    vd.dpf_soot_raw = ((int16_t)can_rx_buf[di] << 8) | can_rx_buf[di + 1];
  } else if (can_rx_len >= di + 1) {
    vd.dpf_soot_raw = (int16_t)can_rx_buf[di];
  }
}

void query_turbo(void) {
  if (!uds_request(DID_TURBO_HI, DID_TURBO_LO)) return;
  if (!uds_wait_response()) return;

  uint8_t di = parse_uds_response(DID_TURBO_HI, DID_TURBO_LO);
  if (di == 0) return;

  vd.ecu_resp = true;
  if (can_rx_len >= di + 2) {
    vd.turbo_raw = ((uint16_t)can_rx_buf[di] << 8) | can_rx_buf[di + 1];
  } else if (can_rx_len >= di + 1) {
    vd.turbo_raw = (uint16_t)can_rx_buf[di];
  }
}

void query_dsg_temp(void) {
  if (!uds_request(DID_DSG_TEMP_HI, DID_DSG_TEMP_LO)) return;
  if (!uds_wait_response()) return;

  uint8_t di = parse_uds_response(DID_DSG_TEMP_HI, DID_DSG_TEMP_LO);
  if (di == 0) return;

  vd.ecu_resp = true;
  if (can_rx_len >= di + 2) {
    vd.dsg_temp_raw = ((int16_t)can_rx_buf[di] << 8) | can_rx_buf[di + 1];
  } else if (can_rx_len >= di + 1) {
    vd.dsg_temp_raw = (int16_t)can_rx_buf[di];
  }
}

void query_coolant(void) {
  if (!uds_request(DID_COOLANT_HI, DID_COOLANT_LO)) return;
  if (!uds_wait_response()) return;

  uint8_t di = parse_uds_response(DID_COOLANT_HI, DID_COOLANT_LO);
  if (di == 0) return;

  vd.ecu_resp = true;
  if (can_rx_len >= di + 1) {
    vd.coolant_raw = (int16_t)can_rx_buf[di];
  }
}

// ============================================================
//  EKRAN FONKSIYONLARI (U8g2 Page Buffer Modu)
// ============================================================

// Açılış / Karşılama Ekranı (sadece setup() içinde bir kez çalışır)
void draw_splash_screen(void) {
  u8g2.firstPage();
  do {
    // Dış çerçeve (U8g2: drawRFrame(x, y, w, h, r))
    u8g2.drawRFrame(2, 2, 124, 60, 5);
    // İç çerçeve (dekoratif çift çizgi)
    u8g2.drawRFrame(5, 5, 118, 54, 3);

    // Logo / Başlık
    u8g2.setFont(u8g2_font_logisoso16_tr);  // Büyük font
    // Metni yatay ortala: 128px - metin genişliği / 2
    // "SKODA" = ~5 karakter * 11px ~ 55px genişlik -> (128-55)/2 ~ 36
    u8g2.setCursor(33, 28);
    u8g2.print(F("SKODA"));

    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(18, 40);
    u8g2.print(F("CAN MONITOR V1.0"));

    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.setCursor(22, 52);
    u8g2.print(F("1.6 TDI | EDC17C64"));

  } while (u8g2.nextPage());
}

// CAN başlatma hatası ekranı
void draw_can_error(void) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(10, 12);
    u8g2.print(F("! CAN BAGLANTI !"));

    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.setCursor(5, 26);
    u8g2.print(F("MCP2515 baslatma hatasi"));
    u8g2.setCursor(5, 36);
    u8g2.print(F("Kontrol et:"));
    u8g2.setCursor(5, 46);
    u8g2.print(F(" - SPI baglantilar"));
    u8g2.setCursor(5, 56);
    u8g2.print(F(" - 8MHz kristal"));
  } while (u8g2.nextPage());
}

// ECU yanıt bekleniyor ekranı
void draw_waiting_ecu(void) {
  static uint8_t dots = 0;
  dots = (dots + 1) % 4;

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(8, 20);
    u8g2.print(F("ECU bekleniyor"));

    // Animasyonlu nokta
    u8g2.setCursor(8, 32);
    for (uint8_t i = 0; i < dots; i++) u8g2.print('.');

    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.setCursor(8, 48);
    u8g2.print(F("Kontak acik mi?"));
    u8g2.setCursor(8, 58);
    u8g2.print(F("OBD2 Pin6/14 kontrol"));
  } while (u8g2.nextPage());
}

// Ana veri ekranı (Sayfa 0): Turbo + Soğutma Suyu
void draw_main_screen(void) {
  char buf[16];

  u8g2.firstPage();
  do {
    // ── Başlık Çubuğu ──────────────────────────────────────
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawBox(0, 0, 128, 10);         // Siyah arka plan
    u8g2.setDrawColor(0);                // Beyaz metin
    u8g2.setCursor(2, 8);
    u8g2.print(F("CANLI VERI  [1/3]"));
    u8g2.setDrawColor(1);                // Siyaha geri dön

    // ── Turbo Basıncı ───────────────────────────────────────
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(0, 22);
    u8g2.print(F("TURBO:"));

    turbo_to_str(vd.turbo_raw, buf, sizeof(buf));
    u8g2.setFont(u8g2_font_logisoso16_tr);
    u8g2.setCursor(0, 42);
    u8g2.print(buf);

    // ── Soğutma Suyu Sıcaklığı (sağ üst, küçük) ───────────
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.setCursor(80, 22);
    u8g2.print(F("SOGUTMA"));

    coolant_to_str(vd.coolant_raw, buf, sizeof(buf));
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(80, 34);
    u8g2.print(buf);

    // ── Alt Bilgi Çubuğu ────────────────────────────────────
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawHLine(0, 54, 128);
    u8g2.setCursor(2, 63);
    u8g2.print(F("SKODA OCTAVIA  CLHA"));

  } while (u8g2.nextPage());
}

// DPF Ekranı (Sayfa 1)
void draw_dpf_screen(void) {
  char buf[16];

  u8g2.firstPage();
  do {
    // ── Başlık ─────────────────────────────────────────────
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawBox(0, 0, 128, 10);
    u8g2.setDrawColor(0);
    u8g2.setCursor(2, 8);
    u8g2.print(F("DPF DURUM   [2/3]"));
    u8g2.setDrawColor(1);

    // ── DPF Kurum Kütlesi ───────────────────────────────────
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(0, 22);
    u8g2.print(F("KURUM KUTLESI:"));

    dpf_to_str(vd.dpf_soot_raw, buf, sizeof(buf));
    u8g2.setFont(u8g2_font_logisoso16_tr);
    u8g2.setCursor(0, 42);
    u8g2.print(buf);

    // ── Durum Göstergesi ────────────────────────────────────
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.setCursor(0, 54);

    // Referans eşikleri: <5.0 g/L TEMIZ, 5-7 g/L ORTA, >7 g/L TEMIZLE
    float dpf_val = vd.dpf_soot_raw / 100.0;
    if (dpf_val < 5.0) {
      u8g2.print(F("DURUM: TEMIZ  [OK]"));
    } else if (dpf_val < 7.0) {
      u8g2.print(F("DURUM: ORTA   [!]"));
    } else {
      u8g2.print(F("DURUM: TEMIZLE [!!]"));
    }

    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawHLine(0, 56, 128);
    u8g2.setCursor(2, 64);
    u8g2.print(F("DID: F407"));

  } while (u8g2.nextPage());
}

// DSG Şanzıman Ekranı (Sayfa 2)
void draw_dsg_screen(void) {
  char buf[16];

  u8g2.firstPage();
  do {
    // ── Başlık ─────────────────────────────────────────────
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawBox(0, 0, 128, 10);
    u8g2.setDrawColor(0);
    u8g2.setCursor(2, 8);
    u8g2.print(F("DSG SICAKLIK [3/3]"));
    u8g2.setDrawColor(1);

    // ── DSG Sıcaklığı ───────────────────────────────────────
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(0, 22);
    u8g2.print(F("DSG / SANZIMAN:"));

    dsg_temp_to_str(vd.dsg_temp_raw, buf, sizeof(buf));
    u8g2.setFont(u8g2_font_logisoso16_tr);
    u8g2.setCursor(0, 42);
    u8g2.print(buf);

    // ── Sıcaklık Durum Çubuğu ──────────────────────────────
    // 0-80°C: Soguk, 80-100°C: Normal, >100°C: Sicak
    u8g2.setFont(u8g2_font_5x7_tr);
    int16_t dsg_c = vd.dsg_temp_raw / 10;
    u8g2.setCursor(0, 54);
    if (dsg_c < 80) {
      u8g2.print(F("DURUM: SOGUK (Isin.)"));
    } else if (dsg_c < 100) {
      u8g2.print(F("DURUM: NORMAL  [OK]"));
    } else {
      u8g2.print(F("DURUM: ASIRI SICAK!"));
    }

    u8g2.drawHLine(0, 56, 128);
    u8g2.setCursor(2, 64);
    u8g2.print(F("DID: F4AE"));

  } while (u8g2.nextPage());
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  // SPI CS pinlerini önce HIGH'a çek (çakışma önleme)
  pinMode(CAN_CS_PIN, OUTPUT);
  digitalWrite(CAN_CS_PIN, HIGH);
  pinMode(OLED_CS_PIN, OUTPUT);
  digitalWrite(OLED_CS_PIN, HIGH);
  pinMode(CAN_INT_PIN, INPUT);  // gelecek için hazır

  // OLED başlat
  u8g2.begin();
  u8g2.setContrast(200);  // Parlaklık (0-255)

  // Açılış ekranını göster
  draw_splash_screen();
  delay(2500);  // 2.5 saniye açılış bekle

  // ── MCP2515 CAN Modülü Başlatma ──────────────────────────
  // CAN_500KBPS: ISO 15765-4 OBD2 standart hızı
  // MCP_8MHz: Modüldeki 8MHz kristale göre
  uint8_t can_init_retry = 0;
  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    can_init_retry++;
    if (can_init_retry >= 3) {
      // 3 denemede başarısız -> hata ekranı, sonsuz döngü
      vd.can_ok = false;
      draw_can_error();
      while (1) { delay(1000); }
    }
    delay(200);
  }

  // Normal mod (dinleme değil, aktif CAN katılımcısı)
  CAN.setMode(MCP_NORMAL);

  vd.can_ok  = true;
  vd.ecu_resp = false;

  // Sıfırlama tamamlandı, zamanlayıcıları başlat
  last_fast_update = millis();
  last_slow_update = millis();
  last_page_switch  = millis();
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  uint32_t now = millis();

  // ── Hızlı Sorgular (Turbo + DSG + Soğutma Suyu, 500ms) ──
  if ((now - last_fast_update) >= UPDATE_INTERVAL_FAST) {
    last_fast_update = now;
    query_turbo();
    query_coolant();
    // DSG sorgusunu sadece DSG ekranı görüntüleniyorken yap
    // (CAN trafiğini azalt, pil/ECU yükü kısıt)
    if (vd.screen_page == 2) {
      query_dsg_temp();
    }
  }

  // ── Yavaş Sorgular (DPF, 2000ms) ─────────────────────────
  if ((now - last_slow_update) >= UPDATE_INTERVAL_SLOW) {
    last_slow_update = now;
    query_dpf();
    // DSG'yi de burada sorgula (ekran sayfasından bağımsız)
    query_dsg_temp();
  }

  // ── Otomatik Sayfa Geçişi ─────────────────────────────────
  if ((now - last_page_switch) >= PAGE_SWITCH_INTERVAL) {
    last_page_switch = now;
    vd.screen_page = (vd.screen_page + 1) % 3;
  }

  // ── Ekranı Güncelle ───────────────────────────────────────
  if (!vd.ecu_resp) {
    // Henüz ECU yanıtı yok -> bekleme animasyonu
    draw_waiting_ecu();
    delay(300);
    return;
  }

  // ECU yanıtladı -> aktif sayfayı çiz
  switch (vd.screen_page) {
    case 0: draw_main_screen(); break;
    case 1: draw_dpf_screen();  break;
    case 2: draw_dsg_screen();  break;
    default: draw_main_screen(); break;
  }

  // Nano'ya nefes aldır - watchdog reset için minimal bekleme
  delay(20);
}

// ============================================================
//  EOF - skoda_mqb_display.ino
//  Kütüphane gereksinimleri (Arduino IDE Library Manager):
//    - "mcp_can" by coryjfowler (v1.5.0+)
//    - "U8g2" by olikraus (v2.35+)
//
//  Önemli Notlar:
//  1. DID değerleri (F407, 0295, F4AE) araca/ECU yazılım
//     versiyonuna göre farklılık gösterebilir.
//     VCDS / OBD11 ile doğrulayın.
//  2. Turbo basınç dönüşüm faktörü (raw->kPa) ECU ölçeklendirme
//     tablosuna göre ayarlanmalıdır.
//  3. DSG şanzıman (DQ200/DQ250) ayrı bir ECU ID üzerinde
//     yanıt verebilir. TCU yanıt ID'si: 0x7E9 veya 0x7EA
//     (araçta VCDS ile tespit edin).
//  4. Security Access (0x27) KULLANILMIYOR - sadece okuma.
// ============================================================
