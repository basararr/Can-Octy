// =============================================================================
// VAG MQB CAN-BUS MODULE SCANNER (GATEWAY DISCOVERY & IDENTIFICATION)
// Version: v1.1.0 | Platform: VAG MQB (Skoda / VW / Seat / Audi)
// Hardware: Arduino Nano (ATmega328P) + MCP2515 (8MHz/500kbps) + SSD1306 OLED
// =============================================================================

#include <SPI.h>
#include <mcp2515.h>
#include <U8g2lib.h>
#include <stdlib.h>

#define SKETCH_VERSION "v1.1"

// -----------------------------------------------------------------------------
// PIN DEFINITIONS & HARDWARE CONSTANTS
// -----------------------------------------------------------------------------
#define PIN_CAN_CS 10
#define PIN_CAN_INT 2
#define PIN_OLED_CS 9
#define PIN_OLED_DC 8
#define PIN_OLED_RES 7

// -----------------------------------------------------------------------------
// GLOBAL OBJECTS
// -----------------------------------------------------------------------------
// SSD1306 OLED in Page Buffer Mode (minimum SRAM usage)
U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, PIN_OLED_CS, PIN_OLED_DC, PIN_OLED_RES);

// MCP2515 CAN Controller
MCP2515 mcp2515(PIN_CAN_CS);

// -----------------------------------------------------------------------------
// VAG MQB CAN MODULE DEFINITIONS
// -----------------------------------------------------------------------------
struct ModuleEntry {
  uint8_t  vagAddr;     // VCDS hex address
  uint16_t canTx;       // CAN TX ID (We send to this)
  char     label[9];    // Short name
};

// RX ID Calculation Rule
// if canTx >= 0x7E0 then RX = canTx + 8
// else RX = canTx + 0x6A
uint16_t getRxId(uint16_t txId) {
    if (txId >= 0x7E0) return txId + 8;
    return txId + 0x6A;
}

// PROGMEM Table to save SRAM (44 VAG MQB Modules)
static const ModuleEntry MODULE_TABLE[] PROGMEM = {
  // Powertrain & Drivetrain
  {0x01, 0x7E0, "Engine"  }, // Motor ECU (01)
  {0x02, 0x7E1, "Trans"   }, // Şanzıman TCU (02)
  {0x03, 0x713, "ABS/ESP" }, // Fren & Denge Kontrol (03)
  {0x14, 0x724, "DCC-Damp"}, // Adaptif Amortisör / DCC (14)
  {0x22, 0x722, "4Motion" }, // 4 Çeker Haldex (22)
  {0x32, 0x732, "DiffLock"}, // Elektronik Ön Kilit VAQ (32)

  // Driver Assistance & Safety
  {0x13, 0x711, "ACC-Rad" }, // Otomatik Mesafe / ACC Radar (13)
  {0x15, 0x745, "Airbag"  }, // Hava Yastığı / Güvenlik (15)
  {0x3C, 0x74C, "LaneChgL"}, // Kör Nokta Asistanı Sol (3C)
  {0xCF, 0x7DF, "LaneChgR"}, // Kör Nokta Asistanı Sağ (CF)
  {0x6C, 0x786, "RearCam" }, // Geri Görüş Kamerası (6C)
  {0xA5, 0x787, "FrontCam"}, // Ön Cam Şerit/Tabela Kamera (A5)
  {0x10, 0x744, "ParkAid2"}, // Park Direksiyon Asistanı (10)
  {0x76, 0x76A, "ParkDist"}, // Park Mesafe Sensörleri PDC (76)

  // Body & Comfort & Security
  {0x05, 0x705, "KESSY"   }, // Giriş & Çalıştırma Yetkisi (05)
  {0x08, 0x715, "HVAC"    }, // Otomatik Klima (08)
  {0x09, 0x765, "CentElec"}, // Merkezi Elektrik / BCM (09)
  {0x16, 0x716, "SteerCol"}, // Direksiyon Kolon Modülü (16)
  {0x17, 0x714, "Cluster" }, // Gösterge Paneli / Kombi (17)
  {0x18, 0x718, "AuxHeat" }, // Bağımsız Yakıtlı Isıtıcı (18)
  {0x19, 0x710, "Gateway" }, // CAN Ağ Geçidi Gateway (19)
  {0x2B, 0x763, "SteerLok"}, // Elektronik Direksiyon Kilidi (2B)
  {0x36, 0x736, "SeatDrv" }, // Hafızalı Sürücü Koltuğu (36)
  {0x06, 0x706, "SeatPass"}, // Hafızalı Yolcu Koltuğu (06)
  {0x42, 0x742, "DrvDoor" }, // Sürücü Kapı Modülü (42)
  {0x52, 0x754, "PassDoor"}, // Yolcu Kapı Modülü (52)
  {0xBB, 0x77B, "RearDrvD"}, // Sol Arka Kapı (BB)
  {0xBC, 0x77C, "RearPasD"}, // Sağ Arka Kapı (BC)
  {0x44, 0x712, "PwrSteer"}, // Elektrikli Direksiyon Kutusu (44)
  {0x46, 0x746, "Comfort" }, // Merkezi Konfor Modülü (46)
  {0x4B, 0x74B, "MultiFnc"}, // Çok Fonksiyonlu Modül (4B)
  {0x65, 0x769, "TirePres"}, // Lastik Basınç TPMS (65)
  {0x69, 0x769, "Trailer" }, // Orijinal Römork Modülü (69)
  {0x6D, 0x76D, "Tailgate"}, // Elektrikli Bagaj Kapağı (6D)
  {0xCA, 0x77A, "Sunroof" }, // Açılır Cam Tavan (CA)

  // Lighting
  {0x55, 0x757, "Headlamp"}, // Far Yükseklik / AFS (55)
  {0xD6, 0x796, "LightL"  }, // Sol Matrix/LED Far (D6)
  {0xD7, 0x797, "LightR"  }, // Sağ Matrix/LED Far (D7)

  // Infotainment & Audio & Telematics
  {0x0F, 0x770, "DigiRad" }, // Dijital Radyo DAB (0F)
  {0x47, 0x747, "SoundSys"}, // Ses Sistemi / Amfi (47)
  {0x56, 0x756, "Radio"   }, // Radyo Ünitesi (56)
  {0x57, 0x757, "TV-Tuner"}, // TV Alıcısı (57)
  {0x5F, 0x773, "InfoEnt" }, // MIB Bilgi-Eğlence / Navigasyon (5F)
  {0x75, 0x775, "eCallSOS"}, // Acil Çağrı / Telematik OCU (75)
};
#define MODULE_COUNT (sizeof(MODULE_TABLE) / sizeof(ModuleEntry))

// -----------------------------------------------------------------------------
// RESULT STORAGE
// -----------------------------------------------------------------------------
struct ModuleResult {
  uint8_t vagAddr;       // VCDS Address
  char    label[9];      // Copied from PROGMEM
  char    partNum[14];   // Null-terminated part number
  char    swVer[7];      // Null-terminated software version
  char    hwVer[5];      // Null-terminated hardware version
  uint8_t status;        // 0=not found, 1=found
};

#define MAX_FOUND 24
ModuleResult foundModules[MAX_FOUND];
uint8_t foundCount = 0;
uint8_t totalScanned = 0;
uint8_t timeoutCount = 0;

// Flag array for Gateway-discovered modules
bool shouldScan[MODULE_COUNT];

// -----------------------------------------------------------------------------
// ISO-TP MULTI-FRAME BUFFER
// -----------------------------------------------------------------------------
#define ISOTP_BUF_SIZE 64
static uint8_t isotpBuf[ISOTP_BUF_SIZE];
static uint8_t isotpLen;

// -----------------------------------------------------------------------------
// ISO-TP & UDS FUNCTIONS
// -----------------------------------------------------------------------------

// Sends a raw Single Frame or First Frame UDS request with 0x55 padding
void sendUDS(uint16_t txId, uint8_t sid, const uint8_t* payload, uint8_t payloadLen) {
    struct can_frame frame;
    frame.can_id = txId;
    frame.can_dlc = 8;
    frame.data[0] = payloadLen + 1; // Length includes SID
    frame.data[1] = sid;
    for(int i = 0; i < 6; i++) {
        if (i < payloadLen) {
            frame.data[i+2] = payload[i];
        } else {
            frame.data[i+2] = 0x55; // UDS padding
        }
    }
    mcp2515.sendMessage(&frame);
}

// Sends Tester Present to wake up or keep session alive
void sendTesterPresent(uint16_t txId) {
    uint8_t payload[1] = {0x00};
    sendUDS(txId, 0x3E, payload, 1);
    delay(20);
}

// Receives ISO-TP frames and assembles multi-frame responses
// Returns length of combined data (which starts with SID response), or 0 on timeout
uint8_t receiveISOTP(uint16_t expectedRxId, uint16_t targetTxId, uint8_t* outBuf, uint16_t timeoutMs) {
    uint32_t startMs = millis();
    uint8_t totalLen = 0;
    uint8_t receivedLen = 0;
    
    while(millis() - startMs < timeoutMs) {
        struct can_frame frame;
        if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
            if (frame.can_id == expectedRxId && frame.can_dlc > 0) {
                uint8_t pciType = frame.data[0] >> 4;
                
                // Single Frame (PCI = 0x0)
                if (pciType == 0) {
                    totalLen = frame.data[0] & 0x0F;
                    if (totalLen > 7) totalLen = 7;
                    for (int i=0; i<totalLen; i++) outBuf[i] = frame.data[i+1];
                    return totalLen;
                } 
                // First Frame (PCI = 0x1)
                else if (pciType == 1) {
                    totalLen = ((frame.data[0] & 0x0F) << 8) | frame.data[1];
                    if (totalLen > ISOTP_BUF_SIZE) totalLen = ISOTP_BUF_SIZE; // Prevent overflow
                    
                    for (int i=0; i<6; i++) outBuf[i] = frame.data[i+2];
                    receivedLen = 6;
                    
                    // Send Flow Control (FC): [0x30, 0x00, 0x0A, 0x55, 0x55...]
                    struct can_frame fc;
                    fc.can_id = targetTxId;
                    fc.can_dlc = 8;
                    fc.data[0] = 0x30;
                    fc.data[1] = 0x00; // Block size (0 = all)
                    fc.data[2] = 0x0A; // STmin (10ms)
                    for(int i=3; i<8; i++) fc.data[i] = 0x55;
                    mcp2515.sendMessage(&fc);
                    
                    startMs = millis(); // Reset timeout for next frames
                } 
                // Consecutive Frame (PCI = 0x2)
                else if (pciType == 2) {
                    // uint8_t seq = frame.data[0] & 0x0F; // Sequence check omitted for brevity
                    uint8_t bytesToCopy = totalLen - receivedLen;
                    if (bytesToCopy > 7) bytesToCopy = 7;
                    
                    for(int i=0; i<bytesToCopy; i++) {
                        if (receivedLen + i < ISOTP_BUF_SIZE) {
                            outBuf[receivedLen + i] = frame.data[i+1];
                        }
                    }
                    receivedLen += bytesToCopy;
                    
                    if (receivedLen >= totalLen) {
                        return (totalLen > ISOTP_BUF_SIZE) ? ISOTP_BUF_SIZE : totalLen;
                    }
                    startMs = millis(); // Reset timeout
                }
            }
        }
    }
    return 0; // Timeout
}

// Queries a Data Identifier (DID) and stores result in outBuf
// Returns number of bytes read
uint8_t queryDID(uint16_t txId, uint16_t rxId, uint16_t did, uint8_t* outBuf, uint16_t timeoutMs) {
    uint8_t payload[2] = { (uint8_t)(did >> 8), (uint8_t)(did & 0xFF) };
    sendUDS(txId, 0x22, payload, 2);
    
    uint8_t rawBuf[ISOTP_BUF_SIZE];
    uint8_t len = receiveISOTP(rxId, txId, rawBuf, timeoutMs);
    
    // Check for Positive Response to 0x22 (which is 0x62) and matching DID
    if (len > 3 && rawBuf[0] == 0x62 && rawBuf[1] == payload[0] && rawBuf[2] == payload[1]) {
        uint8_t dataLen = len - 3;
        for(int i=0; i<dataLen; i++) outBuf[i] = rawBuf[i+3];
        return dataLen;
    }
    return 0;
}

// Opens a Diagnostic Session (Extended = 0x03)
bool openSession(uint16_t txId, uint16_t rxId, uint8_t sessionType) {
    uint8_t payload[1] = {sessionType};
    sendUDS(txId, 0x10, payload, 1);
    
    uint8_t rawBuf[16];
    uint8_t len = receiveISOTP(rxId, txId, rawBuf, 300);
    
    // Positive response for 0x10 is 0x50
    return (len >= 2 && rawBuf[0] == 0x50 && rawBuf[1] == sessionType);
}

// -----------------------------------------------------------------------------
// UI DRAWING FUNCTIONS
// -----------------------------------------------------------------------------

void drawWelcomeScreen() {
    u8g2.firstPage();
    do {
        u8g2.drawFrame(0, 0, 128, 64);
        u8g2.drawLine(0, 48, 128, 48);
        
        u8g2.setFont(u8g2_font_7x14B_tr);
        u8g2.setCursor(8, 16);
        u8g2.print(F("VAG MQB PLATFORM"));
        
        u8g2.setFont(u8g2_font_6x10_tr);
        u8g2.setCursor(8, 32);
        u8g2.print(F("Modul Tara " SKETCH_VERSION));
        
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.setCursor(6, 42);
        u8g2.print(F("Gateway Sorgulaniyor"));
        
        u8g2.setCursor(18, 58);
        u8g2.print(F("500kbps CAN Gateway"));
    } while ( u8g2.nextPage() );
}

void drawScanningScreen(uint8_t currentIndex, uint8_t maxIndex, const char* name, uint8_t vagAddr, uint16_t txId, uint16_t rxId, uint8_t progressPercent, const char* statusMsg) {
    u8g2.firstPage();
    do {
        u8g2.drawFrame(0, 0, 128, 64);
        u8g2.drawLine(0, 14, 128, 14);
        
        // Header
        u8g2.setFont(u8g2_font_6x10_tr);
        u8g2.setCursor(4, 10);
        u8g2.print(F("MODUL TARAMA"));
        
        u8g2.setCursor(86, 10);
        u8g2.print(currentIndex);
        u8g2.print(F("/"));
        u8g2.print(maxIndex);
        
        // Module Name
        u8g2.setFont(u8g2_font_7x14_tr);
        u8g2.setCursor(4, 28);
        u8g2.print(name);
        u8g2.print(F(" ["));
        if (vagAddr < 0x10) u8g2.print(F("0"));
        u8g2.print(vagAddr, HEX);
        u8g2.print(F("]"));
        
        // IDs
        u8g2.setFont(u8g2_font_6x10_tr);
        u8g2.setCursor(4, 40);
        u8g2.print(F("0x")); u8g2.print(txId, HEX);
        u8g2.print(F(" > 0x")); u8g2.print(rxId, HEX);
        
        // Progress Bar
        u8g2.drawFrame(4, 44, 90, 8);
        uint8_t barWidth = (uint8_t)((90UL * progressPercent) / 100);
        if (barWidth > 0) {
            u8g2.drawBox(4, 44, barWidth, 8);
        }
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.setCursor(98, 51);
        u8g2.print(progressPercent);
        u8g2.print(F("%"));
        
        // Status message
        u8g2.setCursor(4, 60);
        u8g2.print(statusMsg);
    } while ( u8g2.nextPage() );
}

void drawSummaryScreen() {
    u8g2.firstPage();
    do {
        u8g2.drawFrame(0, 0, 128, 64);
        u8g2.drawLine(0, 16, 128, 16);
        u8g2.drawLine(0, 52, 128, 52);
        
        u8g2.setFont(u8g2_font_7x14B_tr);
        u8g2.setCursor(8, 12);
        u8g2.print(F("TARAMA TAMAMLANDI"));
        
        u8g2.setFont(u8g2_font_6x10_tr);
        u8g2.setCursor(8, 28);
        u8g2.print(F("Taranan : ")); u8g2.print(totalScanned);
        
        u8g2.setCursor(8, 38);
        u8g2.print(F("Bulunan : ")); u8g2.print(foundCount);
        
        u8g2.setCursor(8, 48);
        u8g2.print(F("Timeout : ")); u8g2.print(timeoutCount);
        
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.setCursor(8, 60);
        if (foundCount == 0) {
            u8g2.print(F("Hicbir modul bulunamadi!"));
        } else {
            u8g2.print(F("Sonuclar gosteriliyor..."));
        }
    } while ( u8g2.nextPage() );
}

void drawNoConnectionScreen() {
    u8g2.firstPage();
    do {
        u8g2.drawFrame(0, 0, 128, 64);
        u8g2.drawLine(0, 15, 128, 15);
        
        // Header
        u8g2.setFont(u8g2_font_6x10_tr);
        u8g2.setCursor(4, 11);
        u8g2.print(F("! OBD BAGLANTISI YOK !"));
        
        // Message body
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.setCursor(6, 26);
        u8g2.print(F("- Modul yaniti alinamadi"));
        u8g2.setCursor(6, 36);
        u8g2.print(F("- OBD soketini takin"));
        u8g2.setCursor(6, 46);
        u8g2.print(F("- Kontagi acin (ON/ACC)"));
        
        // Footer
        u8g2.drawLine(0, 50, 128, 50);
        u8g2.setCursor(6, 59);
        u8g2.print(F("Tekrar tara: RESET tusuna bas"));
    } while ( u8g2.nextPage() );
}

void drawResultScreen(uint8_t idx) {
    u8g2.firstPage();
    do {
        ModuleResult& r = foundModules[idx];
        
        u8g2.drawFrame(0, 0, 128, 64);
        u8g2.drawLine(0, 14, 128, 14);
        u8g2.drawLine(0, 52, 128, 52);
        
        // Header
        u8g2.setFont(u8g2_font_6x10_tr);
        u8g2.setCursor(4, 10);
        u8g2.print(r.label);
        u8g2.print(F(" ["));
        if (r.vagAddr < 0x10) u8g2.print(F("0"));
        u8g2.print(r.vagAddr, HEX);
        u8g2.print(F("]"));
        
        u8g2.setCursor(96, 10);
        u8g2.print(idx + 1);
        u8g2.print(F("/"));
        u8g2.print(foundCount);
        
        // Part Number
        u8g2.setFont(u8g2_font_7x14B_tr);
        u8g2.setCursor(4, 30);
        if (strlen(r.partNum) > 0) u8g2.print(r.partNum);
        else u8g2.print(F("Bilinmiyor"));
        
        // Versions
        u8g2.setFont(u8g2_font_6x10_tr);
        u8g2.setCursor(4, 46);
        u8g2.print(F("SW:")); u8g2.print(strlen(r.swVer)>0 ? r.swVer : "?");
        u8g2.setCursor(64, 46);
        u8g2.print(F("HW:")); u8g2.print(strlen(r.hwVer)>0 ? r.hwVer : "?");
        
        // Status footer
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.setCursor(4, 60);
        u8g2.print(F("BASARILI OKUNDU"));
        
    } while ( u8g2.nextPage() );
}

// -----------------------------------------------------------------------------
// MAIN LOGIC
// -----------------------------------------------------------------------------

void setup() {
    
    // Init OLED
    u8g2.begin();
    u8g2.setContrast(200);
    
    // Init CAN
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    
    // Start Application
    drawWelcomeScreen();
    delay(2000);
    
    // PHASE 1: Gateway Discovery
    bool gwContacted = false;
    uint16_t gwTx = 0x710;
    uint16_t gwRx = 0x77A; // 0x710 + 0x6A
    
    // Try Ext Session on Gateway
    if (openSession(gwTx, gwRx, 0x03)) {
        gwContacted = true;
        
        uint8_t ansBuf[64];
        uint8_t ansLen = queryDID(gwTx, gwRx, 0x04A0, ansBuf, 1000);
        
        if (ansLen == 0) {
            ansLen = queryDID(gwTx, gwRx, 0x0500, ansBuf, 1000);
        }
        
        if (ansLen > 0) {
            // Parse installed modules list
            for (uint8_t i = 0; i < MODULE_COUNT; i++) shouldScan[i] = false;
            
            for (uint8_t i = 0; i < ansLen; i++) {
                uint8_t addr = ansBuf[i];
                for (uint8_t j = 0; j < MODULE_COUNT; j++) {
                    ModuleEntry e;
                    memcpy_P(&e, &MODULE_TABLE[j], sizeof(ModuleEntry));
                    if (e.vagAddr == addr) {
                        shouldScan[j] = true;
                    }
                }
            }
        } else {
            // Failed to read DID, scan all
            for (uint8_t i = 0; i < MODULE_COUNT; i++) shouldScan[i] = true;
        }
    } else {
        // Gateway no response, fallback to scanning all
        for (uint8_t i = 0; i < MODULE_COUNT; i++) shouldScan[i] = true;
    }
    
    // Count modules to scan
    uint8_t scanTargetCount = 0;
    for (uint8_t i = 0; i < MODULE_COUNT; i++) {
        if (shouldScan[i]) scanTargetCount++;
    }
    
    // PHASE 2: Module Scanning
    uint8_t scanIdx = 0;
    
    for (uint8_t i = 0; i < MODULE_COUNT; i++) {
        if (!shouldScan[i]) continue;
        if (foundCount >= MAX_FOUND) break; // Array full
        
        scanIdx++;
        
        ModuleEntry entry;
        memcpy_P(&entry, &MODULE_TABLE[i], sizeof(ModuleEntry));
        
        uint16_t txId = entry.canTx;
        uint16_t rxId = getRxId(txId);
        
        drawScanningScreen(scanIdx, scanTargetCount, entry.label, entry.vagAddr, txId, rxId, 10, "Uyandiriliyor...");
        totalScanned++;
        
        // Wakeup
        sendTesterPresent(txId);
        delay(50);
        
        // Try extended session (optional)
        drawScanningScreen(scanIdx, scanTargetCount, entry.label, entry.vagAddr, txId, rxId, 30, "Oturum aciliyor...");
        openSession(txId, rxId, 0x03);
        delay(50);
        
        // Query Part Number F187
        drawScanningScreen(scanIdx, scanTargetCount, entry.label, entry.vagAddr, txId, rxId, 50, "F187 Part No...");
        uint8_t buf[ISOTP_BUF_SIZE];
        uint8_t pLen = queryDID(txId, rxId, 0xF187, buf, 500);
        
        if (pLen > 0) {
            // Module found!
            ModuleResult& r = foundModules[foundCount];
            r.vagAddr = entry.vagAddr;
            strncpy(r.label, entry.label, sizeof(r.label));
            
            memset(r.partNum, 0, sizeof(r.partNum));
            memset(r.swVer, 0, sizeof(r.swVer));
            memset(r.hwVer, 0, sizeof(r.hwVer));
            
            // Extract ASCII (limit to buffer size minus null)
            uint8_t copyLen = (pLen > 13) ? 13 : pLen;
            for(uint8_t k=0; k<copyLen; k++) {
                char c = (char)buf[k];
                if (c >= 32 && c <= 126) r.partNum[k] = c;
                else r.partNum[k] = ' ';
            }
            
            delay(50);
            
            // Query SW Ver F189
            drawScanningScreen(scanIdx, scanTargetCount, entry.label, entry.vagAddr, txId, rxId, 70, "F189 SW Ver...");
            uint8_t sLen = queryDID(txId, rxId, 0xF189, buf, 500);
            if (sLen > 0) {
                copyLen = (sLen > 6) ? 6 : sLen;
                for(uint8_t k=0; k<copyLen; k++) {
                    char c = (char)buf[k];
                    if (c >= 32 && c <= 126) r.swVer[k] = c;
                    else r.swVer[k] = ' ';
                }
            }
            
            delay(50);
            
            // Query HW Ver F191
            drawScanningScreen(scanIdx, scanTargetCount, entry.label, entry.vagAddr, txId, rxId, 90, "F191 HW Ver...");
            uint8_t hLen = queryDID(txId, rxId, 0xF191, buf, 500);
            if (hLen > 0) {
                copyLen = (hLen > 4) ? 4 : hLen;
                for(uint8_t k=0; k<copyLen; k++) {
                    char c = (char)buf[k];
                    if (c >= 32 && c <= 126) r.hwVer[k] = c;
                    else r.hwVer[k] = ' ';
                }
            }
            
            r.status = 1;
            foundCount++;
        } else {
            timeoutCount++;
        }
        
        drawScanningScreen(scanIdx, scanTargetCount, entry.label, entry.vagAddr, txId, rxId, 100, "Tamamlandi!");
        delay(100);
    }
}

void loop() {
    // If no modules were found (e.g. OBD disconnected or ignition off)
    if (foundCount == 0) {
        static bool summaryShown = false;
        if (!summaryShown) {
            drawSummaryScreen();
            delay(4000); // 4 seconds summary
            summaryShown = true;
        }
        drawNoConnectionScreen();
        delay(2000);
        return;
    }
    
    // Normal results display cycle (modules found)
    drawSummaryScreen();
    delay(4000);
    
    for (uint8_t i = 0; i < foundCount; i++) {
        drawResultScreen(i);
        delay(3000); // 3 seconds per module screen
    }
}
