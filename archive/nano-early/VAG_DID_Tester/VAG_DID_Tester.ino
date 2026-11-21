// =============================================================================
// VAG_DID_Tester.ino — EA288 (ECU 0x7E0) + DQ200 (TCU 0x7E1) Brute-Force Tarayici
// Donanim: Arduino Nano + MCP2515 (CS:D10, INT:D2, 8MHz kristal, 500kbps)
// Cikti   : Serial 115200 baud — CSV formatli ham kayit
// Android : USB-OTG + Serial USB Terminal -> .txt kaydet -> PC analiz
//
// CSV Sutunlari:
//   TICK_MS, ECU_ID, PROTO, ADDR_HEX, RESP_LEN, RESP_HEX_BYTES, NOTE
//
// Komutlar (Serial Monitor):
//   S = Tarama BASLAT
//   P = DURDUR / Devam ettir
//   R = SIFIRLA
//
// Tarama Sirasi:
//   1) OBD Mode 01 PID : 0x00-0xFF (ECU broadcast 0x7DF)
//   2) OBD Mode 09 PID : 0x00-0x1F (VIN, Kalibrasyon)
//   3) UDS ECU DID LOW : 0x0001-0x07FF (Kimlik/Sistem)
//   4) UDS ECU DID MID : 0x1000-0x27FF (Motor Sensorler - ANA BLOK)
//   5) UDS ECU DID HIGH: 0x4000-0x40FF (Adaptasyon)
//   6) UDS TCU DID LOW : 0x0001-0x07FF (DSG Kimlik)
//   7) UDS TCU DID MID : 0x1000-0x17FF (DSG Sensorler)
// =============================================================================

#include <SPI.h>
#include <mcp_can.h>

// --- Pin Tanimlari ---
#define PIN_CAN_CS     10
#define PIN_CAN_INT     2

// --- UDS / OBD Sabitleri ---
#define UDS_SID_READ_DID  0x22
#define OBD_SID_01        0x01
#define OBD_SID_09        0x09

// --- CAN Kimlikleri ---
#define ECU_TX_ID    0x7E0
#define ECU_RX_ID    0x7E8
#define TCU_TX_ID    0x7E1
#define TCU_RX_ID    0x7E9
#define OBD_BCAST    0x7DF

// --- Zamanlama ---
#define TIMEOUT_MS         120
#define INTER_MSG_DELAY     15
#define FLOW_CTRL_DELAY     10

MCP_CAN CAN(PIN_CAN_CS);

enum ScanPhase {
  PHASE_OBD_PID,
  PHASE_OBD_MODE9,
  PHASE_UDS_ECU_LOW,
  PHASE_UDS_ECU_MID,
  PHASE_UDS_ECU_HIGH,
  PHASE_UDS_TCU_LOW,
  PHASE_UDS_TCU_MID,
  PHASE_COMPLETE
};

ScanPhase currentPhase = PHASE_OBD_PID;
uint16_t  currentAddr  = 0x0000;
uint32_t  totalFound   = 0;
uint32_t  totalTried   = 0;
bool      scanRunning  = false;
bool      pauseScan    = false;

// --- Fonksiyon Bildirimleri ---
bool sendOBD(uint32_t txID, uint8_t mode, uint8_t pid, uint8_t *respBuf, uint8_t &respLen);
bool sendUDS(uint32_t txID, uint32_t rxID, uint16_t did, uint8_t *respBuf, uint8_t &respLen);
bool waitForResponse(uint32_t expectedID, uint8_t *buf, uint8_t &len, uint16_t timeoutMs);
void logCSV(const char* proto, uint32_t addr, uint8_t *data, uint8_t len, const char* note);
void logInfo(const char* msg);
void sendFlowControl(uint32_t txID);
void printPhaseHeader();
void nextPhase();
void annotateECU(uint16_t did, char* note);
void annotateTCU(uint16_t did, char* note);

// =============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println(F("# VAG_DID_Tester v1.0 — EA288 + DQ200 Brute-Force Scanner"));
  Serial.println(F("# Donanim: Arduino Nano + MCP2515 CS:D10 INT:D2 8MHz 500kbps"));
  Serial.println(F("# Komutlar: S=Baslat  P=Durdur/Devam  R=Sifirla"));
  Serial.println(F("TICK_MS,ECU_ID,PROTO,ADDR_HEX,RESP_LEN,RESP_HEX,NOTE"));

  byte initResult;
  for (uint8_t attempt = 0; attempt < 5; attempt++) {
    initResult = CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
    if (initResult == CAN_OK) break;
    delay(300);
  }

  if (initResult != CAN_OK) {
    Serial.println(F("# HATA: MCP2515 baslanamadi! Baglantilari kontrol edin."));
    while (true) { delay(1000); }
  }

  CAN.setMode(MCP_NORMAL);
  pinMode(PIN_CAN_INT, INPUT);
  Serial.println(F("# MCP2515 HAZIR — 500kbps — 8MHz"));
  Serial.println(F("# 'S' gondererek taramayi baslatabilirsiniz"));
}

// =============================================================================
void loop() {
  if (Serial.available()) {
    char cmd = toupper((char)Serial.read());
    if (cmd == 'S' && !scanRunning) {
      scanRunning  = true;
      pauseScan    = false;
      currentPhase = PHASE_OBD_PID;
      currentAddr  = 0x0000;
      totalFound   = 0;
      totalTried   = 0;
      logInfo("TARAMA BASLATILDI");
      printPhaseHeader();
    } else if (cmd == 'P') {
      pauseScan = !pauseScan;
      logInfo(pauseScan ? "DURAKLATILDI" : "DEVAM EDİYOR");
    } else if (cmd == 'R') {
      scanRunning  = false;
      pauseScan    = false;
      currentPhase = PHASE_OBD_PID;
      currentAddr  = 0x0000;
      totalFound   = 0;
      totalTried   = 0;
      logInfo("SIFIRLANDI");
    }
  }

  if (!scanRunning || pauseScan || currentPhase == PHASE_COMPLETE) return;

  uint8_t respBuf[16] = {0};
  uint8_t respLen     = 0;
  bool    gotResp     = false;
  char    note[32]    = "";
  char    protoStr[12]= "";

  switch (currentPhase) {

    case PHASE_OBD_PID: {
      uint8_t pid = (uint8_t)currentAddr;
      strcpy(protoStr, "OBD01");
      gotResp = sendOBD(OBD_BCAST, OBD_SID_01, pid, respBuf, respLen);
      if (gotResp) {
        if ((pid & 0x1F) == 0x00 && respLen >= 4) strcpy(note, "SUPPORT_BITMAP");
        logCSV(protoStr, pid, respBuf, respLen, note);
        totalFound++;
      }
      totalTried++;
      currentAddr++;
      if (currentAddr > 0xFF) nextPhase();
      break;
    }

    case PHASE_OBD_MODE9: {
      uint8_t pid = (uint8_t)currentAddr;
      strcpy(protoStr, "OBD09");
      gotResp = sendOBD(OBD_BCAST, OBD_SID_09, pid, respBuf, respLen);
      if (gotResp) {
        if (pid == 0x02) strcpy(note, "VIN_17CHAR");
        else if (pid == 0x04) strcpy(note, "ECU_CALIBRATION_ID");
        else if (pid == 0x06) strcpy(note, "ECU_PART_NUMBER");
        logCSV(protoStr, pid, respBuf, respLen, note);
        totalFound++;
      }
      totalTried++;
      currentAddr++;
      if (currentAddr > 0x1F) nextPhase();
      break;
    }

    case PHASE_UDS_ECU_LOW: {
      strcpy(protoStr, "UDS_ECU");
      gotResp = sendUDS(ECU_TX_ID, ECU_RX_ID, currentAddr, respBuf, respLen);
      if (gotResp) { annotateECU(currentAddr, note); logCSV(protoStr, currentAddr, respBuf, respLen, note); totalFound++; }
      totalTried++;
      currentAddr++;
      if (currentAddr > 0x07FF) nextPhase();
      break;
    }

    case PHASE_UDS_ECU_MID: {
      strcpy(protoStr, "UDS_ECU");
      gotResp = sendUDS(ECU_TX_ID, ECU_RX_ID, currentAddr, respBuf, respLen);
      if (gotResp) { annotateECU(currentAddr, note); logCSV(protoStr, currentAddr, respBuf, respLen, note); totalFound++; }
      totalTried++;
      currentAddr++;
      if (currentAddr > 0x27FF) nextPhase();
      break;
    }

    case PHASE_UDS_ECU_HIGH: {
      strcpy(protoStr, "UDS_ECU");
      gotResp = sendUDS(ECU_TX_ID, ECU_RX_ID, currentAddr, respBuf, respLen);
      if (gotResp) { annotateECU(currentAddr, note); logCSV(protoStr, currentAddr, respBuf, respLen, note); totalFound++; }
      totalTried++;
      currentAddr++;
      if (currentAddr > 0x40FF) nextPhase();
      break;
    }

    case PHASE_UDS_TCU_LOW: {
      strcpy(protoStr, "UDS_TCU");
      gotResp = sendUDS(TCU_TX_ID, TCU_RX_ID, currentAddr, respBuf, respLen);
      if (gotResp) { annotateTCU(currentAddr, note); logCSV(protoStr, currentAddr, respBuf, respLen, note); totalFound++; }
      totalTried++;
      currentAddr++;
      if (currentAddr > 0x07FF) nextPhase();
      break;
    }

    case PHASE_UDS_TCU_MID: {
      strcpy(protoStr, "UDS_TCU");
      gotResp = sendUDS(TCU_TX_ID, TCU_RX_ID, currentAddr, respBuf, respLen);
      if (gotResp) { annotateTCU(currentAddr, note); logCSV(protoStr, currentAddr, respBuf, respLen, note); totalFound++; }
      totalTried++;
      currentAddr++;
      if (currentAddr > 0x17FF) nextPhase();
      break;
    }

    case PHASE_COMPLETE:
    default:
      break;
  }

  delay(INTER_MSG_DELAY);
}

// =============================================================================
void nextPhase() {
  currentPhase = (ScanPhase)((int)currentPhase + 1);
  switch (currentPhase) {
    case PHASE_OBD_MODE9:     currentAddr = 0x00;   break;
    case PHASE_UDS_ECU_LOW:   currentAddr = 0x0001; break;
    case PHASE_UDS_ECU_MID:   currentAddr = 0x1000; break;
    case PHASE_UDS_ECU_HIGH:  currentAddr = 0x4000; break;
    case PHASE_UDS_TCU_LOW:   currentAddr = 0x0001; break;
    case PHASE_UDS_TCU_MID:   currentAddr = 0x1000; break;
    case PHASE_COMPLETE: {
      char buf[80];
      snprintf(buf, sizeof(buf), "TAMAMLANDI — Denenen:%lu Bulunan:%lu", totalTried, totalFound);
      logInfo(buf);
      scanRunning = false;
      break;
    }
    default: break;
  }
  if (currentPhase != PHASE_COMPLETE) printPhaseHeader();
}

// =============================================================================
bool sendOBD(uint32_t txID, uint8_t mode, uint8_t pid, uint8_t *respBuf, uint8_t &respLen) {
  uint8_t msg[8] = {0x02, mode, pid, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint32_t rxID = (txID == OBD_BCAST) ? ECU_RX_ID : (txID + 8);
  if (CAN.sendMsgBuf(txID, 0, 8, msg) != CAN_OK) return false;
  return waitForResponse(rxID, respBuf, respLen, TIMEOUT_MS);
}

// =============================================================================
bool sendUDS(uint32_t txID, uint32_t rxID, uint16_t did, uint8_t *respBuf, uint8_t &respLen) {
  uint8_t msg[8] = {
    0x03, UDS_SID_READ_DID,
    (uint8_t)(did >> 8), (uint8_t)(did & 0xFF),
    0x00, 0x00, 0x00, 0x00
  };
  if (CAN.sendMsgBuf(txID, 0, 8, msg) != CAN_OK) return false;

  uint8_t rawBuf[8] = {0};
  uint8_t rawLen    = 0;
  if (!waitForResponse(rxID, rawBuf, rawLen, TIMEOUT_MS)) return false;

  uint8_t pciType = (rawBuf[0] & 0xF0) >> 4;

  if (pciType == 0x0) {
    if (rawBuf[1] == 0x7F) return false;  // Negatif cevap
    uint8_t dataLen = rawBuf[0] & 0x0F;
    respLen = min(dataLen, (uint8_t)7);
    memcpy(respBuf, rawBuf + 1, respLen);
    return true;
  }

  if (pciType == 0x1) {
    uint16_t totalLen = ((uint16_t)(rawBuf[0] & 0x0F) << 8) | rawBuf[1];
    if (rawBuf[2] == 0x7F) return false;

    uint8_t maxCollect = 15;
    uint8_t collected  = 6;
    memcpy(respBuf, rawBuf + 2, min(collected, maxCollect));

    sendFlowControl(txID);
    delay(FLOW_CTRL_DELAY);

    while (collected < totalLen && collected < maxCollect) {
      uint8_t cf[8] = {0}; uint8_t cfLen = 0;
      if (!waitForResponse(rxID, cf, cfLen, TIMEOUT_MS)) break;
      if ((cf[0] & 0xF0) != 0x20) break;
      uint8_t copyCount = min((uint8_t)7, (uint8_t)(maxCollect - collected));
      memcpy(respBuf + collected, cf + 1, copyCount);
      collected += 7;
    }
    respLen = min(collected, (uint8_t)15);
    return true;
  }
  return false;
}

// =============================================================================
bool waitForResponse(uint32_t expectedID, uint8_t *buf, uint8_t &len, uint16_t timeoutMs) {
  uint32_t start = millis();
  while ((millis() - start) < timeoutMs) {
    if (digitalRead(PIN_CAN_INT) == LOW || CAN.checkReceive() == CAN_MSGAVAIL) {
      long unsigned int rxID;
      uint8_t rxLen;
      uint8_t rxBuf[8];
      if (CAN.readMsgBuf(&rxID, &rxLen, rxBuf) == CAN_OK) {
        if (rxID == expectedID) {
          memcpy(buf, rxBuf, rxLen);
          len = rxLen;
          return true;
        }
      }
    }
  }
  return false;
}

// =============================================================================
void sendFlowControl(uint32_t txID) {
  uint8_t fc[8] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  CAN.sendMsgBuf(txID, 0, 8, fc);
}

// =============================================================================
void logCSV(const char* proto, uint32_t addr, uint8_t *data, uint8_t len, const char* note) {
  const char* ecuStr = (strstr(proto, "TCU") != NULL) ? "TCU" : "ECU";
  Serial.print(millis()); Serial.print(',');
  Serial.print(ecuStr);   Serial.print(',');
  Serial.print(proto);    Serial.print(",0x");
  if (addr < 0x0010) Serial.print('0');
  if (addr < 0x0100) Serial.print('0');
  if (addr < 0x1000) Serial.print('0');
  Serial.print(addr, HEX); Serial.print(',');
  Serial.print(len); Serial.print(',');
  for (uint8_t i = 0; i < len; i++) {
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    if (i < len - 1) Serial.print(' ');
  }
  Serial.print(',');
  Serial.println(note);
}

void logInfo(const char* msg) {
  Serial.print(F("# [INFO] "));
  Serial.print(millis());
  Serial.print(F("ms — "));
  Serial.println(msg);
}

void printPhaseHeader() {
  Serial.print(F("# === FAZ: "));
  switch (currentPhase) {
    case PHASE_OBD_PID:      Serial.println(F("OBD Mode 01 PID (0x00-0xFF)")); break;
    case PHASE_OBD_MODE9:    Serial.println(F("OBD Mode 09 (0x00-0x1F) VIN/Kalibrasyon")); break;
    case PHASE_UDS_ECU_LOW:  Serial.println(F("UDS ECU DID LOW (0x0001-0x07FF) Kimlik")); break;
    case PHASE_UDS_ECU_MID:  Serial.println(F("UDS ECU DID MID (0x1000-0x27FF) Motor Sensorler")); break;
    case PHASE_UDS_ECU_HIGH: Serial.println(F("UDS ECU DID HIGH (0x4000-0x40FF) Adaptasyon")); break;
    case PHASE_UDS_TCU_LOW:  Serial.println(F("UDS TCU DID LOW (0x0001-0x07FF) DSG Kimlik")); break;
    case PHASE_UDS_TCU_MID:  Serial.println(F("UDS TCU DID MID (0x1000-0x17FF) DSG Sensorler")); break;
    default: break;
  }
}

// =============================================================================
void annotateECU(uint16_t did, char* note) {
  note[0] = '\0';
  switch (did) {
    case 0xF186: strcpy(note, "UDS_ACTIVE_SESSION"); break;
    case 0xF187: strcpy(note, "SPARE_PART_NUMBER"); break;
    case 0xF18A: strcpy(note, "SYSTEM_SUPPLIER_ID"); break;
    case 0xF18C: strcpy(note, "ECU_SERIAL_NUMBER"); break;
    case 0xF190: strcpy(note, "VIN_17CHAR"); break;
    case 0xF191: strcpy(note, "HW_VERSION"); break;
    case 0xF193: strcpy(note, "SW_VERSION"); break;
    case 0x1001: strcpy(note, "ENGINE_COOLANT_TEMP"); break;
    case 0x1002: strcpy(note, "ENGINE_OIL_TEMP"); break;
    case 0x1003: strcpy(note, "OIL_PRESSURE"); break;
    case 0x1004: strcpy(note, "FUEL_TEMP"); break;
    case 0x1005: strcpy(note, "INTAKE_AIR_TEMP"); break;
    case 0x1006: strcpy(note, "CHARGE_AIR_TEMP"); break;
    case 0x1007: strcpy(note, "ENGINE_SPEED_RPM"); break;
    case 0x1008: strcpy(note, "INJECTION_QUANTITY"); break;
    case 0x100A: strcpy(note, "RAIL_PRESSURE_ACTUAL"); break;
    case 0x100B: strcpy(note, "RAIL_PRESSURE_SETPOINT"); break;
    case 0x100C: strcpy(note, "MANIFOLD_PRESS_ABS"); break;
    case 0x100D: strcpy(note, "BOOST_PRESS_ACTUAL"); break;
    case 0x100E: strcpy(note, "BOOST_PRESS_SETPOINT"); break;
    case 0x100F: strcpy(note, "TURBO_SPEED"); break;
    case 0x1010: strcpy(note, "VGT_POSITION_ACTUAL"); break;
    case 0x1011: strcpy(note, "VGT_POSITION_SETPOINT"); break;
    case 0x1012: strcpy(note, "MAF_ACTUAL_KGH"); break;
    case 0x1013: strcpy(note, "MAF_SETPOINT_KGH"); break;
    case 0x1014: strcpy(note, "EGR_VALVE_ACTUAL"); break;
    case 0x1015: strcpy(note, "EGR_VALVE_SETPOINT"); break;
    case 0x1016: strcpy(note, "LAMBDA_SENSOR_1"); break;
    case 0x101A: strcpy(note, "ACCELERATOR_PEDAL"); break;
    case 0x101B: strcpy(note, "VEHICLE_SPEED_KMH"); break;
    case 0x101C: strcpy(note, "BATTERY_VOLTAGE"); break;
    case 0x101E: strcpy(note, "FUEL_CONSUMPTION"); break;
    case 0x1150: strcpy(note, "DPF_STATUS_FLAGS"); break;
    case 0x1151: strcpy(note, "DPF_REGEN_ACTIVE"); break;
    case 0x1152: strcpy(note, "DPF_REGEN_TYPE"); break;
    case 0x1153: strcpy(note, "DPF_DIFF_PRESSURE_hPa"); break;
    case 0x1154: strcpy(note, "DPF_REGEN_COUNTER"); break;
    case 0x1155: strcpy(note, "DPF_REGEN_DIST_KM"); break;
    case 0x1156: strcpy(note, "DPF_EXHAUST_TEMP_x001C"); break;
    case 0x1157: strcpy(note, "EGT_UPSTREAM_TURBO"); break;
    case 0x1158: strcpy(note, "EGT_DOWNSTREAM_TURBO"); break;
    case 0x1159: strcpy(note, "EGT_UPSTREAM_DPF"); break;
    case 0x115A: strcpy(note, "DPF_SOOT_MASS_G"); break;
    case 0x115B: strcpy(note, "DPF_ASH_MASS_G"); break;
    case 0x115C: strcpy(note, "DPF_LOADING_PERCENT"); break;
    case 0x115D: strcpy(note, "DPF_REGEN_REQUEST"); break;
    case 0x115E: strcpy(note, "DPF_TEMP_MODEL"); break;
    case 0x115F: strcpy(note, "DPF_SUBSTRATE_TEMP"); break;
    case 0x1160: strcpy(note, "SCR_CATALYST_TEMP"); break;
    case 0x1161: strcpy(note, "NOX_SENSOR_PPM"); break;
    case 0x11E0: strcpy(note, "KM_SINCE_LAST_SERVICE"); break;
    case 0x11E1: strcpy(note, "KM_SINCE_LAST_REGEN"); break;
    case 0x11E2: strcpy(note, "TOTAL_REGEN_COUNT"); break;
    case 0x11E3: strcpy(note, "TOTAL_RUNTIME_H"); break;
    case 0x11E4: strcpy(note, "TOTAL_DISTANCE_KM"); break;
    case 0x11E5: strcpy(note, "OIL_QUALITY_INDEX"); break;
    case 0x2024: strcpy(note, "ENGINE_OIL_TEMP_2024"); break;
    case 0x2025: strcpy(note, "OIL_PRESSURE_SWITCH"); break;
    case 0x2026: strcpy(note, "OIL_LEVEL_SENSOR"); break;
    case 0x2027: strcpy(note, "OIL_DILUTION_PERCENT"); break;
    case 0x2100: strcpy(note, "SUPPLY_VOLTAGE_mV"); break;
    case 0x2101: strcpy(note, "ALTERNATOR_TARGET_V"); break;
    case 0x2103: strcpy(note, "ECU_SUPPLY_VOLTAGE"); break;
    default: break;
  }
}

// =============================================================================
void annotateTCU(uint16_t did, char* note) {
  note[0] = '\0';
  switch (did) {
    case 0x040F: strcpy(note, "DSG_MECHATRONIC_TEMP_C"); break;
    case 0x0410: strcpy(note, "DSG_OIL_TEMP_C"); break;
    case 0x0411: strcpy(note, "DSG_CLUTCH1_TEMP_C"); break;
    case 0x0412: strcpy(note, "DSG_CLUTCH2_TEMP_C"); break;
    case 0x0420: strcpy(note, "DSG_CURRENT_GEAR"); break;
    case 0x0421: strcpy(note, "DSG_TARGET_GEAR"); break;
    case 0x0422: strcpy(note, "DSG_SHIFT_PROGRAM"); break;
    case 0x0423: strcpy(note, "DSG_MODE_D_S_M"); break;
    case 0x0424: strcpy(note, "DSG_KICKDOWN_ACTIVE"); break;
    case 0x0425: strcpy(note, "DSG_GEAR_RATIO"); break;
    case 0x0426: strcpy(note, "DSG_OUTPUT_SHAFT_RPM"); break;
    case 0x0427: strcpy(note, "DSG_INPUT_SHAFT_RPM"); break;
    case 0x0430: strcpy(note, "CLUTCH1_ENGAGE_PRESSURE"); break;
    case 0x0431: strcpy(note, "CLUTCH2_ENGAGE_PRESSURE"); break;
    case 0x0432: strcpy(note, "CLUTCH1_SLIP_RPM"); break;
    case 0x0433: strcpy(note, "CLUTCH2_SLIP_RPM"); break;
    case 0x0434: strcpy(note, "CLUTCH1_ACTUATOR_POS"); break;
    case 0x0435: strcpy(note, "CLUTCH2_ACTUATOR_POS"); break;
    case 0x0436: strcpy(note, "CLUTCH_WEAR_PERCENT"); break;
    case 0x0437: strcpy(note, "CLUTCH_ENERGY_KJ"); break;
    case 0x0440: strcpy(note, "HYD_SYSTEM_PRESSURE"); break;
    case 0x0441: strcpy(note, "HYD_PUMP_ACTIVE"); break;
    case 0x0442: strcpy(note, "HYD_ACCUMULATOR_PRESS"); break;
    case 0x1000: strcpy(note, "TCM_SUPPLY_VOLTAGE_NOT_TEMP"); break;
    case 0x1001: strcpy(note, "DSG_TORQUE_ACTUAL_NM"); break;
    case 0x1002: strcpy(note, "DSG_TORQUE_REQUESTED_NM"); break;
    case 0x1003: strcpy(note, "DSG_TORQUE_LIMIT_NM"); break;
    case 0x1004: strcpy(note, "DSG_ENGINE_TORQUE_RCV"); break;
    case 0x1005: strcpy(note, "DSG_ENGINE_SPEED_RCV"); break;
    case 0x100A: strcpy(note, "DSG_SELECTOR_POSITION"); break;
    case 0x100B: strcpy(note, "DSG_INHIBITOR_STATUS"); break;
    case 0x100C: strcpy(note, "DSG_SPORT_MODE_ACTIVE"); break;
    case 0x100D: strcpy(note, "DSG_LAUNCH_CTRL_ACTIVE"); break;
    case 0x100E: strcpy(note, "DSG_CREEP_ACTIVE"); break;
    case 0x100F: strcpy(note, "DSG_HILL_HOLD_ACTIVE"); break;
    case 0x1100: strcpy(note, "DSG_TOTAL_SHIFTS"); break;
    case 0x1101: strcpy(note, "DSG_TOTAL_CLUTCH_OPS"); break;
    case 0x1102: strcpy(note, "DSG_TOTAL_RUNTIME_H"); break;
    case 0x1103: strcpy(note, "DSG_CLUTCH_LIFE_REMAIN"); break;
    case 0x1104: strcpy(note, "DSG_OIL_CHANGE_DUE_KM"); break;
    case 0x1105: strcpy(note, "DSG_LEARN_ADAPT_FLAGS"); break;
    case 0x1106: strcpy(note, "DSG_CLUTCH_ADAPT_K1"); break;
    case 0x1107: strcpy(note, "DSG_CLUTCH_ADAPT_K2"); break;
    default: break;
  }
}
