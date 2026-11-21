#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoOTA.h>
#include <pico/mutex.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "secrets.h"

// ==============================================================================
// VAG PICO 2W - MFD + SCANNER + VALIDATOR + CUSTOM RAW UDS QUERY (RP2350)
// Core 1: MCP2515 CAN-Bus, ISO-TP (ISO 15765-2), UDS (ISO 14229), Power FSM
// Core 0: Wi-Fi AP, HTTP Server, 30 Hz Zero-Heap WebSocket Telemetry, ArduinoOTA
// ==============================================================================

#define PIN_CAN_MISO 16
#define PIN_CAN_CS   17
#define PIN_CAN_SCK  18
#define PIN_CAN_MOSI 19

static constexpr uint32_t CAN_SPI_HZ          = 8000000UL;
static constexpr uint32_t WDT_TIMEOUT_MS      = 8000UL;
static constexpr uint32_t CORE1_STALL_MS      = 5000UL;    // Core 1 hang threshold before WDT reboots
static constexpr uint32_t WS_BROADCAST_MS     = 33UL;      // ~30 Hz when data changes
static constexpr uint32_t WS_HEARTBEAT_MS     = 500UL;     // 2 Hz bus state heartbeat
static constexpr uint32_t IDLE_PROBE_MS       = 5000UL;    // Gentle probe interval only when bus has traffic
static constexpr uint32_t IDLE_TRAFFIC_WIN_MS = 2000UL;
static constexpr uint8_t  BOOT_PROBE_MAX      = 2;         // Max probes on silent boot before passive wait
static constexpr uint32_t SLEEP_AFTER_CAR_MS  = 180000UL;  // 3 min complete bus silence
static constexpr uint32_t SLEEP_NO_CAR_MS     = 300000UL;  // 5 min boot silence (when 0 WS clients)
static constexpr uint8_t  ACTIVE_FAIL_LIMIT   = 8;         // ~1 s of ECU silence -> Listen-Only
static constexpr uint32_t WAKE_DEBOUNCE_MS    = 1000UL;

// ==============================================================================
// TYPES & SHARED MULTI-CORE STATE
// ==============================================================================

enum SystemMode : uint8_t {
  MODE_MFD      = 0,
  MODE_SCAN     = 1,
  MODE_VALIDATE = 2,
  MODE_CUSTOM   = 3
};

enum BusState : uint8_t {
  BUS_ACTIVE    = 0,
  BUS_IDLE      = 1,
  BUS_SLEEP     = 2,
  BUS_INIT_FAIL = 3
};

enum DidFormula : uint8_t {
  FMT_U8_M40 = 0,
  FMT_U16_DIV10_M2731,
  FMT_BOOST_REL,
  FMT_U16_DIV50,
  FMT_U16_DIV100,
  FMT_S16_DIV100,
  FMT_U16_DIV10,
  FMT_S16_DIV10,
  FMT_U32_DIV1000,
  FMT_U32_DIV60,
  FMT_U16_DIV8192,
  FMT_RAW
};

struct DidDef {
  uint16_t   did;
  char       hexKey[5];
  uint8_t    byteLen;
  DidFormula formula;
};

static const DidDef DID_DEFS[24] = {
  { 0xF405, "F405", 1, FMT_U8_M40          }, // 0:  Soğutma Suyu
  { 0x11BE, "11BE", 2, FMT_U16_DIV10_M2731 }, // 1:  Motor Yağı
  { 0x181A, "181A", 2, FMT_BOOST_REL       }, // 2:  Turbo Basıncı (VIP)
  { 0x105E, "105E", 2, FMT_U16_DIV50       }, // 3:  Akü Voltajı
  { 0x100C, "100C", 2, FMT_U16_DIV100      }, // 4:  Yakıt Seviyesi
  { 0x114F, "114F", 2, FMT_U16_DIV100      }, // 5:  Soot Calc
  { 0x114E, "114E", 2, FMT_S16_DIV100      }, // 6:  Soot Meas (signed)
  { 0x14F5, "14F5", 2, FMT_S16_DIV10       }, // 7:  DPF dP (signed)
  { 0x178C, "178C", 2, FMT_U16_DIV10       }, // 8:  Oil Ash
  { 0x1156, "1156", 4, FMT_U32_DIV1000     }, // 9:  Mesafe / Regen
  { 0x115E, "115E", 4, FMT_U32_DIV60       }, // 10: Zaman / Regen
  { 0x11F6, "11F6", 2, FMT_U16_DIV100      }, // 11: DPF Sınır
  { 0x11B2, "11B2", 2, FMT_U16_DIV10_M2731 }, // 12: EGT 1 (Pre)
  { 0x10F9, "10F9", 2, FMT_U16_DIV10_M2731 }, // 13: EGT 2 (Post)
  { 0x1628, "1628", 2, FMT_U16_DIV10_M2731 }, // 14: İntake Temp
  { 0x163A, "163A", 2, FMT_U16_DIV100      }, // 15: Ana Enj.
  { 0x163B, "163B", 2, FMT_U16_DIV100      }, // 16: Post Enj 2
  { 0x167D, "167D", 2, FMT_U16_DIV100      }, // 17: Post Enj 3
  { 0x1314, "1314", 2, FMT_U16_DIV100      }, // 18: LP-EGR
  { 0x105C, "105C", 2, FMT_U16_DIV8192     }, // 19: Şarj Yükü
  { 0x1116, "1116", 2, FMT_U16_DIV100      }, // 20: Radyatör Fanı
  { 0x16A9, "16A9", 4, FMT_RAW             }, // 21: Odometer
  { 0x11B6, "11B6", 2, FMT_RAW             }, // 22: Regen Sayısı
  { 0x1347, "1347", 4, FMT_U32_DIV60       }  // 23: Regen Süresi
};

static constexpr size_t NUM_DIDS = sizeof(DID_DEFS) / sizeof(DID_DEFS[0]);
static constexpr size_t IDX_BOOST = 2;
static constexpr size_t IDX_SOOT_CALC = 5;
static constexpr size_t IDX_DPF_LIMIT = 11;

// Auto-initialized before main() so Core 0 and Core 1 never race at startup.
auto_init_mutex(dataMutex);
auto_init_mutex(cmdMutex);
auto_init_mutex(logMutex);

// Shared Telemetry (written by Core 1, read by Core 0 under dataMutex)
static float    g_didValues[NUM_DIDS]  = {0.0f};
static bool     g_didUpdated[NUM_DIDS] = {false};
static uint32_t g_dataSeq              = 0;
static BusState g_busState             = BUS_IDLE;

// Shared Commands (written by Core 0, consumed by Core 1 under cmdMutex)
static SystemMode g_mode          = MODE_MFD;
static bool       g_reqScan       = false;
static bool       g_reqVal        = false;
static bool       g_reqCustom     = false;
static uint16_t   g_customReqId   = 0x7E0;
static uint16_t   g_customResId   = 0x7E8;
static bool       g_customResAuto = true;
static uint16_t   g_customDid     = 0x181A;

// Shared Status Flags
static volatile bool    g_isUpdating     = false;
static volatile bool    g_core1Paused    = false;
static volatile bool    g_probeRequested = false;
static volatile bool    g_sleepRequested = false;
static volatile bool    g_core0Sleeping  = false;
static volatile uint8_t  g_wsClientCount    = 0;
static volatile uint32_t g_core1HeartbeatMs = 0;

// Fixed-capacity Ring Buffer for Log Messages (zero heap allocations)
static constexpr size_t LOG_RING_CAP = 32;
static constexpr size_t LOG_LINE_MAX = 160;
static char   g_logRing[LOG_RING_CAP][LOG_LINE_MAX];
static size_t g_logHead  = 0;
static size_t g_logTail  = 0;
static size_t g_logCount = 0;

static void pushLogLine(const char* line) {
  mutex_enter_blocking(&logMutex);
  if (g_logCount == LOG_RING_CAP) {
    g_logTail = (g_logTail + 1) % LOG_RING_CAP;
    g_logCount--;
  }
  snprintf(g_logRing[g_logHead], LOG_LINE_MAX, "%s", line);
  g_logHead = (g_logHead + 1) % LOG_RING_CAP;
  g_logCount++;
  mutex_exit(&logMutex);
}

static void pushLogf(const char* fmt, ...) {
  char buf[LOG_LINE_MAX];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  pushLogLine(buf);
}

// ==============================================================================
// CORE 1: MCP2515, ISO-TP, UDS & POWER MANAGEMENT
// ==============================================================================

static MCP2515* can = nullptr;
static bool     g_diagFilterActive   = false;
static bool     g_normalModeDesired  = false;
static bool     g_hwModeValid        = false;
static bool     g_hwNormalMode       = false;
static uint32_t g_lastBusTrafficMs   = 0;
static uint32_t g_lastIdleProbeMs    = 0;
static uint32_t g_lastInitRetryMs    = 0;
static uint8_t  g_activeTimeouts     = 0;
static uint8_t  g_bootProbesDone     = 0;
static size_t   g_rrIndex            = 0;

enum UdsStatus : uint8_t {
  UDS_OK_SF = 0,
  UDS_OK_FF = 1,
  UDS_ERR_TX = 2,
  UDS_ERR_NRC = 3,
  UDS_ERR_TIMEOUT = 4,
  UDS_ERR_PROTO = 5
};

static uint16_t getVagRxId(uint16_t txId) {
  if (txId >= 0x7E0 && txId <= 0x7E7) {
    return txId + 0x08;
  }
  return txId + 0x6A;
}

static const char* getNrcText(uint8_t nrc) {
  switch (nrc) {
    case 0x10: return "GeneralReject";
    case 0x11: return "ServiceNotSupported";
    case 0x12: return "SubFunctionNotSupported";
    case 0x13: return "IncorrectLengthOrFormat";
    case 0x22: return "ConditionsNotCorrect";
    case 0x24: return "RequestSequenceError";
    case 0x31: return "RequestOutOfRange";
    case 0x33: return "SecurityAccessDenied";
    case 0x35: return "InvalidKey";
    case 0x72: return "GeneralProgrammingFailure";
    case 0x78: return "ResponsePending";
    case 0x7E: return "SubFunctionNotSupportedInActiveSession";
    case 0x7F: return "ServiceNotSupportedInActiveSession";
    default:   return "UnknownNRC";
  }
}

static void setBusState(uint8_t st) {
  BusState next = (BusState)st;
  mutex_enter_blocking(&dataMutex);
  if (g_busState != next) {
    g_busState = next;
    g_dataSeq++;
  }
  mutex_exit(&dataMutex);
}

static void clearTelemetryValues() {
  mutex_enter_blocking(&dataMutex);
  memset(g_didValues, 0, sizeof(g_didValues));
  memset(g_didUpdated, 0, sizeof(g_didUpdated));
  g_dataSeq++;
  mutex_exit(&dataMutex);
}

static void handleCanErrors() {
  if (!can) return;
  uint8_t eflg = can->getErrorFlags();
  if (eflg == 0) return;

  if (eflg & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR)) {
    can->clearRXnOVRFlags();
  }
  if (eflg & MCP2515::EFLG_TXBO) {
    // Recover cleanly from Bus-Off while respecting current passive/normal mode.
    g_hwModeValid = false;
    can->clearInterrupts();
    can->setConfigMode();
    if (g_normalModeDesired) {
      if (can->setNormalMode() == MCP2515::ERROR_OK) {
        g_hwNormalMode = true;
        g_hwModeValid = true;
      }
    } else {
      if (can->setListenOnlyMode() == MCP2515::ERROR_OK) {
        g_hwNormalMode = false;
        g_hwModeValid = true;
      }
    }
  } else {
    can->clearERRIF();
  }
}

static void drainRxBuffer(uint8_t maxFrames) {
  if (!can) return;
  struct can_frame rx;
  uint8_t n = 0;
  while (n < maxFrames && can->readMessage(&rx) == MCP2515::ERROR_OK) {
    if (!(rx.can_id & CAN_RTR_FLAG)) {
      g_lastBusTrafficMs = millis();
    }
    n++;
  }
  handleCanErrors();
}

// Note: setFilterMask / setFilter enter Config mode internally, so caller must
// call setNormalMode() or setListenOnlyMode() immediately after!
static bool setCanFiltersDiagnostic() {
  if (!can) return false;
  g_hwModeValid = false;
  if (can->setFilterMask(MCP2515::MASK0, false, 0x700) != MCP2515::ERROR_OK) return false;
  if (can->setFilterMask(MCP2515::MASK1, false, 0x700) != MCP2515::ERROR_OK) return false;
  const MCP2515::RXF rxf[6] = {
    MCP2515::RXF0, MCP2515::RXF1, MCP2515::RXF2,
    MCP2515::RXF3, MCP2515::RXF4, MCP2515::RXF5
  };
  for (uint8_t i = 0; i < 6; i++) {
    if (can->setFilter(rxf[i], false, 0x700) != MCP2515::ERROR_OK) return false;
  }
  g_diagFilterActive = true;
  return true;
}

static bool setCanFiltersPromiscuous() {
  if (!can) return false;
  g_hwModeValid = false;
  if (can->setFilterMask(MCP2515::MASK0, false, 0x000) != MCP2515::ERROR_OK) return false;
  if (can->setFilterMask(MCP2515::MASK1, false, 0x000) != MCP2515::ERROR_OK) return false;
  const MCP2515::RXF rxf[6] = {
    MCP2515::RXF0, MCP2515::RXF1, MCP2515::RXF2,
    MCP2515::RXF3, MCP2515::RXF4, MCP2515::RXF5
  };
  for (uint8_t i = 0; i < 6; i++) {
    bool ext = (i == 1);
    if (can->setFilter(rxf[i], ext, 0x000) != MCP2515::ERROR_OK) return false;
  }
  g_diagFilterActive = false;
  return true;
}

static void enterListenOnlyPassive() {
  if (!can) return;
  g_normalModeDesired = false;
  if (!g_diagFilterActive && g_hwModeValid && !g_hwNormalMode) return;
  if (g_diagFilterActive) {
    setCanFiltersPromiscuous();
  }
  if (can->setListenOnlyMode() == MCP2515::ERROR_OK) {
    g_hwNormalMode = false;
    g_hwModeValid = true;
  } else {
    g_hwModeValid = false;
  }
  handleCanErrors();
}

static void enterNormalDiagnostic() {
  if (!can) return;
  g_normalModeDesired = true;
  if (g_diagFilterActive && g_hwModeValid && g_hwNormalMode) return;
  if (!g_diagFilterActive) {
    setCanFiltersDiagnostic();
  }
  if (can->setNormalMode() == MCP2515::ERROR_OK) {
    g_hwNormalMode = true;
    g_hwModeValid = true;
  } else {
    g_hwModeValid = false;
  }
  handleCanErrors();
}

static bool initCanController() {
  g_hwModeValid = false;
  g_hwNormalMode = false;
  g_diagFilterActive = false;

  if (can == nullptr) {
    SPI.setRX(PIN_CAN_MISO);
    SPI.setSCK(PIN_CAN_SCK);
    SPI.setTX(PIN_CAN_MOSI);
    SPI.begin();
    static MCP2515 canInstance(PIN_CAN_CS, CAN_SPI_HZ, &SPI);
    can = &canInstance;
  }

  if (can->reset() != MCP2515::ERROR_OK) {
    return false;
  }
  if (can->setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
    return false;
  }
  if (!setCanFiltersDiagnostic()) {
    return false;
  }
  if (can->setNormalMode() != MCP2515::ERROR_OK) {
    return false;
  }
  g_normalModeDesired = true;
  g_hwNormalMode = true;
  g_hwModeValid = true;
  return true;
}

static bool udsStartSession(uint16_t reqId, uint16_t resId, uint8_t sessionType) {
  drainRxBuffer(12);

  struct can_frame tx;
  tx.can_id  = reqId;
  tx.can_dlc = 8;
  tx.data[0] = 0x02;
  tx.data[1] = 0x10;
  tx.data[2] = sessionType;
  for (uint8_t i = 3; i < 8; i++) tx.data[i] = 0x55;

  if (can->sendMessage(&tx) != MCP2515::ERROR_OK) {
    return false;
  }

  uint32_t startMs = millis();
  uint32_t timeoutMs = 120;
  struct can_frame rx;
  while (millis() - startMs < timeoutMs) {
    g_core1HeartbeatMs = millis();
    if (g_isUpdating) return false;
    if (can->readMessage(&rx) == MCP2515::ERROR_OK) {
      g_lastBusTrafficMs = millis();
      if ((rx.can_id & CAN_EFF_FLAG) || (rx.can_id & CAN_RTR_FLAG)) continue;
      if (rx.can_id != resId || rx.can_dlc < 3) continue;
      if ((rx.data[0] >> 4) == 0) {
        if (rx.data[1] == 0x50 && rx.data[2] == sessionType) {
          return true;
        }
        if (rx.data[1] == 0x7F && rx.data[2] == 0x10) {
          if (rx.can_dlc >= 4 && rx.data[3] == 0x78) {
            startMs = millis();
            timeoutMs = 1000;
            continue;
          }
          return false;
        }
      }
    } else {
      handleCanErrors();
      delayMicroseconds(100);
    }
  }
  return false;
}

static bool udsReadDid(uint16_t reqId, uint16_t resId, uint16_t did, uint8_t* outBuf, size_t maxBuf, size_t* outLen, size_t* outTotalLen, uint8_t* outNrc, uint8_t* outStatus, uint32_t baseTimeoutMs) {
  *outLen = 0;
  if (outTotalLen) *outTotalLen = 0;
  *outNrc = 0;
  *outStatus = UDS_ERR_TIMEOUT;

  drainRxBuffer(12);

  struct can_frame tx;
  tx.can_id  = reqId;
  tx.can_dlc = 8;
  tx.data[0] = 0x03;
  tx.data[1] = 0x22;
  tx.data[2] = (uint8_t)((did >> 8) & 0xFF);
  tx.data[3] = (uint8_t)(did & 0xFF);
  for (uint8_t i = 4; i < 8; i++) tx.data[i] = 0x55;

  if (can->sendMessage(&tx) != MCP2515::ERROR_OK) {
    handleCanErrors();
    *outStatus = UDS_ERR_TX;
    return false;
  }

  uint32_t startMs = millis();
  uint32_t timeoutMs = baseTimeoutMs;
  bool     isFirstFrame = false;
  size_t   expectDataLen = 0;
  size_t   rxDataCount = 0;
  uint8_t  expectedSN = 1;
  struct can_frame rx;

  while (millis() - startMs < timeoutMs) {
    g_core1HeartbeatMs = millis();
    if (g_isUpdating) return false;
    if (can->readMessage(&rx) != MCP2515::ERROR_OK) {
      handleCanErrors();
      delayMicroseconds(100);
      continue;
    }

    g_lastBusTrafficMs = millis();
    if ((rx.can_id & CAN_EFF_FLAG) || (rx.can_id & CAN_RTR_FLAG)) continue;
    if (rx.can_id != resId || rx.can_dlc < 2) continue;

    uint8_t pci = rx.data[0] >> 4;

    if (pci == 0) {
      uint8_t sfLen = rx.data[0] & 0x0F;
      if (sfLen < 3 || (uint8_t)(sfLen + 1) > rx.can_dlc) {
        continue;
      }

      if (rx.data[1] == 0x7F && rx.data[2] == 0x22) {
        uint8_t nrc = rx.data[3];
        if (nrc == 0x78) { // ResponsePending: extend deadline
          startMs = millis();
          timeoutMs = 2000;
          continue;
        }
        *outNrc = nrc;
        *outStatus = UDS_ERR_NRC;
        return false;
      }

      if (!isFirstFrame &&
          rx.data[1] == 0x62 &&
          rx.data[2] == tx.data[2] &&
          rx.data[3] == tx.data[3]) {
        size_t dataBytes = (size_t)(sfLen - 3);
        size_t copyBytes = (dataBytes < maxBuf) ? dataBytes : maxBuf;
        for (size_t i = 0; i < copyBytes; i++) {
          outBuf[i] = rx.data[4 + i];
        }
        *outLen = copyBytes;
        if (outTotalLen) *outTotalLen = dataBytes;
        *outStatus = UDS_OK_SF;
        return true;
      }
    } else if (pci == 1 && !isFirstFrame) {
      if (rx.can_dlc < 6) continue;
      uint16_t totalIsoLen = ((uint16_t)(rx.data[0] & 0x0F) << 8) | rx.data[1];
      // ISO 15765-2: First Frame is only valid for payloads >= 8 bytes (1..7 bytes must be SF).
      if (totalIsoLen < 8) continue;

      if (rx.data[2] == 0x62 &&
          rx.data[3] == tx.data[2] &&
          rx.data[4] == tx.data[3]) {
        isFirstFrame = true;
        expectDataLen = (size_t)(totalIsoLen - 3);
        rxDataCount = 0;

        for (uint8_t i = 5; i < rx.can_dlc && rxDataCount < expectDataLen; i++) {
          if (rxDataCount < maxBuf) {
            outBuf[rxDataCount] = rx.data[i];
          }
          rxDataCount++;
        }

        // Request unlimited block size with STmin = 5 ms to avoid RX FIFO overrun
        struct can_frame fc;
        fc.can_id  = reqId;
        fc.can_dlc = 8;
        fc.data[0] = 0x30;
        fc.data[1] = 0x00;
        fc.data[2] = 0x05;
        for (uint8_t k = 3; k < 8; k++) fc.data[k] = 0x55;
        can->sendMessage(&fc);

        expectedSN = 1;
        startMs = millis();
        timeoutMs = 400;

        if (rxDataCount >= expectDataLen) {
          *outLen = (rxDataCount < maxBuf) ? rxDataCount : maxBuf;
          if (outTotalLen) *outTotalLen = expectDataLen;
          *outStatus = UDS_OK_FF;
          return true;
        }
      }
    } else if (pci == 2 && isFirstFrame) {
      uint8_t sn = rx.data[0] & 0x0F;
      if (sn != expectedSN) {
        *outStatus = UDS_ERR_PROTO;
        return false;
      }
      expectedSN = (uint8_t)((expectedSN + 1) & 0x0F);

      for (uint8_t i = 1; i < rx.can_dlc && rxDataCount < expectDataLen; i++) {
        if (rxDataCount < maxBuf) {
          outBuf[rxDataCount] = rx.data[i];
        }
        rxDataCount++;
      }

      startMs = millis();
      if (rxDataCount >= expectDataLen) {
        *outLen = (rxDataCount < maxBuf) ? rxDataCount : maxBuf;
        if (outTotalLen) *outTotalLen = expectDataLen;
        *outStatus = UDS_OK_FF;
        return true;
      }
    }
  }

  return false;
}

static float decodeDidValue(size_t idx, const uint8_t* data, size_t len) {
  const DidDef& def = DID_DEFS[idx];
  uint32_t raw = 0;
  size_t take = (def.byteLen < len) ? def.byteLen : len;
  for (size_t i = 0; i < take && i < 4; i++) {
    raw = (raw << 8) | data[i];
  }

  switch (def.formula) {
    case FMT_U8_M40:          return (float)((uint8_t)raw) - 40.0f;
    case FMT_U16_DIV10_M2731: return ((float)((uint16_t)raw) / 10.0f) - 273.1f;
    case FMT_BOOST_REL:       return (((float)((uint16_t)raw) * 0.25f) - 1000.0f) / 1000.0f;
    case FMT_U16_DIV50:       return (float)((uint16_t)raw) / 50.0f;
    case FMT_U16_DIV100:      return (float)((uint16_t)raw) / 100.0f;
    case FMT_S16_DIV100:      return (float)((int16_t)((uint16_t)raw)) / 100.0f;
    case FMT_U16_DIV10:       return (float)((uint16_t)raw) / 10.0f;
    case FMT_S16_DIV10:       return (float)((int16_t)((uint16_t)raw)) / 10.0f;
    case FMT_U32_DIV1000:     return (float)raw / 1000.0f;
    case FMT_U32_DIV60:       return (float)raw / 60.0f;
    case FMT_U16_DIV8192:     return (float)((uint16_t)raw) / 81.92f;
    case FMT_RAW:
    default:                  return (float)raw;
  }
}

static bool pollDidToShared(size_t idx, uint32_t timeoutMs) {
  if (idx >= NUM_DIDS) return false;

  uint8_t buf[16];
  size_t  rxLen = 0;
  uint8_t nrc = 0;
  uint8_t st = UDS_ERR_TIMEOUT;

  if (!udsReadDid(0x7E0, 0x7E8, DID_DEFS[idx].did, buf, sizeof(buf), &rxLen, nullptr, &nrc, &st, timeoutMs)) {
    return false;
  }
  if (rxLen == 0) return false;

  float val = decodeDidValue(idx, buf, rxLen);

  mutex_enter_blocking(&dataMutex);
  g_didValues[idx] = val;
  g_didUpdated[idx] = true;
  g_dataSeq++;
  mutex_exit(&dataMutex);
  return true;
}

static void executeGatewayScan() {
  pushLogLine("--- VAG MQB MODUL & GATEWAY TARAMASI ---");
  enterNormalDiagnostic();

  uint8_t buf[128];
  size_t rxLen = 0;
  size_t totalLen = 0;
  uint8_t nrc = 0;
  uint8_t st = UDS_ERR_TIMEOUT;

  bool sessOk = udsStartSession(0x710, 0x77A, 0x03);
  pushLogf("[0x710] Extended Session (10 03): %s", sessOk ? "OK" : "Yanit yok/Varsayilan");

  if (udsReadDid(0x710, 0x77A, 0x04A0, buf, sizeof(buf), &rxLen, &totalLen, &nrc, &st, 350)) {
    pushLogf("[0x77A] DID 0x04A0 (%u byte):", (unsigned)rxLen);
    for (size_t off = 0; off < rxLen; off += 16) {
      char hexLine[80];
      size_t pos = 0;
      for (size_t i = off; i < rxLen && i < off + 16; i++) {
        int w = snprintf(hexLine + pos, sizeof(hexLine) - pos, "%02X ", buf[i]);
        if (w > 0) pos += (size_t)w;
      }
      pushLogf("  %s", hexLine);
    }
    if (totalLen > rxLen) {
      pushLogf("  [UYARI] Yanit %u byte, ilk %u byte gosterildi.", (unsigned)totalLen, (unsigned)rxLen);
    }
  } else {
    pushLogf("[0x77A] DID 0x04A0 basarisiz (st=%u, nrc=0x%02X %s)", st, nrc, getNrcText(nrc));
  }

  struct ScanTarget {
    uint16_t txId;
    const char* name;
  };
  static const ScanTarget targets[] = {
    { 0x7E0, "Engine (01)"   },
    { 0x7E1, "Trans (02)"    },
    { 0x713, "ABS/ESP (03)"  },
    { 0x714, "Cluster (17)"  },
    { 0x710, "Gateway (19)"  },
    { 0x765, "CentElec (09)" },
    { 0x715, "HVAC (08)"     },
    { 0x712, "PwrSteer (44)" },
    { 0x745, "Airbag (15)"   },
    { 0x773, "InfoEnt (5F)"  }
  };

  for (size_t t = 0; t < sizeof(targets) / sizeof(targets[0]); t++) {
    g_core1HeartbeatMs = millis();
    if (g_isUpdating) break;
    uint16_t txId = targets[t].txId;
    uint16_t rxId = getVagRxId(txId);
    if (udsReadDid(txId, rxId, 0xF187, buf, sizeof(buf), &rxLen, nullptr, &nrc, &st, 150)) {
      char part[24];
      size_t pLen = 0;
      for (size_t i = 0; i < rxLen && pLen < sizeof(part) - 1; i++) {
        uint8_t ch = buf[i];
        if (ch >= 0x20 && ch <= 0x7E) {
          part[pLen++] = (char)ch;
        }
      }
      part[pLen] = '\0';
      pushLogf("[BULUNDU] %-14s 0x%03X->0x%03X | PN: %s",
               targets[t].name, txId, rxId, pLen ? part : "(Ham)");
    } else if (st == UDS_ERR_NRC) {
      pushLogf("[VAR]     %-14s 0x%03X->0x%03X | NRC: 0x%02X (%s)",
               targets[t].name, txId, rxId, nrc, getNrcText(nrc));
    } else {
      pushLogf("[YOK]     %-14s 0x%03X->0x%03X | Zaman asimi",
               targets[t].name, txId, rxId);
    }
    delay(8);
  }
  pushLogLine("--- TARAMA TAMAMLANDI ---");
}

static void executeDidValidation() {
  pushLogLine("--- 24 ECU DID DOGRULAMASI (0x7E0 -> 0x7E8) ---");
  enterNormalDiagnostic();

  uint8_t buf[16];
  size_t rxLen = 0;
  uint8_t nrc = 0;
  uint8_t st = UDS_ERR_TIMEOUT;
  int okCount = 0;

  for (size_t i = 0; i < NUM_DIDS; i++) {
    g_core1HeartbeatMs = millis();
    if (g_isUpdating) break;
    bool ok = udsReadDid(0x7E0, 0x7E8, DID_DEFS[i].did, buf, sizeof(buf), &rxLen, nullptr, &nrc, &st, 160);
    if (ok) {
      okCount++;
      float val = decodeDidValue(i, buf, rxLen);
      pushLogf("[%s] 0x%04X (%2u B) -> %.2f",
               (st == UDS_OK_FF) ? "OK-FF" : "OK-SF",
               DID_DEFS[i].did, (unsigned)rxLen, val);
    } else if (st == UDS_ERR_NRC) {
      pushLogf("[NRC  ] 0x%04X -> 0x%02X (%s)", DID_DEFS[i].did, nrc, getNrcText(nrc));
    } else if (st == UDS_ERR_PROTO) {
      pushLogf("[PROTO] 0x%04X -> ISO-TP sira hatasi", DID_DEFS[i].did);
    } else {
      pushLogf("[FAIL ] 0x%04X -> Zaman asimi", DID_DEFS[i].did);
    }
    delay(10);
  }
  pushLogf("--- DOGRULAMA BITTI: %d / %u BASARILI ---", okCount, (unsigned)NUM_DIDS);
}

static void executeCustomQuery(uint16_t reqId, uint16_t resId, bool autoRes, uint16_t did) {
  if (reqId < 0x700 || reqId > 0x7EF || resId < 0x700 || resId > 0x7FF) {
    pushLogf("[GECERSIZ] TX=0x%03X RX=0x%03X (Sadece 0x700..0x7FF izinli)", reqId, resId);
    return;
  }

  pushLogLine("--- Ozel UDS Sorgu ---");
  pushLogf("TX: 0x%03X | RX: 0x%03X (%s) | DID: 0x%04X",
           reqId, resId, autoRes ? "Oto" : "Manuel", did);

  enterNormalDiagnostic();

  uint8_t buf[256];
  size_t rxLen = 0;
  size_t totalLen = 0;
  uint8_t nrc = 0;
  uint8_t st = UDS_ERR_TIMEOUT;

  if (udsReadDid(reqId, resId, did, buf, sizeof(buf), &rxLen, &totalLen, &nrc, &st, 800)) {
    pushLogf("[YANIT %s] %u byte:", (st == UDS_OK_FF) ? "Multi-Frame" : "Single-Frame", (unsigned)rxLen);

    for (size_t off = 0; off < rxLen; off += 16) {
      char hexLine[80];
      size_t pos = 0;
      for (size_t i = off; i < rxLen && i < off + 16; i++) {
        int w = snprintf(hexLine + pos, sizeof(hexLine) - pos, "%02X ", buf[i]);
        if (w > 0) pos += (size_t)w;
      }
      pushLogf("  %s", hexLine);
    }

    char ascii[65];
    size_t aPos = 0;
    size_t printable = 0;
    for (size_t i = 0; i < rxLen && aPos < sizeof(ascii) - 1; i++) {
      uint8_t ch = buf[i];
      if (ch >= 0x20 && ch <= 0x7E) {
        ascii[aPos++] = (char)ch;
        printable++;
      } else {
        ascii[aPos++] = '.';
      }
    }
    ascii[aPos] = '\0';
    if (printable >= 3) {
      pushLogf("  [ASCII]: %s", ascii);
    }
    if (totalLen > rxLen) {
      pushLogf("  [UYARI] Yanit %u byte, ilk %u byte gosterildi.", (unsigned)totalLen, (unsigned)rxLen);
    }
  } else if (st == UDS_ERR_NRC) {
    pushLogf("[UDS REDDI]: 7F 22 %02X (%s)", nrc, getNrcText(nrc));
  } else if (st == UDS_ERR_PROTO) {
    pushLogLine("[PROTOKOL HATASI]: ISO-TP ardisik cerceve sirasi bozuldu.");
  } else if (st == UDS_ERR_TX) {
    pushLogLine("[CAN TX HATASI]: Mesaj gonderilemedi (hat / baglanti kontrol edin).");
  } else {
    pushLogf("[ZAMAN ASIMI]: 0x%03X adresinden cevap gelmedi.", resId);
  }
  pushLogLine("--- ISLEM TAMAMLANDI ---");
}

void setup1() {
  g_core1HeartbeatMs = millis();
  if (initCanController()) {
    g_lastBusTrafficMs = 0;
    setBusState(BUS_IDLE);
    // Start passive until first ECU reply or active traffic is seen
    enterListenOnlyPassive();
  } else {
    setBusState(BUS_INIT_FAIL);
    g_lastInitRetryMs = millis();
  }
  g_core1HeartbeatMs = millis();
}

void loop1() {
  g_core1HeartbeatMs = millis();
  if (g_isUpdating) {
    if (!g_core1Paused) {
      enterListenOnlyPassive();
      g_core1Paused = true;
    }
    delay(20);
    return;
  }
  g_core1Paused = false;

  BusState curBus = g_busState;
  uint32_t now = millis();

  if (curBus == BUS_INIT_FAIL) {
    if (now - g_lastInitRetryMs >= 5000UL) {
      g_lastInitRetryMs = now;
      if (initCanController()) {
        setBusState(BUS_IDLE);
        enterListenOnlyPassive();
      }
    }
    delay(50);
    return;
  }

  if (curBus == BUS_SLEEP) {
    // Passive Listen-Only wake monitor: zero TX, zero ACK on CAN bus.
    static uint32_t firstWakeMs = 0;
    static uint8_t  wakeFrames  = 0;
    struct can_frame rx;

    handleCanErrors();
    if (can && can->readMessage(&rx) == MCP2515::ERROR_OK) {
      if (!(rx.can_id & CAN_RTR_FLAG)) {
        if (wakeFrames == 0 || (now - firstWakeMs > WAKE_DEBOUNCE_MS)) {
          firstWakeMs = now;
          wakeFrames = 1;
        } else {
          wakeFrames++;
        }
        if (wakeFrames >= 2) {
          rp2040.reboot();
        }
      }
    }
    delay(25);
    return;
  }

  // Pull pending commands safely
  SystemMode mode;
  bool doScan = false;
  bool doVal = false;
  bool doCustom = false;
  uint16_t cReq = 0x7E0;
  uint16_t cRes = 0x7E8;
  bool     cAuto = true;
  uint16_t cDid = 0x181A;

  mutex_enter_blocking(&cmdMutex);
  mode = g_mode;
  if (g_reqScan)   { doScan = true;   g_reqScan = false; }
  if (g_reqVal)    { doVal = true;    g_reqVal = false; }
  if (g_reqCustom) {
    doCustom = true;
    g_reqCustom = false;
    cReq = g_customReqId;
    cRes = g_customResId;
    cAuto = g_customResAuto;
    cDid = g_customDid;
  }
  mutex_exit(&cmdMutex);

  if (doScan) {
    executeGatewayScan();
    enterListenOnlyPassive();
    return;
  }
  if (doVal) {
    executeDidValidation();
    enterListenOnlyPassive();
    return;
  }
  if (doCustom) {
    executeCustomQuery(cReq, cRes, cAuto, cDid);
    enterListenOnlyPassive();
    return;
  }

  if (mode == MODE_MFD) {
    if (curBus == BUS_ACTIVE) {
      enterNormalDiagnostic();
      bool okBoost = pollDidToShared(IDX_BOOST, 110);

      if (g_rrIndex == IDX_BOOST) g_rrIndex++;
      if (g_rrIndex >= NUM_DIDS) g_rrIndex = 0;
      bool okRr = pollDidToShared(g_rrIndex, 110);
      g_rrIndex++;

      if (okBoost || okRr) {
        g_activeTimeouts = 0;
      } else {
        g_activeTimeouts++;
        if (g_activeTimeouts >= ACTIVE_FAIL_LIMIT) {
          // Ignition OFF or ECU stopped responding: clear stale values & stop TX!
          clearTelemetryValues();
          setBusState(BUS_IDLE);
          enterListenOnlyPassive();
          g_lastIdleProbeMs = millis();
        }
      }
      delay(4);
    } else {
      // BUS_IDLE: passive Listen-Only monitoring; probe only on traffic, <=2 boot checks, or user request.
      drainRxBuffer(8);
      now = millis();

      bool hasRecentTraffic = (g_lastBusTrafficMs > 0) && (now - g_lastBusTrafficMs < IDLE_TRAFFIC_WIN_MS);
      bool allowBootProbe   = (g_lastBusTrafficMs == 0) && (g_bootProbesDone < BOOT_PROBE_MAX);
      bool probeDue         = (g_bootProbesDone == 0 && g_lastBusTrafficMs == 0)
                              ? (now >= 500UL)
                              : (now - g_lastIdleProbeMs >= IDLE_PROBE_MS);
      bool userProbe        = g_probeRequested;

      if (userProbe || ((hasRecentTraffic || allowBootProbe) && probeDue)) {
        g_probeRequested = false;
        g_lastIdleProbeMs = now;
        if (allowBootProbe) g_bootProbesDone++;
        enterNormalDiagnostic();
        if (pollDidToShared(IDX_BOOST, 110)) {
          g_activeTimeouts = 0;
          setBusState(BUS_ACTIVE);
        } else {
          enterListenOnlyPassive();
        }
      }
      delay(20);
    }
  } else {
    // Non-MFD tool tabs: stay completely passive (Listen-Only) between user clicks
    if (g_normalModeDesired) {
      enterListenOnlyPassive();
    } else {
      drainRxBuffer(8);
    }
    delay(20);
  }

  // Global deep-sleep check applies across ALL tabs (MFD / Custom / Scanner / Validator)
  curBus = g_busState;
  now = millis();
  if (curBus != BUS_ACTIVE) {
    bool carSlept = (g_lastBusTrafficMs > 0) && (now - g_lastBusTrafficMs >= SLEEP_AFTER_CAR_MS);
    bool benchIdleSlept = (g_lastBusTrafficMs == 0) && (now >= SLEEP_NO_CAR_MS) && (g_wsClientCount == 0);
    if (carSlept || benchIdleSlept) {
      clearTelemetryValues();
      enterListenOnlyPassive();
      setBusState(BUS_SLEEP);
      g_sleepRequested = true;
    }
  } else if ((g_lastBusTrafficMs > 0) && (now - g_lastBusTrafficMs >= SLEEP_AFTER_CAR_MS)) {
    // Safety fallback if user left a tool tab open while BUS_ACTIVE and car slept
    clearTelemetryValues();
    enterListenOnlyPassive();
    setBusState(BUS_SLEEP);
    g_sleepRequested = true;
  }
}

// ==============================================================================
// CORE 0: EMBEDDED WEB UI (PROGMEM)
// ==============================================================================

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>VAG CAN MFD</title>
    <style>
        :root { --bg: #000; --card: #111; --border: #222; --accent: #fff; --text: #fff; --muted: #888; }
        * { box-sizing: border-box; }
        body { margin: 0; padding: 10px; background: var(--bg); color: var(--text); font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; user-select: none; }
        .header { display: flex; justify-content: space-between; align-items: center; font-size: 13px; color: var(--muted); margin-bottom: 10px; position: relative; gap: 8px; }
        .badges { display: flex; align-items: center; gap: 6px; flex-wrap: wrap; }
        .badge { background: #161616; border: 1px solid #2b2b2b; color: #aaa; padding: 4px 10px; border-radius: 12px; font-weight: 600; font-size: 12px; }
        .badge.ok { color: #4ade80; border-color: #1f4d2e; }
        .badge.warn { color: #fbbf24; border-color: #4d3b14; }
        .badge.err { color: #f87171; border-color: #5c1f1f; }
        .edit-btn { background: #2a2a2a; color: #fff; border: 1px solid #3a3a3a; padding: 5px 13px; border-radius: 14px; cursor: pointer; font-size: 13px; }
        .edit-btn:active { background: #444; }
        #nav_menu { display: none; position: absolute; right: 0; top: 36px; background: #1a1a1a; padding: 4px; border-radius: 12px; z-index: 200; border: 1px solid #333; box-shadow: 0 6px 20px rgba(0,0,0,0.85); width: 185px; }
        .nav-item { padding: 13px; font-size: 14px; color: #fff; border-bottom: 1px solid #2a2a2a; cursor: pointer; text-align: center; }
        .nav-item:last-child { border-bottom: none; }
        .nav-item:active { background: #333; border-radius: 8px; }
        .boost-container { background: var(--card); border-radius: 15px; padding: 18px 20px; text-align: center; margin-bottom: 12px; border: 1px solid var(--border); }
        .boost-lbl { font-size: 13px; color: var(--muted); text-transform: uppercase; letter-spacing: 2px; }
        .boost-val { font-size: 56px; font-weight: 800; line-height: 1.05; margin: 6px 0 4px; font-variant-numeric: tabular-nums; }
        .bar-bg { position: relative; background: #222; height: 14px; border-radius: 7px; margin-top: 12px; overflow: hidden; }
        .bar-zero { position: absolute; left: 23%; top: 0; bottom: 0; width: 2px; background: #666; z-index: 2; }
        .bar-fill-boost { position: absolute; left: 23%; top: 0; bottom: 0; width: 77%; background: var(--accent); transform-origin: left center; transform: scaleX(0); will-change: transform; }
        .bar-fill-vac { position: absolute; right: 77%; top: 0; bottom: 0; width: 23%; background: var(--accent); transform-origin: right center; transform: scaleX(0); will-change: transform; }
        .bar-scale { display: flex; justify-content: space-between; font-size: 11px; color: #555; margin-top: 5px; font-variant-numeric: tabular-nums; }
        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
        .slot { background: var(--card); border-radius: 12px; padding: 14px; display: flex; flex-direction: column; justify-content: center; border: 1px solid var(--border); min-height: 82px; }
        .slot-lbl { font-size: 12px; color: var(--muted); margin-bottom: 5px; }
        .slot-val { font-size: 26px; font-weight: 700; font-variant-numeric: tabular-nums; }
        .dpf-mini-bg { background: #2a2a2a; height: 8px; border-radius: 4px; width: 100%; margin-top: 7px; overflow: hidden; }
        .dpf-mini-fill { background: #fff; height: 100%; width: 0%; transition: width 0.25s; }
        #modal { display: none; position: fixed; top: 0; left: 0; width: 100%; height: 100%; background: rgba(0,0,0,0.92); z-index: 100; padding: 16px; overflow-y: auto; }
        .modal-content { background: var(--card); padding: 18px; border-radius: 14px; border: 1px solid var(--border); margin-bottom: 30px; }
        .modal-title { font-size: 18px; font-weight: 700; margin-bottom: 16px; text-align: center; }
        .slot-select { margin-bottom: 12px; }
        .slot-select label { display: block; margin-bottom: 5px; color: var(--muted); font-size: 13px; }
        select, input[type="text"] { width: 100%; padding: 10px; background: #1d1d1d; color: #fff; border: 1px solid #383838; border-radius: 8px; font-size: 15px; }
        .save-btn { background: #fff; color: #000; width: 100%; padding: 13px; border: none; border-radius: 10px; font-size: 16px; font-weight: 700; margin-top: 14px; cursor: pointer; }
        .view { display: none; }
        .view.active { display: block; }
        .term { background: #0c0c0c; padding: 12px; border-radius: 10px; font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size: 12px; color: #4ade80; height: 48vh; overflow-y: auto; white-space: pre-wrap; border: 1px solid #2b2b2b; word-break: break-word; user-select: text; }
        .action-btn { background: #222; color: #fff; border: 1px solid #3c3c3c; padding: 13px; border-radius: 10px; width: 100%; font-size: 15px; font-weight: 600; margin-bottom: 12px; cursor: pointer; }
        .action-btn:active { background: #3a3a3a; }
        .btn-row { display: flex; gap: 8px; margin-bottom: 12px; }
        .btn-row .action-btn { margin-bottom: 0; }
        .btn-secondary { flex: 0 0 110px; background: #161616; color: #aaa; }
        .presets { display: flex; flex-wrap: wrap; gap: 6px; margin-bottom: 12px; }
        .preset-btn { background: #1d1d1d; color: #ddd; border: 1px solid #333; padding: 6px 10px; border-radius: 8px; font-size: 12px; cursor: pointer; }
        .preset-btn:active { background: #333; }
        .hint { font-size: 12px; color: #4ade80; margin-top: 5px; font-family: ui-monospace, monospace; }
    </style>
</head>
<body>

<div class="header">
    <div class="badges">
        <span id="status" class="badge err">KOPUK</span>
        <span id="can_status" class="badge">CAN: --</span>
    </div>
    <div style="display:flex; gap:8px;">
        <button id="btn_edit" class="edit-btn" onclick="openSettings()">Düzenle</button>
        <button class="edit-btn" onclick="toggleMenu()">☰ Menü</button>
    </div>
    <div id="nav_menu">
        <div class="nav-item" onclick="switchTab('mfd')">Dashboard</div>
        <div class="nav-item" onclick="switchTab('custom')">Özel Sorgu (Raw Hex)</div>
        <div class="nav-item" onclick="switchTab('scanner')">Modül & Gateway Tarayıcı</div>
        <div class="nav-item" onclick="switchTab('validator')">DID Doğrulayıcı (24 DID)</div>
    </div>
</div>

<!-- DASHBOARD VIEW -->
<div id="view_mfd" class="view active">
    <div class="boost-container">
        <div class="boost-lbl">Turbo Basıncı</div>
        <div class="boost-val" id="val_boost">0.00 BAR</div>
        <div class="bar-bg">
            <div class="bar-zero"></div>
            <div class="bar-fill-vac" id="bar_vac"></div>
            <div class="bar-fill-boost" id="bar_boost"></div>
        </div>
        <div class="bar-scale"><span>-0.6</span><span>0.0</span><span>+1.0</span><span>+2.0 BAR</span></div>
    </div>
    <div class="grid" id="main_grid"></div>
</div>

<!-- CUSTOM QUERY VIEW -->
<div id="view_custom" class="view">
    <div class="modal-content" style="margin-top:6px;">
        <div class="presets">
            <button class="preset-btn" onclick="setMod('7E0')">ECU 7E0</button>
            <button class="preset-btn" onclick="setMod('7E1')">TCU 7E1</button>
            <button class="preset-btn" onclick="setMod('710')">Gateway 710</button>
            <button class="preset-btn" onclick="setMod('714')">Kombi 714</button>
            <button class="preset-btn" onclick="setMod('713')">ABS 713</button>
        </div>
        <div class="slot-select">
            <label>Modül TX Adresi (Hex, Örn: 7E0 veya 710)</label>
            <input type="text" id="inp_mod" value="7E0" oninput="updateRxHint()" style="text-transform:uppercase;">
            <div class="hint" id="rx_hint">Beklenen Yanıt (RX): 0x7E8 (Otomatik)</div>
        </div>
        <div class="slot-select">
            <label>Opsiyonel Manuel RX ID (Boş = Otomatik VAG Kuralı)</label>
            <input type="text" id="inp_rx" placeholder="Otomatik (ör: 7E8 veya 77A)" oninput="updateRxHint()" style="text-transform:uppercase;">
        </div>
        <div class="slot-select">
            <label>DID (Veri Kodu - Hex, Örn: 181A, F187, 04A0)</label>
            <input type="text" id="inp_did" value="181A" style="text-transform:uppercase;">
        </div>
        <div class="btn-row">
            <button class="action-btn" onclick="sendCustom()">Sorgula</button>
            <button class="action-btn btn-secondary" onclick="clearTerm('term_custom')">Temizle</button>
        </div>
        <div class="term" id="term_custom" style="height:36vh;">Sonuclar (Ham Hex) burada goruntulenecek...</div>
    </div>
</div>

<!-- SCANNER VIEW -->
<div id="view_scanner" class="view">
    <div class="btn-row">
        <button class="action-btn" onclick="sendCmd('CMD:START_SCAN')">Gateway & Modülleri Tara</button>
        <button class="action-btn btn-secondary" onclick="clearTerm('term_scan')">Temizle</button>
    </div>
    <div class="term" id="term_scan">Loglar bekleniyor...</div>
</div>

<!-- VALIDATOR VIEW -->
<div id="view_validator" class="view">
    <div class="btn-row">
        <button class="action-btn" onclick="sendCmd('CMD:START_VAL')">24 DID'i Doğrula (0x7E0)</button>
        <button class="action-btn btn-secondary" onclick="clearTerm('term_val')">Temizle</button>
    </div>
    <div class="term" id="term_val">Loglar bekleniyor...</div>
</div>

<!-- SETTINGS MODAL -->
<div id="modal">
    <div class="modal-content">
        <div class="modal-title">Ekranı Özelleştir</div>
        <div id="modal_slots"></div>
        <button class="save-btn" onclick="saveSettings()">Kaydet ve Çık</button>
    </div>
</div>

<script>
const DICT = {
    "F405": { name: "Soğutma Suyu", unit: "°C", fmt: v=>v.toFixed(0) },
    "11BE": { name: "Motor Yağı", unit: "°C", fmt: v=>v.toFixed(0) },
    "105E": { name: "Akü Voltajı", unit: "V", fmt: v=>v.toFixed(1) },
    "100C": { name: "Yakıt", unit: "L", fmt: v=>v.toFixed(1) },
    "114F": { name: "Soot Calc", unit: "g", fmt: v=>v.toFixed(2) },
    "114E": { name: "Soot Meas", unit: "g", fmt: v=>v.toFixed(2) },
    "14F5": { name: "DPF dP", unit: "hPa", fmt: v=>v.toFixed(1) },
    "178C": { name: "Oil Ash", unit: "g", fmt: v=>v.toFixed(1) },
    "1156": { name: "Mesafe / Regen", unit: "km", fmt: v=>v.toFixed(0) },
    "115E": { name: "Zaman / Regen", unit: "dk", fmt: v=>v.toFixed(0) },
    "11F6": { name: "DPF Sınır", unit: "g", fmt: v=>v.toFixed(2) },
    "11B2": { name: "EGT 1 (Pre)", unit: "°C", fmt: v=>v.toFixed(0) },
    "10F9": { name: "EGT 2 (Post)", unit: "°C", fmt: v=>v.toFixed(0) },
    "1628": { name: "İntake Temp", unit: "°C", fmt: v=>v.toFixed(0) },
    "163A": { name: "Ana Enj.", unit: "mg", fmt: v=>v.toFixed(2) },
    "163B": { name: "Post Enj 2", unit: "mg", fmt: v=>v.toFixed(2) },
    "167D": { name: "Post Enj 3", unit: "mg", fmt: v=>v.toFixed(2) },
    "1314": { name: "LP-EGR", unit: "%", fmt: v=>v.toFixed(1) },
    "105C": { name: "Şarj Yükü", unit: "%", fmt: v=>v.toFixed(0) },
    "1116": { name: "Radyatör Fanı", unit: "%", fmt: v=>v.toFixed(0) },
    "16A9": { name: "Odometer", unit: "km", fmt: v=>v.toFixed(0) },
    "11B6": { name: "Regen Sayısı", unit: "", fmt: v=>v.toFixed(0) },
    "1347": { name: "Regen Süresi", unit: "dk", fmt: v=>v.toFixed(0) },
    "DPF_PERC": { name: "DPF Doluluk", unit: "%", fmt: v=>v.toFixed(1) }
};

const defaultLayout = ["F405", "11BE", "105E", "DPF_PERC", "11B2", "10F9", "114F", "11F6"];
function loadLayout() {
    try {
        const raw = localStorage.getItem("mfd_layout");
        if (raw) {
            const arr = JSON.parse(raw);
            if (Array.isArray(arr) && arr.length === 8 && arr.every(k => DICT[k])) {
                return arr;
            }
        }
    } catch (e) {}
    return defaultLayout.slice();
}

let currentLayout = loadLayout();
let currentTab = "mfd";
let ws = null;
let reconnectTimer = null;
let lastTelemetry = null;
let targetBoost = 0, currentBoost = 0;

function tabToMode(tab) {
    if (tab === 'scanner') return "1";
    if (tab === 'validator') return "2";
    if (tab === 'custom') return "3";
    return "0";
}

function toggleMenu() {
    const m = document.getElementById('nav_menu');
    m.style.display = (m.style.display === 'block') ? 'none' : 'block';
}

function switchTab(tab) {
    document.querySelectorAll('.view').forEach(v => v.classList.remove('active'));
    document.getElementById('view_' + tab).classList.add('active');
    document.getElementById('btn_edit').style.display = (tab === 'mfd') ? 'inline-block' : 'none';
    document.getElementById('nav_menu').style.display = 'none';
    currentTab = tab;
    sendCmd("MODE:" + tabToMode(tab));
}

function sendCmd(cmd) {
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(cmd);
}

function setMod(hex) {
    document.getElementById('inp_mod').value = hex;
    document.getElementById('inp_rx').value = "";
    updateRxHint();
}

function updateRxHint() {
    const modVal = parseInt(document.getElementById('inp_mod').value.trim(), 16);
    const rxRaw = document.getElementById('inp_rx').value.trim();
    const el = document.getElementById('rx_hint');
    if (isNaN(modVal) || modVal < 0x700 || modVal > 0x7EF) {
        el.innerText = "Geçersiz TX adresi (0x700..0x7EF arası girin)";
        el.style.color = "#f87171";
        return;
    }
    if (rxRaw) {
        const rxVal = parseInt(rxRaw, 16);
        if (isNaN(rxVal) || rxVal < 0x700 || rxVal > 0x7FF) {
            el.innerText = "Geçersiz manuel RX adresi (0x700..0x7FF)";
            el.style.color = "#f87171";
            return;
        }
        el.innerText = "Beklenen Yanıt (RX): 0x" + rxVal.toString(16).toUpperCase() + " (Manuel)";
        el.style.color = "#fbbf24";
        return;
    }
    const autoRx = (modVal >= 0x7E0 && modVal <= 0x7E7) ? (modVal + 8) : (modVal + 0x6A);
    if (autoRx > 0x7FF) {
        el.innerText = "Otomatik RX (0x" + autoRx.toString(16).toUpperCase() + ") > 0x7FF — Manuel RX girin";
        el.style.color = "#f87171";
        return;
    }
    el.innerText = "Beklenen Yanıt (RX): 0x" + autoRx.toString(16).toUpperCase() + " (Otomatik)";
    el.style.color = "#4ade80";
}

function sendCustom() {
    const mod = document.getElementById('inp_mod').value.trim().toUpperCase();
    const did = document.getElementById('inp_did').value.trim().toUpperCase();
    const rx  = document.getElementById('inp_rx').value.trim().toUpperCase();
    if (!mod || !did) return;
    sendCmd(rx ? ("CMD:CST:" + mod + ":" + did + ":" + rx) : ("CMD:CST:" + mod + ":" + did));
}

function clearTerm(id) {
    document.getElementById(id).textContent = "";
}

function appendTerm(id, line) {
    const el = document.getElementById(id);
    if (!el) return;
    let lines = el.textContent ? el.textContent.split('\n') : [];
    lines.push(line);
    if (lines.length > 200) lines = lines.slice(-200);
    el.textContent = lines.join('\n');
    el.scrollTop = el.scrollHeight;
}

function renderGrid() {
    let html = "";
    for (let i = 0; i < 8; i++) {
        const key = currentLayout[i];
        const info = DICT[key] || { name: "--", unit: "", fmt: () => "--" };
        html += `<div class="slot" id="slot_${i}">
            <div class="slot-lbl">${info.name}</div>
            <div class="slot-val" id="val_${i}">-- ${info.unit}</div>
            ${key === 'DPF_PERC' ? `<div class="dpf-mini-bg"><div class="dpf-mini-fill" id="dpf_bar_${i}"></div></div>` : ''}
        </div>`;
    }
    document.getElementById("main_grid").innerHTML = html;
}

function openSettings() {
    let html = "";
    for (let i = 0; i < 8; i++) {
        html += `<div class="slot-select"><label>Kutu ${i + 1}</label><select id="sel_${i}">`;
        for (const [k, v] of Object.entries(DICT)) {
            html += `<option value="${k}" ${currentLayout[i] === k ? "selected" : ""}>${v.name}</option>`;
        }
        html += `</select></div>`;
    }
    document.getElementById("modal_slots").innerHTML = html;
    document.getElementById("modal").style.display = "block";
}

function saveSettings() {
    for (let i = 0; i < 8; i++) {
        currentLayout[i] = document.getElementById(`sel_${i}`).value;
    }
    try { localStorage.setItem("mfd_layout", JSON.stringify(currentLayout)); } catch (e) {}
    document.getElementById("modal").style.display = "none";
    renderGrid();
    if (lastTelemetry) applyTelemetry(lastTelemetry);
}

function updateCanBadge(s) {
    const el = document.getElementById("can_status");
    if (s === 0) { el.innerText = "CAN: AKTİF"; el.className = "badge ok"; }
    else if (s === 1) { el.innerText = "CAN: BEKLEME"; el.className = "badge warn"; }
    else if (s === 2) { el.innerText = "CAN: UYKU"; el.className = "badge warn"; }
    else if (s === 3) { el.innerText = "CAN: HATA"; el.className = "badge err"; }
}

function applyTelemetry(j) {
    lastTelemetry = j;
    if (j.S !== undefined) updateCanBadge(j.S);
    if (currentTab !== 'mfd') return;

    if (j.B !== undefined) {
        targetBoost = j.B;
        const prefix = targetBoost > 0.005 ? "+" : "";
        document.getElementById("val_boost").innerText = prefix + targetBoost.toFixed(2) + " BAR";
    } else if (j.S !== undefined && j.S !== 0) {
        targetBoost = 0;
        document.getElementById("val_boost").innerText = "--.-- BAR";
    }
    if (j.D !== undefined) {
        const d = j.D;
        if (j.DPF !== undefined) d["DPF_PERC"] = j.DPF;
        for (let i = 0; i < 8; i++) {
            const key = currentLayout[i];
            const info = DICT[key];
            if (!info) continue;
            const valEl = document.getElementById(`val_${i}`);
            if (d[key] !== undefined) {
                valEl.innerText = info.fmt(d[key]) + (info.unit ? " " + info.unit : "");
                if (key === "DPF_PERC") {
                    const pct = Math.max(0, Math.min(100, d[key]));
                    const bar = document.getElementById(`dpf_bar_${i}`);
                    if (bar) {
                        bar.style.width = pct + "%";
                        bar.style.background = pct > 90 ? "#ef4444" : (pct > 70 ? "#f59e0b" : "#fff");
                    }
                }
            } else {
                valEl.innerText = "--" + (info.unit ? " " + info.unit : "");
            }
        }
    }
}

function scheduleReconnect() {
    if (reconnectTimer) return;
    reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        connectWS();
    }, 1000);
}

function connectWS() {
    ws = new WebSocket("ws://" + location.hostname + ":81");
    ws.onopen = () => {
        const st = document.getElementById("status");
        st.innerText = "BAĞLI";
        st.className = "badge ok";
        sendCmd("MODE:" + tabToMode(currentTab));
    };
    ws.onclose = () => {
        const st = document.getElementById("status");
        st.innerText = "KOPUK";
        st.className = "badge err";
        scheduleReconnect();
    };
    ws.onerror = () => {
        if (ws && ws.readyState === WebSocket.OPEN) ws.close();
    };
    ws.onmessage = (e) => {
        if (e.data.charCodeAt(0) === 123) { // '{'
            try { applyTelemetry(JSON.parse(e.data)); } catch (err) {}
        } else {
            if (currentTab === 'scanner') appendTerm("term_scan", e.data);
            else if (currentTab === 'validator') appendTerm("term_val", e.data);
            else if (currentTab === 'custom') appendTerm("term_custom", e.data);
        }
    };
}

const elBarBoost = document.getElementById("bar_boost");
const elBarVac   = document.getElementById("bar_vac");

function anim() {
    currentBoost += (targetBoost - currentBoost) * 0.18;
    if (currentBoost >= 0) {
        const s = Math.min(1, currentBoost / 2.0);
        elBarBoost.style.transform = `scaleX(${s.toFixed(4)})`;
        elBarVac.style.transform = "scaleX(0)";
    } else {
        const s = Math.min(1, Math.abs(currentBoost) / 0.6);
        elBarVac.style.transform = `scaleX(${s.toFixed(4)})`;
        elBarBoost.style.transform = "scaleX(0)";
    }
    requestAnimationFrame(anim);
}

renderGrid();
updateRxHint();
connectWS();
requestAnimationFrame(anim);
</script>
</body>
</html>
)rawliteral";

// ==============================================================================
// CORE 0: WEBSERVER, WEBSOCKETS, OTA & SLEEP ORCHESTRATION
// ==============================================================================

static WebServer server(80);
static WebSocketsServer webSocket(81);

static uint32_t g_lastWsSendMs     = 0;
static uint32_t g_lastWsHeartbeat  = 0;
static uint32_t g_lastSentSeq      = 0;
static bool     g_otaInitialized   = false;

static void handleRoot() {
  server.send_P(200, PSTR("text/html; charset=utf-8"), INDEX_HTML, sizeof(INDEX_HTML) - 1);
}

static bool parseHexU16(const char* str, uint16_t* outVal) {
  if (!str || *str == '\0') return false;
  char* endPtr = nullptr;
  unsigned long val = strtoul(str, &endPtr, 16);
  if (endPtr == str || *endPtr != '\0' || val > 0xFFFFUL) {
    return false;
  }
  *outVal = (uint16_t)val;
  return true;
}

static void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  (void)num;
  if (type == WStype_CONNECTED || type == WStype_DISCONNECTED) {
    g_wsClientCount = (uint8_t)webSocket.connectedClients(false);
    return;
  }
  if (type != WStype_TEXT || length == 0 || payload == nullptr) return;

  char cmdBuf[64];
  size_t copyLen = (length < sizeof(cmdBuf) - 1) ? length : (sizeof(cmdBuf) - 1);
  memcpy(cmdBuf, payload, copyLen);
  cmdBuf[copyLen] = '\0';

  if (strncmp(cmdBuf, "MODE:", 5) == 0) {
    int m = atoi(cmdBuf + 5);
    if (m >= 0 && m <= 3) {
      mutex_enter_blocking(&cmdMutex);
      g_mode = (SystemMode)m;
      mutex_exit(&cmdMutex);
      if (m == MODE_MFD) {
        g_probeRequested = true;
      }
    }
  } else if (strcmp(cmdBuf, "CMD:START_SCAN") == 0) {
    mutex_enter_blocking(&cmdMutex);
    g_mode = MODE_SCAN;
    g_reqScan = true;
    mutex_exit(&cmdMutex);
  } else if (strcmp(cmdBuf, "CMD:START_VAL") == 0) {
    mutex_enter_blocking(&cmdMutex);
    g_mode = MODE_VALIDATE;
    g_reqVal = true;
    mutex_exit(&cmdMutex);
  } else if (strncmp(cmdBuf, "CMD:CST:", 8) == 0) {
    // Format: CMD:CST:<MOD_HEX>:<DID_HEX>[:<RX_HEX>]
    char* p1 = cmdBuf + 8;
    char* c1 = strchr(p1, ':');
    if (!c1) return;
    *c1 = '\0';

    char* p2 = c1 + 1;
    char* c2 = strchr(p2, ':');
    char* p3 = nullptr;
    if (c2) {
      *c2 = '\0';
      p3 = c2 + 1;
    }

    uint16_t reqId = 0, did = 0, resId = 0;
    if (!parseHexU16(p1, &reqId) || !parseHexU16(p2, &did)) {
      pushLogLine("[HATA] Gecersiz Hex modul veya DID formati.");
      return;
    }
    bool autoRes = true;
    if (p3 && *p3 != '\0') {
      if (!parseHexU16(p3, &resId)) {
        pushLogLine("[HATA] Gecersiz Hex manuel RX formati.");
        return;
      }
      autoRes = false;
    } else {
      resId = getVagRxId(reqId);
    }

    mutex_enter_blocking(&cmdMutex);
    g_mode = MODE_CUSTOM;
    g_customReqId = reqId;
    g_customResId = resId;
    g_customResAuto = autoRes;
    g_customDid = did;
    g_reqCustom = true;
    mutex_exit(&cmdMutex);
  }
}

static void broadcastTelemetryIfDue(uint32_t now) {
  if (g_wsClientCount == 0) return;

  bool elapsed30Hz = (now - g_lastWsSendMs >= WS_BROADCAST_MS);
  bool elapsedHb   = (now - g_lastWsHeartbeat >= WS_HEARTBEAT_MS);
  if (!elapsed30Hz && !elapsedHb) return;

  float    vals[NUM_DIDS];
  bool     upd[NUM_DIDS];
  uint32_t seq;
  BusState bState;

  mutex_enter_blocking(&dataMutex);
  memcpy(vals, g_didValues, sizeof(vals));
  memcpy(upd, g_didUpdated, sizeof(upd));
  seq = g_dataSeq;
  bState = g_busState;
  mutex_exit(&dataMutex);

  bool changed = (seq != g_lastSentSeq);
  if (!changed && !elapsedHb) return;

  g_lastWsSendMs = now;
  if (elapsedHb) g_lastWsHeartbeat = now;
  g_lastSentSeq = seq;

  static char jsonBuf[1024];
  int pos = 0;
  int rem = (int)sizeof(jsonBuf);

  int n = snprintf(jsonBuf + pos, rem, "{\"S\":%u", (unsigned)bState);
  if (n > 0 && n < rem) { pos += n; rem -= n; }

  if (upd[IDX_BOOST]) {
    n = snprintf(jsonBuf + pos, rem, ",\"B\":%.2f", vals[IDX_BOOST]);
    if (n > 0 && n < rem) { pos += n; rem -= n; }
  }

  if (upd[IDX_SOOT_CALC] && upd[IDX_DPF_LIMIT] && vals[IDX_DPF_LIMIT] > 0.0f) {
    float dpfPct = (vals[IDX_SOOT_CALC] / vals[IDX_DPF_LIMIT]) * 100.0f;
    n = snprintf(jsonBuf + pos, rem, ",\"DPF\":%.1f", dpfPct);
    if (n > 0 && n < rem) { pos += n; rem -= n; }
  }

  n = snprintf(jsonBuf + pos, rem, ",\"D\":{");
  if (n > 0 && n < rem) { pos += n; rem -= n; }

  bool first = true;
  for (size_t i = 0; i < NUM_DIDS; i++) {
    if (!upd[i]) continue;
    n = snprintf(jsonBuf + pos, rem, "%s\"%s\":%.2f",
                 first ? "" : ",", DID_DEFS[i].hexKey, vals[i]);
    if (n <= 0 || n >= rem) break;
    pos += n;
    rem -= n;
    first = false;
  }

  n = snprintf(jsonBuf + pos, rem, "}}");
  if (n > 0 && n < rem) { pos += n; }

  webSocket.broadcastTXT(jsonBuf, (size_t)pos);
}

static void flushLogQueue() {
  char batch[8][LOG_LINE_MAX];
  size_t count = 0;

  mutex_enter_blocking(&logMutex);
  while (g_logCount > 0 && count < 8) {
    memcpy(batch[count], g_logRing[g_logTail], LOG_LINE_MAX);
    g_logTail = (g_logTail + 1) % LOG_RING_CAP;
    g_logCount--;
    count++;
  }
  mutex_exit(&logMutex);

  for (size_t i = 0; i < count; i++) {
    webSocket.broadcastTXT(batch[i]);
  }
}

void setup() {
  g_core1HeartbeatMs = millis();
  rp2040.wdt_begin(WDT_TIMEOUT_MS);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", handleRoot);
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  ArduinoOTA.setHostname("vag-mfd-pico");
  ArduinoOTA.onStart([]() {
    g_isUpdating = true;
    uint32_t waitStart = millis();
    while (!g_core1Paused && (millis() - waitStart < 250UL)) {
      rp2040.wdt_reset();
      delay(5);
    }
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    (void)progress;
    (void)total;
    rp2040.wdt_reset();
  });
  ArduinoOTA.onEnd([]() {
    rp2040.wdt_reset();
    g_isUpdating = false;
  });
  ArduinoOTA.onError([](ota_error_t err) {
    (void)err;
    rp2040.wdt_reset();
    g_isUpdating = false;
  });
  ArduinoOTA.begin();
  g_otaInitialized = true;
}

void loop() {
  uint32_t core1Hb = g_core1HeartbeatMs;
  uint32_t loopNow = millis();
  if (g_isUpdating || ((int32_t)(loopNow - core1Hb) < (int32_t)CORE1_STALL_MS)) {
    rp2040.wdt_reset();
  }

  if (g_core0Sleeping) {
    delay(200);
    return;
  }

  if (g_sleepRequested) {
    g_core0Sleeping = true;
    if (g_otaInitialized) {
      ArduinoOTA.end();
      g_otaInitialized = false;
    }
    webSocket.close();
    server.close();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    return;
  }

  ArduinoOTA.handle();
  server.handleClient();
  webSocket.loop();

  g_wsClientCount = (uint8_t)webSocket.connectedClients(false);

  flushLogQueue();
  broadcastTelemetryIfDue(millis());
}
