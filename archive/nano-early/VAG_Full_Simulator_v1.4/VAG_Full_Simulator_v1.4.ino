/* ==========================================================================
 * Proje: VAG MQB 5-Modül Simülatör (UDS 0x19 DTC + DPF Ash Destekli) (v1.4)
 * ========================================================================== */

#include <SPI.h>
#include <mcp_can.h>

#define PIN_CAN_CS 10
MCP_CAN CAN0(PIN_CAN_CS);

#define ECU_REQ_ID      0x7E0
#define ECU_RESP_ID     0x7E8
#define TCU_REQ_ID      0x7E1
#define TCU_RESP_ID     0x7E9
#define ABS_REQ_ID      0x7E2
#define ABS_RESP_ID     0x7EA
#define CLUSTER_REQ_ID  0x714
#define CLUSTER_RESP_ID 0x77E
#define GW_REQ_ID       0x710
#define GW_RESP_ID      0x77A

const float   SIM_DPF_ASH_MASS_G = 41.25;
const uint16_t SIM_LAST_REGEN_KM = 385;

float   sim_turbo_bar   = 0.0;
float   sim_dpf_soot    = 23.4;
int16_t sim_oil_temp    = 91;
int16_t sim_dsg_temp    = 83;
int16_t sim_egt_temp    = 310;
bool    sim_in_regen    = false;
bool    sim_has_fault   = false; // Sahte arıza kodu bayrağı (P0234 testi için)

uint32_t last_sim_tick  = 0;
float    sim_angle      = 0.0;

void sendISOTPString(uint32_t resp_id, uint32_t req_id, uint16_t did, const char* str) {
  uint8_t str_len = strlen(str);
  uint16_t total = 3 + str_len;

  if (total <= 7) {
    uint8_t sf[8] = {(uint8_t)total, 0x62, (uint8_t)(did >> 8), (uint8_t)(did & 0xFF), 0x55, 0x55, 0x55, 0x55};
    for (uint8_t i = 0; i < str_len; i++) sf[4 + i] = str[i];
    CAN0.sendMsgBuf(resp_id, 0, 8, sf);
  } else {
    uint8_t ff[8] = {(uint8_t)(0x10 | ((total >> 8) & 0x0F)), (uint8_t)(total & 0xFF), 0x62, (uint8_t)(did >> 8), (uint8_t)(did & 0xFF), (uint8_t)str[0], (uint8_t)str[1], (uint8_t)str[2]};
    CAN0.sendMsgBuf(resp_id, 0, 8, ff);

    uint32_t start_w = millis();
    bool fc = false;
    while (millis() - start_w < 300) {
      if (CAN0.checkReceive() == CAN_MSGAVAIL) {
        uint32_t rx_id; uint8_t len; uint8_t buf[8];
        CAN0.readMsgBuf(&rx_id, &len, buf);
        if (rx_id == req_id && (buf[0] & 0xF0) == 0x30) { fc = true; break; }
      }
    }
    if (!fc) return;

    uint8_t bytes_sent = 3; uint8_t seq = 1;
    while (bytes_sent < str_len) {
      uint8_t cf[8] = {(uint8_t)(0x20 | (seq & 0x0F)), 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
      for (uint8_t i = 1; i < 8 && bytes_sent < str_len; i++) cf[i] = str[bytes_sent++];
      seq = (seq + 1) & 0x0F;
      CAN0.sendMsgBuf(resp_id, 0, 8, cf);
      delay(5);
    }
  }
}

// UDS 0x19 DTC Cevaplayıcı (Usulüne uygun 0x59 pozitif yanıtı)
void sendDTCResponse(uint32_t resp_id, bool with_fault) {
  if (!with_fault) {
    // 0 Adet Arıza Kodu (Temiz Modül): [Boyut=3, Service=0x59, Sub=0x02, Mask=0xFF]
    uint8_t clean_resp[8] = {0x03, 0x59, 0x02, 0xFF, 0x55, 0x55, 0x55, 0x55};
    CAN0.sendMsgBuf(resp_id, 0, 8, clean_resp);
  } else {
    // 1 Adet Arıza Kodu: P0234 (Turbo Aşırı Basınç) -> Byte[4]=0x02, Byte[5]=0x34, Status=0x2F
    uint8_t fault_resp[8] = {0x07, 0x59, 0x02, 0xFF, 0x02, 0x34, 0x00, 0x2F};
    CAN0.sendMsgBuf(resp_id, 0, 8, fault_resp);
  }
}

void updateDynamicCarPhysics() {
  uint32_t now = millis();
  if (now - last_sim_tick >= 30) {
    last_sim_tick = now;
    sim_angle += 0.06;
    if (sim_angle > 6.28) sim_angle = 0.0;
    float raw_sin = sin(sim_angle);
    sim_turbo_bar = (raw_sin > 0) ? (raw_sin * 1.55) : 0.05;

    static uint32_t regen_timer = 0;
    if (now - regen_timer > 1000) {
      regen_timer = now;
      if (sim_dpf_soot >= 24.0 && !sim_in_regen) {
        sim_in_regen = true;
      }
      if (sim_in_regen) {
        sim_egt_temp = 620;
        sim_dpf_soot -= 0.35;
        if (sim_dpf_soot <= 3.5) sim_in_regen = false;
      } else {
        sim_egt_temp = 280 + (int16_t)(sim_turbo_bar * 120.0);
        sim_dpf_soot += 0.04;
      }
    }
  }
}

void handleIncomingRequests() {
  if (CAN0.checkReceive() == CAN_MSGAVAIL) {
    uint32_t rx_id; uint8_t len; uint8_t buf[8];
    CAN0.readMsgBuf(&rx_id, &len, buf);

    // 1. MOTOR BEYNİ (0x7E0)
    if (rx_id == ECU_REQ_ID) {
      if (buf[1] == 0x19) { // UDS 0x19 DTC İstegi
        sendDTCResponse(ECU_RESP_ID, sim_has_fault);
      }
      else if (buf[1] == 0x22) {
        uint16_t did = ((uint16_t)buf[2] << 8) | buf[3];
        if (did == 0x202E) {
          uint16_t raw_ash = (uint16_t)(SIM_DPF_ASH_MASS_G * 100.0);
          uint8_t resp[8] = {0x05, 0x62, 0x20, 0x2E, (uint8_t)(raw_ash >> 8), (uint8_t)(raw_ash & 0xFF), 0x55, 0x55};
          CAN0.sendMsgBuf(ECU_RESP_ID, 0, 8, resp);
        }
        else if (did == 0x2029) {
          uint16_t raw_soot = (uint16_t)(sim_dpf_soot * 100.0);
          uint8_t resp[8] = {0x05, 0x62, 0x20, 0x29, (uint8_t)(raw_soot >> 8), (uint8_t)(raw_soot & 0xFF), 0x55, 0x55};
          CAN0.sendMsgBuf(ECU_RESP_ID, 0, 8, resp);
        }
        else if (did == 0x202A) {
          uint16_t raw_meas = (uint16_t)(sim_dpf_soot * 0.88 * 100.0);
          uint8_t resp[8] = {0x05, 0x62, 0x20, 0x2A, (uint8_t)(raw_meas >> 8), (uint8_t)(raw_meas & 0xFF), 0x55, 0x55};
          CAN0.sendMsgBuf(ECU_RESP_ID, 0, 8, resp);
        }
        else if (did == 0x2035) {
          uint8_t resp[8] = {0x05, 0x62, 0x20, 0x35, (uint8_t)(SIM_LAST_REGEN_KM >> 8), (uint8_t)(SIM_LAST_REGEN_KM & 0xFF), 0x55, 0x55};
          CAN0.sendMsgBuf(ECU_RESP_ID, 0, 8, resp);
        }
        else if (did == 0x115B) {
          uint16_t mbar = 1013 + (uint16_t)(sim_turbo_bar * 1000.0);
          uint8_t resp[8] = {0x05, 0x62, 0x11, 0x5B, (uint8_t)(mbar >> 8), (uint8_t)(mbar & 0xFF), 0x55, 0x55};
          CAN0.sendMsgBuf(ECU_RESP_ID, 0, 8, resp);
        }
        else if (did == 0x2045) {
          uint16_t raw_egt = (uint16_t)((sim_egt_temp + 40) * 10);
          uint8_t resp[8] = {0x05, 0x62, 0x20, 0x45, (uint8_t)(raw_egt >> 8), (uint8_t)(raw_egt & 0xFF), 0x55, 0x55};
          CAN0.sendMsgBuf(ECU_RESP_ID, 0, 8, resp);
        }
        else if (did == 0x2024) {
          uint8_t resp[8] = {0x04, 0x62, 0x20, 0x24, (uint8_t)(sim_oil_temp + 40), 0x55, 0x55, 0x55};
          CAN0.sendMsgBuf(ECU_RESP_ID, 0, 8, resp);
        }
        else if (did == 0xF187) sendISOTPString(ECU_RESP_ID, ECU_REQ_ID, 0xF187, "04L906021M");
        else if (did == 0xF197) sendISOTPString(ECU_RESP_ID, ECU_REQ_ID, 0xF197, "R4 1.6l TDI EDC17C64");
        else if (did == 0xF189) sendISOTPString(ECU_RESP_ID, ECU_REQ_ID, 0xF189, "9970");
      }
    }
    // 2. SANZIMAN (0x7E1)
    else if (rx_id == TCU_REQ_ID) {
      if (buf[1] == 0x19) sendDTCResponse(TCU_RESP_ID, false);
      else if (buf[1] == 0x22) {
        uint16_t did = ((uint16_t)buf[2] << 8) | buf[3];
        if (did == 0x1000) {
          uint8_t resp[8] = {0x04, 0x62, 0x10, 0x00, (uint8_t)(sim_dsg_temp + 40), 0x55, 0x55, 0x55};
          CAN0.sendMsgBuf(TCU_RESP_ID, 0, 8, resp);
        }
        else if (did == 0xF187) sendISOTPString(TCU_RESP_ID, TCU_REQ_ID, 0xF187, "0CW300047C");
        else if (did == 0xF197) sendISOTPString(TCU_RESP_ID, TCU_REQ_ID, 0xF197, "GSG DQ200-MQB");
        else if (did == 0xF189) sendISOTPString(TCU_RESP_ID, TCU_REQ_ID, 0xF189, "6502");
      }
    }
    // 3. ABS/ESP (0x7E2)
    else if (rx_id == ABS_REQ_ID) {
      if (buf[1] == 0x19) sendDTCResponse(ABS_RESP_ID, false);
      else if (buf[1] == 0x22) {
        uint16_t did = ((uint16_t)buf[2] << 8) | buf[3];
        if (did == 0xF187) sendISOTPString(ABS_RESP_ID, ABS_REQ_ID, 0xF187, "5Q0614517AH");
        else if (did == 0xF197) sendISOTPString(ABS_RESP_ID, ABS_REQ_ID, 0xF197, "ESC MK100-MQB");
        else if (did == 0xF189) sendISOTPString(ABS_RESP_ID, ABS_REQ_ID, 0xF189, "0122");
      }
    }
    // 4. GOSTERGE (0x714)
    else if (rx_id == CLUSTER_REQ_ID) {
      if (buf[1] == 0x19) sendDTCResponse(CLUSTER_RESP_ID, false);
      else if (buf[1] == 0x22) {
        uint16_t did = ((uint16_t)buf[2] << 8) | buf[3];
        if (did == 0xF187) sendISOTPString(CLUSTER_RESP_ID, CLUSTER_REQ_ID, 0xF187, "5E0920781A");
        else if (did == 0xF197) sendISOTPString(CLUSTER_RESP_ID, CLUSTER_REQ_ID, 0xF197, "KOMBI MQB-High");
        else if (did == 0xF189) sendISOTPString(CLUSTER_RESP_ID, CLUSTER_REQ_ID, 0xF189, "8420");
      }
    }
    // 5. GATEWAY (0x710)
    else if (rx_id == GW_REQ_ID) {
      if (buf[1] == 0x19) sendDTCResponse(GW_RESP_ID, false);
      else if (buf[1] == 0x22) {
        uint16_t did = ((uint16_t)buf[2] << 8) | buf[3];
        if (did == 0xF187) sendISOTPString(GW_RESP_ID, GW_REQ_ID, 0xF187, "5Q0907530AF");
        else if (did == 0xF197) sendISOTPString(GW_RESP_ID, GW_REQ_ID, 0xF197, "GW MQB-High");
        else if (did == 0xF189) sendISOTPString(GW_RESP_ID, GW_REQ_ID, 0xF189, "2214");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CAN_CS, OUTPUT);
  digitalWrite(PIN_CAN_CS, HIGH);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN0.setMode(MCP_NORMAL);
    Serial.println(F("[SIMULATOR v1.4] 5 Modül + UDS 0x19 DTC Motoru Aktif!"));
    Serial.println(F("[KOMUTLAR]"));
    Serial.println(F("  'e' -> Sahte Arıza Kodu Enjekte Et (P0234 - Turbo Overboost)"));
    Serial.println(F("  'c' -> Arıza Kodlarını Temizle (0 DTC)"));
    Serial.println(F("  'r' -> DPF Rejenerasyonunu Tetikle"));
  }
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'e' || c == 'E') {
      sim_has_fault = true;
      Serial.println(F("[TEST] Motora Sahte Ariza Kodu Enjekte Edildi: P0234"));
    } else if (c == 'c' || c == 'C') {
      sim_has_fault = false;
      Serial.println(F("[TEST] Ariza Kodlari Temizlendi (0 DTC)."));
    } else if (c == 'r' || c == 'R') {
      sim_dpf_soot = 24.5;
      Serial.println(F("[MANUEL] DPF Rejenerasyonu Tetiklendi!"));
    }
  }
  updateDynamicCarPhysics();
  handleIncomingRequests();
}