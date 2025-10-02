// TetherIA - Open Source Hand
// Aero Hand Firmware Source Code
#include <Arduino.h>
#include <Wire.h>
#include <HLSCL.h>
#include <Preferences.h>
#include "HandConfig.h"
#include "Homing.h"

HLSCL hlscl;
Preferences prefs;

// ---- UART pins to the servo bus (ESP32-S3 XIAO: RX=2, TX=3) ----
#define SERIAL2_TX_PIN 3
#define SERIAL2_RX_PIN 2

// ---- Servo IDs (declare at top) ----
const uint8_t SERVO_IDS[7] = { 0, 1, 2, 3, 4, 5, 6 };

ServoData sd[7];

// ---- Constants for Control Code byte ----
static const uint8_t HOMING   = 0x01;
static const uint8_t ZERO_ALL = 0x02;
static const uint8_t SET_ID   = 0x03;
static const uint8_t TRIM     = 0x04;
static const uint8_t CTRL_POS = 0x11;
static const uint8_t GET_ALL  = 0x21;
static const uint8_t GET_POS  = 0x22;
static const uint8_t GET_VEL  = 0x23;
static const uint8_t GET_CURR  = 0x24;
static const uint8_t GET_TEMP  = 0x25;

// ---- Defaults for SyncWritePosEx ----
static uint16_t g_speed[7]  = {2400,2400,2400,2400,2400,2400,2400};
static uint8_t  g_accel[7]  = {255,255,255,255,255,255,255};     // 0..255
static uint16_t g_torque[7] = {1023,1023,1023,1023,1023,1023,1023};

// ----- Registers / constants (Mapped as per Feetech Servo HLS3606M) -----
#define REG_ID                 0x05       // ID register
#define REG_CURRENT_LIMIT      28         // decimal address (word)
#define BROADCAST_ID           0xFE
#define SCAN_MIN               0
#define SCAN_MAX               253
#define REG_BLOCK_LEN          15
#define REG_BLOCK_START        56

// ----- Structure for the Metrics of Servo -------
struct ServoMetrics {
  uint16_t pos[7];
  uint16_t vel[7];
  uint16_t cur[7];
  uint16_t tmp[7];
};
static ServoMetrics gMetrics;

//Initialise Servodata sd Once at startup only
struct SdInitOnce {
  SdInitOnce() { resetSdToBaseline(); } 
} _sd_init_once;

// ----- Semaphores for Metrics and Bus for acquiring lock and release it -----
static SemaphoreHandle_t gMetricsMux;
SemaphoreHandle_t gBusMux = nullptr;

// ----- Homing module API Calls (provided below as .h/.cpp) --------
bool HOMING_isBusy();
void HOMING_start();

// ---- Set-ID helpers for setting ID ---
extern void runReIdScanAndSet(uint8_t Id, uint16_t currentLimit);
static volatile int g_lastFoundId; 

// ----- Helper Functions for Set-ID Mode -----
static int scanFirstServoId() {
  for (int id = SCAN_MIN; id <= SCAN_MAX; ++id) {
    if (id == BROADCAST_ID) continue;
    int r = hlscl.Ping((uint8_t)id);
    if (!hlscl.getLastError()) return id;
  }
  return -1;
}
static void sendSetIdAck(uint8_t oldId, uint8_t newId, uint16_t curLimitWord) {
  uint16_t vals[7] = {0};
  vals[0] = oldId;
  vals[1] = newId;
  vals[2] = curLimitWord;
  uint8_t out[2 + 7*2];
  out[0] = SET_ID;  // 0x03
  out[1] = 0x00;    // filler
  for (int i = 0; i < 7; ++i) {
    out[2 + 2*i + 0] = (uint8_t)(vals[i] & 0xFF);
    out[2 + 2*i + 1] = (uint8_t)((vals[i] >> 8) & 0xFF);
  }
  Serial.write(out, sizeof(out));
}
// ---- Helper function to loadManual extends from NVS ------
static void loadManualExtendsFromNVS() {
  prefs.begin("hand", true);  // read-only
  for (uint8_t i = 0; i < 7; ++i) {
    uint8_t ch = SERVO_IDS[i];
    String key = "ext" + String(ch);
    int v = prefs.getInt(key.c_str(), -1);
    if (v >= 0 && v <= 4095) {
      sd[ch].extend_count = v;
      //Serial.printf("[NVS] ext override id=%u <- %d\n", ch, v);
    }
  }
}

// ---- Helper function for u16 to raw 4095 ----
static inline uint16_t u16_to_raw4095(uint16_t u) {
  // Linear map: 0..65535  ->  0..4095
  // Use 32-bit math to avoid overflow, then clamp just in case.
  uint32_t raw = (uint32_t)u * 4095u / 65535u;
  if (raw > 4095u) raw = 4095u;
  return (uint16_t)raw;
}

// ---- Helper Functions for u16, Decode to sign and copy values in u16 format----
static inline uint16_t leu_u16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline int16_t decode_signmag15(uint8_t lo, uint8_t hi) {
  uint16_t mag = ((uint16_t)(hi & 0x7F) << 8) | lo;  // 15-bit magnitude
  return (hi & 0x80) ? -(int16_t)mag : (int16_t)mag;
}
static inline void copy7_u16(uint16_t dst[7], const uint16_t src[7]) {
  for (int i = 0; i < 7; ++i) dst[i] = src[i];
}

// ----- Helper function to send u16 frame for POS,SPD,VEL,CURR and sendACK Packet ----
static inline void sendU16Frame(uint8_t header, const uint16_t data[7]) {
  uint8_t out[2 + 7*2];
  out[0] = header;
  out[1] = 0x00; // filler
  for (int i = 0; i < 7; ++i) {
    out[2 + 2*i + 0] = (uint8_t)(data[i] & 0xFF);
    out[2 + 2*i + 1] = (uint8_t)((data[i] >> 8) & 0xFF);
  }
  Serial.write(out, sizeof(out)); 
}
static inline void sendAckFrame(uint8_t header, const uint8_t* payload, size_t n) {
  uint8_t out[16];
  out[0] = header;
  out[1] = 0x00; // filler
  memset(out + 2, 0, 14);
  if (payload && n) {
    if (n > 14) n = 14;
    memcpy(out + 2, payload, n);
  }
  Serial.write(out, sizeof(out));
}

// ----- Functions to Send POS, VEL, CURR, TEMP -----
void sendPositions() {
  uint16_t buf[7];
  xSemaphoreTake(gMetricsMux, portMAX_DELAY);
  copy7_u16(buf, gMetrics.pos);
  xSemaphoreGive(gMetricsMux);
  sendU16Frame(GET_POS, buf);
}
void sendVelocities() {
  uint16_t buf[7];
  xSemaphoreTake(gMetricsMux, portMAX_DELAY);
  copy7_u16(buf, gMetrics.vel);
  xSemaphoreGive(gMetricsMux);
  sendU16Frame(GET_VEL, buf);
}
void sendCurrents() {
  uint16_t buf[7];
  xSemaphoreTake(gMetricsMux, portMAX_DELAY);
  copy7_u16(buf, gMetrics.cur);
  xSemaphoreGive(gMetricsMux);
  sendU16Frame(GET_CURR, buf);
}
void sendTemps() {
  uint16_t buf[7];
  xSemaphoreTake(gMetricsMux, portMAX_DELAY);
  copy7_u16(buf, gMetrics.tmp);
  xSemaphoreGive(gMetricsMux);
  sendU16Frame(GET_TEMP, buf);
}

// ----- Task - Sync Read running always on Core 1 ----- 
static void TaskSyncRead_Core1(void *arg) {
  uint8_t  rx[REG_BLOCK_LEN];          // 15 bytes
  uint16_t pos[7], vel[7], cur[7], tmp[7];
  const TickType_t period = pdMS_TO_TICKS(20);   // Change Frequency of Running here, 5 -200 Hz, 10-100 Hz, 20 -50 Hz
  TickType_t nextWake = xTaskGetTickCount();
  for (;;) {
    // try-lock: if control is using the bus, skip this cycle
    if (gBusMux && xSemaphoreTake(gBusMux, 0) != pdTRUE) {
      vTaskDelayUntil(&nextWake, period);
      continue;
    }
    bool ok = true;
    // one TX for the whole group (15-byte slice)
    hlscl.syncReadPacketTx((uint8_t*)SERVO_IDS, 7, REG_BLOCK_START, REG_BLOCK_LEN);
    for (uint8_t i = 0; i < 7; ++i) {
      if (!hlscl.syncReadPacketRx(SERVO_IDS[i], rx)) { ok = false; break; }
      // bytes: pos(0..1), vel(2..3 signMag15), tmp(7), cur(13..14 signMag15)
      pos[i] = leu_u16(&rx[0]);                                    // position (unsigned)
      vel[i] = decode_signmag15(rx[2], rx[3]);                // velocity (signed)
      tmp[i] = rx[7];                                            // temperature (unsigned, 1 byte)
      cur[i] = decode_signmag15(rx[13], rx[14]);              // current (signed)
      //vTaskDelay(1);
    }
    if (gBusMux) xSemaphoreGive(gBusMux);
    if (ok) {
      xSemaphoreTake(gMetricsMux, portMAX_DELAY);
      for (int i = 0; i < 7; ++i) {
        gMetrics.pos[i] = pos[i];
        gMetrics.vel[i] = vel[i];
        gMetrics.tmp[i] = tmp[i];
        gMetrics.cur[i] = cur[i];
      }
      xSemaphoreGive(gMetricsMux);
    } 
    vTaskDelayUntil(&nextWake, period);
  }
}
//Set-ID and Trim Servo functions
static bool handleSetIdCmd(const uint8_t* payload) {
  // Parse request: two u16 words, little-endian
  uint16_t w0 = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8); // newId in low byte
  uint16_t w1 = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8); // requested current limit
  uint8_t  newId    = (uint8_t)(w0 & 0xFF);
  uint16_t reqLimit = (w1 > 1023) ? 1023 : w1;
  // Invalid newId → ACK with oldId=0xFF, newId, cur=0
  if (newId > 253) {
    uint8_t ack[6] = { 0xFF, 0x00, newId, 0x00, 0x00, 0x00 };
    sendAckFrame(SET_ID, ack, sizeof(ack));
    return true;
  }
  // Find any servo present
  int found = scanFirstServoId();
  if (found < 0) {
    uint8_t ack[6] = { 0xFF, 0x00, newId, 0x00, 0x00, 0x00 };
    sendAckFrame(SET_ID, ack, sizeof(ack));
    return true;
  }
  uint8_t oldId = (uint8_t)found;
  // Program device
  if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);
  (void)hlscl.unLockEprom(oldId);
  (void)hlscl.writeWord(oldId, REG_CURRENT_LIMIT, reqLimit);
  delay(10);
  uint8_t targetId = oldId;
  if (newId != oldId) {
    (void)hlscl.writeByte(oldId, REG_ID, newId);   // REG_ID = 0x05
    delay(10);
    targetId = newId;
  }
  (void)hlscl.LockEprom(targetId);
  delay(10);
  
  // Read back limit for ACK
  uint16_t curLimitRead = 0;
  int rd = hlscl.readWord(targetId, REG_CURRENT_LIMIT);
  if (rd >= 0) curLimitRead = (uint16_t)rd;
  if (gBusMux) xSemaphoreGive(gBusMux);
  // Build 6-byte payload: oldId(LE16), newId(LE16), curLimit(LE16) and send
  uint8_t ack[6];
  ack[0] = oldId;                      // oldId lo
  ack[1] = 0x00;                       // oldId hi
  ack[2] = targetId;                   // newId lo
  ack[3] = 0x00;                       // newId hi
  ack[4] = (uint8_t)(curLimitRead & 0xFF);
  ack[5] = (uint8_t)((curLimitRead >> 8) & 0xFF);
  sendAckFrame(SET_ID, ack, sizeof(ack));
  return true;
}
static bool handleTrimCmd(const uint8_t* payload) {
  // Parse little-endian fields
  uint16_t rawCh  = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
  uint16_t rawDeg = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
  int      ch      = (int)rawCh;         // 0..6
  int16_t  degrees = (int16_t)rawDeg;    // signed degrees
  // Validate channel (0..6)
  if (ch < 0 || ch >= 7) {
    // Optionally send a NACK or silent-accept to preserve framing
    return true;
  }
  // Degrees -> counts (≈11.375 counts/deg), clamp to 0..4095
  int delta_counts = (int)((float)degrees * 11.375f);
  int new_ext = (int)sd[ch].extend_count + delta_counts;
  if (new_ext < 0)    new_ext = 0;
  if (new_ext > 4095) new_ext = 4095;
  sd[ch].extend_count = (uint16_t)new_ext;
  // Persist to NVS
  prefs.begin("hand", false);
  prefs.putInt(String("ext" + String(ch)).c_str(), sd[ch].extend_count);
  prefs.end();
  // ACK payload: ch (u16, LE), extend_count (u16, LE)
  uint8_t ack[4];
  ack[0] = (uint8_t)(ch & 0xFF);
  ack[1] = (uint8_t)((ch >> 8) & 0xFF);
  ack[2] = (uint8_t)(sd[ch].extend_count & 0xFF);
  ack[3] = (uint8_t)((sd[ch].extend_count >> 8) & 0xFF);
  sendAckFrame(TRIM, ack, sizeof(ack));   // 16 bytes on the wire
  return true;
}
// ----- Returns true if a valid 16-byte frame was consumed and handled -----
static bool handleHostFrame(uint8_t op) {
  // Wait until full frame is buffered: filler + 14 payload
  while (Serial.available() < 15) { /* wait */ }
  uint8_t buf[15];
  for (int i = 0; i < 15; ++i) {
    int ch = Serial.read(); if (ch < 0) return false;
    buf[i] = (uint8_t)ch;
  }
  const uint8_t* payload = &buf[1]; // buf[0] = filler 0x00 (ignored)

  switch (op) {
    case CTRL_POS: {
      int16_t pos[7];
      for (int i = 0; i < 7; ++i) {
        // 0..65535 from payload
        uint16_t u16 = (uint16_t)payload[2*i] | ((uint16_t)payload[2*i+1] << 8);
        uint8_t  ch  = SERVO_IDS[i];
        uint16_t ext = sd[ch].extend_count;  // open
        uint16_t gra = sd[ch].grasp_count;   // closed
        int32_t raw32;
        if (ext == 0 && gra == 0) {
          raw32 = (int32_t)(((uint64_t)u16 * 4095u) / 65535u);
        } else {
          raw32 = (int32_t)ext + (int32_t)(((int64_t)u16 * ((int32_t)gra - (int32_t)ext)) / 65535LL);
        }
        if (raw32 < 0)    raw32 = 0;
        if (raw32 > 4095) raw32 = 4095;
        pos[i] = (int16_t)raw32;
      }
      if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);
      hlscl.SyncWritePosEx((uint8_t*)SERVO_IDS, 7, pos, g_speed, g_accel, g_torque);
      if (gBusMux) xSemaphoreGive(gBusMux);
      return true;
    }

    case HOMING: {
      HOMING_start();   // blocks
      sendAckFrame(HOMING, nullptr, 0);
      return true;
    }

    case ZERO_ALL: {
      int16_t pos[7];
      for (int i = 0; i < 7; ++i) {
        uint8_t ch  = SERVO_IDS[i];
        int32_t raw32 = sd[ch].extend_count;   // force to EXTEND
        if (raw32 < 0)    raw32 = 0;
        if (raw32 > 4095) raw32 = 4095;
        pos[i] = (int16_t)raw32;
      }
      if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);
      hlscl.SyncWritePosEx((uint8_t*)SERVO_IDS, 7, pos, g_speed, g_accel, g_torque);
      if (gBusMux) xSemaphoreGive(gBusMux);
      return true;
    }

    case SET_ID: {
      return handleSetIdCmd(payload);
    }

    case TRIM: {
      return handleTrimCmd(payload);
    }

    case GET_POS: {
      sendPositions();
      return true;
    }

    case GET_VEL: {
      sendVelocities();
      return true;
    }

    case GET_TEMP: {
      sendTemps();
      return true;
    }

    case GET_CURR: {
      sendCurrents();
      return true;
    }

    default:
      //Unknown Control Code — consume frame to preserve alignment
      return true;
  }
}

void setup() {
  // USB debug
  Serial.begin(921600);
  delay(100);

  // Servo bus UART @ 1 Mbps
  Serial2.begin(1000000, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);
  hlscl.pSerial = &Serial2;
  delay(50);

  prefs.begin("hand", false);
  loadManualExtendsFromNVS();
  // ---- Presence Check on Every Boot -----
  Serial.println("\n[Init] Pinging servos...");
  for (uint8_t i = 0; i < 7; ++i) {
    uint8_t id = SERVO_IDS[i];
    int resp = hlscl.Ping(id);
    if (!hlscl.getLastError()) {
      Serial.print("  ID "); Serial.print(id); Serial.println(": OK");
    } else {
      Serial.print("  ID "); Serial.print(id); Serial.println(": NO REPLY");
    }
    delay(20);
  }

  // SyncReadBegin to start the sync read
  hlscl.syncReadBegin(sizeof(SERVO_IDS), REG_BLOCK_LEN, /*rx_fix*/ 8);

  //Initialisation of Mutex and Task serial pinned to Core 1
  gBusMux =xSemaphoreCreateMutex();
  gMetricsMux = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(TaskSyncRead_Core1, "SyncRead", 4096, NULL, 1, NULL, 1); // run on Core1
}

void loop() {
  static uint32_t last_cmd_ms = 0; 

  // Gate all host input during homing
  if (HOMING_isBusy()) {
    while (Serial.available()) { Serial.read(); }
    vTaskDelay(pdMS_TO_TICKS(5));
    return;
  }

  // Process exactly one complete 16-byte frame when available
  if (Serial.available() >= 16) {
    int op = Serial.read();
    if (op >= 0) {
      if (handleHostFrame((uint8_t)op)) {
        last_cmd_ms = millis();
        return;
      }
    }
  }
}