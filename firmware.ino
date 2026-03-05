// ═══════════════════════════════════════════════════════════
//  ZSC31014 Load Cell + Servo Motor — Xiao S3
//  Binary telemetry + command protocol
// ═══════════════════════════════════════════════════════════

#include <Wire.h>
#include <ModbusMaster.h>

// ── Pin Assignments ──────────────────────────────
#define LC_SDA_PIN   D4
#define LC_SCL_PIN   D5
#define LC_I2C_FREQ  100000

#define MB_TX_GPIO   43   // Physical D6
#define MB_RX_GPIO   44   // Physical D7
#define MB_EN_GPIO   D3   // DE/RE direction control

// ── Device Addresses ─────────────────────────────
#define ZSC31014_ADDR 0x28

// ── Load Cell Status Bits ────────────────────────
#define LC_STATUS_VALID   0x00
#define LC_STATUS_STALE   0x01
#define LC_STATUS_COMMAND 0x02

// ── Binary Protocol ──────────────────────────────
#define SYNC_0      0xAA
#define SYNC_1      0x55
#define PACKET_ID   0x02  // v2: load cell + motor

// Command IDs (Python → ESP)
#define CMD_START     0x01  // payload: uint8 mode (0=speed, 1=torque)
#define CMD_STOP      0x02  // no payload
#define CMD_SET_PARAM 0x03  // payload: uint8 param_id, int16_le value

// Param IDs for CMD_SET_PARAM
#define PARAM_TORQUE_REF  0x01  // torque x10 (e.g. 40 = 4.0%)
#define PARAM_SPEED_REF   0x02  // RPM
#define PARAM_TORQUE_LIM_POS 0x03
#define PARAM_TORQUE_LIM_NEG 0x04

// ── Error Codes ─────────────────────────────────
#define ERR_NONE              0x00
#define ERR_LC_STALE_TIMEOUT  0x01  // Load cell stuck returning stale data >50ms
#define ERR_LC_I2C_FAIL       0x02  // I2C read returned no bytes
#define ERR_LC_NOT_FOUND      0x03  // Load cell not detected at startup
#define ERR_SERVO_CONFIG_FAIL 0x04  // Servo configuration write failed
#define ERR_SERVO_SCAN_FAIL   0x05  // No servo found on bus (full scan)

// ── State Machine ────────────────────────────────
enum State {
  STATE_IDLE,        // Waiting for Python START command
  STATE_SCANNING,    // Scanning Modbus bus
  STATE_CONFIGURING, // Setting up servo
  STATE_RUNNING,     // Main loop: read + telemetry + motor
  STATE_ERROR        // Something went wrong
};

State currentState = STATE_IDLE;
uint8_t errorCode = ERR_NONE;
bool modeTorque = true;
uint8_t servoID = 0;
uint8_t scanID = 1;
unsigned long lastActionTime = 0;
unsigned long lastTelemetryTime = 0;

// Motor monitoring values
int16_t motorRPM = 0;
int16_t motorTorqueX10 = 0;  // torque * 10

// Load cell non-blocking state machine
enum LCPhase { LC_REQUEST, LC_WAIT };
LCPhase lcPhase = LC_REQUEST;
unsigned long lcRequestTime = 0;
uint16_t lastBridge = 0;
uint8_t lastLCStatus = 0;
bool lastLCValid = false;

// ── Modbus ───────────────────────────────────────
ModbusMaster servo;

void preTransmission()  { digitalWrite(MB_EN_GPIO, HIGH); }
void postTransmission() { digitalWrite(MB_EN_GPIO, LOW);  }

// ── CRC8 ─────────────────────────────────────────
static uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
  }
  return crc;
}

// ═════════════════════════════════════════════════
//  TELEMETRY (ESP → Python)
// ═════════════════════════════════════════════════
// Packet v2 payload (15 bytes):
//   0     packet_id     uint8
//   1-4   t_ms          uint32 LE
//   5-6   bridge        uint16 LE
//   7     lc_status     uint8
//   8     lc_flags      uint8   (bit0=valid, bit1=stale_timeout; or error_code when state=ERROR)
//   9-10  rpm           int16 LE
//   11-12 torque_x10    int16 LE
//   13    servo_state   uint8
//   14    mode          uint8   (0=speed, 1=torque)

static uint8_t txBuf[64];

void telemetrySend(uint32_t t, uint16_t bridge, uint8_t lcStatus, bool lcValid,
                   int16_t rpm, int16_t torqueX10,
                   uint8_t state, uint8_t mode) {
  uint8_t payloadLen = 15;
  uint8_t *p = txBuf;

  *p++ = PACKET_ID;
  *p++ = (t >>  0) & 0xFF;
  *p++ = (t >>  8) & 0xFF;
  *p++ = (t >> 16) & 0xFF;
  *p++ = (t >> 24) & 0xFF;
  *p++ = (bridge >> 0) & 0xFF;
  *p++ = (bridge >> 8) & 0xFF;
  *p++ = lcStatus;
  // lc_flags: bit0 = valid during normal operation; error_code when state == ERROR
  uint8_t flags = (state == STATE_ERROR) ? errorCode : (lcValid ? 0x01 : 0x00);
  *p++ = flags;
  *p++ = (rpm >>  0) & 0xFF;
  *p++ = (rpm >>  8) & 0xFF;
  *p++ = (torqueX10 >> 0) & 0xFF;
  *p++ = (torqueX10 >> 8) & 0xFF;
  *p++ = state;
  *p++ = mode;

  uint8_t frame[3 + payloadLen + 1];
  frame[0] = SYNC_0;
  frame[1] = SYNC_1;
  frame[2] = payloadLen;
  memcpy(&frame[3], txBuf, payloadLen);
  frame[3 + payloadLen] = crc8(txBuf, payloadLen);

  Serial.write(frame, sizeof(frame));
}

// ═════════════════════════════════════════════════
//  COMMAND RECEPTION (Python → ESP)
// ═════════════════════════════════════════════════
static uint8_t rxBuf[64];
static uint8_t rxPos = 0;
static enum { RX_SYNC0, RX_SYNC1, RX_LEN, RX_PAYLOAD } rxState = RX_SYNC0;
static uint8_t rxLen = 0;

void processCommand(const uint8_t *payload, uint8_t len) {
  if (len < 1) return;
  uint8_t cmd = payload[0];

  switch (cmd) {
    case CMD_START:
      if (len >= 2) {
        modeTorque = (payload[1] != 0);
        scanID = 1;
        servoID = 0;
        currentState = STATE_SCANNING;
      }
      break;

    case CMD_STOP:
      // Attempt servo OFF before going idle
      if (servoID > 0) {
        servo.begin(servoID, Serial0);
        servo.writeSingleRegister(1041, 0);  // Servo OFF (C04.11)
      }
      motorRPM = 0;
      motorTorqueX10 = 0;
      currentState = STATE_IDLE;
      break;

    case CMD_SET_PARAM:
      if (len >= 4 && currentState == STATE_RUNNING && servoID > 0) {
        uint8_t paramID = payload[1];
        int16_t value = (int16_t)(payload[2] | (payload[3] << 8));
        servo.begin(servoID, Serial0);
        switch (paramID) {
          case PARAM_TORQUE_REF:     servo.writeSingleRegister(833, value);  break;
          case PARAM_SPEED_REF:      servo.writeSingleRegister(4614, value); break;
          case PARAM_TORQUE_LIM_POS: servo.writeSingleRegister(839, value);  break;
          case PARAM_TORQUE_LIM_NEG: servo.writeSingleRegister(840, value);  break;
        }
      }
      break;
  }
}

void checkCommands() {
  while (Serial.available()) {
    uint8_t b = Serial.read();

    switch (rxState) {
      case RX_SYNC0:
        if (b == SYNC_0) rxState = RX_SYNC1;
        break;
      case RX_SYNC1:
        rxState = (b == SYNC_1) ? RX_LEN : RX_SYNC0;
        break;
      case RX_LEN:
        rxLen = b;
        rxPos = 0;
        if (rxLen == 0 || rxLen > 60) { rxState = RX_SYNC0; break; }
        rxState = RX_PAYLOAD;
        break;
      case RX_PAYLOAD:
        rxBuf[rxPos++] = b;
        if (rxPos >= rxLen + 1) {  // payload + crc
          uint8_t crcRx = rxBuf[rxLen];
          uint8_t crcCalc = crc8(rxBuf, rxLen);
          if (crcRx == crcCalc) {
            processCommand(rxBuf, rxLen);
          }
          rxState = RX_SYNC0;
        }
        break;
    }
  }
}

// ═════════════════════════════════════════════════
//  LOAD CELL
// ═════════════════════════════════════════════════
bool loadCellInit() {
  Wire.begin(LC_SDA_PIN, LC_SCL_PIN);
  Wire.setClock(LC_I2C_FREQ);

  Wire.beginTransmission(ZSC31014_ADDR);
  uint8_t err = Wire.endTransmission();
  if (err != 0) return false;

  // Discard initial stale read
  Wire.requestFrom((uint8_t)ZSC31014_ADDR, (uint8_t)2);
  while (Wire.available()) Wire.read();
  delay(5);
  return true;
}

void loadCellMeasurementRequest() {
  Wire.requestFrom((uint8_t)ZSC31014_ADDR, (uint8_t)2);
  while (Wire.available()) Wire.read();
}

bool loadCellFetch(uint16_t &bridgeRaw, uint8_t &status) {
  uint8_t count = Wire.requestFrom((uint8_t)ZSC31014_ADDR, (uint8_t)2);
  if (count < 2) return false;

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();

  status = (msb >> 6) & 0x03;
  bridgeRaw = ((uint16_t)(msb & 0x3F) << 8) | lsb;
  return true;
}

bool loadCellRead(uint16_t &bridgeRaw, uint8_t &status) {
  loadCellMeasurementRequest();
  delay(2);

  uint32_t start = millis();
  while (millis() - start < 50) {
    if (!loadCellFetch(bridgeRaw, status)) return false;
    if (status == LC_STATUS_VALID) return true;
    delayMicroseconds(500);
  }
  return false;
}

// ═════════════════════════════════════════════════
//  SERVO MOTOR
// ═════════════════════════════════════════════════
void servoInit() {
  pinMode(MB_EN_GPIO, OUTPUT);
  digitalWrite(MB_EN_GPIO, LOW);
  Serial0.begin(115200, SERIAL_8N1, MB_RX_GPIO, MB_TX_GPIO);
  Serial0.setTimeout(100);
  servo.preTransmission(preTransmission);
  servo.postTransmission(postTransmission);
}

bool servoScanStep() {
  servo.begin(scanID, Serial0);
  uint8_t result = servo.readHoldingRegisters(0, 1);
  if (result == servo.ku8MBSuccess) {
    servoID = scanID;
    return true;
  }
  scanID++;
  if (scanID > 247) scanID = 1;
  return false;
}

bool servoConfigure(uint8_t id) {
  servo.begin(id, Serial0);

  if (modeTorque) {
    if (servo.writeSingleRegister(0, 2) != 0) return false;      // Torque mode
    if (servo.writeSingleRegister(833, 40) != 0) return false;   // Ref torque 4.0%
    if (servo.writeSingleRegister(839, 100) != 0) return false;  // Torque limit+
    if (servo.writeSingleRegister(840, 100) != 0) return false;  // Torque limit-
  } else {
    if (servo.writeSingleRegister(0, 1) != 0) return false;      // Speed mode
    if (servo.writeSingleRegister(4614, 100) != 0) return false; // 100 RPM
  }

  // Servo ON
  if (servo.writeSingleRegister(1041, 1) != 0) return false;
  return true;
}

void servoReadMonitoring() {
  servo.begin(servoID, Serial0);

  uint8_t r1 = servo.readHoldingRegisters(16385, 1);
  if (r1 == servo.ku8MBSuccess)
    motorRPM = (int16_t)servo.getResponseBuffer(0);

  uint8_t r2 = servo.readHoldingRegisters(16387, 1);
  if (r2 == servo.ku8MBSuccess)
    motorTorqueX10 = (int16_t)servo.getResponseBuffer(0);
}

// ═════════════════════════════════════════════════
//  MAIN
// ═════════════════════════════════════════════════
bool loadCellOK = false;

void setup() {
  Serial.begin(115200);
  { unsigned long t0 = millis(); while (!Serial && millis() - t0 < 3000) delay(10); }

  // Debug header (text, Python skips # lines)
  Serial.println("# ═══════════════════════════════════════");
  Serial.println("#  ZSC31014 + Servo — Xiao S3");
  Serial.println("# ═══════════════════════════════════════");

  // Init load cell
  loadCellOK = loadCellInit();
  Serial.printf("# Load Cell: %s\n", loadCellOK ? "OK" : "NOT FOUND");

  // Init modbus hardware (don't scan yet — wait for command)
  servoInit();
  Serial.println("# Modbus: initialized");
  Serial.println("# Waiting for START command...");
  Serial.flush();
  delay(50);
}

void loop() {
  unsigned long now = millis();

  // Always check for commands from Python
  checkCommands();

  switch (currentState) {

    case STATE_IDLE:
      // Send heartbeat telemetry at 2Hz so Python knows we're alive
      if (now - lastTelemetryTime >= 500) {
        telemetrySend(now, 0, 0, false, 0, 0, (uint8_t)currentState, modeTorque ? 1 : 0);
        lastTelemetryTime = now;
      }
      break;

    case STATE_SCANNING:
      if (now - lastActionTime >= 15) {
        if (servoScanStep()) {
          currentState = STATE_CONFIGURING;
        }
        lastActionTime = now;
      }
      // Keep sending telemetry during scan
      if (now - lastTelemetryTime >= 200) {
        telemetrySend(now, 0, 0, false, 0, 0, (uint8_t)currentState, modeTorque ? 1 : 0);
        lastTelemetryTime = now;
      }
      break;

    case STATE_CONFIGURING:
      if (servoConfigure(servoID)) {
        errorCode = ERR_NONE;
        lcPhase = LC_REQUEST;
        lastBridge = 0;
        lastLCStatus = 0;
        lastLCValid = false;
        currentState = STATE_RUNNING;
      } else {
        // Retry scan from next ID
        scanID = servoID + 1;
        servoID = 0;
        currentState = STATE_SCANNING;
      }
      break;

    case STATE_RUNNING:
      if (now - lastActionTime >= 10) {
        // Synchronized: fetch LC (requested last cycle), read motor, then send
        if (loadCellOK && lcPhase == LC_WAIT && now - lcRequestTime >= 2) {
          uint16_t bridge;
          uint8_t status;
          if (loadCellFetch(bridge, status)) {
            if (status == LC_STATUS_VALID) {
              lastBridge = bridge;
              lastLCStatus = status;
              lastLCValid = true;
            } else if (now - lcRequestTime > 50) {
              errorCode = ERR_LC_STALE_TIMEOUT;
              goto error_stop;
            }
          } else {
            errorCode = ERR_LC_I2C_FAIL;
            goto error_stop;
          }
        }

        servoReadMonitoring();
        telemetrySend(now, lastBridge, lastLCStatus, lastLCValid,
                      motorRPM, motorTorqueX10,
                      (uint8_t)currentState, modeTorque ? 1 : 0);

        // Request next LC measurement (will be fetched at start of next cycle)
        if (loadCellOK) {
          loadCellMeasurementRequest();
          lcRequestTime = now;
          lcPhase = LC_WAIT;
        }

        lastActionTime = now;
        lastTelemetryTime = now;
        break;

      error_stop:
        // Emergency stop: servo OFF
        servo.begin(servoID, Serial0);
        servo.writeSingleRegister(1041, 0);
        motorRPM = 0;
        motorTorqueX10 = 0;
        lastLCValid = false;
        telemetrySend(now, lastBridge, lastLCStatus, false,
                      0, 0, (uint8_t)STATE_ERROR, modeTorque ? 1 : 0);
        currentState = STATE_ERROR;
      }
      break;

    case STATE_ERROR:
      if (now - lastTelemetryTime >= 1000) {
        telemetrySend(now, 0, 0, false, 0, 0, (uint8_t)currentState, modeTorque ? 1 : 0);
        lastTelemetryTime = now;
      }
      break;
  }
}
