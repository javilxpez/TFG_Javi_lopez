// ═══════════════════════════════════════════════════════════
//  ZSC31014 Load Cell + Servo Motor — Raspberry Pi Pico 2
//  Modo velocidad A→B — sin lazo cerrado de célula de carga
//  Dual-core: Core0 = LC/telemetría/comandos, Core1 = Modbus
// ═══════════════════════════════════════════════════════════

#include <Wire.h>
#include <ModbusMaster.h>
#include <SerialPIO.h>

static TwoWire I2C1Bus(i2c1, 2, 3);

// ── Pins ─────────────────────────────────────────
#define LC_SDA_PIN   2
#define LC_SCL_PIN   3
#define LC_I2C_FREQ  100000
#define MB_TX_GPIO   14
#define MB_RX_GPIO   13
#define MB_EN_GPIO   12

#define ZSC31014_ADDR 0x28
#define LC_STATUS_VALID   0x00
#define LC_STATUS_STALE   0x01
#define LC_STATUS_COMMAND 0x02

// ── Protocolo binario ─────────────────────────────
#define SYNC_0     0xAA
#define SYNC_1     0x55
#define PACKET_ID  0x02

// Comandos Python → Pico
#define CMD_INIT      0x01
#define CMD_STOP      0x02
#define CMD_SET_PARAM 0x03
#define CMD_MOVE_A    0x04
#define CMD_MOVE_B    0x05
#define CMD_SHUTDOWN  0x06
#define CMD_SCAN      0x07

// IDs de parámetro
#define PARAM_SPEED       0x01
#define PARAM_FORCE_LIMIT 0x02
#define PARAM_ZERO_OFFSET 0x03
#define PARAM_ACCEL       0x04
#define PARAM_DECEL       0x05
#define PARAM_STIFFNESS   0x06
#define PARAM_FORCE_CALIB 0x07

// ── Registros Modbus ──────────────────────────────
#define MB_REG_CONTROL_MODE  0
#define MB_REG_SPEED_REF     801
#define MB_REG_SERVO_ENABLE  1041
#define MB_REG_MON_RPM       16385
#define MB_REG_MON_TORQUE    16387
#define MB_C00_05            5
#define MB_C03_22            802
#define MB_C03_24            804

#define SERVO_ID     1     // Modbus address of the drive (known — no scan)
#define TELEMETRY_MS 200   // telemetry send period, ms (USB CDC has plenty of headroom)
#define LOOP_MS      50    // minimum loop period, ms — caps loop (and thus read/bus) rate

#define PIN_LIMIT_A  16   // DI1 — final de carrera lado A
#define PIN_LIMIT_B  17   // DI2 — final de carrera lado B

#define MB_REG_BRAKE_RES_SEL   16
#define MB_REG_BRAKE_RES_POW   17
#define MB_REG_BRAKE_RES_OHM   18
#define MB_REG_BRAKE_RES_DISS  19
#define BRAKE_RES_SEL_VAL      1
#define BRAKE_RES_POW_VAL      1000
#define BRAKE_RES_OHM_VAL      235
#define BRAKE_RES_DISS_VAL     30

// ── Errores ───────────────────────────────────────
#define ERR_NONE              0x00
#define ERR_LC_NOT_FOUND      0x01
#define ERR_SERVO_SCAN_FAIL   0x02
#define ERR_SERVO_CONFIG_FAIL 0x03
#define ERR_FORCE_LIMIT       0x04

// ── Estados ───────────────────────────────────────
enum State {
  STATE_IDLE        = 0,
  STATE_SCANNING    = 1,
  STATE_CONFIGURING = 2,
  STATE_STOPPED     = 3,
  STATE_MOVING_A    = 4,
  STATE_MOVING_B    = 5,
  STATE_ERROR       = 6
};
#define MODE_STOPPED  0
#define MODE_MOVING_A 1
#define MODE_MOVING_B 2

enum LCPhase { LC_REQUEST, LC_WAIT };

// ── Control State ─────────────────────────────────
struct ControlState {
  State    currentState    = STATE_IDLE;
  uint8_t  errorCode       = ERR_NONE;

  unsigned long lastActionTime    = 0;
  unsigned long lastTelemetryTime = 0;

  uint8_t  servoID         = 0;
  uint8_t  scanID          = 1;
  bool     servoConnected  = false;
  unsigned long scanStartTime    = 0;
  uint8_t  configureRetries      = 0;
  int16_t  motorRPM        = 0;
  int16_t  motorTorqueX10  = 0;

  int16_t  moveSpeed       = 10;
  int16_t  currentSpeedCmd = 0;
  int16_t  stiffness       = 10;
  int16_t  accelRate       = 5;
  int16_t  decelRate       = 5;

  bool     loadCellOK      = false;
  bool     lcConfigApplied = false;
  LCPhase  lcPhase         = LC_REQUEST;
  unsigned long lcRequestTime = 0;
  uint16_t lastBridge      = 0;
  uint8_t  lastLCStatus    = 0;
  bool     lastLCValid     = false;

  int16_t  zeroOffset      = 8210;
  int16_t  baseRead        = 0;
  uint16_t forceLimit      = 0;
  int16_t  forceCalib      = 0;

  static const uint8_t MA_SIZE = 8;
  uint16_t maBuf[8]        = {};
  uint8_t  maIdx           = 0;
  uint32_t maSum           = 0;
  uint16_t lastBridgeFilt  = 0;
};

static ControlState ctrl;

// ── Servo bus context (single owner — touched only by servoReadState/servoAction) ──
enum ServoPhase { SV_IDLE, SV_CONFIGURING, SV_RUNNING, SV_FAULT };

struct Servo {
  ServoPhase    phase       = SV_IDLE;
  uint8_t       id          = SERVO_ID;   // Modbus address in use (set at Init)
  bool          connected   = false;
  uint8_t       failCount   = 0;
  uint8_t       cfgRetries  = 0;
  bool          reqShutdown = false;
  int16_t       speedCmd    = 0;   // target speed ref (0 = stopped, still enabled)
  int16_t       rpm         = 0;
  int16_t       torqueX10   = 0;
  uint8_t       rpmStatus   = 0;   // ModbusMaster return code of last RPM read (diag)
  uint8_t       trqStatus   = 0;   // ModbusMaster return code of last torque read (diag)
} sv;

// ── Modbus (Serial485 is touched ONLY inside servoReadState / servoAction) ──
static SerialPIO Serial485(MB_TX_GPIO, MB_RX_GPIO);
ModbusMaster servo;

void preTransmission()  { digitalWrite(MB_EN_GPIO, HIGH); }
void postTransmission() { delayMicroseconds(200); digitalWrite(MB_EN_GPIO, LOW); }  // no flush() — it breaks SerialPIO RX turnaround

// Core1 unused — single owner design, everything runs on Core0.
void setup1() {}
void loop1()  { delay(1000); }

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

// ── Telemetría por LOTES (header + N muestras) ────
// Frame: [SYNC0][SYNC1][LEN][payload][CRC8]   (LEN = header 14 + N*12 ≤ 254)
// ── Header (14 B) — datos comunes, una vez por lote ──
//  0     packet_id
//  1-4   base_t_ms   (referencia de tiempo del lote)
//  5-6   bridge      (LC crudo)
//  7     lc_status
//  8     flags       (b0=lc_valid, b1=servo_conn, b2=lc_cfg)
//  9     error
//  10    servo_id
//  11    rpm_status
//  12    trq_status
//  13    n_samples
// ── Muestra (12 B) × n_samples — una por iteración de loop ──
//  +0-1  dt_ms       (ms desde base_t — el "loop time")
//  +2-3  rpm         (int16)
//  +4-5  current_x10 (int16)
//  +6-7  ref_cmd     (int16)
//  +8-9  base_read   (int16)
//  +10   io          (b0=limit_A, b1=limit_B)
//  +11   state
#define MAX_SAMPLES 8      // LOOP_MS=50 → ~4-5 por ventana de 200 ms; margen a 8
struct Sample {
  uint16_t dt_ms;
  int16_t  rpm, current, ref, base;
  uint8_t  io, state;
};
static Sample   sampleBuf[MAX_SAMPLES];
static uint8_t  sampleCount = 0;
static uint32_t frameBaseT  = 0;
static uint8_t  txBuf[14 + MAX_SAMPLES*12];

// Cache one loop iteration into the batch (no I/O)
void recordSample(uint32_t now){
  if(sampleCount>=MAX_SAMPLES) return;
  if(sampleCount==0) frameBaseT = now;
  Sample &s = sampleBuf[sampleCount++];
  s.dt_ms   = (uint16_t)(now - frameBaseT);
  s.rpm     = ctrl.motorRPM;
  s.current = ctrl.motorTorqueX10;
  s.ref     = ctrl.currentSpeedCmd;
  s.base    = ctrl.baseRead;
  s.io      = (digitalRead(PIN_LIMIT_A)==HIGH ? 0x01 : 0x00)
            | (digitalRead(PIN_LIMIT_B)==HIGH ? 0x02 : 0x00);
  s.state   = (uint8_t)ctrl.currentState;
}

// Send the whole batch as one frame, then reset
void telemetryFlush(){
  if(sampleCount==0) return;
  uint8_t flags = (ctrl.lastLCValid     ? 0x01 : 0x00)
                | (ctrl.servoConnected  ? 0x02 : 0x00)
                | (ctrl.lcConfigApplied ? 0x04 : 0x00);
  uint8_t *p = txBuf;
  *p++ = PACKET_ID;
  *p++ = (frameBaseT>> 0)&0xFF; *p++ = (frameBaseT>> 8)&0xFF;
  *p++ = (frameBaseT>>16)&0xFF; *p++ = (frameBaseT>>24)&0xFF;
  *p++ = (ctrl.lastBridge>>0)&0xFF; *p++ = (ctrl.lastBridge>>8)&0xFF;
  *p++ = ctrl.lastLCStatus;
  *p++ = flags;
  *p++ = ctrl.errorCode;
  *p++ = sv.id;
  *p++ = sv.rpmStatus;
  *p++ = sv.trqStatus;
  *p++ = sampleCount;
  for(uint8_t i=0;i<sampleCount;i++){
    Sample &s = sampleBuf[i];
    *p++ = (s.dt_ms  >>0)&0xFF; *p++ = (s.dt_ms  >>8)&0xFF;
    *p++ = (s.rpm    >>0)&0xFF; *p++ = (s.rpm    >>8)&0xFF;
    *p++ = (s.current>>0)&0xFF; *p++ = (s.current>>8)&0xFF;
    *p++ = (s.ref    >>0)&0xFF; *p++ = (s.ref    >>8)&0xFF;
    *p++ = (s.base   >>0)&0xFF; *p++ = (s.base   >>8)&0xFF;
    *p++ = s.io;
    *p++ = s.state;
  }
  uint8_t payloadLen = (uint8_t)(p - txBuf);   // 14 + n*12, ≤ 110 for MAX_SAMPLES=8
  uint8_t frame[3 + sizeof(txBuf) + 1];
  frame[0]=SYNC_0; frame[1]=SYNC_1; frame[2]=payloadLen;
  memcpy(&frame[3], txBuf, payloadLen);
  frame[3+payloadLen] = crc8(txBuf, payloadLen);
  Serial.write(frame, 3 + payloadLen + 1);
  sampleCount = 0;
}

// ── Comandos ─────────────────────────────────────
// processCommand only updates state (sv/ctrl) — never touches the bus.
static uint8_t rxBuf[64];
static uint8_t rxPos = 0;
static enum { RX_SYNC0, RX_SYNC1, RX_LEN, RX_PAYLOAD } rxState = RX_SYNC0;
static uint8_t rxLen = 0;

void processCommand(const uint8_t *payload, uint8_t len) {
  if (len < 1) return;
  switch (payload[0]) {

    case CMD_INIT:
      if (len >= 2 && payload[1] > 0 && payload[1] <= 247) sv.id = payload[1];  // address from Init
      sv.cfgRetries = 0; sv.speedCmd = 0;
      ctrl.errorCode = ERR_NONE;
      sv.phase = SV_CONFIGURING;                   // known address → connect directly, no scan
      break;

    case CMD_STOP:
      if (sv.phase == SV_RUNNING)          sv.speedCmd = 0;
      else if (sv.phase == SV_CONFIGURING) sv.phase = SV_IDLE;
      break;

    case CMD_SCAN: {                                // manual, blocking bus scan
      uint8_t found = scanServo();
      if (found) sv.id = found;
      break;
    }

    case CMD_MOVE_A:
      if (sv.phase == SV_RUNNING) sv.speedCmd = -ctrl.moveSpeed;
      break;

    case CMD_MOVE_B:
      if (sv.phase == SV_RUNNING) sv.speedCmd =  ctrl.moveSpeed;
      break;

    case CMD_SHUTDOWN:
      sv.reqShutdown = true;                        // servoAction() disables + goes idle
      break;

    case CMD_SET_PARAM:
      if (len >= 4) {
        uint8_t  pid   = payload[1];
        int16_t  value = (int16_t)(payload[2] | ((uint16_t)payload[3] << 8));
        switch (pid) {
          case PARAM_SPEED:
            ctrl.moveSpeed = (value > 0) ? value : -value;
            if (sv.phase == SV_RUNNING && sv.speedCmd < 0) sv.speedCmd = -ctrl.moveSpeed;
            if (sv.phase == SV_RUNNING && sv.speedCmd > 0) sv.speedCmd =  ctrl.moveSpeed;
            break;
          case PARAM_FORCE_LIMIT: ctrl.forceLimit = (uint16_t)value; break;
          case PARAM_ZERO_OFFSET: ctrl.zeroOffset  = value; break;
          case PARAM_ACCEL:       ctrl.accelRate   = value; break;
          case PARAM_DECEL:       ctrl.decelRate   = value; break;
          case PARAM_STIFFNESS:   ctrl.stiffness   = value; break;
          case PARAM_FORCE_CALIB: ctrl.forceCalib  = value; break;
        }
      }
      break;
  }
}

void checkCommands() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (b == 'i') { uint8_t p[]={CMD_INIT};     processCommand(p,1); continue; }
    if (b == 'a') { uint8_t p[]={CMD_MOVE_A};   processCommand(p,1); continue; }
    if (b == 'b') { uint8_t p[]={CMD_MOVE_B};   processCommand(p,1); continue; }
    if (b == 'x') { uint8_t p[]={CMD_STOP};     processCommand(p,1); continue; }
    if (b == 'q') { uint8_t p[]={CMD_SHUTDOWN}; processCommand(p,1); continue; }

    switch (rxState) {
      case RX_SYNC0: if (b==SYNC_0) rxState=RX_SYNC1; break;
      case RX_SYNC1: rxState=(b==SYNC_1)?RX_LEN:RX_SYNC0; break;
      case RX_LEN:
        rxLen=b; rxPos=0;
        if (!rxLen||rxLen>60) { rxState=RX_SYNC0; break; }
        rxState=RX_PAYLOAD; break;
      case RX_PAYLOAD:
        rxBuf[rxPos++]=b;
        if (rxPos>=rxLen+1) {
          if (rxBuf[rxLen]==crc8(rxBuf,rxLen)) processCommand(rxBuf,rxLen);
          rxState=RX_SYNC0;
        }
        break;
    }
  }
}

// ── Load Cell ─────────────────────────────────────
bool loadCellInit() {
  I2C1Bus.beginTransmission(ZSC31014_ADDR);
  if (I2C1Bus.endTransmission()!=0) return false;
  I2C1Bus.requestFrom((uint8_t)ZSC31014_ADDR,(uint8_t)2);
  while(I2C1Bus.available()) I2C1Bus.read();
  delay(5); return true;
}

void loadCellMeasurementRequest() {
  I2C1Bus.requestFrom((uint8_t)ZSC31014_ADDR,(uint8_t)2);
  while(I2C1Bus.available()) I2C1Bus.read();
}

bool loadCellFetch(uint16_t &br, uint8_t &st) {
  if (I2C1Bus.requestFrom((uint8_t)ZSC31014_ADDR,(uint8_t)2)<2) return false;
  uint8_t msb=I2C1Bus.read(), lsb=I2C1Bus.read();
  st=(msb>>6)&0x03; br=((uint16_t)(msb&0x3F)<<8)|lsb; return true;
}

bool loadCellRead(uint16_t &br, uint8_t &st) {
  loadCellMeasurementRequest(); delay(2);
  uint32_t s=millis();
  while(millis()-s<50) {
    if(!loadCellFetch(br,st)) return false;
    if(st==LC_STATUS_VALID) return true;
    delayMicroseconds(500);
  }
  return false;
}

// LC read — non-blocking two-phase (I2C only; fully independent of the servo/RS485)
void loadCellUpdate(){
  if(!ctrl.loadCellOK) return;
  unsigned long now=millis();
  if(ctrl.lcPhase==LC_REQUEST){
    loadCellMeasurementRequest();
    ctrl.lcRequestTime=now; ctrl.lcPhase=LC_WAIT;
  } else if(now-ctrl.lcRequestTime>=2){
    uint16_t br; uint8_t st;
    if(loadCellFetch(br,st)&&st==LC_STATUS_VALID){
      ctrl.lastBridge=br;
      ctrl.maSum-=ctrl.maBuf[ctrl.maIdx];
      ctrl.maBuf[ctrl.maIdx]=br; ctrl.maSum+=br;
      ctrl.maIdx=(ctrl.maIdx+1)%ctrl.MA_SIZE;
      ctrl.lastBridgeFilt=(uint16_t)(ctrl.maSum/ctrl.MA_SIZE);
      ctrl.baseRead=(int16_t)ctrl.lastBridgeFilt-ctrl.zeroOffset;
      ctrl.lastLCStatus=st; ctrl.lastLCValid=true;
      ctrl.lcPhase=LC_REQUEST;
    }
  }
}

#define LC_PREAMP_GAIN   0b111
#define LC_A2D_OFFSET    0x08
#define LC_GAIN_POLARITY 1
#define LC_DISABLE_NULLING 0

static const uint16_t OFFSET_B_LUT[]={
  0xE000,0xE400,0xE800,0xEC00,0xF000,0xF400,0xF800,0xFC00,
  0x0000,0x0400,0x0800,0x0C00,0x1000,0x1400,0x1800,0x1C00
};

static bool zscCmd(uint8_t cmd,uint16_t d){
  I2C1Bus.beginTransmission(ZSC31014_ADDR);
  I2C1Bus.write(cmd);I2C1Bus.write((d>>8)&0xFF);I2C1Bus.write(d&0xFF);
  return I2C1Bus.endTransmission()==0;
}
static bool zscResp(uint16_t &v){
  if(I2C1Bus.requestFrom((uint8_t)ZSC31014_ADDR,(uint8_t)3)<3) return false;
  uint8_t a=I2C1Bus.read(),m=I2C1Bus.read(),l=I2C1Bus.read();
  if(a!=0x5A) return false; v=((uint16_t)m<<8)|l; return true;
}

bool loadCellConfigureEEPROM(){
  if(!zscCmd(0xA0,0)) return false; delayMicroseconds(100);
  bool ok=false; uint16_t cur=0;
  if(!zscCmd(0x0F,0)) goto done; delayMicroseconds(100);
  if(!zscResp(cur)) goto done;
  {
    uint16_t des=(cur&0xE000)|((uint16_t)(LC_DISABLE_NULLING&1)<<12)|(0b10<<10)
      |(1<<9)|(1<<8)|((uint16_t)(LC_GAIN_POLARITY&1)<<7)
      |((uint16_t)(LC_PREAMP_GAIN&7)<<4)|(LC_A2D_OFFSET&0x0F);
    if(cur!=des){
      if(!zscCmd(0x4F,des)) goto done; delay(15);
      if(!zscCmd(0x43,OFFSET_B_LUT[LC_A2D_OFFSET&0x0F])) goto done; delay(15);
    }
  }
  ok=true;
done:
  zscCmd(0x80,0); delay(15); return ok;
}

// ── Servo (single owner: Core0) ───────────────────
void servoInit(){
  pinMode(MB_EN_GPIO,OUTPUT); digitalWrite(MB_EN_GPIO,LOW);
  Serial485.begin(115200); Serial485.setTimeout(100);
  servo.preTransmission(preTransmission);
  servo.postTransmission(postTransmission);
}

static uint8_t servoWriteReg(uint16_t reg,uint16_t val){
  return servo.writeSingleRegister(reg,val);
}

bool servoConfigure(uint8_t id){
  servo.begin(id,Serial485); Serial485.setTimeout(50);
  servo.writeSingleRegister(MB_REG_SERVO_ENABLE,0); delay(50);

  if(servoWriteReg(MB_REG_CONTROL_MODE, 1)!=0) return false;
  if(servoWriteReg(MB_REG_SPEED_REF,    0)!=0) return false;
  servoWriteReg(MB_C00_05,  ctrl.stiffness);
  servoWriteReg(MB_C03_22,  ctrl.accelRate);
  servoWriteReg(MB_C03_24,  ctrl.decelRate);
  servoWriteReg(MB_REG_BRAKE_RES_SEL,  BRAKE_RES_SEL_VAL);
  servoWriteReg(MB_REG_BRAKE_RES_POW,  BRAKE_RES_POW_VAL);
  servoWriteReg(MB_REG_BRAKE_RES_OHM,  BRAKE_RES_OHM_VAL);
  servoWriteReg(MB_REG_BRAKE_RES_DISS, BRAKE_RES_DISS_VAL);
  if(servoWriteReg(MB_REG_SERVO_ENABLE, 1)!=0) return false;

  return true;
}

// Standalone bus scan (blocking, NOT part of the control loop). Returns id or 0.
// Slow: ModbusMaster's response timeout (~1 s) applies per empty address.
uint8_t scanServo(){
  for(uint8_t id=1; id<=247; id++){
    servo.begin(id,Serial485);
    if(servo.readHoldingRegisters(MB_REG_CONTROL_MODE,1)==servo.ku8MBSuccess) return id;
  }
  return 0;
}

// ═══ The ONLY two functions that touch the RS485 bus in the loop ═══

// READ — pull RPM/torque every loop (loop rate is capped by LOOP_MS)
void servoReadState(){
  if(sv.phase!=SV_RUNNING || !sv.connected) return;   // don't read a dead bus
  servo.begin(sv.id,Serial485);
  sv.rpmStatus=servo.readHoldingRegisters(MB_REG_MON_RPM,1);
  if(sv.rpmStatus==servo.ku8MBSuccess) sv.rpm=(int16_t)servo.getResponseBuffer(0);
  delayMicroseconds(700);   // Modbus RTU inter-frame silence (>3.5 char @115200) before next frame
  sv.trqStatus=servo.readHoldingRegisters(MB_REG_MON_TORQUE,1);
  if(sv.trqStatus==servo.ku8MBSuccess) sv.torqueX10=(int16_t)servo.getResponseBuffer(0);
}

// ACT — servo state machine + all writes (one call, end of loop). Known address, no scan.
void servoAction(){
  if(sv.reqShutdown){
    sv.reqShutdown=false;
    if(sv.id){ servo.begin(sv.id,Serial485);
               servo.writeSingleRegister(MB_REG_SERVO_ENABLE,0); }
    sv.connected=false; sv.rpm=0; sv.torqueX10=0; sv.speedCmd=0;
    sv.phase=SV_IDLE;
    return;
  }

  switch(sv.phase){
    case SV_IDLE:
    case SV_FAULT:
      break;

    case SV_CONFIGURING:                     // connect directly to sv.id (set at Init)
      if(servoConfigure(sv.id)){
        sv.connected=true; sv.failCount=0; sv.cfgRetries=0;
        sv.rpm=0; sv.torqueX10=0; sv.speedCmd=0;
        ctrl.errorCode=ERR_NONE; sv.phase=SV_RUNNING;
      } else if(++sv.cfgRetries>=3){
        ctrl.errorCode=ERR_SERVO_CONFIG_FAIL; sv.phase=SV_FAULT;
      }
      break;

    case SV_RUNNING:                         // one write/loop; retries same id, never scans
      servo.begin(sv.id,Serial485);
      if(servo.writeSingleRegister(MB_REG_SPEED_REF,(uint16_t)sv.speedCmd)==servo.ku8MBSuccess){
        sv.failCount=0; sv.connected=true;
      } else {
        sv.connected=false; sv.rpm=0; sv.torqueX10=0;
        if(sv.failCount<255) sv.failCount++;
      }
      break;
  }
}

// ── Setup / Loop (Core0) ──────────────────────────
void setup(){
  Serial.begin(115200);
  { unsigned long t0=millis(); while(!Serial&&millis()-t0<3000) delay(10); }

  pinMode(PIN_LIMIT_A, INPUT_PULLUP);
  pinMode(PIN_LIMIT_B, INPUT_PULLUP);
  pinMode(1,OUTPUT); digitalWrite(1,HIGH); delay(50);
  I2C1Bus.begin(); I2C1Bus.setClock(LC_I2C_FREQ);

  ctrl.lcConfigApplied = loadCellConfigureEEPROM();
  ctrl.loadCellOK = loadCellInit();

  servoInit();                 // always bring up RS485 — servo runs even if the LC is absent
  Serial.flush(); delay(50);
}

void loop(){
  unsigned long now=millis();

  // ══ READ — independent subsystem reads ═══════════
  servoReadState();      // servo  (RS485)
  loadCellUpdate();      // load cell (I2C) — fully independent

  // ══ LOGIC — no bus access, pure state ════════════
  checkCommands();                         // parse USB commands → sv/ctrl

  // safety: force limit + endstops → zero the speed target
  if(ctrl.forceLimit>0 && ctrl.lastLCValid && sv.phase==SV_RUNNING && sv.speedCmd!=0){
    int16_t af=(ctrl.baseRead<0)?-ctrl.baseRead:ctrl.baseRead;
    if((uint16_t)af>ctrl.forceLimit) sv.speedCmd=0;
  }
  if(digitalRead(PIN_LIMIT_A)==HIGH && sv.speedCmd<0) sv.speedCmd=0;  // running into A
  if(digitalRead(PIN_LIMIT_B)==HIGH && sv.speedCmd>0) sv.speedCmd=0;  // running into B

  // publish servo values + derive app state for telemetry
  ctrl.motorRPM=sv.rpm; ctrl.motorTorqueX10=sv.torqueX10;
  ctrl.servoConnected=sv.connected; ctrl.currentSpeedCmd=sv.speedCmd;
  switch(sv.phase){                          // app state follows the servo (LC is independent)
    case SV_IDLE:        ctrl.currentState=STATE_IDLE;        break;
    case SV_CONFIGURING: ctrl.currentState=STATE_CONFIGURING; break;
    case SV_RUNNING:     ctrl.currentState=(sv.speedCmd<0)?STATE_MOVING_A
                                          :(sv.speedCmd>0)?STATE_MOVING_B:STATE_STOPPED; break;
    case SV_FAULT:       ctrl.currentState=STATE_ERROR;       break;
  }

  // ══ ACT — one call, servo state machine + all writes ══
  servoAction();

  // ══ record this loop, flush the whole batch every 200 ms (or if full) ══
  recordSample(now);
  uint32_t period=(ctrl.currentState==STATE_ERROR)?1000:TELEMETRY_MS;
  if(sampleCount>=MAX_SAMPLES || now-ctrl.lastTelemetryTime>=period){
    telemetryFlush();
    ctrl.lastTelemetryTime=now;
  }

  // ══ pace the loop to a fixed period (caps read/bus rate) ══
  unsigned long dt=millis()-now;
  if(dt<LOOP_MS) delay(LOOP_MS-dt);
}
