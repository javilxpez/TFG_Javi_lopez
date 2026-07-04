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

// ── Core1 Modbus Worker ───────────────────────────
struct MBWorker {
  volatile bool    active    = false;
  volatile bool    busy      = false;
  volatile uint8_t slaveID   = 0;
  volatile int16_t speedCmd  = 0;
  volatile bool    connected = false;
  volatile uint8_t failCount = 0;
  volatile int16_t motorRPM  = 0;
  volatile int16_t motorTorqueX10 = 0;
} mbw;

static void stopCore1() {
  mbw.active = false;
  unsigned long t = millis();
  while (mbw.busy && millis() - t < 100) delayMicroseconds(100);
}

// ── Modbus ────────────────────────────────────────
static SerialPIO Serial485(MB_TX_GPIO, MB_RX_GPIO);
ModbusMaster servo;

void preTransmission()  { digitalWrite(MB_EN_GPIO, HIGH); }
void postTransmission() { delayMicroseconds(200); digitalWrite(MB_EN_GPIO, LOW); }

void setup1() {}

void loop1() {
  if (!mbw.active || mbw.slaveID == 0) { delay(1); return; }
  mbw.busy = true;
  servo.begin(mbw.slaveID, Serial485);
  Serial485.setTimeout(20);

  uint8_t r = servo.writeSingleRegister(MB_REG_SPEED_REF, (uint16_t)mbw.speedCmd);
  if (r == 0) {
    mbw.failCount = 0;
    mbw.connected = true;
    static uint8_t monCnt = 0;
    if (++monCnt >= 10) {
      monCnt = 0;
      if (servo.readHoldingRegisters(MB_REG_MON_RPM, 1) == 0)
        mbw.motorRPM = (int16_t)servo.getResponseBuffer(0);
      if (servo.readHoldingRegisters(MB_REG_MON_TORQUE, 1) == 0)
        mbw.motorTorqueX10 = (int16_t)servo.getResponseBuffer(0);
    }
  } else {
    if (++mbw.failCount >= 5) {
      mbw.connected      = false;
      mbw.motorRPM       = 0;
      mbw.motorTorqueX10 = 0;
    }
  }
  mbw.busy = false;
}

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

// ── Telemetría (19 bytes) ─────────────────────────
//  0     packet_id
//  1-4   t_ms
//  5-6   bridge
//  7     lc_status
//  8     flags (b0=lc_valid, b1=servo_conn, b2=lc_cfg)
//  9-10  rpm (int16)
//  11-12 torque_x10 (int16)
//  13    state
//  14    mode
//  15-16 speed_cmd (int16)
//  17-18 base_read (int16)
static uint8_t txBuf[64];

void telemetrySend(uint32_t t, uint8_t modeFlag) {
  const uint8_t payloadLen = 19;
  uint8_t *p = txBuf;

  uint8_t flags;
  if (ctrl.currentState == STATE_ERROR)
    flags = ctrl.errorCode;
  else
    flags = (ctrl.lastLCValid     ? 0x01 : 0x00)
          | (ctrl.servoConnected  ? 0x02 : 0x00)
          | (ctrl.lcConfigApplied ? 0x04 : 0x00);

  *p++ = PACKET_ID;
  *p++ = (t>> 0)&0xFF; *p++ = (t>> 8)&0xFF;
  *p++ = (t>>16)&0xFF; *p++ = (t>>24)&0xFF;
  *p++ = (ctrl.lastBridge>>0)&0xFF; *p++ = (ctrl.lastBridge>>8)&0xFF;
  *p++ = ctrl.lastLCStatus;
  *p++ = flags;
  *p++ = (ctrl.motorRPM>>0)&0xFF;      *p++ = (ctrl.motorRPM>>8)&0xFF;
  *p++ = (ctrl.motorTorqueX10>>0)&0xFF; *p++ = (ctrl.motorTorqueX10>>8)&0xFF;
  *p++ = (uint8_t)ctrl.currentState;
  *p++ = modeFlag;
  *p++ = (ctrl.currentSpeedCmd>>0)&0xFF; *p++ = (ctrl.currentSpeedCmd>>8)&0xFF;
  *p++ = (ctrl.baseRead>>0)&0xFF;        *p++ = (ctrl.baseRead>>8)&0xFF;

  uint8_t frame[3 + payloadLen + 1];
  frame[0]=SYNC_0; frame[1]=SYNC_1; frame[2]=payloadLen;
  memcpy(&frame[3], txBuf, payloadLen);
  frame[3+payloadLen] = crc8(txBuf, payloadLen);
  Serial.write(frame, sizeof(frame));
}

// ── Movimiento ────────────────────────────────────
static void startMovement(int16_t speed) {
  ctrl.currentSpeedCmd = speed;
  mbw.speedCmd  = speed;
  mbw.slaveID   = ctrl.servoID;
  mbw.failCount = 0;
  mbw.connected = true;
  if (!mbw.active) mbw.active = true;
}

static void stopMovement() {
  ctrl.currentSpeedCmd = 0;
  mbw.speedCmd = 0;
}

// ── Comandos ─────────────────────────────────────
static uint8_t rxBuf[64];
static uint8_t rxPos = 0;
static enum { RX_SYNC0, RX_SYNC1, RX_LEN, RX_PAYLOAD } rxState = RX_SYNC0;
static uint8_t rxLen = 0;

void processCommand(const uint8_t *payload, uint8_t len) {
  if (len < 1) return;
  switch (payload[0]) {

    case CMD_INIT:
      if (ctrl.currentState == STATE_ERROR) break;
      if (mbw.active) { stopCore1(); ctrl.currentSpeedCmd = 0; }
      ctrl.scanID = 1; ctrl.servoID = 0;
      ctrl.scanStartTime = millis(); ctrl.configureRetries = 0;
      ctrl.currentState = STATE_SCANNING;
      Serial.println("# CMD_INIT");
      break;

    case CMD_STOP:
      if (ctrl.currentState == STATE_MOVING_A || ctrl.currentState == STATE_MOVING_B) {
        stopMovement();
        ctrl.currentState = STATE_STOPPED;
        Serial.println("# CMD_STOP");
      } else if (ctrl.currentState == STATE_SCANNING || ctrl.currentState == STATE_CONFIGURING) {
        if (mbw.active) stopCore1();
        ctrl.currentState = STATE_IDLE;
      }
      break;

    case CMD_MOVE_A:
      if (ctrl.currentState == STATE_STOPPED || ctrl.currentState == STATE_MOVING_B) {
        startMovement(-ctrl.moveSpeed);
        ctrl.currentState = STATE_MOVING_A;
        Serial.printf("# CMD_MOVE_A: %d RPM\n", -ctrl.moveSpeed);
      }
      break;

    case CMD_MOVE_B:
      if (ctrl.currentState == STATE_STOPPED || ctrl.currentState == STATE_MOVING_A) {
        startMovement(ctrl.moveSpeed);
        ctrl.currentState = STATE_MOVING_B;
        Serial.printf("# CMD_MOVE_B: +%d RPM\n", ctrl.moveSpeed);
      }
      break;

    case CMD_SHUTDOWN:
      if (mbw.active) stopCore1();
      ctrl.currentSpeedCmd = 0;
      if (ctrl.servoID > 0) {
        servo.begin(ctrl.servoID, Serial485);
        servo.writeSingleRegister(MB_REG_SERVO_ENABLE, 0);
      }
      ctrl.servoConnected = false;
      ctrl.motorRPM = 0; ctrl.motorTorqueX10 = 0;
      ctrl.currentState = STATE_IDLE;
      Serial.println("# CMD_SHUTDOWN");
      break;

    case CMD_SET_PARAM:
      if (len >= 4) {
        uint8_t  pid   = payload[1];
        int16_t  value = (int16_t)(payload[2] | ((uint16_t)payload[3] << 8));
        switch (pid) {
          case PARAM_SPEED:
            ctrl.moveSpeed = (value > 0) ? value : -value;
            if (ctrl.currentState == STATE_MOVING_A) { mbw.speedCmd = -ctrl.moveSpeed; ctrl.currentSpeedCmd = -ctrl.moveSpeed; }
            if (ctrl.currentState == STATE_MOVING_B) { mbw.speedCmd =  ctrl.moveSpeed; ctrl.currentSpeedCmd =  ctrl.moveSpeed; }
            Serial.printf("# PARAM_SPEED=%d RPM\n", ctrl.moveSpeed);
            break;
          case PARAM_FORCE_LIMIT:
            ctrl.forceLimit = (uint16_t)value;
            Serial.printf("# PARAM_FORCE_LIMIT=%u\n", ctrl.forceLimit);
            break;
          case PARAM_ZERO_OFFSET:
            ctrl.zeroOffset = value;
            Serial.printf("# PARAM_ZERO_OFFSET=%d\n", value);
            break;
          case PARAM_ACCEL:
            ctrl.accelRate = value;
            Serial.printf("# PARAM_ACCEL=%d\n", value);
            break;
          case PARAM_DECEL:
            ctrl.decelRate = value;
            Serial.printf("# PARAM_DECEL=%d\n", value);
            break;
          case PARAM_STIFFNESS:
            ctrl.stiffness = value;
            Serial.printf("# PARAM_STIFFNESS=%d\n", value);
            break;
          case PARAM_FORCE_CALIB:
            ctrl.forceCalib = value;
            Serial.printf("# PARAM_FORCE_CALIB=%d cnt/N\n", value);
            break;
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

// ── Servo (Core0) ─────────────────────────────────
void servoInit(){
  pinMode(MB_EN_GPIO,OUTPUT); digitalWrite(MB_EN_GPIO,LOW);
  Serial485.begin(115200); Serial485.setTimeout(100);
  servo.preTransmission(preTransmission);
  servo.postTransmission(postTransmission);
}

bool servoScanStep(){
  servo.begin(ctrl.scanID,Serial485);
  if(servo.readHoldingRegisters(MB_REG_CONTROL_MODE,1)==servo.ku8MBSuccess){
    ctrl.servoID=ctrl.scanID; return true;
  }
  if(++ctrl.scanID>247) ctrl.scanID=1; return false;
}

static uint8_t servoWriteReg(uint16_t reg,uint16_t val,const char *name){
  uint8_t r=servo.writeSingleRegister(reg,val);
  if(r) Serial.printf("# FAIL %s err=0x%02X\n",name,r);
  else  Serial.printf("# OK   %s=%u\n",name,val);
  return r;
}

bool servoConfigure(uint8_t id){
  servo.begin(id,Serial485);
  Serial.printf("# servoConfigure ID=%u\n",id);
  servo.writeSingleRegister(MB_REG_SERVO_ENABLE,0); delay(50);

  if(servoWriteReg(MB_REG_CONTROL_MODE, 1,                  "ctrl_mode")!=0) return false;
  if(servoWriteReg(MB_REG_SPEED_REF,    0,                  "speed_ref")!=0) return false;
  servoWriteReg(MB_C00_05,  ctrl.stiffness,                  "stiffness");
  servoWriteReg(MB_C03_22,  ctrl.accelRate,                  "accel");
  servoWriteReg(MB_C03_24,  ctrl.decelRate,                  "decel");
  servoWriteReg(MB_REG_BRAKE_RES_SEL,  BRAKE_RES_SEL_VAL,   "brk_sel");
  servoWriteReg(MB_REG_BRAKE_RES_POW,  BRAKE_RES_POW_VAL,   "brk_pow");
  servoWriteReg(MB_REG_BRAKE_RES_OHM,  BRAKE_RES_OHM_VAL,   "brk_ohm");
  servoWriteReg(MB_REG_BRAKE_RES_DISS, BRAKE_RES_DISS_VAL,  "brk_diss");
  if(servoWriteReg(MB_REG_SERVO_ENABLE, 1,                  "servo_on" )!=0) return false;

  Serial.printf("# servoConfigure OK (stiffness=%d)\n", ctrl.stiffness);
  return true;
}

// ── Setup / Loop (Core0) ──────────────────────────
void setup(){
  Serial.begin(115200);
  { unsigned long t0=millis(); while(!Serial&&millis()-t0<3000) delay(10); }
  Serial.println("# Firmware Posicion A→B — Pico 2");
  Serial.println("# i=init  b=mover_B  a=mover_A  x=stop  q=shutdown");

  pinMode(1,OUTPUT); digitalWrite(1,HIGH); delay(50);
  I2C1Bus.begin(); I2C1Bus.setClock(LC_I2C_FREQ);

  ctrl.lcConfigApplied = loadCellConfigureEEPROM();
  ctrl.loadCellOK = loadCellInit();
  Serial.printf("# LC: %s\n", ctrl.loadCellOK?"OK":"NOT FOUND");

  if(!ctrl.loadCellOK){
    ctrl.errorCode=ERR_LC_NOT_FOUND;
    ctrl.currentState=STATE_ERROR;
    Serial.flush(); delay(50); return;
  }
  servoInit();
  Serial.flush(); delay(50);
}

void loop(){
  unsigned long now=millis();
  checkCommands();

  uint8_t modeFlag=MODE_STOPPED;
  if(ctrl.currentState==STATE_MOVING_A) modeFlag=MODE_MOVING_A;
  if(ctrl.currentState==STATE_MOVING_B) modeFlag=MODE_MOVING_B;

  switch(ctrl.currentState){

    case STATE_IDLE:
      if(now-ctrl.lastTelemetryTime>=200){
        uint16_t br; uint8_t st;
        if(ctrl.loadCellOK&&loadCellRead(br,st)){
          ctrl.lastBridge=br; ctrl.lastLCStatus=st;
          ctrl.lastLCValid=(st==LC_STATUS_VALID);
          ctrl.baseRead=(int16_t)br-ctrl.zeroOffset;
        }
        telemetrySend(now,MODE_STOPPED);
        ctrl.lastTelemetryTime=now;
      }
      break;

    case STATE_SCANNING:
      if(now-ctrl.scanStartTime>10000){
        ctrl.errorCode=ERR_SERVO_SCAN_FAIL;
        ctrl.currentState=STATE_ERROR;
        break;
      }
      if(now-ctrl.lastActionTime>=15){
        if(servoScanStep()) ctrl.currentState=STATE_CONFIGURING;
        ctrl.lastActionTime=now;
      }
      if(now-ctrl.lastTelemetryTime>=200){
        uint16_t br; uint8_t st;
        if(ctrl.loadCellOK&&loadCellRead(br,st)){
          ctrl.lastBridge=br; ctrl.lastLCStatus=st;
          ctrl.lastLCValid=(st==LC_STATUS_VALID);
          ctrl.baseRead=(int16_t)br-ctrl.zeroOffset;
        }
        telemetrySend(now,MODE_STOPPED);
        ctrl.lastTelemetryTime=now;
      }
      break;

    case STATE_CONFIGURING:
      if(servoConfigure(ctrl.servoID)){
        ctrl.configureRetries=0;
        ctrl.errorCode=ERR_NONE;
        ctrl.servoConnected=true;
        ctrl.currentSpeedCmd=0;
        mbw.slaveID=ctrl.servoID;
        mbw.speedCmd=0;
        mbw.failCount=0;
        mbw.connected=true;
        mbw.motorRPM=0;
        mbw.motorTorqueX10=0;
        mbw.active=true;
        ctrl.lcPhase=LC_REQUEST;
        ctrl.currentState=STATE_STOPPED;
      } else {
        if(++ctrl.configureRetries>=3){
          ctrl.errorCode=ERR_SERVO_CONFIG_FAIL;
          ctrl.currentState=STATE_ERROR;
        } else {
          ctrl.scanID=ctrl.servoID; ctrl.servoID=0;
          ctrl.scanStartTime=millis();
          ctrl.currentState=STATE_SCANNING;
        }
      }
      break;

    case STATE_STOPPED:
    case STATE_MOVING_A:
    case STATE_MOVING_B:
      {
        if(ctrl.loadCellOK){
          if(ctrl.lcPhase==LC_REQUEST){
            loadCellMeasurementRequest();
            ctrl.lcRequestTime=now;
            ctrl.lcPhase=LC_WAIT;
          } else if(now-ctrl.lcRequestTime>=2){
            uint16_t br; uint8_t st;
            if(loadCellFetch(br,st)&&st==LC_STATUS_VALID){
              ctrl.lastBridge=br;
              ctrl.maSum-=ctrl.maBuf[ctrl.maIdx];
              ctrl.maBuf[ctrl.maIdx]=br;
              ctrl.maSum+=br;
              ctrl.maIdx=(ctrl.maIdx+1)%ctrl.MA_SIZE;
              ctrl.lastBridgeFilt=(uint16_t)(ctrl.maSum/ctrl.MA_SIZE);
              ctrl.baseRead=(int16_t)ctrl.lastBridgeFilt-ctrl.zeroOffset;
              ctrl.lastLCStatus=st;
              ctrl.lastLCValid=true;
              ctrl.lcPhase=LC_REQUEST;
            }
          }
        }

        ctrl.motorRPM      =mbw.motorRPM;
        ctrl.motorTorqueX10=mbw.motorTorqueX10;
        ctrl.servoConnected=mbw.connected;

        if(ctrl.forceLimit>0&&ctrl.lastLCValid){
          int16_t af=(ctrl.baseRead<0)?-ctrl.baseRead:ctrl.baseRead;
          if((uint16_t)af>ctrl.forceLimit&&
             (ctrl.currentState==STATE_MOVING_A||ctrl.currentState==STATE_MOVING_B)){
            stopMovement();
            ctrl.currentState=STATE_STOPPED;
            Serial.printf("# FORCE LIMIT |%d|>%u\n",ctrl.baseRead,ctrl.forceLimit);
          }
        }

        if(!mbw.connected&&mbw.failCount>=5){
          stopCore1();
          ctrl.servoConnected=false; ctrl.motorRPM=0; ctrl.motorTorqueX10=0;
          ctrl.currentSpeedCmd=0;
          ctrl.scanID=ctrl.servoID; ctrl.servoID=0;
          ctrl.scanStartTime=millis(); ctrl.configureRetries=0;
          Serial485.setTimeout(100);
          ctrl.currentState=STATE_SCANNING;
          break;
        }

        if(now-ctrl.lastTelemetryTime>=200){
          telemetrySend(now,modeFlag);
          ctrl.lastTelemetryTime=now;
        }
      }
      break;

    case STATE_ERROR:
      if(now-ctrl.lastTelemetryTime>=1000){
        telemetrySend(now,MODE_STOPPED);
        ctrl.lastTelemetryTime=now;
      }
      break;
  }
}
