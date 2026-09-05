// ═══════════════════════════════════════════════════════════
//  ZSC31014 Load Cell + Servo Motor — Raspberry Pi Pico 2  (v2)
//  Modo velocidad A→B — sin lazo cerrado de célula de carga
//  Dual-core: Core0 = LC/telemetría/comandos, Core1 = Modbus
//
//  ── NOVEDAD v2: HOMING con detección de ambos finales de carrera ──
//  Secuencia: busca A (datum, pos=0) → busca B (mide recorrido A→B)
//             → va a un offset configurable desde A → fija HOME=0.
//  La posición se integra a partir de RPM DENTRO del firmware (rev).
//  Comando: CMD_HOME (0x08) o tecla 'h'. Aborta con STOP.
//  Offset: PARAM_HOME_OFFSET (centi-rev). Velocidad: PARAM_HOMING_SPEED (RPM).
//  Estado/recorrido se reportan en un frame propio (PACKET_ID_HOME=0x03).
//
//  ── v2.1: SEGUNDA CÉLULA DE CARGA ──────────────────────────────
//  Load Cell 4 Click (NAU7802 @ 0x2A) en el mismo bus I2C1 que la
//  Load Cell 2 Click (ZSC31014 @ 0x28). Se lee sin filtrar y viaja
//  en cada muestra de telemetría (campo lc2, int32).
// ═══════════════════════════════════════════════════════════

#include <Wire.h>
#include <hardware/gpio.h>
#include <ModbusMaster.h>
#include <SerialPIO.h>
#include "i2c_bus.h"      // árbitro del bus I2C1: quién lo tiene y quién espera turno

static TwoWire I2C1Bus(i2c1, 2, 3);

// ── Pins ─────────────────────────────────────────
#define LC_SDA_PIN   2
#define LC_SCL_PIN   3
// 100 kHz. Se probó a 400 kHz y las lecturas se rompieron: no lo subas sin medirlo.
// El bus lo comparten el ZSC31014 y la NAU7802, con los pull-ups y la capacidad de dos
// Click en paralelo, y el ZSC además tiene requisitos propios de flancos (SDA no puede
// bajar entre el START y el primer flanco de SCL). A 100 kHz todo eso cumple holgado.
#define LC_I2C_FREQ     100000
// El modo comando se queda a 100 kHz a propósito: es la velocidad con la que se ha
// conseguido entrar en la ventana de 1.5 ms, y no se toca lo que ya funciona. Además
// ahí la velocidad no aprieta — sólo tiene que caber el Start_CM, y a 100 kHz son
// ~380 µs de los 1500 disponibles.
#define LC_CM_I2C_FREQ  100000
#define MB_TX_GPIO   14
#define MB_RX_GPIO   13
#define MB_EN_GPIO   12

#define LC1_EN_GPIO  1     // alimentación/enable Load Cell 2 Click (ZSC31014)
// GPIO al que llega el INT/SS de la Click. -1 = sin cablear / desconocido.
// IMPORTANTE: si la Click lleva pull-up en INT y ese pull-up cuelga del 3V3 del
// mikroBUS (que NO está gobernado por EN), la corriente entra por el pin INT del
// ZSC31014, cruza su diodo de protección y le sostiene el rail: el chip nunca se
// apaga del todo y no hay Power-On-Reset. Con el pad del Pico en reposo (entrada con
// pull-down de ~50 k) contra un pull-up de ~10 k salen ~2,7 V, que es justo el orden
// de lo medido. Poniendo aquí el GPIO correcto, el corte lo lleva a 0 de verdad.
#define LC1_INT_GPIO  4
// Durante el corte se le quita también la tensión a la LC2 (su enable es GP0). No es
// por su INT —GP0 ya es LC2_EN_GPIO, son redes distintas— sino porque la NAU7802
// comparte SDA/SCL: alimentada, sus propios pines sostienen el bus y vuelven a ofrecer
// un camino hacia el ZSC31014. Cuesta que la LC2 se reinicie después, cosa que la
// grabación ya hace de todas formas. Ponlo a 0 si prefieres no tocarla.
#define LC_PARK_LC2   1
#define LC2_EN_GPIO  0     // alimentación/enable Load Cell 4 Click (NAU7802)

#define ZSC31014_ADDR 0x28
// Ganancia y offset del A/D con los que ARRANCA la GUI. No se graban en el arranque:
// el chip usa lo último que se le programó y esto es sólo lo que se propone por
// defecto en la interfaz, hasta que el operario pulse "Programar" con el eje parado.
#define LC_PREAMP_GAIN   0b111
#define LC_A2D_OFFSET    0x08
#define LC_GAIN_POLARITY 1
#define LC_DISABLE_NULLING 0
#define LC_RESET_LOW_MS  250   // reset abajo: margen de sobra para que el rail caiga a 0
// La ventana de modo comando son 1.5 ms desde que sube la alimentación, y hay que
// llegar DESPUÉS de que el chip acabe de arrancar. Los 500 µs son los que le funcionan
// al driver de mbed; MikroE manda el Start_CM inmediatamente y reintenta el ciclo
// entero hasta que entra. Aquí se hacen las dos cosas: esperar poco y reintentar.
#define LC_CM_RAMP_MS     50   // espera máxima a que el chip vuelva a contestar tras el corte
#define LC_CM_ATTEMPTS     8   // ciclos de alimentación: cada uno prueba un retardo distinto
#define LC_CM_STEP_US    150   // el barrido va de 0 a 7·150 = 1050 µs, dentro de la ventana
#define LC_HOLD_MS      8000   // cuánto se mantiene el corte en el modo de medida
#define LC_EEPROM_WRITE_MS  20 // que cuaje una escritura de palabra
// La firma MISR (palabra 0x12) se regenera al SALIR del modo comando con Start_NOM, y
// sólo si hubo escritura. Si se corta la alimentación antes de que esa firma esté
// grabada, queda mal y el chip falla su comprobación en cada arranque: sale dando
// estado de diagnóstico para siempre. Los 15 ms de antes eran justos para una escritura
// de EEPROM; aquí no se escatima, que el precio de equivocarse es una placa inservible.
#define LC_EEPROM_SETTLE_MS 150
#define LC_STATUS_VALID   0x00
#define LC_STATUS_STALE   0x01
#define LC_STATUS_COMMAND 0x02

// ── LC2: Load Cell 4 Click — NAU7802 @ 0x2A, 24 bit con signo ──
// Config validada en test_loadcell_dual_v2 (ruido de reposo ~350 cuentas p-p):
// el LDO interno a 3.0 V (la Click va a 3V3, 4.5 V lo dejaba en dropout), el
// chopper del PGA ACTIVO (desactivarlo multiplica por 10 la deriva lenta) y la
// calibración interna de offset. AVDD debe salir del LDO interno: esta Click no
// alimenta el pin AVDD por fuera y con AVDDS=0 el ADC se clava en -8388608.
#define NAU7802_ADDR  0x2A
#define NAU_PU_CTRL   0x00
#define NAU_CTRL1     0x01
#define NAU_CTRL2     0x02
#define NAU_ADCO_B2   0x12
#define NAU_ADC_REG   0x15
#define NAU_PGA       0x1B
#define NAU_PWR_CTRL  0x1C
#define NAU_REVISION  0x1F

#define NAU_RR    0x01
#define NAU_PUD   0x02
#define NAU_PUA   0x04
#define NAU_PUR   0x08
#define NAU_CS    0x10
#define NAU_CR    0x20
#define NAU_AVDDS 0x80

#define NAU_GAIN   7     // x128
#define NAU_VLDO   5     // 3.0 V
#define NAU_RATE   3     // 80 SPS
#define NAU_CALMOD 0     // calibración interna de offset

// ── Protocolo binario ─────────────────────────────
#define SYNC_0     0xAA
#define SYNC_1     0x55
#define PACKET_ID       0x02   // telemetría por lotes
#define PACKET_ID_HOME  0x03   // estado de homing (posición/recorrido)
#define PACKET_ID_LC    0x04   // resultado de programar la EEPROM de la célula

// Comandos Python → Pico
#define CMD_INIT      0x01
#define CMD_STOP      0x02
#define CMD_SET_PARAM 0x03
#define CMD_MOVE_A    0x04
#define CMD_MOVE_B    0x05
#define CMD_SHUTDOWN  0x06
#define CMD_SCAN      0x07
#define CMD_HOME      0x08   // v2: lanza la rutina de homing
#define CMD_LC_PROGRAM 0x09  // graba ganancia/offset en la EEPROM de la célula (sólo parado)
#define CMD_LC_QUERY   0x0A  // lee lo que hay grabado, sin escribir nada (sólo parado)
#define CMD_LC_HOLD    0x0B  // mantiene la célula en reset para medirla con el polímetro

// Resultado de la última programación, para que la GUI diga qué ha pasado
enum LcProgResult : uint8_t {
  LCPROG_NONE      = 0,   // nunca se ha intentado
  LCPROG_OK        = 1,   // grabado y verificado tras el ciclo de alimentación
  LCPROG_UNCHANGED = 2,   // ya estaba así: no se ha tocado la EEPROM
  LCPROG_RUNNING   = 3,   // rechazado: el eje no está parado
  LCPROG_BUS_BUSY  = 4,   // no se pudo reservar el bus a tiempo
  LCPROG_NO_WINDOW = 5,   // el chip no entró en modo comando tras el reset
  LCPROG_FAILED    = 6,   // se escribió pero la relectura no coincide
  LCPROG_READ_ONLY = 7,   // sólo lectura de la config
  LCPROG_EN_DEAD   = 8,   // el pin de reset no corta: la célula sigue contestando con él abajo
  LCPROG_NO_ACK    = 9,   // la célula no contesta nada al reset: sin tensión o mal cableada
  LCPROG_HELD      = 10   // se ha mantenido en reset para medir; nada que programar
};

// Diagnóstico de la última entrada en modo comando. Sin esto, "no entró en modo
// comando" tapa tres averías muy distintas: que el pin de reset no corte la tensión,
// que la célula no esté ni conectada, o que conteste algo que no es el 0x5A esperado.
// Vive aquí arriba porque telemetryFlushLc() lo manda a la GUI mucho antes de que el
// fichero llegue a zscResp(), que es quien lo rellena.
static uint8_t lcDiagAck   = 0xFF;   // primer byte de la última respuesta leída
static bool    lcDiagEnCut = false;  // ¿se calló al bajar el pin de reset?
static bool    lcDiagWrAck = false;  // ¿llegó a hacer ACK a algún Start_CM?
static uint8_t lcDiagAttempt = 0;    // qué intento del barrido entró (retardo = n·150 µs)

// Cómo se dejan SDA, SCL e INT mientras la célula está sin tensión: 0 = alta impedancia
// (sueltas), 1 = forzadas a masa.
//
// MEDIDO en esta placa: sólo funciona forzándolas a masa. Soltándolas, el rail se queda
// en ~2,5 V y el chip nunca se apaga — es decir, los pull-ups de la Click cuelgan del
// 3V3 permanente, no del rail gobernado por EN, y realimentan al ZSC31014 por los
// diodos de protección de sus pines. Por eso el valor por defecto es 1: no es una
// preferencia, es lo único que abre la ventana de modo comando.
#define LC_PARK_DEFAULT 1
static uint8_t lcParkMode = LC_PARK_DEFAULT;

// IDs de parámetro
#define PARAM_SPEED        0x01
#define PARAM_FORCE_LIMIT  0x02
#define PARAM_ZERO_OFFSET  0x03
#define PARAM_ACCEL        0x04
#define PARAM_DECEL        0x05
#define PARAM_STIFFNESS    0x06
#define PARAM_FORCE_CALIB  0x07
#define PARAM_HOME_OFFSET  0x08   // v2: offset del home desde A, en centi-rev (0.01 rev)
#define PARAM_HOMING_SPEED 0x09   // v2: velocidad de búsqueda del homing (RPM)

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

// ── Homing (v2) ───────────────────────────────────
#define HOMING_TIMEOUT_MS  30000   // seguridad: aborta un tramo que tarda demasiado
#define HOMING_TOL_REV     0.02f   // tolerancia para dar por alcanzado el home
#define HOMING_REPORT_MS   1500    // tras DONE/FAIL, sigue reportando este tiempo

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
  STATE_ERROR       = 6,
  STATE_HOMING      = 7,   // v2: rutina de homing en curso
  STATE_PROGRAMMING = 8    // grabando la EEPROM de la célula: el resto está parado
};
#define MODE_STOPPED  0
#define MODE_MOVING_A 1
#define MODE_MOVING_B 2

// ── Fases del homing (v2) ─────────────────────────
enum HomingPhase {
  HOME_IDLE  = 0,   // sin homing
  HOME_SEEK_A,      // buscando final A (datum → pos=0)
  HOME_SEEK_B,      // buscando final B (mide recorrido A→B)
  HOME_GOTO,        // yendo al offset deseado desde A
  HOME_DONE,        // completado (home fijado)
  HOME_FAIL         // abortado (timeout / servo no listo)
};

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
  bool     lcProgBusy      = false;         // grabando: el resto del sistema está parado
  uint8_t  lcProgResult    = LCPROG_NONE;   // resultado de la última programación
  uint8_t  lcProgGain      = LC_PREAMP_GAIN;   // lo que hay grabado ahora mismo
  uint8_t  lcProgOffset    = LC_A2D_OFFSET;
  uint16_t lcCfgBefore     = 0;             // palabra 0x0F antes y después, para la GUI
  uint16_t lcCfgAfter      = 0;
  unsigned long lcLastGood = 0;   // última muestra válida, para invalidar por timeout
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

  // ── Segunda célula (NAU7802) — sin filtrar, tal cual sale del ADC ──
  bool     lc2OK           = false;
  bool     lc2CalOK        = false;
  bool     lc2Valid        = false;
  int32_t  lc2Raw          = 0;
  uint8_t  lc2Discard      = 0;      // muestras a tirar tras (re)configurar
  unsigned long lc2LastGood = 0;

  // ── Posición + Homing (v2) ──
  float    posRev          = 0.0f;   // posición integrada de RPM (rev), signo + = hacia B
  unsigned long lastPosTime = 0;     // t de la última integración (0 = sin base aún)
  HomingPhase homePhase    = HOME_IDLE;
  float    rangeRev        = 0.0f;   // recorrido A→B medido (rev)
  float    homeOffsetRev   = 0.0f;   // offset deseado del home desde A (rev)
  int16_t  homingSpeed     = 8;      // velocidad de búsqueda del homing (RPM, lenta)
  bool     homeRangeValid  = false;  // true tras medir A→B con éxito
  unsigned long homePhaseStart  = 0; // t de inicio del tramo actual (timeout)
  unsigned long homeReportUntil = 0; // seguir reportando tras DONE/FAIL hasta este t
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
// ── Muestra (14 B) × n_samples — una por iteración de loop ──
//  +0-1  dt_ms       (ms desde base_t — timestamp exacto del loop: millis())
//  +2-3  rpm         (int16)
//  +4-5  current_x10 (int16)
//  +6-7  ref_cmd     (int16)
//  +8-9  base_read   (int16)
//  +10   io          (b0=limit_A, b1=limit_B)
//  +11   state
//  +12-13 work_us    (duración de trabajo del loop en µs, sin el pacing — rendimiento)
//  +14-17 lc2        (int32 — célula 2 en crudo, 24 bit con signo)
#define MAX_SAMPLES 8      // LOOP_MS=50 → ~4-5 por ventana de 200 ms; margen a 8
struct Sample {
  uint16_t dt_ms;
  int16_t  rpm, current, ref, base;
  uint8_t  io, state;
  uint16_t work_us;
  int32_t  lc2;
};
static Sample   sampleBuf[MAX_SAMPLES];
static uint8_t  sampleCount = 0;
static uint32_t frameBaseT  = 0;
static uint8_t  txBuf[14 + MAX_SAMPLES*18];

// Cache one loop iteration into the batch (no I/O). work = loop work time (µs, pre-pacing)
void recordSample(uint32_t now, uint16_t work){
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
  s.work_us = work;
  s.lc2     = ctrl.lc2Raw;
}

// Send the whole batch as one frame, then reset
void telemetryFlush(){
  if(sampleCount==0) return;
  uint8_t flags = (ctrl.lastLCValid     ? 0x01 : 0x00)
                | (ctrl.servoConnected  ? 0x02 : 0x00)
                | (ctrl.lcConfigApplied ? 0x04 : 0x00)
                | (ctrl.lc2Valid        ? 0x08 : 0x00)
                | (ctrl.lc2OK           ? 0x10 : 0x00);
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
    *p++ = (s.work_us>>0)&0xFF; *p++ = (s.work_us>>8)&0xFF;
    *p++ = (s.lc2>> 0)&0xFF; *p++ = (s.lc2>> 8)&0xFF;
    *p++ = (s.lc2>>16)&0xFF; *p++ = (s.lc2>>24)&0xFF;
  }
  uint8_t payloadLen = (uint8_t)(p - txBuf);   // 14 + n*18, ≤ 158 for MAX_SAMPLES=8
  uint8_t frame[3 + sizeof(txBuf) + 1];
  frame[0]=SYNC_0; frame[1]=SYNC_1; frame[2]=payloadLen;
  memcpy(&frame[3], txBuf, payloadLen);
  frame[3+payloadLen] = crc8(txBuf, payloadLen);
  Serial.write(frame, 3 + payloadLen + 1);
  sampleCount = 0;
}

// ── Respuesta a CMD_LC_PROGRAM / CMD_LC_QUERY — PACKET_ID_LC ──
// Modelo pregunta-respuesta: cada comando de la GUI produce exactamente UNA de estas,
// también cuando se rechaza. El byte `req` devuelve el opcode contestado para que la
// interfaz sepa a qué pregunta corresponde y no se quede esperando.
void telemetryFlushLc(uint8_t req, uint8_t result){
  uint8_t payload[11];
  payload[0] = PACKET_ID_LC;
  payload[1] = req;
  payload[2] = result;
  payload[3] = ctrl.lcProgGain;
  payload[4] = ctrl.lcProgOffset;
  payload[5] = (ctrl.lcCfgBefore>>0)&0xFF; payload[6] = (ctrl.lcCfgBefore>>8)&0xFF;
  payload[7] = (ctrl.lcCfgAfter >>0)&0xFF; payload[8] = (ctrl.lcCfgAfter >>8)&0xFF;
  payload[9]  = lcDiagAck;                 // lo que contestó de verdad (bits [7:6] = estado)
  payload[10] = (lcDiagEnCut ? 0x01 : 0x00)      // el pin de reset sí corta la tensión
              | (lcDiagWrAck ? 0x02 : 0x00)      // hubo ACK a algún Start_CM
              | ((lcDiagAttempt & 0x0F) << 4);   // intento del barrido que entró
  uint8_t frame[3 + sizeof(payload) + 1];
  frame[0]=SYNC_0; frame[1]=SYNC_1; frame[2]=(uint8_t)sizeof(payload);
  memcpy(&frame[3], payload, sizeof(payload));
  frame[3+sizeof(payload)] = crc8(payload, sizeof(payload));
  Serial.write(frame, 3 + sizeof(payload) + 1);
  ctrl.lcProgResult = result;
}

// ── Homing status frame (v2) — PACKET_ID_HOME ─────
// Frame: [SYNC0][SYNC1][LEN=11][payload][CRC8]
// payload (11 B):
//  0     packet_id  (0x03)
//  1-4   pos_mrev   (int32 LE)  posición actual × 1000 (mili-rev)
//  5-8   range_mrev (int32 LE)  recorrido A→B × 1000 (mili-rev)
//  9     home_phase (HomingPhase)
//  10    flags      (b0 = range_valid)
void telemetryFlushHome(){
  int32_t pm = (int32_t)(ctrl.posRev   * 1000.0f);
  int32_t rm = (int32_t)(ctrl.rangeRev * 1000.0f);
  uint8_t payload[11];
  payload[0] = PACKET_ID_HOME;
  payload[1] = (pm>> 0)&0xFF; payload[2] = (pm>> 8)&0xFF;
  payload[3] = (pm>>16)&0xFF; payload[4] = (pm>>24)&0xFF;
  payload[5] = (rm>> 0)&0xFF; payload[6] = (rm>> 8)&0xFF;
  payload[7] = (rm>>16)&0xFF; payload[8] = (rm>>24)&0xFF;
  payload[9]  = (uint8_t)ctrl.homePhase;
  payload[10] = ctrl.homeRangeValid ? 0x01 : 0x00;
  uint8_t frame[3 + sizeof(payload) + 1];
  frame[0]=SYNC_0; frame[1]=SYNC_1; frame[2]=(uint8_t)sizeof(payload);
  memcpy(&frame[3], payload, sizeof(payload));
  frame[3+sizeof(payload)] = crc8(payload, sizeof(payload));
  Serial.write(frame, 3 + sizeof(payload) + 1);
}

// ── Comandos ─────────────────────────────────────
// Petición de programación de la célula pendiente. Se declara aquí porque la apunta
// processCommand() y la ejecuta lcRequestService(), mucho más abajo, ya en el loop.
#define LC_REQ_TIMEOUT_MS 1000   // margen para que LC1 suelte el bus antes de rendirse

static uint8_t       lcReqPending  = 0;                 // opcode pendiente, 0 = ninguno
static uint8_t       lcReqGain     = LC_PREAMP_GAIN;
static uint8_t       lcReqOffset   = LC_A2D_OFFSET;
static unsigned long lcReqDeadline = 0;
static unsigned long lcHoldUntil   = 0;   // corte en curso: instante en que se suelta

// processCommand only updates state (sv/ctrl) — never touches the bus.
static uint8_t rxBuf[64];
static uint8_t rxPos = 0;
static enum { RX_SYNC0, RX_SYNC1, RX_LEN, RX_PAYLOAD } rxState = RX_SYNC0;
static uint8_t rxLen = 0;

// Arranca la rutina de homing (requiere servo activo). Resetea posición y recorrido.
static void startHoming(){
  if(sv.phase != SV_RUNNING) return;      // el servo debe estar habilitado
  ctrl.posRev         = 0.0f;
  ctrl.lastPosTime    = 0;                 // rebase la integración
  ctrl.rangeRev       = 0.0f;
  ctrl.homeRangeValid = false;
  ctrl.homePhase      = HOME_SEEK_A;
  ctrl.homePhaseStart = millis();
  ctrl.homeReportUntil = 0;
  sv.speedCmd         = 0;
}

// Aborta el homing (sin tocar el bus). Deja el servo parado (habilitado).
static void abortHoming(){
  if(ctrl.homePhase==HOME_SEEK_A || ctrl.homePhase==HOME_SEEK_B || ctrl.homePhase==HOME_GOTO){
    ctrl.homePhase = HOME_IDLE;
    sv.speedCmd    = 0;
  }
}

void processCommand(const uint8_t *payload, uint8_t len) {
  if (len < 1) return;
  switch (payload[0]) {

    case CMD_INIT:
      if (len >= 2 && payload[1] > 0 && payload[1] <= 247) sv.id = payload[1];  // address from Init
      sv.cfgRetries = 0; sv.speedCmd = 0;
      ctrl.errorCode = ERR_NONE;
      ctrl.posRev = 0.0f; ctrl.lastPosTime = 0;   // v2: reset de posición al inicializar
      ctrl.homePhase = HOME_IDLE;
      sv.phase = SV_CONFIGURING;                   // known address → connect directly, no scan
      break;

    case CMD_STOP:
      abortHoming();                               // v2: STOP también aborta el homing
      if (sv.phase == SV_RUNNING)          sv.speedCmd = 0;
      else if (sv.phase == SV_CONFIGURING) sv.phase = SV_IDLE;
      break;

    case CMD_SCAN: {                                // manual, blocking bus scan
      uint8_t found = scanServo();
      if (found) sv.id = found;
      break;
    }

    case CMD_MOVE_A:
      if (ctrl.lcProgBusy) break;                   // célula cortada: no se mueve nada
      if (ctrl.homePhase!=HOME_IDLE) break;         // v2: ignora movimiento manual durante homing
      if (sv.phase == SV_RUNNING) sv.speedCmd = -ctrl.moveSpeed;
      break;

    case CMD_MOVE_B:
      if (ctrl.lcProgBusy) break;                   // célula cortada: no se mueve nada
      if (ctrl.homePhase!=HOME_IDLE) break;         // v2: ignora movimiento manual durante homing
      if (sv.phase == SV_RUNNING) sv.speedCmd =  ctrl.moveSpeed;
      break;

    case CMD_LC_PROGRAM:                            // [op, ganancia 0..7, offset 0..15]
      if (len >= 3) { lcReqGain = payload[1] & 0x07; lcReqOffset = payload[2] & 0x0F; }
      lcReqPending  = CMD_LC_PROGRAM;               // lo hace lcRequestService(), no aquí
      lcReqDeadline = millis() + LC_REQ_TIMEOUT_MS;
      break;

    case CMD_LC_QUERY:                              // sin payload: sólo relee la EEPROM
      lcReqPending  = CMD_LC_QUERY;
      lcReqDeadline = millis() + LC_REQ_TIMEOUT_MS;
      break;

    case CMD_LC_HOLD:                               // diagnóstico con el polímetro
      if (len >= 2) lcParkMode = payload[1] ? 1 : 0;   // 0 = alta impedancia, 1 = a masa
      lcReqPending  = CMD_LC_HOLD;
      lcReqDeadline = millis() + LC_REQ_TIMEOUT_MS;
      break;

    case CMD_HOME:                                  // v2: lanza el homing
      if (ctrl.lcProgBusy) break;                   // célula cortada: no se arranca homing
      startHoming();
      break;

    case CMD_SHUTDOWN:
      abortHoming();
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
          case PARAM_FORCE_LIMIT:  ctrl.forceLimit = (uint16_t)value; break;
          case PARAM_ZERO_OFFSET:  ctrl.zeroOffset  = value; break;
          case PARAM_ACCEL:        ctrl.accelRate   = value; break;
          case PARAM_DECEL:        ctrl.decelRate   = value; break;
          case PARAM_STIFFNESS:    ctrl.stiffness   = value; break;
          case PARAM_FORCE_CALIB:  ctrl.forceCalib  = value; break;
          case PARAM_HOME_OFFSET:  ctrl.homeOffsetRev = (float)value / 100.0f; break;  // centi-rev → rev
          case PARAM_HOMING_SPEED: ctrl.homingSpeed = (value > 0) ? value : -value; break;
        }
      }
      break;
  }
}

void checkCommands() {
  while (Serial.available()) {
    uint8_t b = Serial.read();

    // Los atajos de una letra SÓLO valen con el decodificador en reposo. Antes se
    // miraban en cada byte, incluidos los de dentro de una trama binaria: cualquier
    // longitud, dato o CRC que coincidiera con una de estas letras se lo tragaba el
    // atajo y la trama se perdía sin más. No es hipotético — "grabar con ganancia 7 y
    // cero 8" da la trama AA 55 03 09 07 08 69, y ese 0x69 final es la 'i' de Init:
    // el comando desaparecía Y además reinicializaba el servo. Con ganancia 6 el CRC
    // sale 0x7C y la misma orden funcionaba, que es lo que lo hacía tan desconcertante.
    if (rxState == RX_SYNC0) {
      if (b == 'i') { uint8_t p[]={CMD_INIT};     processCommand(p,1); continue; }
      if (b == 'a') { uint8_t p[]={CMD_MOVE_A};   processCommand(p,1); continue; }
      if (b == 'b') { uint8_t p[]={CMD_MOVE_B};   processCommand(p,1); continue; }
      if (b == 'x') { uint8_t p[]={CMD_STOP};     processCommand(p,1); continue; }
      if (b == 'h') { uint8_t p[]={CMD_HOME};     processCommand(p,1); continue; }   // v2
      if (b == 'q') { uint8_t p[]={CMD_SHUTDOWN}; processCommand(p,1); continue; }
    }

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

// El árbitro del bus (i2cAcquire/i2cRelease) está en i2c_bus.h.

// Grabar la ganancia en la EEPROM la dispara el operario desde la GUI, nunca el
// arranque: al encender, el chip ya trae lo último que se le programó y el firmware
// se limita a leerlo para enseñarlo. La única condición es que el sistema esté
// PARADO — la grabación resetea la célula y la deja ciega ~400 ms, y durante ese rato
// el límite de fuerza se queda sin vigilancia, así que con el eje moviéndose no se
// toca. STATE_IDLE (drive sin habilitar) y STATE_STOPPED (habilitado, consigna 0) son
// los dos estados de reposo; homing en curso cuenta como movimiento.
static bool lcProgramAllowed(){
  return (ctrl.currentState==STATE_IDLE || ctrl.currentState==STATE_STOPPED)
      && sv.speedCmd    == 0
      && ctrl.homePhase == HOME_IDLE;
}

// ── Load Cell ─────────────────────────────────────
bool loadCellInit() {
  if(!i2cAcquire(I2C_LC1)) return false;
  bool ok = false;
  I2C1Bus.beginTransmission(ZSC31014_ADDR);
  if (I2C1Bus.endTransmission()==0) {
    I2C1Bus.requestFrom((uint8_t)ZSC31014_ADDR,(uint8_t)2);
    while(I2C1Bus.available()) I2C1Bus.read();
    delay(5); ok = true;
  }
  i2cRelease(I2C_LC1);
  return ok;
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

// Lectura bloqueante: sólo para arranque/diagnóstico, nunca desde el loop — retiene
// el bus hasta 50 ms. El camino normal es loadCellUpdate(), que no bloquea.
bool loadCellRead(uint16_t &br, uint8_t &st) {
  if(!i2cAcquire(I2C_LC1)) return false;
  bool ok=false;
  loadCellMeasurementRequest(); delay(2);
  uint32_t t0=millis();
  while(millis()-t0<50) {
    if(!loadCellFetch(br,st)) break;
    if(st==LC_STATUS_VALID){ ok=true; break; }
    delayMicroseconds(500);
  }
  i2cRelease(I2C_LC1);
  return ok;
}

#define LC_FETCH_TIMEOUT_MS 150   // > LOOP_MS: da 2-3 intentos antes de rendirse

// Lectura de LC1 — un evento de lectura por VUELTA del loop, y nada más.
//
// El ZSC31014 en Update Rate Mode convierte solo, a su propio ritmo: leerlo no dispara
// ninguna conversión, sólo devuelve el último valor y un estado que dice si es nuevo
// (0b00) o si ya se había leído (0b01, STALE). Así que aquí no hay pareja petición /
// recogida que sincronizar — hay una lectura por vuelta, y se acepta si viene fresca.
//
// La versión anterior hacía dos lecturas separadas 2 ms y se quedaba con la segunda.
// Como el chip no actualiza en 2 ms, la segunda salía siempre STALE y no se registraba
// ni una sola muestra. La de dos fases funcionaba de casualidad: sus dos lecturas caían
// en vueltas distintas del loop, con 50 ms de por medio, tiempo de sobra para que el
// chip hubiera convertido.
//
// El ritmo resultante es el menor entre el del loop y el del propio chip: si convierte
// más rápido que el loop, sale una muestra por vuelta; si va más lento, se repite el
// STALE y se conserva el valor anterior, que es exactamente lo que hay que hacer.
void loadCellUpdate(){
  if(!ctrl.loadCellOK) return;
  if(!i2cAcquire(I2C_LC1)) return;              // bus ocupado: se pierde el turno

  uint16_t br; uint8_t st;
  bool got = loadCellFetch(br, st);
  i2cRelease(I2C_LC1);

  unsigned long now = millis();
  if(got && st==LC_STATUS_VALID){
    ctrl.lastBridge=br;
    ctrl.maSum-=ctrl.maBuf[ctrl.maIdx];
    ctrl.maBuf[ctrl.maIdx]=br; ctrl.maSum+=br;
    ctrl.maIdx=(ctrl.maIdx+1)%ctrl.MA_SIZE;
    ctrl.lastBridgeFilt=(uint16_t)(ctrl.maSum/ctrl.MA_SIZE);
    ctrl.baseRead=(int16_t)ctrl.lastBridgeFilt-ctrl.zeroOffset;
    ctrl.lastLCStatus=st; ctrl.lastLCValid=true;
    ctrl.lcLastGood=now;
  } else if(ctrl.lastLCValid && now-ctrl.lcLastGood > LC_FETCH_TIMEOUT_MS){
    ctrl.lastLCValid=false;   // ni una lectura fresca en 150 ms: la última ya no vale
  }
}

// La palabra 0x0F que queremos, a partir de la que hay. Se tocan SÓLO los dos campos
// que el operario elige —ganancia [6:4] y cero del A/D [3:0], que juntos son los bits
// [6:0]— y todo lo demás se conserva tal cual venía.
//
// Antes esta función reconstruía la palabra entera desde constantes fijas, y con la
// configuración real de esta placa (0x1A68) eso habría cambiado tres campos más sin
// pedirlo: polaridad 0→1 (invirtiendo el signo de la fuerza), integración larga 0→1 y
// nulling 1→0. Esas constantes describían el montaje de otra persona, no éste.
static uint16_t lcDesiredCfg(uint16_t cur, uint8_t gain, uint8_t adOffset){
  return (cur & 0xFF80) | ((uint16_t)(gain & 7) << 4) | (adOffset & 0x0F);
}

static const uint16_t OFFSET_B_LUT[]={
  0xE000,0xE400,0xE800,0xEC00,0xF000,0xF400,0xF800,0xFC00,
  0x0000,0x0400,0x0800,0x0C00,0x1000,0x1400,0x1800,0x1C00
};

static bool zscCmd(uint8_t cmd,uint16_t d){
  I2C1Bus.beginTransmission(ZSC31014_ADDR);
  I2C1Bus.write(cmd);I2C1Bus.write((d>>8)&0xFF);I2C1Bus.write(d&0xFF);
  return I2C1Bus.endTransmission()==0;
}
// El primer byte de la respuesta lleva el estado en los bits [7:6], igual que en las
// lecturas normales: 0b10 = LC_STATUS_COMMAND significa "estoy en modo comando", y los
// otros seis bits ya son datos. El código original comparaba con un 0x5A literal y por
// eso daba por fallida una entrada en modo comando que había funcionado — nunca llegó a
// comprobarse porque tampoco miraba el resultado. Se acepta también el 0x5A por si
// alguna revisión del chip lo usa.
static bool zscResp(uint16_t &v){
  if(I2C1Bus.requestFrom((uint8_t)ZSC31014_ADDR,(uint8_t)3)<3){ lcDiagAck=0xFF; return false; }
  uint8_t a=I2C1Bus.read(),m=I2C1Bus.read(),l=I2C1Bus.read();
  lcDiagAck = a;
  if((a & 0xC0) != (LC_STATUS_COMMAND << 6) && a != 0x5A) return false;
  v=((uint16_t)m<<8)|l; return true;
}

// El pin de reset del ZSC31014 cuelga de GPIO1 (LC1_EN_GPIO). Bajarlo y volver a
// subirlo fuerza un Power-On-Reset limpio, que es la UNICA forma de abrir la ventana
// de modo comando: dura pocos ms y despues el chip pasa a Normal Operation Mode e
// ignora el Start_CM en silencio. Sin este ciclo, tras un reset por USB, por watchdog
// o por reflasheo el chip sigue alimentado y la configuracion no se aplicaba nunca.
// Cortar la alimentación NO basta si el bus se queda arriba. SDA y SCL siguen a 3V3
// por sus pull-ups, y esa tensión entra en el ZSC31014 por los diodos de protección de
// sus propios pines: el chip se queda alimentado de forma parásita, con su rail a unos
// 2,7 V, y nunca ve un Power-On-Reset. Eso es exactamente lo que medimos —seguía
// contestando con GPIO1 abajo—. Así que durante el corte se suelta el I2C y las dos
// líneas se dejan a 0, para que no quede ningún camino por donde colarle corriente.
// Se sueltan todas las señales que van al chip antes de quitarle la tensión.
//
// Alta impedancia, no forzadas a 0: el Pico deja de gobernar esas redes y quedan a lo
// que digan sus pull-ups. Esto cierra la realimentación siempre que esos pull-ups
// cuelguen del rail gobernado por EN, porque mueren con él. Si alguno estuviera en el
// 3V3 permanente volvería a sostener el chip por los diodos de protección de sus
// pines, y se vería en el acto midiendo el rail con "Mantener reset": si no baja a 0,
// ese pull-up es permanente y estas tres líneas tendrían que ir forzadas a 0 en vez
// de sueltas (pinMode OUTPUT + digitalWrite LOW).
//
// LC2_EN es la excepción y va forzado: no es una señal con pull-up, es el enable de
// alimentación de la otra Click, y soltarlo no apagaría nada.
static void zscParkLines(){
  I2C1Bus.end();
  if(lcParkMode){
    pinMode(LC_SDA_PIN, OUTPUT); digitalWrite(LC_SDA_PIN, LOW);
    pinMode(LC_SCL_PIN, OUTPUT); digitalWrite(LC_SCL_PIN, LOW);
    if(LC1_INT_GPIO >= 0){ pinMode(LC1_INT_GPIO, OUTPUT); digitalWrite(LC1_INT_GPIO, LOW); }
  } else {
    pinMode(LC_SDA_PIN, INPUT);                        // sin pull: alta impedancia
    pinMode(LC_SCL_PIN, INPUT);
    if(LC1_INT_GPIO >= 0) pinMode(LC1_INT_GPIO, INPUT);
  }
  if(LC_PARK_LC2) digitalWrite(LC2_EN_GPIO, LOW);
}

static void zscRestoreLines(){
  if(LC_PARK_LC2) digitalWrite(LC2_EN_GPIO, HIGH);
  I2C1Bus.begin(); I2C1Bus.setClock(LC_CM_I2C_FREQ);   // recupera SDA y SCL, a 100 kHz
  // INT se queda como entrada, que es su estado normal de funcionamiento.
}

static void zscPowerCycle(){
  zscParkLines();

  digitalWrite(LC1_EN_GPIO, LOW);
  delay(LC_RESET_LOW_MS);          // descarga de los condensadores de la Click

  // El bus se devuelve ANTES de dar tensión, y no después: Wire.begin() reconfigura
  // los pads y reinicia el periférico, y eso tarda de sobra para comerse los 1.5 ms
  // de ventana. Cuando el pin sube, el I2C tiene que estar ya listo para hablar.
  zscRestoreLines();

  digitalWrite(LC1_EN_GPIO, HIGH);
}

// Sólo para el diagnóstico, y sólo cuando ya ha fallado. Mide EXACTAMENTE la misma
// condición que el corte de verdad: con las líneas del bus parqueadas abajo. La versión
// anterior las dejaba arriba y por tanto siempre detectaba realimentación — estaba
// midiendo el problema que zscPowerCycle() ya evita, y acusaba al pin sin motivo.
//
// Para preguntar hay que devolver el bus un instante, y eso reabre el camino por los
// pull-ups. No importa: recargar los condensadores de la Click a través de ellos es
// cosa de milisegundos (τ de varios ms), y la pregunta son ~90 µs. Si en ese rato
// contesta, es que sigue alimentada por algo que no hemos cortado.
static void zscProbeResetPin(){
  zscParkLines();

  digitalWrite(LC1_EN_GPIO, LOW);
  delay(LC_RESET_LOW_MS);

  zscRestoreLines();
  I2C1Bus.beginTransmission(ZSC31014_ADDR);
  lcDiagEnCut = (I2C1Bus.endTransmission() != 0);   // muda = el corte funciona de verdad

  digitalWrite(LC1_EN_GPIO, HIGH);
  delay(20);
}

// Entre dar tensión y mandar el Start_CM no puede haber NADA: ni Wire.begin(), ni un
// Serial.print, ni un cálculo. El propio autor del driver de mbed avisa de que añadir
// un printf en medio le rompía la entrada en modo comando. Por eso la secuencia va
// seguida y lo único que se reintenta es el ciclo completo.
static bool zscEnterCommandMode(){
  lcDiagAck = 0xFF; lcDiagWrAck = false; lcDiagEnCut = true; lcDiagAttempt = 0;
  uint16_t v;

  for(uint8_t attempt=0; attempt<LC_CM_ATTEMPTS; attempt++){
    zscPowerCycle();

    // Hay que dar en una ventana estrecha y con dos bordes: el Start_CM tiene que llegar
    // DESPUÉS de que el chip termine de arrancar, pero antes de 1.5 ms desde que tiene
    // tensión. Cuánto tarda en arrancar ESTA Click depende de sus condensadores y de su
    // conmutador de alimentación, y ningún número copiado de otra placa vale: los 500 µs
    // del driver de mbed no llegaban y el chip aún no contestaba.
    //
    // Así que no se adivina. Primero se espera a que vuelva a hacer ACK —eso marca el
    // instante en que ha arrancado— y después cada intento prueba un retardo distinto,
    // de 0 a 1050 µs en pasos de 150. El primero que entre gana, y se informa de cuál
    // ha sido para poder fijarlo el día de mañana.
    bool alive = false;
    unsigned long t0 = millis();
    while(millis() - t0 < LC_CM_RAMP_MS){
      I2C1Bus.beginTransmission(ZSC31014_ADDR);      // sólo la dirección: ~90 µs
      if(I2C1Bus.endTransmission() == 0){ alive = true; break; }
    }
    if(!alive) continue;                             // ni siquiera ha vuelto: otro ciclo

    delayMicroseconds(attempt * LC_CM_STEP_US);      // el barrido
    if(!zscCmd(0xA0,0)) continue;                    // Start_CM
    lcDiagWrAck = true;
    lcDiagAttempt = attempt;
    delayMicroseconds(10);

    // Se confirma pidiendo la palabra de configuración, como hace el driver de MikroE:
    // lo que conteste el propio Start_CM no es una comprobación fiable.
    if(!zscCmd(0x0F,0)) continue;              // ReadB_Config
    delayMicroseconds(100);
    if(zscResp(v)) return true;
  }
  zscProbeResetPin();              // ha fallado: ahora sí interesa saber por qué
  return false;
}

// Traduce el fallo de entrada en modo comando al porqué concreto, que es lo único
// que sirve para arreglarlo desde la GUI.
static uint8_t zscWhyNoWindow(){
  if(!lcDiagWrAck) return LCPROG_NO_ACK;    // no contesta ni al reset: sin tensión o sin bus
  if(!lcDiagEnCut) return LCPROG_EN_DEAD;   // habla, pero el pin de reset no la calla
  return LCPROG_NO_WINDOW;                  // se reinicia, pero no da el 0x5A a tiempo
}

// Graba ganancia y offset del A/D en la EEPROM. Dos reglas del datasheet mandan aqui:
//   · ZMDI_Config solo se carga tras un ciclo de alimentacion, asi que escribir y leer
//     en la misma sesion de modo comando devuelve el valor viejo.
//   · La firma MISR de la EEPROM (palabra 0x12) se regenera al SALIR del modo comando
//     con Start_NOM si hubo escritura, y se verifica en el POR siguiente. Cortar la
//     alimentacion sin ese Start_NOM deja la firma mal y el chip se queda dando estado
//     de diagnostico para siempre.
// De ahi el bucle: la primera vuelta escribe y sale con Start_NOM, la segunda arranca
// desde un POR limpio y comprueba que lo que se lee es ya lo pedido. Si coincide a la
// primera no se escribe nada, que la EEPROM tiene los ciclos contados.
// Las dos siguientes dan por hecho que quien llama ya tiene el bus reservado como
// I2C_CFG y ha dejado las dos células paradas: son el trabajo sucio, no la puerta de
// entrada. La puerta es lcRequestService(), que es quien contesta a la GUI.

// Sólo lectura: no escribe, así que ni desgasta la EEPROM ni deja firma que recalcular.
// Aun así necesita modo comando, y modo comando implica resetear el chip.
static uint8_t loadCellQuery(uint16_t &cfg){
  uint8_t res = LCPROG_NO_WINDOW;
  if(zscEnterCommandMode()){
    if(zscCmd(0x0F,0)){ delayMicroseconds(100); if(zscResp(cfg)) res = LCPROG_READ_ONLY; }
  } else res = zscWhyNoWindow();
  zscCmd(0x80,0); delay(15);                              // Start_NOM -> modo normal
  return res;
}

// Escribe y VERIFICA. Dos reglas del datasheet mandan aquí:
//   · ZMDI_Config sólo se carga tras un ciclo de alimentación, así que escribir y leer
//     en la misma sesión de modo comando devuelve el valor viejo.
//   · La firma MISR de la EEPROM (palabra 0x12) se regenera al SALIR del modo comando
//     con Start_NOM si hubo escritura, y se comprueba en el POR siguiente. Cortar la
//     alimentación sin ese Start_NOM deja la firma mal y el chip se queda dando estado
//     de diagnóstico para siempre.
// De ahí el bucle: la primera vuelta escribe y sale con Start_NOM, la segunda arranca
// desde un POR limpio y comprueba que lo que se lee es ya lo pedido. Si coincide a la
// primera no se escribe nada: la EEPROM tiene los ciclos de escritura contados.
static uint8_t loadCellProgram(uint8_t gain, uint8_t adOffset,
                               uint16_t &before, uint16_t &after){
  uint8_t res     = LCPROG_NO_WINDOW;
  bool    wrote   = false;
  bool    touched = false;   // se ha llegado a mandar una escritura de EEPROM
  before = 0; after = 0;
  for(int attempt=0; attempt<2; attempt++){
    if(!zscEnterCommandMode()){ res = zscWhyNoWindow(); break; }
    uint16_t cur=0;
    if(!zscCmd(0x0F,0)){ res=LCPROG_FAILED; break; } delayMicroseconds(100);
    if(!zscResp(cur)) { res=LCPROG_FAILED; break; }
    if(attempt==0) before = cur;
    after = cur;
    uint16_t des = lcDesiredCfg(cur, gain, adOffset);
    if(cur==des){ res = wrote ? LCPROG_OK : LCPROG_UNCHANGED; break; }
    if(attempt==1){ res = LCPROG_FAILED; break; }         // se escribió y no cuajó

    // A partir de aquí la EEPROM queda tocada: pase lo que pase hay que salir con
    // Start_NOM y darle tiempo, o la firma se queda a medias.
    touched = true;
    if(!zscCmd(0x4F,des)){ res=LCPROG_FAILED; break; }    // B_Config (0x0F)
    delay(LC_EEPROM_WRITE_MS);

    // Aquí ANTES se escribía también Offset_B (0x43) con una tabla heredada. Offset_B
    // es el coeficiente de calibración del puente, no el cero del A/D: ese vive en los
    // bits [3:0] de B_Config y ya se ha escrito arriba. Eran dos ajustes distintos
    // confundidos en uno, y escribirlo pisaba una calibración que nadie ha pedido tocar.

    zscCmd(0x80,0);                                       // Start_NOM: regenera la firma
    delay(LC_EEPROM_SETTLE_MS);                           // ...y que termine ANTES de cortar
    wrote = true;
  }
  zscCmd(0x80,0);                                         // Start_NOM -> modo normal
  delay(touched ? LC_EEPROM_SETTLE_MS : 15);
  return res;
}

// ── Load Cell 2 (NAU7802) ─────────────────────────
static bool nauWrite(uint8_t reg, uint8_t val){
  I2C1Bus.beginTransmission(NAU7802_ADDR);
  I2C1Bus.write(reg); I2C1Bus.write(val);
  return I2C1Bus.endTransmission()==0;
}

// STOP en vez de repeated start: el ZSC31014 comparte bus y su protocolo no admite
// la condicion de restart — un flanco de bajada de SDA con SCL alto le rompe la
// comunicacion SIGUIENTE, aunque esta vaya dirigida a la NAU7802.
static uint8_t nauReadReg(uint8_t reg){
  I2C1Bus.beginTransmission(NAU7802_ADDR);
  I2C1Bus.write(reg);
  I2C1Bus.endTransmission();
  I2C1Bus.requestFrom((uint8_t)NAU7802_ADDR,(uint8_t)1);
  return I2C1Bus.available() ? I2C1Bus.read() : 0xFF;
}

// Calibración interna de offset — obligatoria tras fijar ganancia, LDO y tasa
static bool nauCalibrate(){
  uint8_t c2 = ((uint8_t)(NAU_RATE&7)<<4) | (NAU_CALMOD&3);
  nauWrite(NAU_CTRL2, c2 | 0x04);                 // CALS = 1
  uint32_t t = millis();
  while(nauReadReg(NAU_CTRL2) & 0x04){            // espera a que CALS baje
    if(millis()-t > 1000) return false;
    delay(1);
  }
  return (nauReadReg(NAU_CTRL2) & 0x08)==0;       // CAL_ERR = 0
}

static bool loadCell2Setup(){
  if(!nauWrite(NAU_PU_CTRL, NAU_RR)) return false;
  delay(10);
  if(!nauWrite(NAU_PU_CTRL, NAU_AVDDS|NAU_PUA|NAU_PUD)) return false;
  delay(10);
  uint32_t t = millis();
  while(!(nauReadReg(NAU_PU_CTRL) & NAU_PUR)){    // espera power-up ready
    if(millis()-t > 200) return false;
    delay(5);
  }
  nauWrite(NAU_CTRL1, ((uint8_t)(NAU_VLDO&7)<<3) | (NAU_GAIN&7));  // LDO + ganancia
  nauWrite(NAU_CTRL2, (uint8_t)(NAU_RATE&7)<<4);                   // tasa (bits 6:4)
  nauWrite(NAU_ADC_REG,  0x30);                                    // chopper del ADC apagado
  nauWrite(NAU_PWR_CTRL, nauReadReg(NAU_PWR_CTRL) | 0x80);         // cap de 330 pF del PGA
  nauWrite(NAU_PGA,      (uint8_t)(nauReadReg(NAU_PGA) & ~0x01));  // chopper del PGA ACTIVO
  ctrl.lc2CalOK = nauCalibrate();
  nauWrite(NAU_PU_CTRL, NAU_AVDDS|NAU_CS|NAU_PUA|NAU_PUD);         // conversiones continuas
  ctrl.lc2Discard = 4;
  return true;
}

// Corre en setup() con el loop todavía parado, pero pide el bus igual: la regla no
// tiene excepciones, y el cuerpo va aparte porque tiene cinco salidas de error.
bool loadCell2Init(){
  if(!i2cAcquire(I2C_LC2)) return false;
  bool ok = loadCell2Setup();
  i2cRelease(I2C_LC2);
  return ok;
}

// El cuerpo va aparte para que loadCell2Update() sea sólo el pedir y soltar el bus:
// así ninguno de los tres caminos de salida de aquí se puede olvidar de liberarlo.
static void loadCell2Poll(){
  unsigned long now = millis();
  if(!(nauReadReg(NAU_PU_CTRL) & NAU_CR)){
    if(ctrl.lc2Valid && now-ctrl.lc2LastGood > 500) ctrl.lc2Valid=false;
    return;
  }
  I2C1Bus.beginTransmission(NAU7802_ADDR);
  I2C1Bus.write(NAU_ADCO_B2);
  I2C1Bus.endTransmission();                      // sin repeated start (ver nauReadReg)
  if(I2C1Bus.requestFrom((uint8_t)NAU7802_ADDR,(uint8_t)3) < 3) return;
  uint8_t b2=I2C1Bus.read(), b1=I2C1Bus.read(), b0=I2C1Bus.read();
  int32_t v = ((int32_t)b2<<16) | ((int32_t)b1<<8) | b0;
  if(v & 0x800000) v |= 0xFF000000;               // signo 24 → 32 bit
  if(ctrl.lc2Discard){ ctrl.lc2Discard--; return; }
  ctrl.lc2Raw      = v;
  ctrl.lc2Valid    = true;
  ctrl.lc2LastGood = now;
}

// Lectura no bloqueante: sólo cuando el bit CR dice que hay muestra nueva.
void loadCell2Update(){
  if(!ctrl.lc2OK) return;
  if(!i2cAcquire(I2C_LC2)) return;      // LC1 está en mitad de una conversión: turno perdido
  loadCell2Poll();
  i2cRelease(I2C_LC2);
}

// ── Servo (single owner: Core0) ───────────────────
void servoInit(){
  pinMode(MB_EN_GPIO,OUTPUT); digitalWrite(MB_EN_GPIO,LOW);
  Serial485.begin(115200); Serial485.setTimeout(100);
  servo.preTransmission(preTransmission);
  servo.postTransmission(postTransmission);
}

static uint8_t servoWriteReg(uint16_t reg,uint16_t val){
  uint8_t r = servo.writeSingleRegister(reg,val);
  delayMicroseconds(700);   // RTU inter-frame silence before the next config write
  return r;
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

// ── Posición (v2) — integra RPM → rev, sin tocar buses ──
// Signo: + hacia B (mismo signo que la referencia de velocidad).
void positionUpdate(){
  unsigned long now = millis();
  if(ctrl.lastPosTime == 0){ ctrl.lastPosTime = now; return; }  // primera muestra: fija base
  unsigned long dt = now - ctrl.lastPosTime;
  ctrl.lastPosTime = now;
  // rev = rpm(rev/min) · dt(ms) / 60000
  ctrl.posRev += (float)sv.rpm * (float)dt / 60000.0f;
}

// ── Homing (v2) — máquina de estados, sólo fija sv.speedCmd ──
// Busca A (datum→0) → busca B (mide recorrido) → va al offset → fija HOME=0.
void homingUpdate(){
  if(ctrl.homePhase==HOME_IDLE || ctrl.homePhase==HOME_DONE || ctrl.homePhase==HOME_FAIL) return;

  unsigned long now = millis();

  // el homing exige servo habilitado; si no, aborta
  if(sv.phase != SV_RUNNING){
    ctrl.homePhase = HOME_FAIL; sv.speedCmd = 0;
    ctrl.homeReportUntil = now + HOMING_REPORT_MS; return;
  }
  // timeout de seguridad por tramo
  if(now - ctrl.homePhaseStart > HOMING_TIMEOUT_MS){
    ctrl.homePhase = HOME_FAIL; sv.speedCmd = 0;
    ctrl.homeReportUntil = now + HOMING_REPORT_MS; return;
  }

  bool limA = (digitalRead(PIN_LIMIT_A)==HIGH);
  bool limB = (digitalRead(PIN_LIMIT_B)==HIGH);

  switch(ctrl.homePhase){
    case HOME_SEEK_A:
      if(limA){                          // datum A encontrado
        sv.speedCmd = 0;
        ctrl.posRev = 0.0f;              // A = cero de referencia
        ctrl.homePhase = HOME_SEEK_B;
        ctrl.homePhaseStart = now;
      } else {
        sv.speedCmd = -ctrl.homingSpeed; // avanza hacia A
      }
      break;

    case HOME_SEEK_B:
      if(limB){                          // final B encontrado
        sv.speedCmd = 0;
        ctrl.rangeRev = ctrl.posRev;     // recorrido disponible A→B
        ctrl.homeRangeValid = true;
        ctrl.homePhase = HOME_GOTO;
        ctrl.homePhaseStart = now;
      } else {
        sv.speedCmd = +ctrl.homingSpeed; // avanza hacia B
      }
      break;

    case HOME_GOTO: {                     // ir al offset deseado desde A
      float target = ctrl.homeOffsetRev;
      if(target < 0.0f)          target = 0.0f;
      if(target > ctrl.rangeRev) target = ctrl.rangeRev;   // acotado al recorrido medido
      float err = target - ctrl.posRev;
      bool arrived = (err <= HOMING_TOL_REV && err >= -HOMING_TOL_REV)
                  || (err > 0 && limB)                     // tope físico en el sentido de marcha
                  || (err < 0 && limA);
      if(arrived){
        sv.speedCmd = 0;
        ctrl.posRev = 0.0f;              // este punto queda como HOME (0)
        ctrl.homePhase = HOME_DONE;
        ctrl.homeReportUntil = now + HOMING_REPORT_MS;
      } else {
        sv.speedCmd = (err > 0) ? +ctrl.homingSpeed : -ctrl.homingSpeed;
      }
      break;
    }

    default: break;
  }
}

// ══ Programación de la célula: pregunta de la GUI → respuesta ═════
// El comando no se ejecuta dentro del parser porque al llegar puede haber una
// conversión de LC1 a medias con el bus cogido. Se apunta la petición y se atiende
// aquí, en un punto fijo del loop, con el bus libre. Salga como salga, se contesta.
static void lcRequestService(){
  unsigned long now = millis();

  // ── Corte en curso (medida con el polímetro) ──────────────────────────────
  // Aquí NO se espera con delay(). El loop tiene que seguir girando durante los ocho
  // segundos: es lo que alimenta el watchdog, vigila los finales de carrera, lee el
  // servo y manda telemetría. Pararlo dejaría el equipo ciego justo mientras alguien
  // tiene las manos y el polímetro dentro de la máquina — y con watchdog, además,
  // reiniciaría el micro y soltaría el reset a media medición.
  //
  // La célula se queda sin tensión y el bus parqueado porque el token I2C_CFG sigue
  // cogido entre vuelta y vuelta: nadie más puede tocar el bus aunque el loop corra.
  if(lcHoldUntil){
    if((long)(now - lcHoldUntil) < 0) return;      // sigue cortada; se mira y se vuelve
    lcHoldUntil = 0;
    digitalWrite(LC1_EN_GPIO, HIGH);
    zscRestoreLines();
    I2C1Bus.setClock(LC_I2C_FREQ);  // se devuelve el bus a la velocidad de lectura
    lcParkMode = LC_PARK_DEFAULT;   // el modo de la medida era sólo para esa medida
    i2cRelease(I2C_CFG);
    ctrl.loadCellOK        = loadCellInit();
    ctrl.lc2OK             = loadCell2Init();
    ctrl.lastPosTime       = 0;                   // el hueco no cuenta como movimiento
    ctrl.lastTelemetryTime = millis();
    ctrl.lcProgBusy        = false;
    telemetryFlushLc(CMD_LC_HOLD, LCPROG_HELD);
    return;
  }

  if(!lcReqPending) return;
  uint8_t req = lcReqPending;

  // Con el eje en marcha ni se intenta: la grabación deja la célula ciega ~400 ms y
  // el límite de fuerza se quedaría sin vigilancia con carga puesta.
  if(!lcProgramAllowed()){ lcReqPending=0; telemetryFlushLc(req, LCPROG_RUNNING); return; }

  if(!i2cAcquire(I2C_CFG)){                    // LC1 en mitad de una conversión
    if((long)(now-lcReqDeadline) < 0) return;           // aún hay margen: se reintenta
    lcReqPending=0; telemetryFlushLc(req, LCPROG_BUS_BUSY); return;
  }
  lcReqPending = 0;

  // ── Se para TODO antes de empezar ────────────────────────────────────────
  // Grabar bloquea el loop cerca de un segundo: durante ese rato no se lee el servo,
  // no se integra la posición, no se vigila el límite de fuerza y no sale telemetría.
  // Eso pasaba ya por ser una llamada bloqueante, pero conviene que sea una decisión
  // y no un efecto secundario, y sobre todo que se vea desde fuera.
  ctrl.lcProgBusy   = true;
  ctrl.currentState = STATE_PROGRAMMING;
  sv.speedCmd       = 0;              // la consigna a cero; el eje ya venía parado
  telemetryFlush();                   // vacía lo pendiente y avisa del silencio que viene

  // Nadie más toca el bus mientras dure esto: la célula se queda sin tensión varias
  // veces y lo que hubiera leído antes ya no significa nada.
  ctrl.loadCellOK  = false; ctrl.lc2OK    = false;
  ctrl.lastLCValid = false; ctrl.lc2Valid = false;

  if(req==CMD_LC_HOLD){
    // Corta y se va: la célula queda sin tensión y el bus parqueado, exactamente en la
    // misma condición que usa la grabación, y así se puede medir su rail. Quien lo
    // suelta es la comprobación de arriba, LC_HOLD_MS más tarde, sin bloquear nada.
    // Si en ese rato el rail no baja a ~0 V, le entra alimentación por un pin que
    // seguimos sin cerrar.
    zscParkLines();
    digitalWrite(LC1_EN_GPIO, LOW);
    lcHoldUntil = now + LC_HOLD_MS;
    return;                                    // el token I2C_CFG se queda cogido
  }

  uint8_t res;
  if(req==CMD_LC_PROGRAM){
    res = loadCellProgram(lcReqGain, lcReqOffset, ctrl.lcCfgBefore, ctrl.lcCfgAfter);
    if(res==LCPROG_OK || res==LCPROG_UNCHANGED){
      ctrl.lcProgGain   = lcReqGain;           // lo que ha quedado grabado de verdad
      ctrl.lcProgOffset = lcReqOffset;
    }
  } else {
    uint16_t cfg = 0;
    res = loadCellQuery(cfg);
    ctrl.lcCfgBefore = cfg; ctrl.lcCfgAfter = cfg;
    if(res==LCPROG_READ_ONLY){                 // lo que el chip trae ya programado
      ctrl.lcProgGain   = (cfg >> 4) & 0x07;
      ctrl.lcProgOffset =  cfg       & 0x0F;
    }
  }

  // Se suelta el bus ANTES de reinicializar: los init piden su propio turno y con
  // I2C_CFG todavía cogido se lo negarían a sí mismos, dejando las células mudas.
  I2C1Bus.setClock(LC_I2C_FREQ);   // se devuelve el bus a la velocidad de lectura
  i2cRelease(I2C_CFG);

  // El chip ha pasado por varios resets: las dos células se reinician desde cero.
  ctrl.loadCellOK      = loadCellInit();
  ctrl.lc2OK           = loadCell2Init();
  ctrl.lcConfigApplied = (res==LCPROG_OK || res==LCPROG_UNCHANGED);

  // ── Se reanuda, reparando lo que el parón deja mal ───────────────────────
  // La posición se integra como rpm·dt: si no se rebasa el reloj, la próxima vuelta
  // vería un dt de ~1 s y lo multiplicaría por las últimas rpm leídas, inventándose
  // un desplazamiento que nunca ocurrió. Mismo idioma que usa CMD_INIT.
  ctrl.lastPosTime       = 0;
  ctrl.lastTelemetryTime = millis();   // el hueco no cuenta como periodo de telemetría
  ctrl.lcProgBusy        = false;

  telemetryFlushLc(req, res);
}

// ══ Único punto del firmware que mueve el bus I2C1 en marcha ══════
// Una vez por vuelta y siempre en este orden: primero LC1, que es quien puede
// reservar el bus mientras convierte, y después LC2, que aprovecha los huecos.
// Fuera de aquí sólo lo tocan los init y la escritura de EEPROM, todos en setup().
void i2cUpdate(){
  loadCellUpdate();      // LC1 — ZSC31014 (retiene el bus durante la conversión)
  loadCell2Update();     // LC2 — NAU7802  (sólo si LC1 lo ha soltado)
}

// ── Setup / Loop (Core0) ──────────────────────────
void setup(){
  // ── Células primero, sólo para arrancarlas. Aquí no se programa nada: el ZSC31014
  //    ya viene con lo último que se le grabó, y la GUI lo consulta o lo cambia con
  //    CMD_LC_QUERY / CMD_LC_PROGRAM, siempre con el eje parado.
  pinMode(LC1_EN_GPIO,OUTPUT); pinMode(LC2_EN_GPIO,OUTPUT);
  gpio_set_drive_strength(LC1_EN_GPIO, GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_drive_strength(LC2_EN_GPIO, GPIO_DRIVE_STRENGTH_12MA);
  digitalWrite(LC1_EN_GPIO,HIGH); digitalWrite(LC2_EN_GPIO,HIGH);
  delay(50);
  I2C1Bus.begin(); I2C1Bus.setClock(LC_I2C_FREQ);
  // Aquí NO se programa nada: el ZSC31014 arranca con lo último que se le grabó, y
  // leerlo obligaría a resetearlo. La GUI lo pregunta con CMD_LC_QUERY cuando quiere.
  ctrl.loadCellOK      = loadCellInit();
  ctrl.lc2OK           = loadCell2Init();                // la NAU7802 no tiene prisa

  // ── Everything else ──
  Serial.begin(115200);
  { unsigned long t0=millis(); while(!Serial&&millis()-t0<3000) delay(10); }
  pinMode(PIN_LIMIT_A, INPUT_PULLUP);
  pinMode(PIN_LIMIT_B, INPUT_PULLUP);
  servoInit();                 // RS485 — servo runs even if the LC is absent
  Serial.flush(); delay(50);
}

void loop(){
  unsigned long now=millis();
  uint32_t      t0us=micros();   // loop-work start (for per-sample performance)

  // ══ READ — independent subsystem reads ═══════════
  servoReadState();      // servo  (RS485)
  lcRequestService();    // graba/lee la EEPROM de la célula si la GUI lo ha pedido
  i2cUpdate();           // ambas células: único punto del loop que toca el bus I2C1
  positionUpdate();      // v2: integra RPM → posición (pura, sin buses)

  // ══ LOGIC — no bus access, pure state ════════════
  checkCommands();                         // parse USB commands → sv/ctrl

  // Mientras la célula está cortada no se mueve nada. El loop sigue girando —para
  // alimentar el watchdog, vigilar finales de carrera y mandar telemetría— pero el eje
  // se queda quieto: alguien está midiendo con las manos dentro de la máquina, y una
  // orden de movimiento llegada por USB en ese momento no puede arrancarlo.
  if(ctrl.lcProgBusy){
    sv.speedCmd = 0;                       // se reafirma cada vuelta, no sólo al entrar
  } else {
    homingUpdate();                        // v2: máquina de homing → fija sv.speedCmd
  }

  // v2: cierra la ventana de reporte tras DONE/FAIL
  if((ctrl.homePhase==HOME_DONE || ctrl.homePhase==HOME_FAIL)
     && ctrl.homeReportUntil && now > ctrl.homeReportUntil){
    ctrl.homePhase = HOME_IDLE;
  }

  // safety: force limit + endstops → zero the speed target (backstop, también en homing)
  if(ctrl.forceLimit>0 && ctrl.lastLCValid && sv.phase==SV_RUNNING && sv.speedCmd!=0){
    int16_t af=(ctrl.baseRead<0)?-ctrl.baseRead:ctrl.baseRead;
    if((uint16_t)af>ctrl.forceLimit) sv.speedCmd=0;
  }
  if(digitalRead(PIN_LIMIT_A)==HIGH && sv.speedCmd<0) sv.speedCmd=0;  // running into A
  if(digitalRead(PIN_LIMIT_B)==HIGH && sv.speedCmd>0) sv.speedCmd=0;  // running into B

  // publish servo values + derive app state for telemetry
  ctrl.motorRPM=sv.rpm; ctrl.motorTorqueX10=sv.torqueX10;
  ctrl.servoConnected=sv.connected; ctrl.currentSpeedCmd=sv.speedCmd;
  if(ctrl.lcProgBusy){ ctrl.currentState = STATE_PROGRAMMING; } else
  switch(sv.phase){                          // app state follows the servo (LC is independent)
    case SV_IDLE:        ctrl.currentState=STATE_IDLE;        break;
    case SV_CONFIGURING: ctrl.currentState=STATE_CONFIGURING; break;
    case SV_RUNNING:     ctrl.currentState=(sv.speedCmd<0)?STATE_MOVING_A
                                          :(sv.speedCmd>0)?STATE_MOVING_B:STATE_STOPPED; break;
    case SV_FAULT:       ctrl.currentState=STATE_ERROR;       break;
  }
  // v2: durante el homing el estado manda
  if(ctrl.homePhase==HOME_SEEK_A || ctrl.homePhase==HOME_SEEK_B || ctrl.homePhase==HOME_GOTO)
    ctrl.currentState = STATE_HOMING;

  // ══ ACT — one call, servo state machine + all writes ══
  servoAction();

  // ══ record this loop (with its work time), flush the whole batch every 200 ms (or if full) ══
  uint32_t workUs = micros() - t0us;
  recordSample(now, workUs > 65535 ? 65535 : (uint16_t)workUs);
  uint32_t period=(ctrl.currentState==STATE_ERROR)?1000:TELEMETRY_MS;
  if(sampleCount>=MAX_SAMPLES || now-ctrl.lastTelemetryTime>=period){
    telemetryFlush();
    if(ctrl.homePhase!=HOME_IDLE) telemetryFlushHome();   // v2: estado de homing sólo si está activo
    ctrl.lastTelemetryTime=now;
  }

  // ══ pace the loop to a fixed period (caps read/bus rate) ══
  unsigned long dt=millis()-now;
  if(dt<LOOP_MS) delay(LOOP_MS-dt);
}
