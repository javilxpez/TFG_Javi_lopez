// ═══════════════════════════════════════════════════════════
//  Lectura dual de células de carga — Raspberry Pi Pico 2
//
//  LC1: Load Cell 2 Click — ZSC31014 @ 0x28  (14 bit + 2 de status)
//  LC2: Load Cell 4 Click — NAU7802  @ 0x2A  (24 bit con signo)
//
//  Las dos en el MISMO bus I2C1 (GP2 = SDA, GP3 = SCL) a 100 kHz.
//  SIN filtrado: cada línea es la última medida real del ADC, y entre
//  paréntesis esa misma medida menos la tara.
//  Secuencia de arranque igual que firmware_posicion_v2: la ventana de
//  modo comando del ZSC31014 dura pocos ms tras alimentarlo, así que se
//  configura ANTES de inicializar el Serial.
//
//  ── Configuración del NAU7802: por qué cada línea ────────────────
//  Medido sobre esta placa, ruido de reposo pico a pico a ganancia 128:
//    · VLDO a 4.5 V con la Click a 3V3 (LDO en dropout) ..... ~18000
//    · VLDO a 3.0 V + calibración interna de offset ..........  ~3500
//    · + chopper del PGA activo ..............................   ~350
//  350 cuentas son 0,49 µV en la entrada: el suelo de ruido del chip.
//  Con las entradas cortocircuitadas por dentro ('s') sale lo mismo,
//  así que la cadena electrónica está limpia y lo que se mueva a partir
//  de ahí viene del puente o de su cableado.
//  AVDD DEBE venir del LDO interno: esta Click no alimenta el pin AVDD
//  por fuera y con AVDDS=0 el ADC se clava en -8388608 (comprobado).
//
//  ── Comandos por USB (115200) ────────────────────────────────────
//    t          tara de ambas células
//    r          reinicia la tara
//    c          repite la calibración interna de offset del NAU7802
//    s          cortocircuita las entradas de LC2 (aísla el chip del puente)
//    l          conmuta la fuente de AVDD de LC2 (diagnóstico: aquí satura)
//    v          alterna salida legible / CSV para registrar y graficar
//    k1 <valor> calibra LC1 con la carga conocida que tengas puesta
//    k2 <valor> calibra LC2 igual  (ej: "k2 5.0" con 5 kg encima)
//    g <0..7>   ganancia del preamplificador de LC1. Barre los 8 y quedate con
//               el que mas cuentas te mueva a fuerza maxima.
//    w <hex>    escribe la palabra 0x0F de configuracion entera, para explorar
//               los bits que el firmware fija a mano. 'w 0' vuelve a la calculada.
//    o <0..15>  mueve el cero de LC1 (offset del A/D). 8 = centrado, que es
//               donde estás ahora: el reposo cae en ~8192 y media escala se
//               desperdicia si la fuerza va siempre en el mismo sentido.
//               Reconfigura en caliente apagando la Click por su pin EN.
//
//  La calibración vive en RAM: el sketch imprime las constantes para que
//  las pegues en LC1_COUNTS_PER_UNIT / LC2_COUNTS_PER_UNIT y queden fijas.
// ═══════════════════════════════════════════════════════════

#include <Wire.h>
#include <hardware/gpio.h>

static TwoWire I2C1Bus(i2c1, 2, 3);

// ═══ Configuración ═══════════════════════════════

// ── Bus y pines ──
#define LC_SDA_PIN   2
#define LC_SCL_PIN   3
#define LC_I2C_FREQ  100000
#define LC1_EN_PIN   1     // GP1 — reset/enable Load Cell 2 Click (ZSC31014)
#define LC_RESET_LOW_MS  250   // reset abajo: margen de sobra para que el rail caiga a 0
#define LC_CM_WINDOW_MS   20   // margen para pillar la ventana de modo comando
#define LC2_EN_PIN   0     // GP0 — alimentación/enable Load Cell 4 Click

// ── Calibración (0 = sin calibrar → se imprimen cuentas en crudo) ──
#define UNIT_NAME             "kg"
#define LC1_COUNTS_PER_UNIT   0.0f
#define LC2_COUNTS_PER_UNIT   0.0f

// ── Cadencia ──
#define PRINT_MS  250   // periodo de impresión
#define LOOP_MS   5     // periodo del bucle (el muestreo va sin bloquear)
#define LC1_TIMEOUT_MS 100   // sin medida válida → deja de dar el último valor por bueno
#define LC2_TIMEOUT_MS 500

// ── LC1: ZSC31014 ────────────────────────────────
#define ZSC31014_ADDR     0x28
#define LC_STATUS_VALID   0x00
#define LC_STATUS_STALE   0x01
#define LC_STATUS_COMMAND 0x02

// Ganancia del preamplificador, grabada en EEPROM en modo comando
#define LC_PREAMP_GAIN     0b111   // valor de arranque; 'g <n>' lo cambia en caliente
static uint8_t  lc1PreampGain  = LC_PREAMP_GAIN;   // índice 0..7 del preamplificador
static int32_t  lc1CfgOverride = -1;               // >=0: palabra 0x0F cruda ('w')
static uint16_t lc1CfgBefore   = 0;                // lo que había antes de escribir
static uint16_t lc1CfgWritten  = 0;                // lo que se ha dejado grabado
#define LC_A2D_OFFSET      0x08   // valor de arranque; 'o <n>' lo cambia en caliente
static uint8_t lc1AdOffset = LC_A2D_OFFSET;   // índice 0..15 en OFFSET_B_LUT
#define LC_GAIN_POLARITY   1
#define LC_DISABLE_NULLING 0

// ── LC2: NAU7802 ─────────────────────────────────
#define NAU7802_ADDR      0x2A
#define NAU_PU_CTRL       0x00
#define NAU_CTRL1         0x01
#define NAU_CTRL2         0x02
#define NAU_ADCO_B2       0x12   // byte alto del ADC (le siguen B1 y B0)
#define NAU_I2C_CTRL      0x11   // bit 3 = SI (cortocircuita las entradas del PGA)
#define NAU_ADC_REG       0x15   // control del chopper del ADC
#define NAU_PGA           0x1B
#define NAU_PWR_CTRL      0x1C
#define NAU_REVISION      0x1F

// PU_CTRL
#define NAU_RR    0x01   // register reset
#define NAU_PUD   0x02   // power up digital
#define NAU_PUA   0x04   // power up analog
#define NAU_PUR   0x08   // power up ready (solo lectura)
#define NAU_CS    0x10   // cycle start
#define NAU_CR    0x20   // cycle ready (solo lectura) — hay dato nuevo
#define NAU_AVDDS 0x80   // 1 = AVDD desde el LDO interno, 0 = desde el pin AVDD
#define NAU_SI    0x08   // I2C_CTRL bit 3: entradas del PGA en corto

// CTRL1: [2:0] ganancia = 1,2,4,8,16,32,64,128
//        [5:3] VLDO     = 4.5, 4.2, 3.9, 3.6, 3.3, 3.0, 2.7, 2.4 V
#define NAU_GAIN   7     // ×128 (baja a 6=×64 o 5=×32 si satura)
#define NAU_VLDO   5     // 3.0 V — la Click va a 3V3, no puede dar más

// CTRL2: [6:4] tasa = 0,1,2,3,7 → 10,20,40,80,320 SPS
#define NAU_RATE   3     // 80 SPS
#define NAU_CALMOD 0     // 00 = calibración interna de offset

// El chopper del PGA cancela el ruido 1/f y la deriva de offset: DEBE quedarse
// activo. Desactivarlo baja los picos de conmutación pero multiplica por 10 la
// deriva lenta, que es justo el problema que se ve al cargar la célula.
#define NAU_PGA_CHOP_DIS 0

#define NAU_USE_INTERNAL_LDO 1        // ver cabecera: en esta placa es obligatorio
#if NAU_USE_INTERNAL_LDO
  #define NAU_AVDD_BIT NAU_AVDDS
#else
  #define NAU_AVDD_BIT 0x00
#endif
static uint8_t nauAvddBit = NAU_AVDD_BIT;   // conmutable en caliente con 'l'

#define NAU_SAT_LEVEL  8300000L       // por encima de esto el ADC está saturado

// ═══ Estado ══════════════════════════════════════

struct LC1State {                        // ZSC31014
  bool     ok         = false;
  bool     cfgApplied = false;
  bool     valid      = false;
  uint8_t  status     = 0xFF;
  uint16_t bridge     = 0;               // última medida, sin filtrar (0..16383)
  int16_t  zeroOffset = 0;               // tara
  float    scale      = LC1_COUNTS_PER_UNIT;

  enum Phase { REQUEST, WAIT } phase = REQUEST;   // lectura en dos fases
  unsigned long reqTime  = 0;
  unsigned long lastGood = 0;
} lc1;

struct LC2State {                        // NAU7802
  bool    ok      = false;
  bool    calOK   = false;
  bool    valid   = false;
  bool    sat     = false;               // ADC saturado en la ventana actual
  bool    shorted = false;               // modo diagnóstico: entradas en corto
  int32_t raw     = 0;                   // última medida, sin filtrar (24 bit con signo)
  int32_t zeroOffset = 0;                // tara
  float   scale   = LC2_COUNTS_PER_UNIT;

  uint8_t discard = 0;                   // muestras a tirar tras (re)configurar
  unsigned long lastGood = 0;
} lc2;

static bool csvMode = false;

// ═══ LC1 — ZSC31014 ══════════════════════════════

bool lc1Init() {
  I2C1Bus.beginTransmission(ZSC31014_ADDR);
  if (I2C1Bus.endTransmission() != 0) return false;
  I2C1Bus.requestFrom((uint8_t)ZSC31014_ADDR, (uint8_t)2);
  while (I2C1Bus.available()) I2C1Bus.read();
  delay(5);
  return true;
}

// La propia petición dispara la siguiente conversión
void lc1Request() {
  I2C1Bus.requestFrom((uint8_t)ZSC31014_ADDR, (uint8_t)2);
  while (I2C1Bus.available()) I2C1Bus.read();
}

// Recoge la medida: 2 bits de status + 14 bits de puente
bool lc1Fetch(uint16_t &br, uint8_t &st) {
  if (I2C1Bus.requestFrom((uint8_t)ZSC31014_ADDR, (uint8_t)2) < 2) return false;
  uint8_t msb = I2C1Bus.read(), lsb = I2C1Bus.read();
  st = (msb >> 6) & 0x03;
  br = ((uint16_t)(msb & 0x3F) << 8) | lsb;
  return true;
}

// Pide → espera ≥2 ms → recoge. No bloquea el bucle en ningún momento.
void lc1Update() {
  if (!lc1.ok) return;
  unsigned long now = millis();

  if (lc1.phase == LC1State::REQUEST) {
    lc1Request();
    lc1.reqTime = now;
    lc1.phase   = LC1State::WAIT;
  } else if (now - lc1.reqTime >= 2) {
    uint16_t br; uint8_t st;
    if (lc1Fetch(br, st)) {
      lc1.status = st;
      if (st == LC_STATUS_VALID) {
        lc1.bridge   = br;
        lc1.valid    = true;
        lc1.lastGood = now;
        lc1.phase = LC1State::REQUEST;
      }
      // STALE: sigue en WAIT y reintenta la recogida en el próximo ciclo
    } else {
      lc1.phase = LC1State::REQUEST;     // error de bus → vuelve a pedir
    }
  }

  if (lc1.valid && now - lc1.lastGood > LC1_TIMEOUT_MS) lc1.valid = false;
}

// ── Modo comando: graba la ganancia en la EEPROM del ZSC31014 ──
// Sólo accesible durante los primeros ms tras alimentar el chip.
static const uint16_t OFFSET_B_LUT[] = {
  0xE000,0xE400,0xE800,0xEC00,0xF000,0xF400,0xF800,0xFC00,
  0x0000,0x0400,0x0800,0x0C00,0x1000,0x1400,0x1800,0x1C00
};

static bool zscCmd(uint8_t cmd, uint16_t d) {
  I2C1Bus.beginTransmission(ZSC31014_ADDR);
  I2C1Bus.write(cmd); I2C1Bus.write((d >> 8) & 0xFF); I2C1Bus.write(d & 0xFF);
  return I2C1Bus.endTransmission() == 0;
}

static bool zscResp(uint16_t &v) {
  if (I2C1Bus.requestFrom((uint8_t)ZSC31014_ADDR, (uint8_t)3) < 3) return false;
  uint8_t a = I2C1Bus.read(), m = I2C1Bus.read(), l = I2C1Bus.read();
  if (a != 0x5A) return false;
  v = ((uint16_t)m << 8) | l;
  return true;
}

// El pin de reset del ZSC31014 está en GP1 (LC1_EN_PIN): bajarlo y subirlo fuerza un
// Power-On-Reset, la única forma de abrir la ventana de modo comando. Dura pocos ms,
// así que se sondea el Start_CM desde el primer instante en vez de esperar un retardo
// fijo — mientras la alimentación sube el bus NAKea, que es inofensivo.
static bool zscEnterCommandMode() {
  digitalWrite(LC1_EN_PIN, LOW);
  delay(LC_RESET_LOW_MS);               // descarga de los condensadores de la Click
  digitalWrite(LC1_EN_PIN, HIGH);
  unsigned long t0 = millis(); uint16_t v;
  while (millis() - t0 < LC_CM_WINDOW_MS) {
    if (zscCmd(0xA0, 0)) {                         // Start_CM
      delayMicroseconds(100);
      if (zscResp(v)) return true;                 // 0x5A → estamos en modo comando
    }
    delayMicroseconds(200);
  }
  return false;
}

// Dos reglas del datasheet mandan en esta función:
//   · ZMDI_Config sólo se carga tras un ciclo de alimentación, así que escribir y leer
//     dentro de la misma sesión de modo comando devuelve el valor viejo.
//   · La firma MISR de la EEPROM se regenera al SALIR del modo comando con Start_NOM
//     si hubo escritura, y se comprueba en el POR siguiente. Cortar la alimentación
//     sin ese Start_NOM deja la firma mal y el chip se queda en estado de diagnóstico.
// Por eso el bucle: la primera vuelta escribe y sale con Start_NOM, la segunda arranca
// desde un POR limpio y verifica que lo leído ya es lo pedido. Si coincide a la primera
// no se escribe: la EEPROM tiene los ciclos contados.
bool lc1ConfigureEEPROM() {
  bool ok = false;
  for (int attempt = 0; attempt < 2 && !ok; attempt++) {
    if (!zscEnterCommandMode()) break;             // ventana perdida
    uint16_t cur = 0;
    if (!zscCmd(0x0F, 0)) break;                   // lee la config actual
    delayMicroseconds(100);
    if (!zscResp(cur)) break;
    if (attempt == 0) lc1CfgBefore = cur;          // lo que había antes de tocar nada
    uint16_t des = (lc1CfgOverride >= 0) ? (uint16_t)lc1CfgOverride
                 : (cur & 0xE000)
                 | ((uint16_t)(LC_DISABLE_NULLING & 1) << 12) | (0b10 << 10)
                 | (1 << 9) | (1 << 8)
                 | ((uint16_t)(LC_GAIN_POLARITY & 1) << 7)
                 | ((uint16_t)(lc1PreampGain & 7) << 4)
                 | (lc1AdOffset & 0x0F);
    lc1CfgWritten = des;
    if (cur == des) { ok = true; break; }          // ya grabado y cargado
    if (!zscCmd(0x4F, des)) break; delay(15);                          // graba config
    if (!zscCmd(0x43, OFFSET_B_LUT[lc1AdOffset & 0x0F])) break;        // offset del A/D
    delay(15);
    zscCmd(0x80, 0); delay(15);                    // Start_NOM: firma la EEPROM
  }
  zscCmd(0x80, 0); delay(15);                      // Start_NOM → modo normal
  return ok;
}

// Graba un nuevo offset del A/D. La ventana de modo comando la reabre por dentro
// lc1ConfigureEEPROM(), reseteando el chip por LC1_EN_PIN tantas veces como haga
// falta para escribir y verificar. El índice 8 es el centro (offset
// cero, reposo en ~8192); bajar o subir desplaza el cero hacia un extremo y deja
// todo el recorrido de 14 bits para un solo sentido de la fuerza.
//
// Dos avisos: la EEPROM del ZSC31014 tiene ciclos de escritura contados, por eso
// lc1ConfigureEEPROM() sólo escribe si la configuración cambia; y mientras LC1
// está sin tensión sus pines siguen colgados del bus, así que LC2 puede perder
// lecturas durante el apagón.
bool lc1Reconfigure() {
  lc1.ok         = false;               // deja de leer una célula que no está
  lc1.valid      = false;
  lc1.phase      = LC1State::REQUEST;   // la máquina de dos fases vuelve a empezar
  lc1.zeroOffset = 0;                   // la tara anterior ya no significa nada
  lc2.valid      = false;               // el bus queda tocado durante el apagón

  lc1.cfgApplied = lc1ConfigureEEPROM();  // el ciclo de reset lo hace ella misma
  lc1.ok         = lc1Init();
  lc2.discard    = 4;
  return lc1.cfgApplied && lc1.ok;
}

// Enseña lo que había y lo que ha quedado en la palabra 0x0F. Los bits [6:4] son
// la ganancia del preamplificador y los [3:0] el offset del A/D; el resto los fija
// el firmware. Si tras un cambio "antes" y "ahora" coinciden es que ya estaba así.
static void reportReconfig(bool ok) {
  Serial.printf("# LC1 %s | cfg 0x%04X -> 0x%04X | ganancia %d | offset %d (Offset_B 0x%04X)\n",
                ok ? "reconfigurada" : "FALLO al reconfigurar",
                lc1CfgBefore, lc1CfgWritten, lc1PreampGain, lc1AdOffset,
                OFFSET_B_LUT[lc1AdOffset & 0x0F]);
}

// ═══ LC2 — NAU7802 ═══════════════════════════════

static bool nauWrite(uint8_t reg, uint8_t val) {
  I2C1Bus.beginTransmission(NAU7802_ADDR);
  I2C1Bus.write(reg); I2C1Bus.write(val);
  return I2C1Bus.endTransmission() == 0;
}

// STOP en vez de repeated start: el ZSC31014 comparte bus y su protocolo no admite la
// condición de restart — un flanco de bajada de SDA con SCL alto le rompe la
// comunicación SIGUIENTE, aunque ésta vaya dirigida a la NAU7802.
static uint8_t nauReadReg(uint8_t reg) {
  I2C1Bus.beginTransmission(NAU7802_ADDR);
  I2C1Bus.write(reg);
  I2C1Bus.endTransmission();
  I2C1Bus.requestFrom((uint8_t)NAU7802_ADDR, (uint8_t)1);
  return I2C1Bus.available() ? I2C1Bus.read() : 0xFF;
}

// Diagnóstico: cortocircuita las entradas del PGA dentro del chip, dejando fuera
// la célula y su cableado. Lo que siga moviéndose es del NAU7802, no del puente.
void nauShortInputs(bool on) {
  uint8_t v = nauReadReg(NAU_I2C_CTRL);
  nauWrite(NAU_I2C_CTRL, on ? (uint8_t)(v | NAU_SI) : (uint8_t)(v & ~NAU_SI));
  lc2.shorted = on;
  lc2.discard = 4;
}

// Calibración interna de offset. Obligatoria tras cambiar ganancia, LDO o tasa.
bool nauCalibrate() {
  uint8_t c2 = ((uint8_t)(NAU_RATE & 7) << 4) | (NAU_CALMOD & 3);
  nauWrite(NAU_CTRL2, c2 | 0x04);                       // CALS = 1 → arranca
  uint32_t t = millis();
  while (nauReadReg(NAU_CTRL2) & 0x04) {                // espera a que CALS baje
    if (millis() - t > 1000) return false;
    delay(1);
  }
  return (nauReadReg(NAU_CTRL2) & 0x08) == 0;           // CAL_ERR = 0 → OK
}

bool lc2Init() {
  if (!nauWrite(NAU_PU_CTRL, NAU_RR)) return false;                    // reset
  delay(10);
  if (!nauWrite(NAU_PU_CTRL, nauAvddBit | NAU_PUA | NAU_PUD)) return false;
  delay(10);
  uint32_t t = millis();
  while (!(nauReadReg(NAU_PU_CTRL) & NAU_PUR)) {                       // espera PUR
    if (millis() - t > 200) return false;
    delay(5);
  }

  // Ganancia y tensión del LDO comparten registro: hay que escribir las dos
  nauWrite(NAU_CTRL1, ((uint8_t)(NAU_VLDO & 7) << 3) | (NAU_GAIN & 7));
  // La tasa va en los bits [6:4], no en los bajos
  nauWrite(NAU_CTRL2, (uint8_t)(NAU_RATE & 7) << 4);

  // Secuencia de encendido del datasheet (§9.1) — es donde se juega el ruido
  nauWrite(NAU_ADC_REG,  0x30);                                // chopper del ADC apagado
  nauWrite(NAU_PWR_CTRL, nauReadReg(NAU_PWR_CTRL) | 0x80);     // cap de 330 pF en el PGA
#if NAU_PGA_CHOP_DIS
  nauWrite(NAU_PGA,      nauReadReg(NAU_PGA) | 0x01);
#else
  nauWrite(NAU_PGA,      (uint8_t)(nauReadReg(NAU_PGA) & ~0x01));   // chopper del PGA activo
#endif

  lc2.calOK = nauCalibrate();

  nauWrite(NAU_PU_CTRL, nauAvddBit | NAU_CS | NAU_PUA | NAU_PUD);     // conversiones continuas
  lc2.discard = 4;                                                    // las primeras son basura
  delay(300);
  return true;
}

// Lee sólo si hay dato nuevo (bit CR), con el signo ya extendido a 32 bits
bool lc2Read(int32_t &value) {
  if (!(nauReadReg(NAU_PU_CTRL) & NAU_CR)) return false;
  I2C1Bus.beginTransmission(NAU7802_ADDR);
  I2C1Bus.write(NAU_ADCO_B2);
  I2C1Bus.endTransmission();                     // sin repeated start (ver nauReadReg)
  if (I2C1Bus.requestFrom((uint8_t)NAU7802_ADDR, (uint8_t)3) < 3) return false;
  uint8_t b2 = I2C1Bus.read(), b1 = I2C1Bus.read(), b0 = I2C1Bus.read();
  value = ((int32_t)b2 << 16) | ((int32_t)b1 << 8) | b0;
  if (value & 0x800000) value |= 0xFF000000;
  return true;
}

void lc2Update() {
  if (!lc2.ok) return;
  unsigned long now = millis();
  int32_t v;

  if (!lc2Read(v)) {
    if (lc2.valid && now - lc2.lastGood > LC2_TIMEOUT_MS) lc2.valid = false;
    return;
  }
  if (lc2.discard) { lc2.discard--; return; }
  if (v > NAU_SAT_LEVEL || v < -NAU_SAT_LEVEL) lc2.sat = true;

  lc2.raw      = v;
  lc2.valid    = true;
  lc2.lastGood = now;
}

// ═══ Comandos ════════════════════════════════════

static void doTare() {
  lc1.zeroOffset = (int16_t)lc1.bridge;
  lc2.zeroOffset = lc2.raw;
  Serial.printf("# Tara: LC1=%d  LC2=%ld\n", lc1.zeroOffset, (long)lc2.zeroOffset);
}

// Calibra con la carga conocida que haya puesta ahora mismo.
static void doCalibrate(uint8_t cell, float knownLoad) {
  if (knownLoad == 0.0f) { Serial.println("# Carga 0 no sirve para calibrar"); return; }
  if (cell == 1) {
    if (!lc1.valid) { Serial.println("# LC1 sin medida valida"); return; }
    lc1.scale = ((float)lc1.bridge - (float)lc1.zeroOffset) / knownLoad;
    Serial.print("# LC1_COUNTS_PER_UNIT = "); Serial.println(lc1.scale, 4);
  } else {
    if (!lc2.valid) { Serial.println("# LC2 sin medida valida"); return; }
    lc2.scale = ((float)lc2.raw - (float)lc2.zeroOffset) / knownLoad;
    Serial.print("# LC2_COUNTS_PER_UNIT = "); Serial.println(lc2.scale, 4);
  }
  Serial.println("# Pega ese valor en la constante de arriba para dejarlo fijo");
}

static char   cmdBuf[24];
static uint8_t cmdLen  = 0;
static bool    cmdLine = false;   // true mientras se acumula una orden con argumento

static void runLine(const char *line) {
  if (line[0] == 'k' && (line[1] == '1' || line[1] == '2')) {
    doCalibrate(line[1] - '0', atof(line + 2));
  } else if (line[0] == 'o') {
    int idx = atoi(line + 1);
    if (idx < 0 || idx > 15) { Serial.println("# Offset fuera de rango (0..15)"); return; }
    lc1AdOffset = (uint8_t)idx;
    reportReconfig(lc1Reconfigure());
    Serial.println("# 8 = centrado. Baja o sube y mira adonde se va el reposo.");
  } else if (line[0] == 'g') {
    int g = atoi(line + 1);
    if (g < 0 || g > 7) { Serial.println("# Ganancia fuera de rango (0..7)"); return; }
    lc1PreampGain = (uint8_t)g;
    reportReconfig(lc1Reconfigure());
    Serial.println("# Aplica tu fuerza maxima y mira cuantas cuentas se mueve");
  } else if (line[0] == 'w') {
    long v = strtol(line + 1, nullptr, 0);          // acepta 0x....
    lc1CfgOverride = (v == 0) ? -1 : (int32_t)(v & 0xFFFF);
    reportReconfig(lc1Reconfigure());
    if (lc1CfgOverride < 0) Serial.println("# Vuelta a la palabra calculada");
  } else {
    Serial.println("# Ordenes: k1/k2 <valor>, o <0..15>, g <0..7>, w <palabra hex>");
  }
}

void checkCommands() {
  while (Serial.available()) {
    char c = Serial.read();

    if (cmdLine) {                                  // acumulando "k1 5.0"
      if (c == '\n' || c == '\r') {
        cmdBuf[cmdLen] = '\0';
        runLine(cmdBuf);
        cmdLen = 0; cmdLine = false;
      } else if (cmdLen < sizeof(cmdBuf) - 1) {
        cmdBuf[cmdLen++] = c;
      }
      continue;
    }

    switch (c) {
      case 't': doTare(); break;
      case 'r':
        lc1.zeroOffset = 0; lc2.zeroOffset = 0;
        Serial.println("# Tara reiniciada");
        break;
      case 'c':
        if (!lc2.ok) break;
        nauWrite(NAU_PU_CTRL, nauAvddBit | NAU_PUA | NAU_PUD);          // para conversiones
        lc2.calOK = nauCalibrate();
        nauWrite(NAU_PU_CTRL, nauAvddBit | NAU_CS | NAU_PUA | NAU_PUD);
        lc2.discard = 4;
        Serial.printf("# Recalibracion LC2: %s\n", lc2.calOK ? "OK" : "FALLO");
        break;
      case 's':
        if (!lc2.ok) break;
        nauShortInputs(!lc2.shorted);
        Serial.printf("# LC2 entradas en corto: %s\n", lc2.shorted ? "SI" : "no");
        break;
      case 'l':
        if (!lc2.ok) break;
        nauAvddBit = nauAvddBit ? 0x00 : NAU_AVDDS;
        lc2.ok = lc2Init();
        Serial.printf("# LC2 AVDD: %s (init %s)\n",
                      nauAvddBit ? "LDO interno" : "pin de la Click",
                      lc2.ok ? "OK" : "FALLO");
        break;
      case 'v':
        csvMode = !csvMode;
        if (csvMode) Serial.println("t_ms,lc1,lc2");
        break;
      case 'k':
      case 'o':
      case 'g':
      case 'w':
        cmdBuf[0] = c; cmdLen = 1; cmdLine = true;
        break;
      default: break;                               // ignora ruido y saltos de línea
    }
  }
}

// ═══ Salida ══════════════════════════════════════

static void printUnits(float counts, float scale) {
  if (scale == 0.0f) return;
  Serial.print(" = "); Serial.print(counts / scale, 3); Serial.print(" " UNIT_NAME);
}

static void printReadable() {
  if (lc1.valid) {
    Serial.printf("LC1: %5u (%+6d)", lc1.bridge, (int)lc1.bridge - lc1.zeroOffset);
    printUnits((float)lc1.bridge - (float)lc1.zeroOffset, lc1.scale);
    Serial.print("  |  ");
  } else {
    Serial.printf("LC1: %-17s |  ",
      !lc1.ok ? "NO_INIT" : (lc1.status == LC_STATUS_STALE ? "STALE" : "SIN_DATO"));
  }

  if (lc2.valid) {
    Serial.printf("LC2: %9ld (%+9ld)", (long)lc2.raw, (long)(lc2.raw - lc2.zeroOffset));
    printUnits((float)lc2.raw - (float)lc2.zeroOffset, lc2.scale);
    if (lc2.sat)          Serial.print("  SATURADA (baja NAU_GAIN)");
    else if (lc2.shorted) Serial.print("  [entradas en corto]");
    Serial.println();
  } else {
    Serial.println(lc2.ok ? "LC2: SIN_DATO" : "LC2: NO_INIT");
  }
}

static void printCsv(unsigned long now) {
  Serial.printf("%lu,%d,%ld\n", now,
                lc1.valid ? (int)lc1.bridge : -1,
                lc2.valid ? (long)lc2.raw   : 0L);
}

// ═══ Setup ═══════════════════════════════════════

void setup() {
  // Las células PRIMERO: la ventana de modo comando del ZSC31014 dura pocos ms
  // tras alimentarlo, así que hay que entrar antes de inicializar el Serial.
  pinMode(LC1_EN_PIN, OUTPUT);
  pinMode(LC2_EN_PIN, OUTPUT);
  gpio_set_drive_strength(LC1_EN_PIN, GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_drive_strength(LC2_EN_PIN, GPIO_DRIVE_STRENGTH_12MA);
  digitalWrite(LC1_EN_PIN, HIGH);
  digitalWrite(LC2_EN_PIN, HIGH);
  delay(50);

  I2C1Bus.begin();
  I2C1Bus.setClock(LC_I2C_FREQ);

  lc1.cfgApplied = lc1ConfigureEEPROM();
  lc1.ok         = lc1Init();
  lc2.ok         = lc2Init();

  Serial.begin(115200);
  { unsigned long t0 = millis(); while (!Serial && millis() - t0 < 3000) delay(10); }

  Serial.println("# ── Celulas de carga duales (I2C1: GP2/GP3 @ 100 kHz) ──");
  Serial.printf("# LC1 ZSC31014 @ 0x%02X: %-13s config EEPROM: %s\n",
                ZSC31014_ADDR, lc1.ok ? "OK" : "NO ENCONTRADA",
                lc1.cfgApplied ? "aplicada" : "FALLO");
  Serial.printf("# LC2 NAU7802  @ 0x%02X: %s\n",
                NAU7802_ADDR, lc2.ok ? "OK" : "NO ENCONTRADA / error init");
  if (lc2.ok) {
    Serial.printf("#     Revision 0x%02X (esperado 0x0F)  calibracion %s  ganancia x%d  %d SPS\n",
                  nauReadReg(NAU_REVISION) & 0x0F, lc2.calOK ? "OK" : "FALLO",
                  1 << NAU_GAIN, NAU_RATE == 3 ? 80 : (NAU_RATE == 7 ? 320 : 10 << NAU_RATE));
    Serial.printf("#     AVDD: %s\n", nauAvddBit ? "LDO interno 3.0 V" : "pin de la Click");
  }
  Serial.println("# Comandos: t tara | r reset tara | c recalibra LC2 | s corto LC2");
  Serial.println("#           l fuente AVDD | v CSV | k1 <valor> / k2 <valor> calibran");
  Serial.printf("#           o <0..15> cero de LC1 (ahora %d) | g <0..7> ganancia (ahora %d)\n",
                lc1AdOffset, lc1PreampGain);
  if (lc1.scale == 0.0f && lc2.scale == 0.0f)
    Serial.println("# Sin calibrar: se muestran cuentas en crudo");
}

// ═══ Loop ════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  lc1Update();        // ZSC31014 — dos fases, no bloquea
  lc2Update();        // NAU7802  — sólo cuando hay dato nuevo
  checkCommands();

  static unsigned long lastPrint = 0;
  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;
    if (csvMode) printCsv(now); else printReadable();
    lc2.sat = false;
  }

  unsigned long dt = millis() - now;
  if (dt < LOOP_MS) delay(LOOP_MS - dt);
}
