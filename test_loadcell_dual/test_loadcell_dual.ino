#include <Wire.h>

// I2C1 en GP2 (SDA) / GP3 (SCL) — ambos sensores en el mismo bus (distintas dir.)
TwoWire I2C1Bus(i2c1, 2, 3);

// ── LC1: Load Cell 2 Click — ZSC31014 @ 0x28 ─────────────
#define ZSC31014_ADDR   0x28
#define LC1_EN_PIN      1    // GP1

// ── LC2: Load Cell 4 Click — NAU7802 @ 0x2A ──────────────
#define NAU7802_ADDR    0x2A
#define LC2_EN_PIN      0    // GP0

// NAU7802 registros
#define NAU7802_PU_CTRL  0x00
#define NAU7802_CTRL1    0x01
#define NAU7802_CTRL2    0x02
#define NAU7802_ADCO_B2  0x12  // byte alto del ADC (seguido de B1, B0)

// ── NAU7802 helpers ───────────────────────────────────────
static bool nau7802Write(uint8_t reg, uint8_t val) {
  I2C1Bus.beginTransmission(NAU7802_ADDR);
  I2C1Bus.write(reg);
  I2C1Bus.write(val);
  return I2C1Bus.endTransmission() == 0;
}

static uint8_t nau7802ReadReg(uint8_t reg) {
  I2C1Bus.beginTransmission(NAU7802_ADDR);
  I2C1Bus.write(reg);
  I2C1Bus.endTransmission(false);
  I2C1Bus.requestFrom((uint8_t)NAU7802_ADDR, (uint8_t)1);
  return I2C1Bus.available() ? I2C1Bus.read() : 0xFF;
}

bool nau7802Init() {
  // Reset
  if (!nau7802Write(NAU7802_PU_CTRL, 0x01)) return false;
  delay(10);
  // Power up digital + analógico, AVDDS=1 (usa AVDD externo de la placa, no LDO interno)
  if (!nau7802Write(NAU7802_PU_CTRL, 0x86)) return false;  // bit7=AVDDS, bit2=PUA, bit1=PUD
  delay(10);
  // Esperar PUR (power-up ready, bit 3)
  uint32_t t = millis();
  while (!(nau7802ReadReg(NAU7802_PU_CTRL) & 0x08)) {
    if (millis() - t > 200) return false;
    delay(5);
  }
  // Ganancia = 32 (GAINS bits [2:0] = 101)
  nau7802Write(NAU7802_CTRL1, 0x05);
  // Tasa de conversión = 80 SPS
  nau7802Write(NAU7802_CTRL2, 0x0C);
  // Arrancar conversiones (CS bit), mantener AVDDS=1
  nau7802Write(NAU7802_PU_CTRL, 0x96);  // AVDDS + CS + PUA + PUD
  delay(300);  // primera conversión estable
  return true;
}

// Devuelve true si hay dato nuevo (CR bit), escribe el valor de 24 bits con signo
bool nau7802Read(int32_t &value) {
  if (!(nau7802ReadReg(NAU7802_PU_CTRL) & 0x20)) return false;  // CR bit
  I2C1Bus.beginTransmission(NAU7802_ADDR);
  I2C1Bus.write(NAU7802_ADCO_B2);
  I2C1Bus.endTransmission(false);
  if (I2C1Bus.requestFrom((uint8_t)NAU7802_ADDR, (uint8_t)3) < 3) return false;
  uint8_t b2 = I2C1Bus.read();
  uint8_t b1 = I2C1Bus.read();
  uint8_t b0 = I2C1Bus.read();
  value = ((int32_t)b2 << 16) | ((int32_t)b1 << 8) | b0;
  if (value & 0x800000) value |= 0xFF000000;  // sign extend 24→32 bit
  return true;
}

// ── ZSC31014 read ─────────────────────────────────────────
bool zscRead(uint16_t &bridge, uint8_t &status) {
  // Descartar lectura anterior
  I2C1Bus.requestFrom((uint8_t)ZSC31014_ADDR, (uint8_t)2);
  while (I2C1Bus.available()) I2C1Bus.read();
  delay(5);
  // Leer dato fresco
  if (I2C1Bus.requestFrom((uint8_t)ZSC31014_ADDR, (uint8_t)2) < 2) return false;
  uint8_t msb = I2C1Bus.read();
  uint8_t lsb = I2C1Bus.read();
  status = (msb >> 6) & 0x03;
  bridge = ((uint16_t)(msb & 0x3F) << 8) | lsb;
  return true;
}

// ── Setup ─────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("# Test dual load cell — distintas direcciones I2C");

  // Ambos EN en HIGH a la vez (no hay conflicto de direccion)
  pinMode(LC1_EN_PIN, OUTPUT);
  pinMode(LC2_EN_PIN, OUTPUT);
  digitalWrite(LC1_EN_PIN, HIGH);
  digitalWrite(LC2_EN_PIN, HIGH);
  delay(50);

  I2C1Bus.begin();
  I2C1Bus.setClock(100000);

  // Verificar LC1 (ZSC31014)
  I2C1Bus.beginTransmission(ZSC31014_ADDR);
  Serial.printf("# LC1 ZSC31014 @ 0x28: %s\n",
    I2C1Bus.endTransmission() == 0 ? "OK" : "NO ENCONTRADA");

  // Inicializar LC2 (NAU7802)
  bool lc2ok = nau7802Init();
  Serial.printf("# LC2 NAU7802  @ 0x2A: %s\n", lc2ok ? "OK" : "NO ENCONTRADA / error init");

  if (lc2ok) {
    // Leer registro PU_CTRL para confirmar estado
    uint8_t pu = nau7802ReadReg(NAU7802_PU_CTRL);
    Serial.printf("# LC2 PU_CTRL = 0x%02X  (esperado ~0x16, PUR=1 CR=?)\n", pu);
    // Leer revision ID (registro 0x1F)
    uint8_t rev = nau7802ReadReg(0x1F);
    Serial.printf("# LC2 Revision = 0x%02X  (NAU7802 = 0x0F)\n", rev);
  }

  Serial.println("# LC1_bridge (0-16383) | LC2_raw (24-bit con signo)");
}

// ── Filtro media móvil LC2 ────────────────────────────────
#define MA2_SIZE  16
static int32_t ma2Buf[MA2_SIZE] = {};
static uint8_t ma2Idx = 0;
static int64_t ma2Sum = 0;
static uint8_t ma2Count = 0;

int32_t ma2Filter(int32_t newVal) {
  ma2Sum -= ma2Buf[ma2Idx];
  ma2Buf[ma2Idx] = newVal;
  ma2Sum += newVal;
  ma2Idx = (ma2Idx + 1) % MA2_SIZE;
  if (ma2Count < MA2_SIZE) ma2Count++;
  return (int32_t)(ma2Sum / ma2Count);
}

// ── Loop ──────────────────────────────────────────────────
void loop() {
  uint16_t bridge = 0;
  uint8_t  zscStatus = 0xFF;
  int32_t  adc = 0;

  bool ok1    = zscRead(bridge, zscStatus);
  bool valid1 = ok1 && zscStatus == 0x00;
  bool ok2    = nau7802Read(adc);

  if (valid1)
    Serial.printf("LC1: %5d  |  ", bridge);
  else
    Serial.printf("LC1: %-6s |  ", !ok1 ? "I2C_ERR" : "STALE  ");

  if (ok2)
    Serial.printf("LC2: %10d  (filt: %10d)\n", adc, ma2Filter(adc));
  else
    Serial.println("LC2: NO_DATA");

  delay(200);
}
