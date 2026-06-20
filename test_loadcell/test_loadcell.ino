#include <Wire.h>

#define PIN_INT1  0
#define PIN_EN1   1
#define PIN_SDA1  2
#define PIN_SCL1  3

#define ZSC31014_ADDR  0x28

// I2C1 en GP2/GP3
TwoWire I2C1Bus(i2c1, PIN_SDA1, PIN_SCL1);

#define STATUS_VALID    0x00
#define STATUS_STALE    0x01
#define STATUS_COMMAND  0x02

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("# Iniciando...");

  pinMode(PIN_EN1, OUTPUT);
  digitalWrite(PIN_EN1, HIGH);
  delay(50);

  pinMode(PIN_INT1, INPUT);

  I2C1Bus.begin();
  I2C1Bus.setClock(100000);
  Serial.println("# I2C iniciado");

  I2C1Bus.beginTransmission(ZSC31014_ADDR);
  uint8_t err = I2C1Bus.endTransmission();
  if (err == 0)
    Serial.println("# Sensor OK en 0x28");
  else
    Serial.printf("# ERROR: Sensor no encontrado (err=%d)\n", err);

  // Descartar primera lectura
  I2C1Bus.requestFrom((uint8_t)ZSC31014_ADDR, (uint8_t)2);
  while (I2C1Bus.available()) I2C1Bus.read();
  delay(10);

  Serial.println("# Raw   | Status  | Bridge | INT1");
}

void loop() {
  uint8_t count = I2C1Bus.requestFrom((uint8_t)ZSC31014_ADDR, (uint8_t)2);

  if (count < 2) {
    Serial.println("ERROR I2C");
    delay(500);
    return;
  }

  uint8_t msb = I2C1Bus.read();
  uint8_t lsb = I2C1Bus.read();

  uint8_t  status = (msb >> 6) & 0x03;
  uint16_t bridge = ((uint16_t)(msb & 0x3F) << 8) | lsb;
  bool     int1   = digitalRead(PIN_INT1);

  const char* st = (status == STATUS_VALID) ? "VALID  " :
                   (status == STATUS_STALE)  ? "STALE  " : "COMMAND";

  Serial.printf("  %5d | %s | %5d | %s\n",
    (msb << 8) | lsb, st, bridge, int1 ? "HIGH" : "LOW");

  delay(200);
}
