// ═══════════════════════════════════════════════════
//  Test LOOPBACK GPIO — Pico 2
//
//  ANTES DE CARGAR:
//    Conecta un cable directamente de GP14 a GP13
//    (sin pasar por el módulo RS485).
//    Esto verifica que SerialPIO funciona.
//
// ═══════════════════════════════════════════════════

#include <SerialPIO.h>

#define RS485_TX   14
#define RS485_RX   13

SerialPIO Serial485(RS485_TX, RS485_RX);

void setup() {
  Serial.begin(115200);
  delay(3000);

  Serial.println("# Test loopback directo GP14 → GP13");
  Serial.println("# (sin modulo RS485, cable directo entre pines)");
  Serial.println();

  uint32_t bauds[] = {9600, 115200};

  for (int bi = 0; bi < 2; bi++) {
    uint32_t baud = bauds[bi];
    Serial485.end();
    delay(20);
    Serial485.begin(baud);
    delay(50);

    while (Serial485.available()) Serial485.read();

    uint8_t msg[] = {0xAA, 0x01, 0x02, 0x03, 0x55};
    Serial.printf("══ %lu baud ══\n", baud);
    Serial.printf("  TX: AA 01 02 03 55\n");

    Serial485.write(msg, 5);
    Serial485.flush();

    uint32_t t = millis();
    uint8_t rxBuf[16];
    uint8_t n = 0;
    while (millis() - t < 200 && n < 16) {
      if (Serial485.available()) rxBuf[n++] = Serial485.read();
    }

    if (n > 0) {
      Serial.printf("  RX %d bytes: ", n);
      for (uint8_t i = 0; i < n; i++) Serial.printf("%02X ", rxBuf[i]);

      bool match = (n == 5);
      if (match)
        for (uint8_t i = 0; i < 5; i++) if (rxBuf[i] != msg[i]) { match = false; break; }

      Serial.println(match ? "  ✓ SerialPIO OK" : "  ? bytes distintos");
    } else {
      Serial.println("  RX: sin eco → SerialPIO no funciona o cable suelto");
    }
    Serial.println();
    delay(50);
  }

  Serial.println("═══════════════════════════════════════");
  Serial.println("✓ OK  → SerialPIO funciona, problema en modulo RS485 o servo");
  Serial.println("sin eco → GP14/GP13 mal configurados o cable suelto");
  Serial.println("═══════════════════════════════════════");
}

void loop() { delay(10000); }
