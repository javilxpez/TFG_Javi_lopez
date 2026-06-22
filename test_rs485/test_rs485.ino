// ═══════════════════════════════════════════════════
//  Test diagnóstico RS485 + Modbus RTU — Pico 2
//  Prueba baud rates: 9600 / 19200 / 38400 / 115200
//  Prueba IDs Modbus: 1 a 5
//
//  Cableado esperado:
//    GP14 → DI  del módulo RS485  (transmisión)
//    GP13 ← RO  del módulo RS485  (recepción)
//    GP12 → DE/RE del módulo RS485 (dirección)
// ═══════════════════════════════════════════════════

#include <SerialPIO.h>

#define RS485_TX   14
#define RS485_RX   13
#define RS485_EN   12

SerialPIO Serial485(RS485_TX, RS485_RX);

// ── CRC16 Modbus ──────────────────────────────────
uint16_t crc16(const uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

// ── Envía Read Holding Registers (FC=03) ─────────
// Lee registro 0, cantidad 1, desde slave slaveID
// Devuelve bytes recibidos (0 = timeout)
uint8_t modbusRead(uint8_t slaveID, uint8_t rxBuf[], uint8_t rxMax) {
  uint8_t req[8];
  req[0] = slaveID;
  req[1] = 0x03;
  req[2] = 0x00; req[3] = 0x00;  // registro 0
  req[4] = 0x00; req[5] = 0x01;  // 1 registro
  uint16_t crc = crc16(req, 6);
  req[6] = crc & 0xFF;
  req[7] = (crc >> 8) & 0xFF;

  Serial.printf("  TX [ID=%d]: ", slaveID);
  for (int i = 0; i < 8; i++) Serial.printf("%02X ", req[i]);
  Serial.println();

  // Vaciar buffer antes de enviar
  while (Serial485.available()) Serial485.read();

  digitalWrite(RS485_EN, HIGH);
  delayMicroseconds(100);
  Serial485.write(req, 8);
  Serial485.flush();
  delayMicroseconds(200);
  digitalWrite(RS485_EN, LOW);

  // Esperar respuesta (máx 100 ms)
  uint32_t t = millis();
  uint8_t n = 0;
  while (millis() - t < 100 && n < rxMax) {
    if (Serial485.available()) {
      rxBuf[n++] = Serial485.read();
      t = millis();  // resetear timeout si llegan bytes
    }
  }
  return n;
}

// ── Prueba a una baud rate ─────────────────────────
void testBaud(uint32_t baud) {
  Serial.printf("\n══ %lu baud ══════════════════════════\n", baud);
  Serial485.end();
  delay(20);
  Serial485.begin(baud);
  delay(100);

  bool found = false;
  uint8_t rxBuf[32];

  for (uint8_t id = 1; id <= 5; id++) {
    uint8_t n = modbusRead(id, rxBuf, sizeof(rxBuf));

    if (n > 0) {
      Serial.printf("  RX %d bytes: ", n);
      for (uint8_t i = 0; i < n; i++) Serial.printf("%02X ", rxBuf[i]);

      // Verificar si la respuesta tiene CRC válido
      if (n >= 5) {
        uint16_t rxCRC  = (uint16_t)rxBuf[n-1] << 8 | rxBuf[n-2];
        uint16_t calcCRC = crc16(rxBuf, n - 2);
        if (rxCRC == calcCRC)
          Serial.println("  ✓ CRC OK  ← ¡SERVO ENCONTRADO!");
        else
          Serial.println("  ✗ CRC MAL (basura o baud rate incorrecto)");
      } else {
        Serial.println("  ? respuesta corta");
      }
      found = true;
    } else {
      Serial.println("  RX: timeout (sin respuesta)");
    }
    delay(50);
  }

  if (!found)
    Serial.println("  → Ningún esclavo respondió a esta velocidad.");
}

// ── Setup ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(3000);

  Serial.println("# ═══════════════════════════════════════════");
  Serial.println("#  Test diagnóstico RS485 — Raspberry Pi Pico 2");
  Serial.println("#  GP14=TX(DI)   GP13=RX(RO)   GP12=DE/RE");
  Serial.println("# ═══════════════════════════════════════════");
  Serial.println("# Probando IDs Modbus 1-5 a 4 velocidades...\n");

  pinMode(RS485_EN, OUTPUT);
  digitalWrite(RS485_EN, LOW);

  testBaud(9600);
  testBaud(19200);
  testBaud(38400);
  testBaud(115200);

  Serial.println("\n# ═══════════════════════════════════════════");
  Serial.println("# FIN DEL TEST");
  Serial.println("#");
  Serial.println("# Si TODOS los IDs dieron timeout:");
  Serial.println("#   → Revisar cableado (DI/RO invertidos, GP14/13/12)");
  Serial.println("#   → Verificar alimentación RS485 y del servo");
  Serial.println("#   → Comprobar cables A/B entre módulo y servo");
  Serial.println("#");
  Serial.println("# Si aparece 'basura' (bytes pero CRC MAL):");
  Serial.println("#   → El servo responde pero a otra velocidad");
  Serial.println("# ═══════════════════════════════════════════");
}

void loop() {
  delay(10000);
}
