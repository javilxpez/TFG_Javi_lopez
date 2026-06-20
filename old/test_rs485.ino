// ═══════════════════════════════════════════════════════════
//  Test RS485 simple — Raspberry Pi Pico 2
//  Si el PC manda "a" por el adaptador USB-RS485, la Pico
//  responde "b" por RS485. Verifica que TX y RX funcionan.
// ═══════════════════════════════════════════════════════════
//
//  Pines RS485:
//    GP12 → DE/RE  (direction control)
//    GP13 → RO     (RX)
//    GP14 → DI     (TX)
//
//  PC: abre el adaptador USB-RS485 en el monitor serie a 115200
//  Escribe "a" → debe llegar "b"
// ═══════════════════════════════════════════════════════════

#include <SerialPIO.h>

#define MB_TX_GPIO  14
#define MB_RX_GPIO  13
#define MB_EN_GPIO  12

static SerialPIO Serial485(MB_TX_GPIO, MB_RX_GPIO);

void rs485Send(const char *msg) {
  digitalWrite(MB_EN_GPIO, HIGH);  // TX
  delayMicroseconds(200);
  Serial485.print(msg);
  Serial485.flush();
  delayMicroseconds(200);
  digitalWrite(MB_EN_GPIO, LOW);   // RX
}

void setup() {
  Serial.begin(115200);
  { unsigned long t0 = millis(); while (!Serial && millis() - t0 < 3000) delay(10); }

  pinMode(MB_EN_GPIO, OUTPUT);
  digitalWrite(MB_EN_GPIO, LOW);
  Serial485.begin(115200);

  Serial.println("# RS485 echo listo. Manda 'a' desde el adaptador USB.");
}

void loop() {
  if (Serial485.available()) {
    char c = Serial485.read();
    Serial.printf("[RX por RS485] '%c'\n", c);  // log en monitor Pico

    if (c == 'a') {
      rs485Send("b\n");
      Serial.println("[TX por RS485] 'b'");
    }
  }
}
