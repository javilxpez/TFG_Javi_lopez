// Test RX: PC sends bytes via USB-RS485 → RS485 module → GP13 Pico
// Pico prints what it receives on the USB serial monitor

#include <SerialPIO.h>

#define RS485_TX  14
#define RS485_RX  13
#define RS485_EN  12

SerialPIO Serial485(RS485_TX, RS485_RX);

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("# RX test — listening on GP13 at 115200");
  Serial.println("# Send bytes from PC via USB-RS485 adapter");

  pinMode(RS485_EN, OUTPUT);
  digitalWrite(RS485_EN, LOW);  // receive mode, never transmit

  Serial485.begin(115200);
}

void loop() {
  if (Serial485.available()) {
    uint8_t b = Serial485.read();
    Serial.printf("RX: 0x%02X\n", b);
  }
}
