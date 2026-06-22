// Pico transmite por RS485 cada segundo
// PC escucha con USB-RS485 adapter

#include <SerialPIO.h>

#define RS485_TX  14
#define RS485_RX  13
#define RS485_EN  12

SerialPIO Serial485(RS485_TX, RS485_RX);

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("# Transmitiendo por RS485...");

  pinMode(RS485_EN, OUTPUT);
  digitalWrite(RS485_EN, LOW);

  Serial485.begin(115200);
  delay(100);
}

uint8_t counter = 0;

void loop() {
  uint8_t msg[] = {0xAA, 0x55, counter, 0xBB};

  digitalWrite(RS485_EN, HIGH);
  delayMicroseconds(100);
  Serial485.write(msg, 4);
  Serial485.flush();
  delayMicroseconds(500);
  digitalWrite(RS485_EN, LOW);

  Serial.printf("# TX: AA 55 %02X BB\n", counter);
  counter++;
  delay(1000);
}
