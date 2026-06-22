#include <SerialPIO.h>
#include <ModbusMaster.h>

#define RS485_TX  14
#define RS485_RX  13
#define RS485_EN  12

SerialPIO Serial485(RS485_TX, RS485_RX);
ModbusMaster servo;

void preTransmission()  { digitalWrite(RS485_EN, HIGH); }
void postTransmission() { delayMicroseconds(200); digitalWrite(RS485_EN, LOW); }

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("Modbus test — ID=1  GP14 TX / GP13 RX / GP12 DE/RE @ 115200");
  Serial.println("Result codes: 0x00=OK  0xE2=timeout  0xE3=bad CRC");
  Serial.println("----------------------------------------------------");

  pinMode(RS485_EN, OUTPUT);
  digitalWrite(RS485_EN, LOW);

  Serial485.begin(115200);
  Serial485.setTimeout(100);

  servo.preTransmission(preTransmission);
  servo.postTransmission(postTransmission);
}

void loop() {
  servo.begin(1, Serial485);
  uint8_t result = servo.readHoldingRegisters(0, 1);

  if (result == 0x00) {
    Serial.printf("ID=1  OK  reg0=%d\n", servo.getResponseBuffer(0));
  } else {
    const char* desc = (result == 0xE2) ? "timeout" :
                       (result == 0xE3) ? "bad CRC" :
                       (result == 0xE0) ? "invalid slave ID" : "slave error";
    Serial.printf("ID=1  0x%02X  %s\n", result, desc);
  }
  delay(500);
}
