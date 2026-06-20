// Test SIN1 — toggle cada 1 segundo
// SIN1 → GP10

#define SIN1_PIN  10

void setup() {
  Serial.begin(115200);
  pinMode(SIN1_PIN, OUTPUT);
  digitalWrite(SIN1_PIN, LOW);
  Serial.println("# SIN1 toggling cada 1s");
}

void loop() {
  digitalWrite(SIN1_PIN, HIGH);
  Serial.println("SIN1 ON");
  delay(1000);

  digitalWrite(SIN1_PIN, LOW);
  Serial.println("SIN1 OFF");
  delay(1000);
}
