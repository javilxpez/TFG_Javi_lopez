// Test Entradas Digitales — GP16, GP17, GP18, GP19
// Lee el estado cada 500ms y avisa cuando cambia

#define DIN1_PIN  16
#define DIN2_PIN  17
#define DIN3_PIN  18 
#define DIN4_PIN  19

static const uint8_t pins[4] = {DIN1_PIN, DIN2_PIN, DIN3_PIN, DIN4_PIN};
static bool lastState[4] = {false, false, false, false};

void printAll(bool states[4]) {
  for (uint8_t i = 0; i < 4; i++)
    Serial.printf("  DIN%d (GP%02d): %s\n", i + 1, pins[i], states[i] ? "HIGH" : "LOW");
  Serial.println("─────────────────────");
}

void setup() {
  Serial.begin(115200);
  { unsigned long t0 = millis(); while (!Serial && millis() - t0 < 3000) delay(10); }

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(pins[i], INPUT);
    lastState[i] = digitalRead(pins[i]);
  }

  Serial.println("# Test Entradas Digitales — Pico 2");
  Serial.println("# Estado inicial:");
  printAll(lastState);
}

void loop() {
  bool changed = false;
  bool current[4];

  for (uint8_t i = 0; i < 4; i++) {
    current[i] = digitalRead(pins[i]);
    if (current[i] != lastState[i]) {
      Serial.printf(">> CAMBIO DIN%d (GP%02d): %s → %s\n",
        i + 1, pins[i],
        lastState[i] ? "HIGH" : "LOW",
        current[i]  ? "HIGH" : "LOW");
      lastState[i] = current[i];
      changed = true;
    }
  }

  if (changed) printAll(current);

  delay(50);
}
