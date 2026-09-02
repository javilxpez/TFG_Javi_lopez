// Test final de carrera — DI1=GPIO16, DI2=GPIO17
// Conectar: COM→GND, NC→terminal DI1/DI2

#define PIN_LIMIT_A  16   // DI1 del conector externo
#define PIN_LIMIT_B  17   // DI2 del conector externo

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  pinMode(PIN_LIMIT_A, INPUT_PULLUP);
  pinMode(PIN_LIMIT_B, INPUT_PULLUP);

  Serial.println("=== Test Final de Carrera ===");
  Serial.println("LOW  = normal (switch sin pulsar)");
  Serial.println("HIGH = activado (limite alcanzado)");
  Serial.println("=============================");
}

bool lastA = false;
bool lastB = false;

void loop() {
  bool a = digitalRead(PIN_LIMIT_A);
  bool b = digitalRead(PIN_LIMIT_B);

  if (a != lastA || b != lastB) {
    Serial.print("A (DI1/GPIO16): ");
    Serial.print(a ? "HIGH → LIMITE ALCANZADO" : "LOW  → normal");
    Serial.print("   |   B (DI2/GPIO17): ");
    Serial.println(b ? "HIGH → LIMITE ALCANZADO" : "LOW  → normal");
    lastA = a;
    lastB = b;
  }

  delay(20);
}
