// ═══════════════════════════════════════════════════════════
//  Test Entradas Analógicas — Raspberry Pi Pico 2
//  Divisor 17K/33K → factor 0.66 → rango entrada 0-5V
// ═══════════════════════════════════════════════════════════
//
//  Pines ADC:
//    GP26 (ADC0) → AIN3
//    GP27 (ADC1) → AIN1
//    GP28 (ADC2) → AIN2
//
//  Para probar:
//    - Conecta AIN a GND     → debe leer ~0.00 V
//    - Conecta AIN a 3.3V    → debe leer ~3.30 V entrada
//    - Conecta AIN a VSYS 5V → debe leer ~5.00 V entrada
// ═══════════════════════════════════════════════════════════

#define PIN_AIN1  27   // GP27 → ADC1
#define PIN_AIN2  28   // GP28 → ADC2
#define PIN_AIN3  26   // GP26 → ADC0

// Divisor de tensión: 17K serie + 33K a GND → ratio 33/50
#define DIVIDER_RATIO  (33.0f / 50.0f)   // 0.66
#define ADC_REF        3.3f
#define ADC_MAX        4095.0f

float adcToInputVoltage(int raw) {
  float v_adc = raw * ADC_REF / ADC_MAX;
  return v_adc / DIVIDER_RATIO;
}

void setup() {
  Serial.begin(115200);
  { unsigned long t0 = millis(); while (!Serial && millis() - t0 < 3000) delay(10); }

  analogReadResolution(12);  // 12 bits: 0-4095

  Serial.println("# ═══════════════════════════════════════");
  Serial.println("#  Test Analogicas — Pico 2");
  Serial.println("#  Lectura cada 500ms");
  Serial.println("#  Conecta AIN a GND, 3.3V o 5V para verificar");
  Serial.println("# ═══════════════════════════════════════");
  Serial.println("# Canal  |  Raw  |  V_ADC  |  V_Entrada");
  Serial.println("# -----  |  ---  |  -----  |  ---------");
}

void loop() {
  int raw1 = analogRead(PIN_AIN1);
  int raw2 = analogRead(PIN_AIN2);
  int raw3 = analogRead(PIN_AIN3);

  Serial.printf("  AIN1   |  %4d  |  %.3f V  |  %.3f V\n",
    raw1, raw1 * ADC_REF / ADC_MAX, adcToInputVoltage(raw1));
  Serial.printf("  AIN2   |  %4d  |  %.3f V  |  %.3f V\n",
    raw2, raw2 * ADC_REF / ADC_MAX, adcToInputVoltage(raw2));
  Serial.printf("  AIN3   |  %4d  |  %.3f V  |  %.3f V\n",
    raw3, raw3 * ADC_REF / ADC_MAX, adcToInputVoltage(raw3));
  Serial.println("# -----  |  ---  |  -----  |  ---------");

  delay(500);
}
