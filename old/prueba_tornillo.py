#include <ModbusMaster.h>

// --- CONFIGURACIÓN DE PINES ---
#define TX_GPIO 43    
#define RX_GPIO 44    
#define EN_GPIO D3    // Control de dirección RS485 (DE/RE)

ModbusMaster servo;

enum State { SCANNING, CONFIGURING, POLLING };
State currentState = SCANNING;

uint8_t currentID = 1;
unsigned long lastActionTime = 0;

// Callbacks para control del MAX485
void preTransmission()  { digitalWrite(EN_GPIO, HIGH); } 
void postTransmission() { digitalWrite(EN_GPIO, LOW);  } 

void setup() {
  Serial.begin(115200);
  
  pinMode(EN_GPIO, OUTPUT);
  digitalWrite(EN_GPIO, LOW); 

  // Inicialización de Serial para Modbus
  Serial0.begin(115200, SERIAL_8N1, RX_GPIO, TX_GPIO);
  Serial0.setTimeout(100);

  servo.begin(currentID, Serial0);
  servo.preTransmission(preTransmission);
  servo.postTransmission(postTransmission);

  Serial.println("Iniciando escaneo de Servo...");
}

void loop() {
  unsigned long currentMillis = millis();

  switch (currentState) {
    case SCANNING:
      if (currentMillis - lastActionTime >= 20) {
        runScanStep();
        lastActionTime = currentMillis;
      }
      break;

    case CONFIGURING:
      if (configureServoTorque(currentID)) {
        Serial.printf("\n[EXITO] ID %d configurado: Torque 20%%, Límite 100 RPM.\n", currentID);
        currentState = POLLING;
      } else {
        Serial.printf("\n[ERROR CONFIG] ID %d falló. Reintentando escaneo...\n", currentID);
        currentID++;
        currentState = SCANNING;
      }
      break;

    case POLLING:
      if (currentMillis - lastActionTime >= 100) {
        readMonitoring();
        lastActionTime = currentMillis;
      }
      break;
  }
}

void runScanStep() {
  servo.begin(currentID, Serial0);
  // Intentamos leer el registro 0 (Modo de operación)
  uint8_t result = servo.readHoldingRegisters(0, 1);
  if (result == servo.ku8MBSuccess) {
    Serial.printf("\n[ENCONTRADO] Dispositivo en ID: %d\n", currentID);
    currentState = CONFIGURING;
  } else {
    if (currentID % 10 == 0) Serial.print(".");
    currentID++;
    if (currentID > 247) currentID = 1;
  }
}

bool configureServoTorque(uint8_t id) {
  servo.begin(id, Serial0);

  // 1. Modo de Control (C00.00 = 2 para Torque)
  if (servo.writeSingleRegister(0, 2) != 0) return false;
  delay(10);

  // 2. Fuente de Torque (C03.40 = 0 para usar registro interno C03.41)
  if (servo.writeSingleRegister(832, 0) != 0) return false;

  // 3. Valor de Torque (C03.41 = 200 para 20.0%)
  // Nota: Muchos drives usan un decimal (200 = 20.0%). Si el tuyo es entero, usa 20.
  if (servo.writeSingleRegister(833, 200) != 0) return false;

  // 4. Límite de Velocidad Forward (C03.47 = 100 RPM)
  if (servo.writeSingleRegister(839, 100) != 0) return false;

  // 5. Límite de Velocidad Reverse (C03.48 = 100 RPM)
  if (servo.writeSingleRegister(840, 100) != 0) return false;

  // 6. Activar Servo (Servo ON - C04.11 = 1)
  if (servo.writeSingleRegister(1041, 1) != 0) return false;
  
  return true;
}

void readMonitoring() {
  int16_t rpm = 0;
  int16_t torqueRealRaw = 0;
  
  // Leer Velocidad Actual (Registro 0x4001 / 16385)
  if (servo.readHoldingRegisters(16385, 1) == servo.ku8MBSuccess) {
    rpm = (int16_t)servo.getResponseBuffer(0);
  }

  // Leer Torque Actual (Registro 0x4003 / 16387)
  if (servo.readHoldingRegisters(16387, 1) == servo.ku8MBSuccess) {
    torqueRealRaw = (int16_t)servo.getResponseBuffer(0);
  }

  Serial.printf("MODO TORQUE | Torque: %.1f%% | Velocidad: %d RPM    \r", (float)torqueRealRaw/10.0, rpm);
}