#include <ModbusMaster.h>

// --- CONFIGURATION ---
#define TX_GPIO 43    // Physical D6
#define RX_GPIO 44    // Physical D7
#define EN_GPIO D3    // Direction Control (DE/RE)

// Set this to true for Torque Mode, false for Speed Mode
#define TARGET_MODE_TORQUE true 

ModbusMaster servo;

enum State { SCANNING, CONFIGURING, POLLING };
State currentState = SCANNING;

uint8_t currentID = 1;
unsigned long lastActionTime = 0;

// Direction Control Callbacks
void preTransmission()  { digitalWrite(EN_GPIO, HIGH); } 
void postTransmission() { digitalWrite(EN_GPIO, LOW);  } 

void setup() {
  Serial.begin(115200);
  
  pinMode(EN_GPIO, OUTPUT);
  digitalWrite(EN_GPIO, LOW); 

  // Physical Verification Test for D3
  Serial.println("[TEST] Forcing D3 HIGH for 3s... Verify with Multimeter/LED.");
  digitalWrite(EN_GPIO, HIGH);
  delay(3000);
  digitalWrite(EN_GPIO, LOW);
  Serial.println("[TEST] Done. Starting Modbus at 115200.");

  // Initialize Hardware Serial0 (Modbus)
  Serial0.begin(115200, SERIAL_8N1, RX_GPIO, TX_GPIO);
  Serial0.setTimeout(100);

  servo.begin(currentID, Serial0);
  servo.preTransmission(preTransmission);
  servo.postTransmission(postTransmission);
}

void loop() {
  unsigned long currentMillis = millis();

  switch (currentState) {
    case SCANNING:
      if (currentMillis - lastActionTime >= 15) {
        runScanStep();
        lastActionTime = currentMillis;
      }
      break;

    case CONFIGURING:
      if (configureServo(currentID)) {
        Serial.printf("\n[SUCCESS] ID %d configured in %s mode.\n", 
                      currentID, TARGET_MODE_TORQUE ? "TORQUE" : "SPEED");
        currentState = POLLING;
      } else {
        Serial.printf("\n[CONFIG FAIL] ID %d failed. Moving on...\n", currentID);
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
  uint8_t result = servo.readHoldingRegisters(0, 1);
  if (result == servo.ku8MBSuccess) {
    Serial.printf("\n[FOUND] Device at ID: %d\n", currentID);
    currentState = CONFIGURING;
  } else {
    if (currentID % 10 == 0) Serial.print(".");
    currentID++;
    if (currentID > 247) currentID = 1;
  }
}

bool configureServo(uint8_t id) {
  servo.begin(id, Serial0);

  if (TARGET_MODE_TORQUE) {
    // --- TORQUE MODE CONFIGURATION ---
    if (servo.writeSingleRegister(0, 2) != 0) return false;      // Mode: Torque (C00.00)
    if (servo.writeSingleRegister(833, 40) != 0) return false;   // Ref Torque: 4.0% (C03.41)
    if (servo.writeSingleRegister(839, 100) != 0) return false;  // Torque Limit+ (C03.47)
    if (servo.writeSingleRegister(840, 100) != 0) return false;  // Torque Limit- (C03.48)
  } else {
    // --- SPEED MODE CONFIGURATION ---
    if (servo.writeSingleRegister(0, 1) != 0) return false;      // Mode: Speed (C00.00)
    if (servo.writeSingleRegister(4614, 100) != 0) return false; // Set Speed: 100 RPM (C12.06)
  }

  // Common: Set Servo ON (C04.11)
  if (servo.writeSingleRegister(1041, 1) != 0) return false;   
  return true;
}

void readMonitoring() {
  int16_t rpm = 0;
  float torqueReal = 0.0;
  
  // Read Speed (0x4001 / 16385)
  uint8_t res1 = servo.readHoldingRegisters(16385, 1);
  if (res1 == servo.ku8MBSuccess) {
    rpm = (int16_t)servo.getResponseBuffer(0);
  }

  // Read Actual Torque (0x4003 / 16387)
  uint8_t res2 = servo.readHoldingRegisters(16387, 1);
  if (res2 == servo.ku8MBSuccess) {
    torqueReal = (int16_t)servo.getResponseBuffer(0) / 10.0;
  }

  if (res1 == servo.ku8MBSuccess && res2 == servo.ku8MBSuccess) {
    Serial.printf("Torque Real: %.1f%% | Velocidad: %d RPM\r", torqueReal, rpm);
  } else {
    Serial.print("Comm Error...\r");
  }
}