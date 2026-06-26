// =================================================================
// --- ARDUINO A: MASTER MOTOR ENGINE (COMMON-CATHODE FIXED) -------
// =================================================================

// SWAPPED TO MATCH YOUR PHYSICAL BENCH WIRING
#define PUL_PIN_1   4       // Driver A PUL+  
#define DIR_PIN_1   7       // Driver A DIR+

#define PUL_PIN_2   5       // Driver B PUL+
#define DIR_PIN_2   8       // Driver B DIR+

#define PHASE_SWITCH 9      
#define SYNC_OUT_PIN 3      

const unsigned long BASELINE_PERIOD_US = 62; 

unsigned long lastStepTimeMotor1 = 0;
unsigned long lastStepTimeMotor2 = 0;

// Common-Cathode initialization (Idle state is LOW)
bool stepState1 = false;
bool stepState2 = false;

long stepCounterMotor2 = 0; 
bool syncPinState = false;

void setup() {
  pinMode(PUL_PIN_1, OUTPUT); pinMode(DIR_PIN_1, OUTPUT);
  pinMode(PUL_PIN_2, OUTPUT); pinMode(DIR_PIN_2, OUTPUT);
  pinMode(SYNC_OUT_PIN, OUTPUT);
  pinMode(PHASE_SWITCH, INPUT_PULLUP); 
  
  // Set default logic states LOW for common-cathode drivers
  digitalWrite(PUL_PIN_1, LOW);
  digitalWrite(DIR_PIN_1, HIGH); // Keeps direction forward
  digitalWrite(PUL_PIN_2, HIGH);
  digitalWrite(DIR_PIN_2, LOW); // Keeps direction reverse
}

unsigned long currentMicros = micros();

void loop() {
  currentMicros = micros();
  // ENGINE 1: MOTOR 2 (RMCS-1020 Reference)
  if (currentMicros - lastStepTimeMotor2 >= BASELINE_PERIOD_US) {
    stepState2 = !stepState2; 
    digitalWrite(PUL_PIN_2, stepState2); 
    
    // Check pulse trigger on the rising transition edge (LOW to HIGH)
    if (stepState2 == HIGH) {
      stepCounterMotor2++;
      if (stepCounterMotor2 >= 400) {
        syncPinState = !syncPinState; 
        digitalWrite(SYNC_OUT_PIN, syncPinState); 
        stepCounterMotor2 = 0; 
      }
    }
    lastStepTimeMotor2 = currentMicros;
  }

  // ENGINE 2: MOTOR 1 - DRIFT CRAWLER
  unsigned long motor1Period = BASELINE_PERIOD_US;
  if (digitalRead(PHASE_SWITCH) == LOW) {
    motor1Period = BASELINE_PERIOD_US + 4; 
  }

  if (currentMicros - lastStepTimeMotor1 >= motor1Period) {
    stepState1 = !stepState1;
    digitalWrite(PUL_PIN_1, stepState1);
    lastStepTimeMotor1 = currentMicros;
  }
}