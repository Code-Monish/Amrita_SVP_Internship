// =================================================================
// --- ARDUINO A: MASTER MOTOR ENGINE WITH SYNC PULSER -------------
// =================================================================

#define PUL_PIN_1   4       
#define DIR_PIN_1   7       

#define PUL_PIN_2   5       
#define DIR_PIN_2   8       

#define PHASE_SWITCH 9      // Moved to Pin 9 to free up interrupt lines
#define SYNC_OUT_PIN 3      // Sends hardware trigger to Strobe Arduino

const unsigned long BASELINE_PERIOD_US = 187; // Hardcoded ~300 RPM Baseline

unsigned long lastStepTimeMotor1 = 0;
unsigned long lastStepTimeMotor2 = 0;

bool stepState1 = false;
bool stepState2 = false;

long stepCounterMotor2 = 0; // Tracks when a quarter-turn happens
bool syncPinState = false;

void setup() {
  pinMode(PUL_PIN_1, OUTPUT); pinMode(DIR_PIN_1, OUTPUT);
  pinMode(PUL_PIN_2, OUTPUT); pinMode(DIR_PIN_2, OUTPUT);
  pinMode(SYNC_OUT_PIN, OUTPUT);
  pinMode(PHASE_SWITCH, INPUT_PULLUP); 
  
  digitalWrite(DIR_PIN_1, HIGH);
  digitalWrite(DIR_PIN_2, HIGH);
}

void loop() {
  unsigned long currentMicros = micros();

  // ENGINE 1: MOTOR 2 (RMCS-1020) - THE STATIONARY VISUAL REFERENCE
  if (currentMicros - lastStepTimeMotor2 >= BASELINE_PERIOD_US) {
    stepState2 = !stepState2;
    digitalWrite(PUL_PIN_2, stepState2);
    
    // Every full pulse cycle is 2 state changes (HIGH then LOW)
    if (stepState2 == HIGH) {
      stepCounterMotor2++;
      
      // 1600 steps per rev / 4 blades = 400 steps per blade replacement
      if (stepCounterMotor2 >= 400) {
        syncPinState = !syncPinState; 
        digitalWrite(SYNC_OUT_PIN, syncPinState); // Send the edge signal!
        stepCounterMotor2 = 0; // Reset count
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