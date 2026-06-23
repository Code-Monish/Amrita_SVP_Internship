// =================================================================
// --- MASTER ENGINE: HARDCODED 53300 US STROBE LOCK ---------------
// =================================================================

#define PUL_PIN_1   4       
#define DIR_PIN_1   7       

#define PUL_PIN_2   5       
#define DIR_PIN_2   8       

#define PHASE_SWITCH 2      
#define STROBE_PIN   11     

// --- BASELINE SPEED TIMING (300 RPM) ---
const unsigned long BASELINE_PERIOD_US = 62; 

// --- YOUR VERIFIED SWEET SPOT LOCKED IN ---
const unsigned long strobeIntervalUs = 54200; 
const int STROBE_FLASH_WIDTH_US = 200; 

// --- INDEPENDENT MOTOR CLOCKS ---
unsigned long lastStepTimeMotor1 = 0;
unsigned long lastStepTimeMotor2 = 0;
unsigned long lastStrobeTimeUs = 0;
unsigned long strobeFlashEndUs = 0;

bool stepState1 = false;
bool stepState2 = false;
bool isStrobeActive = false;

void setup() {
  pinMode(PUL_PIN_1, OUTPUT); pinMode(DIR_PIN_1, OUTPUT);
  pinMode(PUL_PIN_2, OUTPUT); pinMode(DIR_PIN_2, OUTPUT);
  pinMode(STROBE_PIN, OUTPUT);
  
  pinMode(PHASE_SWITCH, INPUT_PULLUP); 
  
  digitalWrite(DIR_PIN_1, HIGH);
  digitalWrite(DIR_PIN_2, HIGH);
  
  Serial.begin(115200);
  Serial.println("--- Phase Engine: Hardcoded 53300us Lock Ready ---");
  Serial.println("Motor 2 = Rock Solid Baseline reference.");
  Serial.println("Toggle Switch = Crawl Motor 1 into mesh phase.");
}

void loop() {
  unsigned long currentMicros = micros();

  // =================================================================
  // ENGINE 1: MOTOR 2 (RMCS-1020) - ABSOLUTE CONSTANT
  // =================================================================
  if (currentMicros - lastStepTimeMotor2 >= BASELINE_PERIOD_US) {
    stepState2 = !stepState2;
    digitalWrite(PUL_PIN_2, stepState2);
    lastStepTimeMotor2 = currentMicros;
  }

  // =================================================================
  // ENGINE 2: MOTOR 1 - DRIFT CRAWLER
  // =================================================================
  unsigned long motor1Period = BASELINE_PERIOD_US;
  if (digitalRead(PHASE_SWITCH) == LOW) {
    motor1Period = BASELINE_PERIOD_US + 4; // Alters Motor 1 speed only
  }

  if (currentMicros - lastStepTimeMotor1 >= motor1Period) {
    stepState1 = !stepState1;
    digitalWrite(PUL_PIN_1, stepState1);
    lastStepTimeMotor1 = currentMicros;
  }

  // =================================================================
  // ENGINE 3: DEDICATED OPTICAL STROBE (53300 us)
  // =================================================================
  if (!isStrobeActive && (currentMicros - lastStrobeTimeUs >= strobeIntervalUs)) {
    digitalWrite(STROBE_PIN, HIGH);
    lastStrobeTimeUs = currentMicros;
    strobeFlashEndUs = currentMicros + STROBE_FLASH_WIDTH_US;
    isStrobeActive = true;
  }

  if (isStrobeActive && (currentMicros >= strobeFlashEndUs)) {
    digitalWrite(STROBE_PIN, LOW);
    isStrobeActive = false;
  }

  unsigned long actualStepInterval = currentMicros - lastStepTimeMotor2;

  if (actualStepInterval >= BASELINE_PERIOD_US) {
      stepState2 = !stepState2;
      digitalWrite(PUL_PIN_2, stepState2);

      // Calculate the exact delay overshoot for this step
      unsigned long softwareOvershoot = actualStepInterval - BASELINE_PERIOD_US;

      // Print it out every few thousand steps so it doesn't lag the loop
      static int count = 0;
      if (count++ > 2000) {
         Serial.print("Software Latency per loop: "); 
         Serial.print(softwareOvershoot); 
         Serial.println(" us");
         count = 0;
      }

      lastStepTimeMotor2 = currentMicros;
  }
  // Serial.println("Loop cycled!");
  // Serial.println(currentMicros);
}