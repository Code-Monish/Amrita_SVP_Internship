// =================================================================
// --- BARE-METAL STROBOSCOPE: SLOWER 300 RPM TUNING ENGINE -------
// =================================================================

#define PUL_PIN 4       
#define DIR_PIN 7       
#define STROBE_PIN 11   

// --- SLOWER MOTOR CONFIGURATION (300 RPM) ---
// 300 RPM * 1600 steps/rev / 60s = 8000 steps per second.
// 1,000,000us / 8000 steps = 125us total cycle time. Half period = ~62us.
const unsigned long STEP_HALF_PERIOD_US = 62; 

// --- LIVE TUNABLE STROBE PARAMETERS ---
// Start at your mathematical 300 RPM target: 50,000 microseconds (20 Hz)
unsigned long strobeIntervalUs = 50000; 
const int STROBE_FLASH_WIDTH_US = 200; // Brighter flash for slower speed

// --- TIMING STOPWATCHES ---
unsigned long lastStepTimeUs = 0;
unsigned long lastStrobeTimeUs = 0;
unsigned long strobeFlashEndUs = 0;

bool stepPinState = false;
bool isStrobeActive = false;

void setup() {
  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STROBE_PIN, OUTPUT);
  
  digitalWrite(PUL_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH); 
  digitalWrite(STROBE_PIN, LOW);
  
  Serial.begin(115200);
  Serial.println("=================================================");
  Serial.println("--- Slower 300 RPM Tuning Engine Operational ---");
  Serial.print("Target Motor Speed: 300 RPM Locked\n");
  Serial.print("Initial Strobe Interval: "); Serial.print(strobeIntervalUs); Serial.println(" us");
  Serial.println("Type a microsecond value (e.g., 50000) to tune live!");
  Serial.println("=================================================");
}

void loop() {
  unsigned long currentMicros = micros();

  // =================================================================
  // 1. LIVE SERIAL MONITOR INPUT INTERFACE
  // =================================================================
  if (Serial.available() > 0) {
    long incomingValue = Serial.parseInt();
    
    // Safety boundaries for the 300 RPM range
    if (incomingValue >= 10000 && incomingValue <= 500000) {
      strobeIntervalUs = incomingValue;
      Serial.print("► Strobe Interval updated to: ");
      Serial.print(strobeIntervalUs);
      Serial.print(" us (~");
      Serial.print(1000000.0 / strobeIntervalUs, 1);
      Serial.println(" Hz)");
    }
  }

  // =================================================================
  // 2. CONSTANT 300 RPM MOTOR DRIVE
  // =================================================================
  if (currentMicros - lastStepTimeUs >= STEP_HALF_PERIOD_US) {
    stepPinState = !stepPinState;
    digitalWrite(PUL_PIN, stepPinState);
    lastStepTimeUs = currentMicros;
  }

  // =================================================================
  // 3. INDEPENDENT OPTICAL STROBE
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
}