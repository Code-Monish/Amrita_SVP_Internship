// =================================================================
// --- ARDUINO B: STROBE SLAVE WITH MANUAL SERIAL OVERRIDE ---------
// =================================================================

#define SYNC_IN_PIN  2     // Connects to Pin 3 of Motor Arduino
#define STROBE_PIN   11    // Connects to MOSFET Gate driving 24V lamp

const int STROBE_FLASH_WIDTH_US = 200; 

// --- TIMING MECHANICS ---
volatile bool hardwareTriggered = false;
unsigned long manualStrobeIntervalUs = 52450; // 0 = Use Hardware Sync, >0 = Manual Time Mode
unsigned long lastStrobeTimeUs = 0;
unsigned long strobeFlashEndUs = 0;
bool isStrobeActive = false;

void setup() {
  pinMode(STROBE_PIN, OUTPUT);
  pinMode(SYNC_IN_PIN, INPUT);
  
  Serial.begin(115200);
  Serial.println("--- Strobe Controller: Serial Input Engine Active ---");
  Serial.println("Default: Operating in Hardware-Sync Mode.");
  Serial.println("Type a value (e.g., 49600) to enter Manual Override Mode.");
  Serial.println("Type '0' to switch back to Hardware-Sync Mode.");

  // Attach hardware interrupt to Pin 2
  attachInterrupt(digitalPinToInterrupt(SYNC_IN_PIN), triggerStrobeLamp, CHANGE);
}

void loop() {
  unsigned long currentMicros = micros();

  // =================================================================
  // 2. LIGHT TRIGGER SELECTOR
  // =================================================================
  if (manualStrobeIntervalUs == 0) {
    // --- MODE A: HARDWARE SYNC (Default) ---
    if (hardwareTriggered) {
      digitalWrite(STROBE_PIN, HIGH);
      delayMicroseconds(STROBE_FLASH_WIDTH_US);
      digitalWrite(STROBE_PIN, LOW);
      hardwareTriggered = false; // Reset flag
    }
  } 
  else {
    // --- MODE B: MANUAL TIME OVERRIDE (Your Custom Input) ---
    if (!isStrobeActive && (currentMicros - lastStrobeTimeUs >= manualStrobeIntervalUs)) {
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
}

// Interrupt Service Routine for hardware synchronization
void triggerStrobeLamp() {
  hardwareTriggered = true; 
}

