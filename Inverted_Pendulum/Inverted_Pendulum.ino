// INVERTED PENDULUM : Balancing & Damping Module

// PINOUT
#define STEP_PIN 4
#define DIR_PIN 7
#define ENCODER_PIN_A 2  // Green wire
#define ENCODER_PIN_B 3  // White wire
#define BLINK_PIN 13

// GLOBAL VARIABLES
int systemState = 0;  // 0 Homing/Calibration, 1 Active Balancing
bool isSystemArmed = false;
long lastMicros = 0;

// position variables
const long pivot_length = 220;  // mm
const long ORIGIN_PIVOT_STEPLEN = 9.847 * pivot_length;

long currentPosition = 0;

// encoder variables
volatile int encoderTicks = 0;
volatile int lastEncoded = 0;
long lastTicks = 0;
unsigned long lastDamperMicros = 0;
unsigned long lastPrintMillis = 0;
float angularVelocity = 0;
signed long angleError = 0;

// LED variables
const long ANGLE_45_COUNTS = 300;  // 45 degrees in encoder ticks
long currentSector = 1;
long currentCounts = 0;
long lastBoundaryCrossed = 0;
bool ledState = false;

// damper variables
float timeDiff = 0;
float angleDiff = 0;
long stepControlDelay = 0;
unsigned long maxSpeedDelay = 80;    // Calibrated for 800 p/r
unsigned long minSpeedDelay = 1000;  // Snappy baseline crawl
float rawVelocity = 0;

// Upright Target definition
const long UPRIGHT_TARGET_TICKS = 1200;  // Half rotation on an 800 p/r sequence

const float Kp = 2.1;
const float Ki = 0;
const float Kd = 0.4;

void setup() {
  Serial.begin(115200);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BLINK_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(BLINK_PIN, LOW);

  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);
}

void loop() {
  noInterrupts();
  currentCounts = encoderTicks;
  interrupts();

  // --- LED Feedback Logic Retained ---
  currentSector = abs(currentCounts) / ANGLE_45_COUNTS;
  if (currentSector != lastBoundaryCrossed) {
    ledState = !ledState;
    digitalWrite(BLINK_PIN, ledState);
    lastBoundaryCrossed = currentSector;
  }

  unsigned long currentMicros = micros();
  unsigned long currentMillis = millis();

  // Homing logic (Calibrates dead bottom)
  if (!isSystemArmed) {
    if (encoderTicks <= -600) {
      delay(5000);  // 5-second window to let it settle dead still at bottom

      noInterrupts();
      encoderTicks = 0;  // Force-reset raw encoder values to 0 at true dead bottom
      interrupts();

      currentPosition = 0;
      lastMicros = 0;
      isSystemArmed = true;
      systemState = 1;
      Serial.println("System Calibrated! Lift rod to top vertical to engage control.");
    }
  } else {
      long deviationFromTop = abs(currentCounts) - UPRIGHT_TARGET_TICKS;
    if (systemState == 1) {
      digitalWrite(STEP_PIN, LOW);
      lastTicks = currentCounts;

      if (abs(deviationFromTop) < 10){
        systemState = 2;
      }

    }
    else if (systemState == 2){

      if (abs(deviationFromTop) > ANGLE_45_COUNTS) {
        digitalWrite(STEP_PIN, LOW);
        systemState = 1;
        return;
      } else {
        if (currentMicros - lastDamperMicros >= 10000) {
          damper(currentMicros, deviationFromTop);
        }

        if (currentMicros - lastMicros > stepControlDelay) {
          digitalWrite(STEP_PIN, HIGH);
          delayMicroseconds(2);
          digitalWrite(STEP_PIN, LOW);
          lastMicros = currentMicros;
        }

        if (currentMillis - lastPrintMillis >= 100) {
          Serial.print("Error: ");
          Serial.print(angleError);
          Serial.print(" | Velocity: ");
          Serial.println(angularVelocity);
          lastPrintMillis = currentMillis;
        }
      }
    }
  }
}

void damper(unsigned long currentMicros, long targetError) {
  noInterrupts();
  long snapshotTicks = encoderTicks;
  interrupts();

  timeDiff = 10.0;

  // Compute error using the normalized deflection passed from loop
  angleError = targetError;
  angleDiff = (snapshotTicks - lastTicks) * 0.15;
  rawVelocity = (angleDiff * 1000.0) / timeDiff;
  angularVelocity = (0.15 * rawVelocity) + (0.85 * angularVelocity);

  // Deadband check
  if (abs(angleError) <= 4) {
    stepControlDelay = minSpeedDelay;
    angularVelocity = 0;
    angleDiff = 0;
    lastTicks = snapshotTicks;
    return;
  }

  float proportionalControlEffort = abs(angleError) * Kp;
  float derivativeControlEffort = abs(angularVelocity) * Kd;
  float totalControlEffort = proportionalControlEffort + derivativeControlEffort;

  stepControlDelay = constrain(minSpeedDelay - totalControlEffort, maxSpeedDelay, minSpeedDelay);

  lastTicks = snapshotTicks;
  lastDamperMicros = currentMicros;

  if (angleError < 0) {
    digitalWrite(DIR_PIN, HIGH);
  } else {
    digitalWrite(DIR_PIN, LOW);
  }
}

// Encoder readings [Refrence : https://www.rcscomponents.kiev.ua/datasheets/lpd-ab-english2020.pdf?srsltid=AfmBOoqEVmGO062hPS_RaB5_w2TkuZ-YwevjQDEOlr1-3n7S0Rw8zgBE]
void updateEncoder() {
  int MSB = (PIND & (1 << 2)) >> 2;
  int LSB = (PIND & (1 << 3)) >> 3;
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderTicks++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderTicks--;
  }
  lastEncoded = encoded;
}