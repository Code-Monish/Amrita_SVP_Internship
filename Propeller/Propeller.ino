// --- CONFIGURATION & PIN DEFINITIONS ---
#define ENCODER_PIN_A 2  // Green wire (Pendulum)
#define ENCODER_PIN_B 3  // White wire (Pendulum)

#define STEP_PIN 4    // PLS+ on 2HSS57
#define DIR_PIN 7     // DIR+ on 2HSS57
#define BLINK_PIN 13  // Onboard LED

// --- HARDWARE COORDINATE CONSTANTS ---
const long MAX_TRACK_STEPS = 5905;
// 600mm total padded travel line max
const long ANGLE_45_COUNTS = 300;

// --- SHIMMY TIMING PARAMETERS ---
const int SHIMMY_SPEED_DELAY = 100;  // Microsecond pulse delay
const int BASE_SHIMMY_STEPS = 90;
// Nominal step burst size at absolute track center

// --- ORIGIN BIAS PENALTY GAIN ---
const float SHIMMY_BIAS_GAIN = 0.025;

// --- PENDULUM STATE TRACKING ---
volatile int lastEncoded = 0;
volatile long pendulumCounts = 0;
long lastPendulumCounts = 0;

// --- TIMING & SPEED TRACKING VARIABLES ---
unsigned long lastVelocityMicros = 0;
float angularVelocity = 0;
// Velocity threshold in degrees/second before resting the motor
const float MAX_VELOCITY_THRESHOLD = 1200.0;

// --- MOTOR SPATIAL STATE TRACKING ---
long cartStepPosition = 0;
// 0 is dead center of the rail

// --- DIAGNOSTIC LED TRACKING ---
bool ledState = false;
long lastBoundaryCrossed = 0;

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

  Serial.println("--- Origin-Biased Asymmetric Shimmy Online ---");
}

void loop() {
  unsigned long currentMicros = micros();

  // 1. Snapshot the pendulum encoder safely
  noInterrupts();
  long currentAngle = pendulumCounts;
  interrupts();

  // --- LED INDICATOR AT MULTIPLES OF 45° ---
  long currentSector = abs(currentAngle) / ANGLE_45_COUNTS;
  if (currentSector != lastBoundaryCrossed) {
    ledState = !ledState;
    digitalWrite(BLINK_PIN, ledState);
    lastBoundaryCrossed = currentSector;
  }

  // 2. Fixed 10ms Velocity Estimation Engine
  if (currentMicros - lastVelocityMicros >= 10000) {
    long deltaAngle = currentAngle - lastPendulumCounts;
    float angleDiffDegrees = deltaAngle * 0.15;  // Convert counts to degrees

    // Calculate raw degrees/second over our 10ms window
    float rawVelocity = (angleDiffDegrees * 1000.0) / 10.0;
    // Applied low-pass filter to reject structural chattering
    angularVelocity = (0.20 * rawVelocity) + (0.80 * angularVelocity);

    lastPendulumCounts = currentAngle;
    lastVelocityMicros = currentMicros;
  }

  long normalizedAngle = currentAngle % 2400;
  long dynamicPenalty = (long)(cartStepPosition * SHIMMY_BIAS_GAIN);

  if (abs(angularVelocity) > MAX_VELOCITY_THRESHOLD) {
    digitalWrite(STEP_PIN, LOW);
    return;
  }


  // 3. ASYMMETRIC SHIMMY EXECUTION WITH PENALTY BIAS
  long deltaAngle = currentAngle - lastPendulumCounts;  // Retained direction tracking

  if (deltaAngle < 0) {  // Confirmed CCW Motion
    if (normalizedAngle < 0 && normalizedAngle >= -600) {
      int rightSteps = BASE_SHIMMY_STEPS - dynamicPenalty;
      rightSteps = constrain(rightSteps, 30, 150);
      executeSafeStep(HIGH, rightSteps, SHIMMY_SPEED_DELAY);
    } else if (normalizedAngle < -600 && normalizedAngle >= -1200) {
      int leftSteps = BASE_SHIMMY_STEPS + dynamicPenalty;
      leftSteps = constrain(leftSteps, 30, 150);
      executeSafeStep(LOW, leftSteps, SHIMMY_SPEED_DELAY);
    }
  } else if (deltaAngle > 0) {  // Confirmed CW Motion
    if (normalizedAngle > 0 && normalizedAngle <= 600) {
      int leftSteps = BASE_SHIMMY_STEPS - dynamicPenalty;
      leftSteps = constrain(leftSteps, 30, 150);
      executeSafeStep(LOW, leftSteps, SHIMMY_SPEED_DELAY);
    } else if (normalizedAngle > 600 && normalizedAngle <= 1200) {
      int rightSteps = BASE_SHIMMY_STEPS + dynamicPenalty;
      rightSteps = constrain(rightSteps, 30, 150);
      executeSafeStep(HIGH, rightSteps, SHIMMY_SPEED_DELAY);
    }
  }

  delay(1);
}

// Low-level step executor with absolute independent spatial guard rails
void executeSafeStep(bool targetDirection, int stepsToTake, int pulseDelayMicros) {
  digitalWrite(DIR_PIN, targetDirection);
  for (int i = 0; i < stepsToTake; i++) {
    if (targetDirection == HIGH && cartStepPosition >= MAX_TRACK_STEPS) {
      digitalWrite(STEP_PIN, LOW);
      return;
    }
    if (targetDirection == LOW && cartStepPosition <= -MAX_TRACK_STEPS) {
      digitalWrite(STEP_PIN, LOW);
      return;
    }

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(pulseDelayMicros);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(pulseDelayMicros);

    if (targetDirection == HIGH) {
      cartStepPosition++;
    } else {
      cartStepPosition--;
    }
  }
}

// --- HIGH-SPEED QUADRATURE INTERRUPT SERVICE ROUTINE ---
void updateEncoder() {
  int MSB = digitalRead(ENCODER_PIN_A);
  int LSB = digitalRead(ENCODER_PIN_B);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) pendulumCounts++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) pendulumCounts--;

  lastEncoded = encoded;
}