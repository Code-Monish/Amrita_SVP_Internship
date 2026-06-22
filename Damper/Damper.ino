// --- CONFIGURATION & PIN DEFINITIONS ---
#define ENCODER_PIN_A 2  // Green wire (Pendulum)
#define ENCODER_PIN_B 3  // White wire (Pendulum)

#define STEP_PIN 4       // PLS+ on 2HSS57
#define DIR_PIN 7        // DIR+ on 2HSS57
#define BLINK_PIN 13     // Onboard LED

// --- UPDATED TIGHTER TRACK BOUNDARY CONSTANTS ---
// 400mm total padded travel line = +/- 200mm from center
// 200mm * 19.685 steps/mm = 3937 steps max allowed displacement
const long MAX_TRACK_STEPS = 3937; 

// Inward-shifted destinations for the transit routine
const long LEFT_ORIGIN = -2500;
const long RIGHT_ORIGIN = 2500;
const long ANGLE_45_COUNTS = 300;

// --- HIGH-SPEED DAMPING & SHIFT PARAMETERS ---
const int DAMP_SPEED_DELAY = 70;    // FAST: 70us delay for explosive acceleration
const int DAMP_BURST_STEPS = 35;    // Snappy blocks for ultra-high loop refresh frequency
const int TRANSIT_SPEED_DELAY = 80;  // High-velocity cruise between origins

// --- SYSTEM STATE MACHINE ---
enum SystemState { DAMPING, TRANSITING };
SystemState currentState = DAMPING;
long targetOrigin = LEFT_ORIGIN; 

// --- STATE TRACKING VARIABLES ---
volatile int lastEncoded = 0;
volatile long pendulumCounts = 0; // 0 is dead straight down hanging vertical
long lastPendulumCounts = 0;
long cartStepPosition = 0; 

// Velocity tracking for dead-still detection
unsigned long lastVelocityCheckTime = 0;
long positionAtLastCheck = 0;

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
  
  Serial.println("--- Tight-Boundary Micro-Angle Damper Online ---");
}

void loop() {
  // 1. Snapshot encoder data safely
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

  // 2. Track rotational variables
  long deltaAngle = currentAngle - lastPendulumCounts;

  // =================================================================
  // STATE 1: ACTIVE HIGH-SPEED MICRO-ANGLE DAMPING MODE
  // =================================================================
  if (currentState == DAMPING) {
    
    // CHANGED: Core Damping Window tightened down to a narrow range of -200 to 200 (+/- 30 degrees)
    if (currentAngle >= -200 && currentAngle <= 200) {
      
      if (deltaAngle > 0 && abs(deltaAngle) > 1) {
        // Pendulum is moving Clockwise -> Move RIGHT (HIGH) to run under the swing
        executeSafeStep(HIGH, DAMP_BURST_STEPS, DAMP_SPEED_DELAY);
      } 
      else if (deltaAngle < 0 && abs(deltaAngle) > 1) {
        // Pendulum is moving Counter-Clockwise -> Move LEFT (LOW) to cushion the swing
        executeSafeStep(LOW, DAMP_BURST_STEPS, DAMP_SPEED_DELAY);
      }
    }

    // --- DEAD STILL DETECTION LOOP ---
    if (millis() - lastVelocityCheckTime > 80) {
      long angularDisplacement = abs(currentAngle - positionAtLastCheck);
      
      // Tightened stability threshold: must stay within 40 counts of absolute dead center
      if (angularDisplacement <= 2 && abs(currentAngle) < 40) {
        // Switch targets and launch to the other side
        if (targetOrigin == LEFT_ORIGIN) {
          targetOrigin = RIGHT_ORIGIN;
        } else {
          targetOrigin = LEFT_ORIGIN;
        }
        currentState = TRANSITING;
      }
      
      positionAtLastCheck = currentAngle;
      lastVelocityCheckTime = millis();
    }
  }
  // =================================================================
  // STATE 2: METRO HIGH-SPEED TRANSIT TO NEW ORIGIN
  // =================================================================
  else if (currentState == TRANSITING) {
    long distanceToTarget = targetOrigin - cartStepPosition;

    if (abs(distanceToTarget) > 30) {
      bool transitDirection = (distanceToTarget > 0) ? HIGH : LOW;
      executeSafeStep(transitDirection, 40, TRANSIT_SPEED_DELAY);
    } 
    else {
      // Destination reached! Sudden stop kicks off the new swing cycle.
      currentState = DAMPING;
      positionAtLastCheck = currentAngle;
      lastVelocityCheckTime = millis();
    }
  }

  lastPendulumCounts = currentAngle;
}

// Low-level step executor with independent absolute spatial guard rails
void executeSafeStep(bool targetDirection, int stepsToTake, int pulseDelayMicros) {
  digitalWrite(DIR_PIN, targetDirection);

  for (int i = 0; i < stepsToTake; i++) {
    // UPDATED: Rigid safety checks against the new 3937 step boundary limit
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
  
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) pendulumCounts++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) pendulumCounts--;
  
  lastEncoded = encoded; 
}