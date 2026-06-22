// =================================================================
// --- ULTRA-FAST RECOVERY HIGH-FREQUENCY PENDULUM CORE -----------
// =================================================================

#define ENCODER_PIN_A 2  
#define ENCODER_PIN_B 3  

#define STEP_PIN 4       
#define DIR_PIN 7        
#define BLINK_PIN 13     

void updateEncoder();
void executeVariableMotorSpeed(double targetOutputSpeed);

// --- PID TUNING PARAMETERS (restored to original working values) ---
float Kp = 30.0;
float Kd = 50.0;

// --- GRAVITY OFFSET (auto-calibrated at startup from dead bottom) ---
long gravityOffset = 0;

// --- POSITIONING & WORKSPACE BOUNDARIES ---
const long MAX_TRACK_STEPS = 5905; 
const float POSITION_GAIN_KP = 0.015;  // restored to original
const long ANGLE_45_COUNTS = 300;

// --- ENCODER STATE ---
volatile int lastEncoded = 0;
volatile long encoderValue = 0; 

// --- CONTROL STATE ---
long lastCounts = 0;
unsigned long lastTime = 0;
long cartStepPosition = 0; 

// --- DERIVATIVE FILTER (only new addition to fix vibration) ---
double filteredDerivative = 0.0;
const double ALPHA = 0.10;  // light filter — preserves original Kd response

// --- POSITION INTEGRAL (for drift correction) ---
double posIntegral = 0;

// ---------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BLINK_PIN, OUTPUT);
  
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);
  
  Serial.println("Calibrating... Let pendulum hang DEAD STILL at the BOTTOM.");
  delay(2000);
  
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    noInterrupts();
    sum += encoderValue;
    interrupts();
    delay(4);
  }

  long hangingAverage = (sum / 500) % 2400;
  if (hangingAverage > 1200)  hangingAverage -= 2400;
  if (hangingAverage < -1200) hangingAverage += 2400;

  gravityOffset = hangingAverage + 1200;
  if (gravityOffset > 1200)  gravityOffset -= 2400;
  if (gravityOffset < -1200) gravityOffset += 2400;

  Serial.print("Bottom reference: "); Serial.println(hangingAverage);
  Serial.print("Top target offset: "); Serial.println(gravityOffset);
  Serial.println("--- Lift rod to engage balance loop ---");

  lastTime = micros();
}

// ---------------------------------------------------------------

void loop() {
  unsigned long currentTime = micros();
  double dt = (double)(currentTime - lastTime) / 1000000.0;
  if (dt <= 0.0) dt = 0.000001;
  lastTime = currentTime;

  noInterrupts();
  long currentCounts = encoderValue;
  interrupts();

  long normalizedUprightAngle = currentCounts % 2400;
  if (normalizedUprightAngle > 1200)  normalizedUprightAngle -= 2400;
  if (normalizedUprightAngle < -1200) normalizedUprightAngle += 2400;

  normalizedUprightAngle += gravityOffset;

  if (normalizedUprightAngle > 1200)  normalizedUprightAngle -= 2400;
  if (normalizedUprightAngle < -1200) normalizedUprightAngle += 2400;

  // --- FALL DETECTION ---
  if (abs(normalizedUprightAngle) > ANGLE_45_COUNTS) {
    digitalWrite(STEP_PIN, LOW);
    filteredDerivative = 0.0;
    posIntegral = 0.0;
    lastCounts = normalizedUprightAngle;
    return;
  }

  // --- POSITION CENTERING BIAS (restored original + light integral) ---
  posIntegral += cartStepPosition * dt;
  posIntegral = constrain(posIntegral, -5000, 5000);
  double positionVirtualBias = (cartStepPosition * POSITION_GAIN_KP) + (0.002 * posIntegral);

  // --- PID ERROR ---
  double error = (0.0 - positionVirtualBias) - (double)normalizedUprightAngle;

  // --- FILTERED DERIVATIVE (replaces raw derivative — fixes vibration) ---
  double rawDerivative = (double)(normalizedUprightAngle - lastCounts) / dt;
  filteredDerivative = (ALPHA * rawDerivative) + ((1.0 - ALPHA) * filteredDerivative);
  lastCounts = normalizedUprightAngle;

  // --- PID OUTPUT ---
  double pidOutput = -(Kp * error) - (Kd * filteredDerivative);
  
  Serial.print("angle="); Serial.print(normalizedUprightAngle);
  Serial.print(" error="); Serial.print(error);
  Serial.print(" pid="); Serial.println(pidOutput);

  executeVariableMotorSpeed(pidOutput);
}

// ---------------------------------------------------------------

void executeVariableMotorSpeed(double targetOutputSpeed) {
  if (abs(targetOutputSpeed) < 4) {
    digitalWrite(STEP_PIN, LOW); 
    return; 
  }

  bool currentDirection;
  if (targetOutputSpeed > 0) {
    currentDirection = LOW;   // corrected direction
  } else {
    currentDirection = HIGH;
    targetOutputSpeed = abs(targetOutputSpeed);
  }

  if (currentDirection == HIGH && cartStepPosition >= MAX_TRACK_STEPS) {
    digitalWrite(STEP_PIN, LOW);
    return;
  }
  if (currentDirection == LOW && cartStepPosition <= -MAX_TRACK_STEPS) {
    digitalWrite(STEP_PIN, LOW);
    return;
  }

  digitalWrite(DIR_PIN, currentDirection);

  // --- ORIGINAL PARABOLIC SPEED PROFILE (restored) ---
  double speedFraction = targetOutputSpeed / 2500.0;
  if (speedFraction > 1.0) speedFraction = 1.0;

  double parabolicFactor = speedFraction * speedFraction;

  // Original timing restored
  long pulseDelay = 130 - (long)(parabolicFactor * 85.0);
  pulseDelay = constrain(pulseDelay, 45, 130);

  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(pulseDelay);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(5);  // minimum LOW hold only

  if (currentDirection == HIGH) {
    cartStepPosition++;
  } else {
    cartStepPosition--;
  }
}

// ---------------------------------------------------------------

void updateEncoder() {
  int MSB = digitalRead(ENCODER_PIN_A);
  int LSB = digitalRead(ENCODER_PIN_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  // Flipped to correct encoder coordinate direction
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue++;

  lastEncoded = encoded;
}