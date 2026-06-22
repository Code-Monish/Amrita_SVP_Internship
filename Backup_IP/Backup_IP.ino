// =================================================================
// --- ULTRA-FAST RECOVERY HIGH-FREQUENCY PENDULUM CORE -----------
// =================================================================

#define ENCODER_PIN_A 2  
#define ENCODER_PIN_B 3  

#define STEP_PIN 4       
#define DIR_PIN 7        
#define BLINK_PIN 13     

void updateEncoder();

// --- PID TUNING PARAMETERS (Optimized for Instant Velocity Injection) ---
float Kp = 65.0;   // REDUCED: Keep this low initially (60-80) because the new motor profile injects massive speed automatically
float Ki = 0.00;   
float Kd = 18.0;   // Damping adjusted to match lower microstepping tracking

const long GRAVITY_OFFSET = 4; 

// --- POSITIONING & WORKSPACE BOUNDARIES ---
const long MAX_TRACK_STEPS = 5905; 
const float POSITION_GAIN_KP = 0.002; 
const long ANGLE_45_COUNTS = 300;               

volatile int lastEncoded = 0;
volatile long encoderValue = 0; 

long lastCounts = 0;
unsigned long lastTime = 0;
double integralError = 0;
long cartStepPosition = 0; 

bool ledState = false;
long lastBoundaryCrossed = 0;

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
  
  Serial.println("Calibrating absolute gravity vector... keep rod dead still.");
  delay(2000); 
  
  noInterrupts();
  encoderValue = 1200; 
  interrupts();
  
  lastTime = micros();
  Serial.println("--- Symmetrical Instant-Acceleration Loop Active ---");
}

void loop() {
  unsigned long currentTime = micros();
  double dt = (double)(currentTime - lastTime) / 1000000.0; 
  if (dt <= 0.0) dt = 0.001; 
  lastTime = currentTime;

  noInterrupts();
  long currentCounts = encoderValue;
  interrupts();

  long normalizedUprightAngle = currentCounts % 2400;
  if (normalizedUprightAngle > 1200)  normalizedUprightAngle -= 2400;
  if (normalizedUprightAngle < -1200) normalizedUprightAngle += 2400;

  normalizedUprightAngle += GRAVITY_OFFSET;

  if (abs(normalizedUprightAngle) > 230) {
    digitalWrite(STEP_PIN, LOW);
    integralError = 0; 
    lastCounts = normalizedUprightAngle;
    return; 
  }

  double positionVirtualBias = (double)cartStepPosition * POSITION_GAIN_KP;
  double error = (0.0 - positionVirtualBias) - (double)normalizedUprightAngle; 
  
  double derivativeError = (double)(normalizedUprightAngle - lastCounts) / dt;
  lastCounts = normalizedUprightAngle;

  // Raw combined control force
  double pidOutput = (Kp * error) - (Kd * derivativeError);

  executeVariableMotorSpeed(pidOutput);
}

void executeVariableMotorSpeed(double targetOutputSpeed) {
  if (abs(targetOutputSpeed) < 4) {
    digitalWrite(STEP_PIN, LOW); 
    return; 
  }

  bool currentDirection = HIGH;
  if (targetOutputSpeed > 0) {
    currentDirection = HIGH; 
  } else {
    currentDirection = LOW;  
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

  // =================================================================
  // PARABOLIC SMOOTH-CENTER / ROCKET-EDGE ENGINE
  // =================================================================
  // 1. Normalize the speed command against a realistic maximum scaling factor (e.g., 2500)
  double speedFraction = targetOutputSpeed / 2500.0;
  if (speedFraction > 1.0) speedFraction = 1.0; // Prevent over-scaling

  // 2. Square the fraction to create a soft flat zone at the center
  double parabolicFactor = speedFraction * speedFraction;

  // 3. Map to your physical microsecond limits. 
  // Base delay is a calm 130us. The max possible subtraction is 85us (130 - 85 = 45us peak sprint).
  long pulseDelay = 130 - (parabolicFactor * 85.0);          

  // Hard safety limit clamp
  pulseDelay = constrain(pulseDelay, 45, 130); 

  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(pulseDelay);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(pulseDelay);

  if (currentDirection == HIGH) {
    cartStepPosition++; 
  } else {
    cartStepPosition--; 
  }
}

void updateEncoder() {
  int MSB = digitalRead(ENCODER_PIN_A); 
  int LSB = digitalRead(ENCODER_PIN_B); 
  int encoded = (MSB << 1) | LSB; 
  int sum = (lastEncoded << 2) | encoded; 
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue--;
  lastEncoded = encoded; 
}