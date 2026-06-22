// --- CONFIGURATION & PIN DEFINITIONS ---
#define ENCODER_PIN_A 2  // Green wire (Interrupt 0)
#define ENCODER_PIN_B 3  // White wire (Interrupt 1)

#define STEP_PIN 4       // Connects to PLS+ on 2HSS57
#define DIR_PIN 7        // Connects to DIR+ on 2HSS57
#define BLINK_PIN 13     // Onboard Arduino LED (LED_BUILTIN)

// --- KINEMATIC & JOGGING CONSTANTS ---
const float STEPS_PER_MM = 19.6850;
const int PULSE_DELAY_MICROS = 400; // Tuning for motor velocity

// --- THRESHOLD CRITERIA ---
// 2400 counts = 360 degrees. 
// 30 degrees = (30 / 360) * 2400 = 200 counts.
const long THRESHOLD_COUNTS = 200; 
const int MOTOR_STEP_BLOCK = 50;   // How many steps to take per check loop

// --- VOLATILE TRACKING VARIABLES ---
volatile int lastEncoded = 0;
volatile long encoderValue = 0;

// --- DIAGNOSTIC LED TRACKING VARIABLES ---
bool ledState = false; 
long lastBoundaryCrossed = 0;

void setup() {
  Serial.begin(115200);
  
  // Hardware Interface Initialization
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BLINK_PIN, OUTPUT); 
  
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(BLINK_PIN, LOW); // Start with the LED turned off
  
  // Encoder Input Initialization with internal pull-ups
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  
  // Secure robust hardware-agnostic interrupt vectors
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);
  
  Serial.println("--- Joystick Pendulum System Online ---");
  Serial.println("Rotate encoder past +/- 30 degrees to drive the axis.");
}

void loop() {
  // 1. Safely snapshot the encoder value away from interrupt interference
  noInterrupts();
  long currentCounts = encoderValue;
  interrupts();
  
  // --- VISUAL DIAGNOSTIC FEATURE: TOGGLE LED AT EVERY 300-COUNT INTERVAL ($45°) ---
  long currentSector = abs(currentCounts) / 300;
  
  // If we have moved into a brand new 300-count block, flip the light state
  if (currentSector != lastBoundaryCrossed) {
    ledState = !ledState;            // Invert the state (ON -> OFF / OFF -> ON)
    digitalWrite(BLINK_PIN, ledState);
    lastBoundaryCrossed = currentSector; // Update our tracker
  }
  
  // 2. Process our threshold conditions
  if (currentCounts > THRESHOLD_COUNTS) {
    // Encoder turned positive (Clockwise) -> Move Stepper Forward
    digitalWrite(DIR_PIN, HIGH);
    
    Serial.print("Threshold Broken Right! Counts: ");
    Serial.println(currentCounts);
    
    // Execute a fast block of steps
    for (int i = 0; i < MOTOR_STEP_BLOCK; i++) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(PULSE_DELAY_MICROS);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(PULSE_DELAY_MICROS);
    }
  } 
  else if (currentCounts < -THRESHOLD_COUNTS) {
    // Encoder turned negative (Counter-Clockwise) -> Move Stepper Reverse
    digitalWrite(DIR_PIN, LOW);
    
    Serial.print("Threshold Broken Left! Counts: ");
    Serial.println(currentCounts);
    
    // Execute a fast block of steps
    for (int i = 0; i < MOTOR_STEP_BLOCK; i++) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(PULSE_DELAY_MICROS);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(PULSE_DELAY_MICROS);
    }
  }
  // If inside the deadzone (-200 to +200), the loop does absolutely nothing,
  // keeping the motor standing perfectly still.
}

// --- HIGH-SPEED QUADRATURE INTERRUPT SERVICE ROUTINE ---
void updateEncoder() {
  int MSB = digitalRead(ENCODER_PIN_A); 
  int LSB = digitalRead(ENCODER_PIN_B); 
  
  int encoded = (MSB << 1) | LSB; 
  int sum = (lastEncoded << 2) | encoded; 
  
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue--;
  
  lastEncoded = encoded; 
}