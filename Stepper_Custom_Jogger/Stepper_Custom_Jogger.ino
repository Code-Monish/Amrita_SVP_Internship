// --- KINEMATIC & HARDWARE CONSTANTS ---
const float STEPS_PER_MM = 19.6850; // Your verified 16-tooth XL pulley constant

#define STEP_PIN 4  // Connects to PLS+ on 2HSS57
#define DIR_PIN 7   // Connects to DIR+ on 2HSS57

// Tuning parameters for motor speed and smoothness
const int PULSE_DELAY_MICROS = 400; // Time between step transitions (Lower = Faster rotation)

void setup() {
  Serial.begin(115200); // Fast baud rate for industrial debugging
  
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  // Initialize pins to safe low states
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  
  Serial.println("--- Industrial CNC Jogging Terminal Active ---");
  Serial.println("Enter target distance in mm (e.g., '10', '-15.5', '5') and hit Send:");
}

void loop() {
  // Check if user has typed a command into the Serial Monitor
  if (Serial.available() > 0) {
    // Read the incoming input directly as a floating-point number
    float targetDistanceMm = Serial.parseFloat();
    
    // Skip ghost values or accidental newline characters (\n)
    if (targetDistanceMm == 0.00) return; 
    
    Serial.print("\nCommand Received: ");
    Serial.print(targetDistanceMm);
    Serial.println(" mm");
    
    // Execute the linear movement
    jogAxis(targetDistanceMm);
  }
}

// Core motion-control interpolation routine
void jogAxis(float distanceMm) {
  // 1. Determine direction state based on the value sign
  if (distanceMm >= 0) {
    digitalWrite(DIR_PIN, HIGH); // Clockwise / Forward
    Serial.println("Direction: FORWARD (+)");
  } else {
    digitalWrite(DIR_PIN, LOW);  // Counter-Clockwise / Reverse
    Serial.println("Direction: REVERSE (-)");
    distanceMm = abs(distanceMm); // Work with absolute distance for pulse calculation
  }
  
  // 2. Calculate the exact target step allocation
  long totalRequiredSteps = (long)(distanceMm * STEPS_PER_MM);
  
  Serial.print("Executing ");
  Serial.print(totalRequiredSteps);
  Serial.println(" precision step pulses...");
  
  // 3. Generate high-frequency square wave pulse stream
  for (long i = 0; i < totalRequiredSteps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(PULSE_DELAY_MICROS);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(PULSE_DELAY_MICROS);
  }
  
  Serial.println("Target position reached successfully.");
  Serial.println("\nReady for next command...");
}