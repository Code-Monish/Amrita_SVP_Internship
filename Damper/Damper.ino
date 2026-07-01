// INVERTED PENDULUM DAMPER Module

// PINOUT
#define STEP_PIN 4
#define DIR_PIN 7

#define ENCODER_PIN_A 2  //Green wire
#define ENCODER_PIN_B 3  //White wire

#define BLINK_PIN 13

// GLOBAL VARIABLES
int systemState = 0;  // 0 Homing, 1 Human Interaction Mode
bool isSystemArmed = false;
long lastMicros = 0;

// position variables
const long pivot_length = 220;  //mm
const long ORIGIN_PIVOT_STEPLEN = 9.847 * pivot_length;

long currentPosition = 0;
long TargetPosition = 0;

// encoder variables
volatile int encoderTicks = 0;
volatile int lastEncoded = 0;
long lastTicks = 0;
unsigned long lastDamperMicros = 0;
unsigned long lastPrintMillis = 0;
float angularVelocity = 0;
signed long angleError = 0;

// damper variables
float timeDiff = 0;
float angleDiff = 0;
long stepControlDelay = 0;
unsigned long maxSpeedDelay = 80;    // Calibrated for 800 p/r
unsigned long minSpeedDelay = 1000;  // Snappy baseline crawl

const float Kp = 1.5;
const float Ki = 0;
const float Kd = 0.5;

void setup() {
  Serial.begin(115200);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(BLINK_PIN, LOW);

  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);
}


void loop() {
  unsigned long currentMicros = micros();
  unsigned long currentMillis = millis();

  // Homing logic (Manual set)
  if (!isSystemArmed) {
    if (encoderTicks <= -600) {
      delay(5000);

      currentPosition = 0;
      lastMicros = 0;

      isSystemArmed = true;
      systemState = 1;  // Transition straight into Interaction Mode
    }
  } else {
    if (systemState == 1) {
      if (currentMicros - lastDamperMicros >= 10000) {
        damper(currentMicros);
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

void damper(unsigned long currentMicros) {
  noInterrupts();
  long snapshotTicks = encoderTicks;
  interrupts();

  // if (abs(angleError) <= 4) {
  //   // Pendulum is close enough to center; relax the motor entirely
  //   stepControlDelay = minSpeedDelay; 
  //   angularVelocity = 0;
  //   angleDiff = 0;
  //   lastTicks = snapshotTicks; // Keep history updated so it doesn't spike on exit
  //   return; // Exit the function early so no motor forces are computed
  // }

  timeDiff = 10.0;  // Fixed 10ms tracking step

  angleError = snapshotTicks;
  angleDiff = (snapshotTicks - lastTicks) * 0.15;     // Degrees
  angularVelocity = (angleDiff * 1000.0) / timeDiff;  // Clean Deg/Sec

  // PD Control Law execution handling human induced force deflection
  float proportionalControlEffort = abs(angleError) * Kp;
  float derivativeControlEffort = abs(angularVelocity) * Kd;
  float totalControlEffort = proportionalControlEffort + derivativeControlEffort;

  // Map out inverse delay bounds safely
  stepControlDelay = constrain(minSpeedDelay - totalControlEffort, maxSpeedDelay, minSpeedDelay);

  // History Updates
  lastTicks = snapshotTicks;
  lastDamperMicros = currentMicros;

  // Align direction with the physical fall context to resist or assist force
  if (angleError < 0) {
    digitalWrite(DIR_PIN, HIGH);
  } else {
    digitalWrite(DIR_PIN, LOW);
  }
}

// Interrupt Service Routine [Rotary encoder readings]
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