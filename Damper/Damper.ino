// INVERTED PENDULUM DAMPER Module

// PINOUT
#define STEP_PIN 4
#define DIR_PIN 7

#define ENCODER_PIN_A 2 //Green wire
#define ENCODER_PIN_B 3 //White wire

#define BLINK_PIN 13

// GLOBAL VARIABLES
int systemState = 0; // 0 Homing, 1 Transit, 2 Damping
bool isSystemArmed = false;
long lastMicros = 0;

// position variables
const long pivot_length = 350; //mm
const long ORIGIN_PIVOT_STEPLEN = 19.685*pivot_length;

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
// const long QUARTER_ANGLE_COUNT = 300;

// damper variables
float timeDiff = 0;
float angleDiff = 0;
unsigned long dampingStartingMicros = 0;
long stepControlDelay = 0;
unsigned long maxSpeedDelay = 25;
unsigned long minSpeedDelay = 1000;
const int Kp = 3.5;
const int Ki = 0;
const int Kd = 1.8;

void setup(){
  Serial.begin(115200);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  // pinMode(BLINK_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(BLINK_PIN, LOW);

  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);
}


void loop(){
  unsigned long currentMicros = micros();
  unsigned long currentMillis = millis();

  // Homing logic (Manual set)
  if (!isSystemArmed) {
    if (encoderTicks <= -600){
      delay(3000);

      currentPosition = 0;
      TargetPosition = 2000;
      lastMicros = 0;

      isSystemArmed = true;
      systemState = 1;
    }
  }
  else{
    if (systemState == 1){
      // Motor motion
      if (currentPosition != TargetPosition){
        if (currentMicros-lastMicros>200){
          if (currentPosition < TargetPosition) {
            digitalWrite(DIR_PIN, HIGH);
            digitalWrite(STEP_PIN, HIGH);
            delayMicroseconds(2);
            digitalWrite(STEP_PIN, LOW);
            currentPosition++;
          } 
          else {
            digitalWrite(DIR_PIN, LOW);
            digitalWrite(STEP_PIN, HIGH);
            delayMicroseconds(2);
            digitalWrite(STEP_PIN, LOW);
            currentPosition--;
          }
          lastMicros = micros();
        }
      }
      // Target Update
      if (currentPosition == 2000){
          systemState = 2;
          TargetPosition =0;
          dampingStartingMicros = micros();
        } else if (currentPosition == 0) {
          systemState = 2;
          TargetPosition =2000;
          dampingStartingMicros = micros();
      }
    }

    // Damping and debug block
    if (systemState == 2){
      if (micros()-dampingStartingMicros >= 1500000){
        systemState = 1;
      } else if (currentMicros - lastDamperMicros >= 10000){
        damper(currentMicros);  
      }

      if (currentMicros - lastMicros > stepControlDelay){
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(2); 
        digitalWrite(STEP_PIN, LOW);
      
        Serial.print("Error: ");     Serial.print(angleError);
        Serial.print(" | Velocity: ");  Serial.println(angularVelocity);
        lastMicros = currentMicros;
      }

      // if (currentMillis - lastPrintMillis >= 100) {
      //   lastPrintMillis = currentMillis;
      // }

      // lastPrintMillis = currentMillis;
      // lastTicks = snapshotTicks;

    }

    
  }
  
}

void damper(unsigned long currentMicros){
  noInterrupts();
  long snapshotTicks = encoderTicks;
  // unsigned long currentMicros = micros();
  interrupts();


  // timeDiff = (currentMicros-lastDamperMicros)/1000000.0;
  // if(timeDiff<=0){return;}
  timeDiff = 10;

  angleError = snapshotTicks;
  angleDiff = (snapshotTicks - lastTicks)*0.15;
  angularVelocity = angleDiff*1000/timeDiff;

  float propotionalControlEffort = abs(angleError)*Kp;
  float derivativeControlEffort = abs(angularVelocity)*Kd;
  float totalControlEffort = propotionalControlEffort + derivativeControlEffort;

  stepControlDelay = constrain(minSpeedDelay - totalControlEffort, maxSpeedDelay, minSpeedDelay);

  lastTicks = snapshotTicks;
  lastDamperMicros = currentMicros;

  if (angularVelocity < 0){
    digitalWrite(DIR_PIN, HIGH);
  } else {
    digitalWrite(DIR_PIN, LOW);
  }

}

// Interrupt Service Routine [Rotary encoder readings]
void updateEncoder() {
  int MSB = (PIND & (1 << 2)) >> 2; // Most Significant Bit (Ch A)
  int LSB = (PIND & (1 << 3)) >> 3; // Least Significant Bit (Ch B)

  int encoded = (MSB << 1) | LSB; 
  int sum = (lastEncoded << 2) | encoded; 
  
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderTicks++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderTicks--;
  }
  
  lastEncoded = encoded; 
}