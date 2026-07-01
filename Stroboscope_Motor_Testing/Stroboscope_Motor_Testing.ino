// #define PUL_PIN_1 4
// #define DIR_PIN_1 7

#define PUL_PIN_2 5
#define DIR_PIN_2 8

#define PHASE_SWITCH 9
#define SYNC_OUT_PIN 3

const unsigned long TEST_PERIOD_US = 187;

void setup() {
  // put your setup code here, to run once:

  // pinMode(PUL_PIN_1, OUTPUT); pinMode(DIR_PIN_1, OUTPUT);
  pinMode(PUL_PIN_2, OUTPUT); pinMode(DIR_PIN_2, OUTPUT);

  // digitalWrite(PUL_PIN_1, LOW);
  // digitalWrite(DIR_PIN_1, HIGH); 
  digitalWrite(PUL_PIN_2, LOW);
  digitalWrite(DIR_PIN_2, LOW);

  // Serial.begin(115200);
}

unsigned long phaseStartTime = millis();

void loop() {
  // put your main code here, to run repeatedly:
  // phaseStartTime = millis();
  // Serial.println("Motor 1 phase");

  digitalWrite(PUL_PIN_2, HIGH);
  delayMicroseconds(TEST_PERIOD_US);
  digitalWrite(PUL_PIN_2, LOW); 
  delayMicroseconds(TEST_PERIOD_US);

  // delay(2000);
  // Serial.println("Motor 2 phase");

  // phaseStartTime = millis();

  // while(millis() - phaseStartTime <= 5000){
  //   digitalWrite(PUL_PIN_2, HIGH);
  //   delayMicroseconds(TEST_PERIOD_US);
  //   digitalWrite(PUL_PIN_2, LOW);
  //   delayMicroseconds(TEST_PERIOD_US);    
  // }

  // delay(2000);
}
