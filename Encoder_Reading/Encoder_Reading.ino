int encoderPin1 = 2;
int encoderPin2 = 3;

volatile int lastEncoded = 0;
volatile long encoderValue = 0;

long lastencoderValue = 0;

int lastMSB = 0;
int lastLSB = 0;

void setup() {
  Serial.begin (115200);

  pinMode(encoderPin1, INPUT); 
  pinMode(encoderPin2, INPUT);

  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(digitalPinToInterrupt(encoderPin1), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), updateEncoder, CHANGE);

}

void loop(){ 
  // 1. Temporarily pause interrupts to read 'encoderValue' safely.
  // Because 'long' is a 4-byte variable, a background interrupt could 
  // modify it mid-read on an 8-bit Arduino, causing data corruption.
  noInterrupts();
  long currentCounts = encoderValue;
  interrupts(); // Re-enable interrupts immediately

  // 2. Map counts to angles with inverse orientation
  // 1200 counts -> -90.00 degrees | -1200 counts -> +90.00 degrees
  float angle = -((float)currentCounts / 2400.0) * 360.0;

  // 3. Print the processed data
  Serial.print("Counts: ");
  Serial.print(currentCounts);
  Serial.print(" | Angle: ");
  Serial.print(angle, 2); // Prints angle to 2 decimal places
  Serial.println(" deg");

  delay(300); 
}


void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  lastEncoded = encoded; //store this value for next time
}