// ON is towards USB
// OFF is towards outputs
// switch must be ON to upload
// check resistor if opening case

// define variables
int inPin = 8;    // left
int outLow = 7;   // middle
int outHigh = 11; // right
int buzzer = 9;   // test LEDs
int del = 1000;
int on = 1000;

void setup() {
  // define input/output
  pinMode(inPin, INPUT);     
  pinMode(outLow, OUTPUT); 
  pinMode(outHigh, OUTPUT);
  pinMode(buzzer, OUTPUT);

  // turn everything off
  digitalWrite(outLow, LOW);
  digitalWrite(outHigh, LOW);
  digitalWrite(buzzer, LOW);
}

void loop() {

  // run if input detected
  if (digitalRead(inPin)) {
    delay(del); // wait 4 seconds

    digitalWrite(buzzer, HIGH);
    delay(100);
    digitalWrite(buzzer, LOW);
    delay(1000);
    digitalWrite(buzzer, HIGH);
    delay(100);
    digitalWrite(buzzer, LOW);

    // turn on outputs for 30 ms (low has external resistor)
    digitalWrite(outLow, HIGH);  
    digitalWrite(outHigh, HIGH);
    delay(on);

    // turn everything off
    digitalWrite(outLow, LOW);
    digitalWrite(outHigh, LOW);
  }
}
