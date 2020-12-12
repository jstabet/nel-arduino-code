#define INPUT_SIZE ((5+1)*5)-1

int cs = 8;
int us = 7;
int imaging = 11;
int idle = 10;
int buzzer = 9;
int inByte = 0;         // incoming serial byte
int reset = 2;

// define a string for settings input from PC

char in_str[INPUT_SIZE];//((number of digits for each variable)+token)*(# of variables)

int Tcss = 4000; // time in ms
int Tcse = 4500; // time from cs
int Tuss = 4250; // time in ms
int Tuse = 4280; // time from cs
int Tend = 10000; // time from us
int d1 = 10;
int d2 = 10;
int d3 = 10;
int d4 = 10;
int d5 = 10;
bool overlap = true;
bool cse_use = true;

void setup() {
  // Define as output
  pinMode(cs, OUTPUT);
  pinMode(us, OUTPUT);
  pinMode(imaging, OUTPUT);
  pinMode(idle, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(reset, OUTPUT);
  
  //Make sure eveything is turned off
  digitalWrite(cs, LOW);
  digitalWrite(us, LOW);
  digitalWrite(imaging, LOW);
  digitalWrite(idle, LOW);
  digitalWrite(buzzer, LOW);
  digitalWrite(reset, LOW);

  // Get the correct delay values
  figureOutTimes();
  
  // start serial port at 115200 bps:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  establishContact();  // send a byte to establish contact until receiver responds
}

void loop() {
  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) {
    // get incoming byte:
    inByte = Serial.read();
    
    if (inByte!=10){
      
    if (inByte==68) {// if D(efault)
      Serial.write('D');
      initiateStimulation();
    }
    else if (inByte==83) {// if S(et)
      Serial.write('S');// let the pc know you're ready
      // read
      updateParams();
      buzzOK();
    }
    else if (inByte==67) {// if C(S only)
      Serial.write('C');
      digitalWrite(imaging, HIGH);
      delay(Tcss);
      digitalWrite(cs, HIGH);
      delay(Tcse-Tcss);
      digitalWrite(cs, LOW);
      delay(Tend-Tcse);
      digitalWrite(imaging, LOW);
    }
    else if (inByte==85) {// if U(S only)
      Serial.write('U');
      digitalWrite(imaging, HIGH);
      delay(Tuss);
      digitalWrite(us, HIGH);
      delay(Tuse-Tuss);
      digitalWrite(us, LOW);
      delay(Tend-Tuse);
      digitalWrite(imaging, LOW);
    }
    else if (inByte == 82){//R(eset)
      Serial.write('U');
      digitalWrite(reset, HIGH);
    }
    else{// an error has occured
      buzzErr();
      Serial.write(inByte);
    }
    }
    }
}

void initiateStimulation(){ // I'M SURE THERE IS A SMARTER WAY TO DO THIS BUT THIS IS THE EASIEST
  if (overlap==false){
    digitalWrite(imaging, HIGH);
    delay(d1);
    digitalWrite(cs, HIGH);
    delay(d2);
    digitalWrite(cs, LOW);
    delay(d3);
    digitalWrite(us, HIGH);
    delay(d4);
    digitalWrite(us, LOW);
    delay(d5);
    digitalWrite(imaging, LOW);
  }
  else{
    digitalWrite(imaging, HIGH);
    delay(d1);
    digitalWrite(cs, HIGH);
    delay(d2);
    digitalWrite(us, HIGH);
    delay(d3);
    if (cse_use==true){
      digitalWrite(us, LOW);
      delay(d4);
      digitalWrite(cs, LOW);
    }
    else{
      digitalWrite(cs, LOW);
      delay(d4);
      digitalWrite(us, LOW);
    }
    delay(d5);
    digitalWrite(imaging, LOW);    
  }
  
}

void updateParams(){
  char buff[5];
  int count = 0;
  char * pch;
  byte size = Serial.readBytes(in_str, INPUT_SIZE);
  ///////////////////////////////////////////////////////// Test
//  Serial.write(in_str);
//  Serial.write('\n');
  //////////////////////////////////////////////////////////////
  pch = strtok (in_str,"-");
  while (pch != NULL) {
    if (count==0){
      Tcss = atoi(pch);
    }
    else if (count==1){
      Tcse = atoi(pch);
    }
    else if (count==2){
      Tuss = atoi(pch);
    }
    else if (count==3){
      Tuse = atoi(pch);
    }
    else{
      Tend = atoi(pch);
    }
    count++;
    pch = strtok (NULL, "-");
  }
//  itoa(Tcss,buff,10);
//  Serial.write(buff);
//  itoa(Tcse,buff,10);
//  Serial.write(buff);
//  itoa(Tuss,buff,10);
//  Serial.write(buff);
//  itoa(Tuse,buff,10);
//  Serial.write(buff);
//  itoa(Tend,buff,10);
//  Serial.write(buff);
  // Get the correct delay values
  figureOutTimes();
  itoa(d1,buff,10);
  Serial.write(buff);
  itoa(d2,buff,10);
  Serial.write(buff);
  itoa(d3,buff,10);
  Serial.write(buff);
  itoa(d4,buff,10);
  Serial.write(buff);
  itoa(d5,buff,10);
  Serial.write(buff);
}

void buzzOK(){ //
  digitalWrite(buzzer, HIGH);
  delay(100);
  digitalWrite(buzzer, LOW);
  delay(100);
  digitalWrite(buzzer, HIGH);
  delay(100);
  digitalWrite(buzzer, LOW);
}

void buzzErr(){ //
  digitalWrite(buzzer, HIGH);
  delay(600);
  digitalWrite(buzzer, LOW);
}

void establishContact() {
  bool toggle = false;
  bool ok = false;
  while (ok==false){
    while (Serial.available() <= 0) {
      digitalWrite(idle,toggle);
      toggle = !toggle;
      delay(500);
    }
    inByte = Serial.read();
    if (inByte == 65) {//A
      ok = true;
      digitalWrite(idle,LOW); // turn off idle LED
      Serial.write('B');
      buzzOK(); // buzz that connection is established
        }
    else {
        Serial.write('e');
        buzzErr();
        ok = false;
      }
}
}

void figureOutTimes(){
  d1 = Tcss;
  if (Tcse <= Tuss) {// there is no overlap
    overlap = false;
    d2 = Tcse - Tcss;
    d3 = Tuss - Tcse;
    d4 = Tuse - Tuss;
    d5 = Tend - Tuse;
  }
  else {
    overlap = true;
    d2 = Tuss - Tcss;
    d3 = Tcse - Tuss;
    if (Tcse > Tuse){
      cse_use = true;
      d3 = Tuse - Tuss;
      d4 = Tcse - Tuse;
      d5 = Tend - Tcse;
    }
    else{
      cse_use = false;
      d3 = Tcse - Tuss;
      d4 = Tuse - Tcse;
      d5 = Tend - Tuse;
    }
  }
}
