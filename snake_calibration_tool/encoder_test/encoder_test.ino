volatile int temp0, counter1, temp1, counter2 = 0; //This variable will increase or decrease depending on the rotation of encoder
unsigned char peak1, notch1, peak2, notch2 = 0;
unsigned long timeA1, timeB1, timeA2, timeB2;
unsigned long currenttime1, currenttime2 = millis();
unsigned char pinA1;
unsigned char pinB1;
unsigned char pinA2;
unsigned char pinB2;
double degree0 = 0.0; 
double degree1 = 0.0;
 
void setup() {
  Serial.begin (9600);
  pinMode(2, INPUT_PULLUP); // internal pullup input pin 2 
  pinMode(3, INPUT_PULLUP); // internal pullup input pin 3
  pinMode(18, INPUT_PULLUP); // internal pullup input pin 18
  pinMode(19, INPUT_PULLUP); // internal pullup input pin 19
 
  //Setting up interrupts
  //A rising pulse from encoder activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on Arduino.
  attachInterrupt(digitalPinToInterrupt(2), ai0, CHANGE);
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on Arduino.
  attachInterrupt(digitalPinToInterrupt(3), ai0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), ai2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(19), ai2, CHANGE);
}
   
void loop() {
  // Send the value of counter
//  if(( counter0 != temp0 )||(counter1 != temp1)){
    degree0 = (counter1 /2.0)*0.6;
    degree1 = (counter2 /2.0)*0.6;
    Serial.print(degree0);
    Serial.print(F(", "));
    Serial.println(degree1);
//  }
}
   
void ai0() {
  currenttime1 = millis();
  pinA1 = digitalRead(2);
  pinB1 = digitalRead(3);
  if((pinA1 == 1) && (pinB1 == 1)){
    peak1 = 1;
  }
  if ((pinA1 == 0) && (pinB1 == 0)){
    notch1 = 1;
  }
  if ((pinA1 == 0) && (pinB1 == 0) && (peak1 ==1)){
    peak1 = 0;
    if (timeA1 < timeB1){
      counter1 --;
    }if (timeA1 > timeB1){
      counter1 ++;
    }
  }
  if ((pinA1 == 1) && (pinB1 == 1) && (notch1 == 1)){
    notch1 = 0;
    if (timeA1 > timeB1) {
      counter1 --;
    }if (timeA1 < timeB1){
      counter1 ++;
    }
  }

  if (pinA1 == 1){
    timeA1 = currenttime1;
  }
  if (pinB1 == 1){
    timeB1 = currenttime1;
  }
}
   


void ai2() {
  currenttime2 = millis();
  pinA2 = digitalRead(18);
  pinB2 = digitalRead(19);
  if((pinA2 == 1) && (pinB2 == 1)){
    peak2 = 1;
  }
  if ((pinA2 == 0) && (pinB2 == 0)){
    notch2 = 1;
  }
  if ((pinA2 == 0) && (pinB2 == 0) && (peak2 ==1)){
    peak2 = 0;
    if (timeA2 < timeB2){
      counter2 --;
    }if (timeA2 > timeB2){
      counter2 ++;
    }
  }
  if ((pinA2 == 1) && (pinB2 == 1) && (notch2 == 1)){
    notch2 = 0;
    if (timeA2 > timeB2) {
      counter2 --;
    }if (timeA2 < timeB2){
      counter2 ++;
    }
  }

  if (pinA2 == 1){
    timeA2 = currenttime2;
  }
  if (pinB2 == 1){
    timeB2 = currenttime2;
  }
}
   
