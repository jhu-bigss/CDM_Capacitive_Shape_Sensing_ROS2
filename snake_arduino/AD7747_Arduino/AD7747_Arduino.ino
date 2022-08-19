#include <Wire.h> //I2C library

double capacitance = 0; //capacitance reading
double capacitance0 = 0; // capa0 result to send to py
double capacitance1 = 0; // capa1 result to send to py
double capacitance2 = 0; // capa2 result to send to py

int DataReady = B0000; //

int StatusAddress = 0x00;
int CapDataH = 0x01;
int CapDataM = 0x02;
int CapDataL = 0x03;
int Address = B1001000;

int CapSetup = 0x07;        //capacitive channel setup address
int ExcSetup = 0x09;        //excitation setup address
int ConfigSetup = 0x0A;     //configuration setup addrss
int CapDacAReg = 0x0B;      //Capacitive DAC setup address
int CapGainRegH = 0x0F;     //Cap gain high adress
int CapGainRegL = 0x10;     //Cap gain high adress
int CapOffsetH = 0x0D;      //Cap Offset high adress
int CapOffsetL = 0x0E;      //Cap gain Offset adress

int CapChanProp = B10100000;    //capacitive channel properties
int ExcProp = B00001110;        //excitation properties
int ConfigProp = B00010001;     //configuration properties
int CapDacProp = B0;            //Capacitive DAC setup properties
int CapGainPropH = B01011101;   //cap gain properties high
int CapGainPropL = B10111101;   //cap gain properties low
int CapOffsetPropH = B01110111; //cap offset properties high
int CapOffsetPropL = B00011010; //cap Offset properties low
// counter to determine the state
int counter = 0;
// counter to determine the printing state
bool DATA_READY = false;
// button state for button ISR
bool button_state = false;

// interrupt routines
const byte interruptPin = 2;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  // interrupt service routine setup
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), MCU_timer, FALLING);

  Configuration(); //determine configuration properties in AD7745

  Excitation(); //determine the excitation properties of the AD7745

  CapInput(); //Capacitive input properties

  CapDacARegister(); //Capacitive data aquisition properties
  CapOffsetHighAdjust(); //The adjustment of the offset (mainly used in differential mode)
  CapOffsetLowAdjust();  //The adjustment of the offset (mainly used in differential mode)

  // addressRead(); //Reads address and provides binary value (used for debugging)

  continuous(); //sets up continuous reading of the AD7745

  CapGainHighAdjust(); //used for adjusting the gain of AD7745
  CapGainLowAdjust();   //used for adjusting the gain of AD7745


  //chip select DG4051E & DG4052E
  pinMode(7, OUTPUT);
  // initialize DG4052E
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  digitalWrite(7, LOW);
  // initialize DG4051E
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);

  // switch on X0Y0 (DG4052E)
  digitalWrite(12, LOW); // A
  digitalWrite(11, LOW); // B

  // switch on X0 (DG4051E)
//  digitalWrite(10, LOW); // A
//  digitalWrite(9, LOW); // B
//  digitalWrite(8, LOW); // C

  delay(15); // delay before the loop starts

}

void loop()
{
  if (DATA_READY) {

    capacitance = dataRead();
    DATA_READY = false;

    int current_state = counter % 3;

    if (current_state == 0) {
      //      switch to next state
      digitalWrite(12, HIGH);
      digitalWrite(11, LOW);
//      digitalWrite(10, HIGH);
//      digitalWrite(9, LOW);
//      digitalWrite(8, LOW);

      //      Serial.print(F("[0]: "));
    } else if (current_state == 1) {
      digitalWrite(12, HIGH);
      digitalWrite(11, HIGH);
//      digitalWrite(10, HIGH);
//      digitalWrite(9, HIGH);
//      digitalWrite(8, LOW);

      //      Serial.print(F("[1]: "));
    } else {
      digitalWrite(12, LOW);
      digitalWrite(11, LOW);
//      digitalWrite(10, LOW);
//      digitalWrite(9, LOW);
//      digitalWrite(8, LOW);

      //      Serial.print(F("[2]: "));
    }

    if (current_state == 0) {
      //      Serial.println(capacitance - capa0, DEC);
      capacitance0 = capacitance;
    } else if (current_state == 1) {
      //      Serial.println(capacitance - capa1, DEC);
      capacitance1 = capacitance;
    } else {
      //      Serial.println(capacitance - capa2, DEC);
      capacitance2 = capacitance;
      Serial.print(capacitance0, DEC);
      Serial.print(F(", "));
      Serial.print(capacitance1, DEC);
      Serial.print(F(", "));
      Serial.println(capacitance2, DEC);
    }


  }

  //  statusRead(); //reads the status of the AD7745
  //  if (statusRead() & 0 == DataReady) { //if the capacitive conversion has been completed
  //    capacitance = dataRead(); //read the data from capacitive registers of AD7745
  //    Serial.println(capacitance, DEC);
  //
  //  }



}

void MCU_timer() {
  counter ++;
  DATA_READY = true;
}
