#include <Wire.h> //I2C library
#define TCAADDR 0x70

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
int ConfigProp = B00010010;     //configuration properties
int CapDacProp = B0;            //Capacitive DAC setup properties
int CapGainPropH = B01011101;   //cap gain properties high
int CapGainPropL = B10111101;   //cap gain properties low
int CapOffsetPropH = B01110111; //cap offset properties high
int CapOffsetPropL = B00011010; //cap Offset properties low
// counter to determine the state
int counter = 0;

// counter to determine the printing state
volatile bool DATA_READY0 = false;
volatile bool DATA_READY1 = false;
volatile bool DATA_READY2 = false;
// button state for button ISR
bool button_state = false;

// interrupt routines
const byte interruptPin0 = 2;
const byte interruptPin1 = 3;
const byte interruptPin2 = 18;

// capacitance array
const int maxtca = 3;
double capas[maxtca];

// I2C multiplexer helper function
void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup()
{
  Serial.begin(9600);
  Wire.begin();
//  // interrupt service routine setup
//  pinMode(interruptPin0, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(interruptPin0), MCU_callback0, FALLING);
//  pinMode(interruptPin1, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(interruptPin1), MCU_callback1, FALLING);
//  pinMode(interruptPin2, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(interruptPin2), MCU_callback2, FALLING);


  for (int tca = 0; tca < maxtca; tca ++) {
    tcaselect(tca);
    delay(10);

    Wire.beginTransmission(Address);  // reset AD7747
    Wire.write(0xBF);
    Wire.endTransmission();
    delay(15);

    Configuration(); //determine configuration properties in AD7745

    Excitation(); //determine the excitation properties of the AD7745

    CapInput(); //Capacitive input properties

    CapDacARegister(); //Capacitive data aquisition properties
    CapOffsetHighAdjust(); //The adjustment of the offset (mainly used in differential mode)
    CapOffsetLowAdjust();  //The adjustment of the offset (mainly used in differential mode)

    //     addressRead(); //Reads address and provides binary value (used for debugging)

    continuous(); //sets up continuous reading of the AD7745

    //    CapGainHighAdjust(); //used for adjusting the gain of AD7745
    //    CapGainLowAdjust();   //used for adjusting the gain of AD7745

  }


//  Serial.println("Init Finished");

  delay(15); // delay before the loop starts

}

void loop()
{
  int current_state = counter % 3;
  if (current_state == 0){
    tcaselect(0);
    Configuration();
    capas[0] = dataRead();
  }else if (current_state == 1){
    tcaselect(1);
    Configuration();
    capas[1] = dataRead();
  }else{
    tcaselect(2);
    Configuration();
    capas[2] = dataRead();
  }
  counter ++;
  Serial.print(capas[0], DEC);
  Serial.print(", ");
  Serial.print(capas[1], DEC);
  Serial.print(", ");
  Serial.println(capas[2], DEC);
 
}


void MCU_callback0() {
  DATA_READY0 = true;
  DATA_READY1 = false;
  DATA_READY2 = false;
  //  Serial.println("7747_0 is here");

}

void MCU_callback1() {
  DATA_READY0 = false;
  DATA_READY1 = true;
  DATA_READY2 = false;
  //  Serial.println("7747_1 is here");

}

void MCU_callback2() {
  DATA_READY0 = false;
  DATA_READY1 = false;
  DATA_READY2 = true;
  //  Serial.println("7747_2 is here");

}