//read the data from AD7745 data registers and converts it to capacitance
double dataRead() {
   Wire.requestFrom(Address, 4);
  
  while(Wire.available() == 0);
  
  byte byte1 = Wire.read();
  long byte2 = Wire.read();
  long byte3 = Wire.read();
  long byte4 = Wire.read();
  
  long capValue = (byte2 * 0x10000 + byte3 * 0x100 + byte4);
 
  capacitance = ((capValue * 8.192 / 16777216)); //formula used for calculation of true capacitance

//  Serial.println(capacitance, DEC); // will print in the main loop for formatting
  return capacitance;
}
