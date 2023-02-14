#include <Arduino_FreeRTOS.h>
#include <Adafruit_MotorShield.h>

void LeftMotor( void *pvParameters );
void RightMotor( void *pvParameters );
void readSerial( void *pvParameters );

int incomingByte;      // variable stores  serial data
bool flag = false;

// get motor shield I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// motor connected to port 1
Adafruit_StepperMotor *myMotor = AFMS.getStepper(500, 2);
Adafruit_StepperMotor *myMotor1 = AFMS.getStepper(500,1);

void setup() {

  // initialize serial communication at 9600 bits per second:

  Serial.begin(115200);

  xTaskCreate(LeftMotor , "task1" , 128 , NULL, 1 ,  NULL );
  xTaskCreate(RightMotor , "task2" , 128 , NULL, 1 ,  NULL );
  xTaskCreate(readSerial , "task3" , 128 , NULL, 1 ,  NULL );

  vTaskStartScheduler();

}

void loop()

{

}

void LeftMotor(void *pvParameters){
  while (1)
  {
    if ((incomingByte == 'H') && (flag == true)){
      myMotor1->step(500, FORWARD, DOUBLE);
      flag = false;
//      Serial.println("H");
    }else if ((incomingByte == 'L') && (flag == true)){
      myMotor1->step(500, BACKWARD, DOUBLE);
      flag = false;
    }
    vTaskDelay( 200 / portTICK_PERIOD_MS );
    vTaskDelay( 200 / portTICK_PERIOD_MS );
  }
}

void RightMotor(void *pvParameters){
  while (1)
  {
    if ((incomingByte == 'L') && (flag == true)){
      myMotor->step(500, BACKWARD, DOUBLE);
      flag = false;
//      Serial.println("L");
    }else if ((incomingByte == 'H') && (flag == true)){
      myMotor->step(500, FORWARD, DOUBLE);
      flag = false;
    }
    vTaskDelay( 200 / portTICK_PERIOD_MS );
    vTaskDelay( 200 / portTICK_PERIOD_MS );
  }
}

void readSerial(void *pvParameters){
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");
  myMotor->setSpeed(1000);  // 10 rpm

  while (1)
  {
    if (Serial.available() > 0){
      incomingByte = Serial.read();
      flag = true;
//      Serial.println(incomingByte);
    }
    vTaskDelay(400 / portTICK_PERIOD_MS);
  }
}
