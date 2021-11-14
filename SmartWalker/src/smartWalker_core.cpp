#include <Arduino.h>
#include <HCSR04.h>
#include <avr/sleep.h> //this AVR library contains the methods that controls the sleep modes
#define interruptPin 2 //Pin we are going to use to wake up the Arduino

UltraSonicDistanceSensor leftSensor(13, 12);  // Initialize sensor that uses digital pins 13 and 12.
UltraSonicDistanceSensor rightSensor (8, 7);
int leftSensorValue = 0;
int rightSensorValue = 0;
int rightMotorPin = 3;
int leftMotorPin = 4;

int motorStateRight = LOW;
int motorStateLeft = LOW;
unsigned long previousMillisRight = 0;
unsigned long previousMillisLeft = 0;
int intervalRight = 1000;
int intervalLeft = 1000;
unsigned long currentMillis = millis();

int rightOutput;

unsigned long previousMillisSleep = 0;
int intervalSleep = 5000;

void setup () {
  Serial.begin(9600);  // We initialize serial connection so that we could print values from sensor.
  pinMode (rightMotorPin, OUTPUT);
  pinMode (leftMotorPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
}

void wakeUp() {
  Serial.println("Interrrupt Fired");
  sleep_disable();
  detachInterrupt(0);
}

void Going_To_Sleep() {
  sleep_enable();
  attachInterrupt(0, wakeUp, LOW);//attaching a interrupt to pin d2
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  /*
    SLEEP_MODE_EXT_STANDBY Extended Standby Mode.  ------> least
    SLEEP_MODE_IDLE Idle mode.
    SLEEP_MODE_PWR_DOWN Power Down Mode.
    SLEEP_MODE_PWR_SAVE Power Save Mode.
    SLEEP_MODE_STANDBY Standby Mode.
  */
  sleep_cpu();//activating sleep mode
  Serial.println("just woke up!");//next line of code executed after the interrupt
}

int Mapping(int sensorValue){

//  Serial.println(sensorValue);

  if (sensorValue <= 120 && sensorValue > 0){
  
  //  Serial.println(rightSensorValue);

    int sensorOutput = log(sensorValue + 1) / log(120) * 100;
        sensorOutput = map(sensorOutput, 0, 102, 100, 0);
    return sensorOutput;
  }

else if (sensorValue > 120){

  return 0;

  }

  else {
    return -1;
  }
  
}


void driveOutput(int x, int y) {
  
  float dutyCycle = x/100.0;
  int dutyValue = 500;

  if (y == 3)Serial.print ("right: "); 
  else if (y == 4)Serial.print ("left: "); 
  Serial.println (dutyCycle);

  if (y == 3){
  if (currentMillis - previousMillisRight >= intervalRight) {
    previousMillisRight = currentMillis;

    if (motorStateRight == LOW) {
      motorStateRight = HIGH;
      intervalRight = dutyCycle * dutyValue;
    } else {
      motorStateRight = LOW;
      intervalRight = dutyValue - (dutyCycle * dutyValue);
    }

    digitalWrite(rightMotorPin, motorStateRight);
      }
  }
  
  else if (y == 4){
  if (currentMillis - previousMillisLeft >= intervalLeft) {
    previousMillisLeft = currentMillis;

    if (motorStateLeft == LOW) {
      motorStateLeft = HIGH;
      intervalLeft = dutyCycle * dutyValue;
    } else {
      motorStateLeft = LOW;
      intervalLeft = dutyValue - (dutyCycle * dutyValue);
    }

    digitalWrite(leftMotorPin, motorStateLeft);
    }
  }

/*  
  digitalWrite(y, HIGH);
  delay (dutyCycle * dutyValue);
  digitalWrite(y, LOW);
  delay (dutyValue - (dutyCycle * dutyValue)); 
*/
}

void loop () {
  // Every 500 miliseconds, do a measurement using the sensor and print the distance in centimeters.
  leftSensorValue = leftSensor.measureDistanceCm() ;
  rightSensorValue = rightSensor.measureDistanceCm() ;

  currentMillis = millis();

int xyz = Mapping(leftSensorValue);

  if (xyz < 0){
//  Serial.println("error💀 (leftsensor)");
  }
  else if (xyz == 0) {
    return;
  }
  else{
  driveOutput(xyz, leftMotorPin);
  }

  xyz = Mapping(rightSensorValue);
//Serial.println(xyz);

if (xyz < 0){
//  Serial.println("error💀 (rightsensor)");
}
else if (xyz == 0) {
  return;
}
else{
driveOutput(xyz, rightMotorPin);
}

  currentMillis = millis();
  if (currentMillis - previousMillisSleep >= intervalSleep) {
    previousMillisSleep = currentMillis;
    Going_To_Sleep();
  }

  if (digitalRead(interruptPin) == 0) {
    previousMillisSleep = currentMillis;

//Serial.print("Right sensor value: ");
//Serial.println(xyz);

//  Serial.println(leftSensorValue);

}
}