#include <Arduino.h>
#include <HCSR04.h>

UltraSonicDistanceSensor leftSensor(13, 12);  // Initialize sensor that uses digital pins 13 and 12.
UltraSonicDistanceSensor rightSensor (8, 7);
int leftSensorValue = 0;
int rightSensorValue = 0;
int rightOutput;


void setup () {
  Serial.begin(9600);  // We initialize serial connection so that we could print values from sensor.
  pinMode (3, OUTPUT);
  pinMode (5, OUTPUT);
}

void newFunction(int x) {
  
  float rightDecimal = x/100.0;
  int dutyValue = 1000;
  Serial.println(rightSensorValue);

  digitalWrite(3, HIGH);
  delay (rightDecimal * dutyValue);
  digitalWrite(3, LOW);
  delay (dutyValue - (rightDecimal * dutyValue));
  rightOutput = 0;

}

void loop () {
  // Every 500 miliseconds, do a measurement using the sensor and print the distance in centimeters.
  leftSensorValue = leftSensor.measureDistanceCm() ;

  rightSensorValue = rightSensor.measureDistanceCm() ;

if (rightSensorValue <= 120){
  
  //  Serial.println(rightSensorValue);

    rightOutput = log(rightSensorValue + 1) / log(120) * 100;
    rightOutput = map(rightOutput, 0, 102, 100, 0);

    newFunction(rightOutput);

}

else {

  Serial.println("out of range!");

}

  int leftOutput = log(leftSensorValue + 1) / log(120) * 100;
  leftOutput = map(leftOutput, 0, 102, 100, 0);

//  Serial.println(leftSensorValue);

}

