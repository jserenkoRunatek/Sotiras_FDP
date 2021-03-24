#include "DFRobot_EC10.h"

//using named constants for pins
const int pumpIn1 = 8;
const int pumpIn2 = 7;
const int ENA = 10;
const int fromSensor = A0;

//using named constants for parameters
const float targetVoltage = 2.5f;
const int readDelay = 1000;
const int injectTime = 250;
const int period = 0;

//makes instance of object "DFRobot_EC10", used for the probe
DFRobot_EC10 ec;

void setup() 
{
  //set all pins to either input or output
  pinMode(pumpIn1, OUTPUT);
  pinMode(pumpIn2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(fromSensor, INPUT);
  //set PWM full throtle
  digitalWrite(ENA, 100);
  //starts instance of "DFRobot_EC10"
  ec.begin();

}

void loop() 
{
  float voltage;
  unsigned long timepoint = millis();
  if(millis()-timepoint > readDelay)
  {
    voltage = getVoltage();
    Serial.print("Voltage: ");
    Serial.print(voltage);
      if(voltage < targetVoltage)
      {
        inject();
      }
  }
}

void extend()
{
  digitalWrite(pumpIn1, HIGH);
  digitalWrite(pumpIn2, LOW);
}
void retract()
{
  digitalWrite(pumpIn1, LOW);
  digitalWrite(pumpIn2, HIGH);
}
void stopMotor()
{
  digitalWrite(pumpIn1, LOW);
  digitalWrite(pumpIn2, LOW);
}
void inject()
{
  unsigned long timeRan = millis();
  while(millis() - timeRan < injectTime)
  {
    extend();
  }
  stopMotor();
  
}
float getVoltage()
{
  float voltage = analogRead(fromSensor)/1024.0*5000;
}
