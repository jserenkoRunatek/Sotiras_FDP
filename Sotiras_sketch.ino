#include "DFRobot_EC10.h"
#include <EEPROM.h>

//using named constants for pins
#define enA 10
#define in3 7
#define in4 8
#define EC_PIN A1

//using named constants for parameters
const float targetEc = 65;
const int readDelay = 1000;
const int injectTime = 250;
const int period = 0;
float voltage = 0;
float ecValue;
float temperature = 25;
//makes instance of object "DFRobot_EC10", used for the probe
DFRobot_EC10 ec;

void setup()
{
  //set all pins to either input or output
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(EC_PIN, INPUT);
  //set PWM full throtle
  analogWrite(enA, 255);
  Serial.begin(115200);
  //starts instance of "DFRobot_EC10"

  ec.begin();

}

void loop()
{
  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U) //time interval: 1s
  {
    timepoint = millis();
    voltage = analogRead(EC_PIN) * 0.72; // read the voltage
    Serial.print("voltage:");
    Serial.print(voltage);
    //temperature = readTemperature();  // read your temperature sensor to execute temperature compensation
    ecValue =  ec.readEC(voltage, temperature); // convert voltage to EC with temperature compensation
    Serial.print("  temperature:");
    Serial.print(temperature, 1);
    Serial.print("^C  EC:");
    Serial.print(ecValue, 1);
    Serial.println("ms/cm");
  }
  ec.calibration(voltage, temperature);
  if (ecValue < targetEc)
  {
    //Serial.print("Branch!!!\n"); //remove comment in case of screw up
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    delay(injectTime);
    stopMotor();
  }
}

void extend()
{
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
void retract()
{
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}
void stopMotor()
{
  digitalWrite(in3, LOW);
  digitalWrite(in3, LOW);
}
void inject()
{
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(injectTime);
  stopMotor();

}
