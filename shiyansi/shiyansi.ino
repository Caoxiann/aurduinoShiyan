//
// shiyansi
//
// Description of the project
// Developed with [embedXcode](http://embedXcode.weebly.com)
//
// Author 		陈浩贤
// 				陈浩贤
//
// Date			16/10/24 下午8:37
// Version		<#version#>
//
// Copyright	© 陈浩贤, 2016年
// Licence		<#licence#>
//
// See         ReadMe.txt for references
//


// Core library for code-sense - IDE-based
#if defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#include "WProgram.h"
#elif defined(MPIDE) // chipKIT specific
#include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
#include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
#include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
#include "Arduino.h"
#elif defined(SPARK) || defined(PARTICLE) // Particle / Spark specific
#include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
#include "Arduino.h"
#elif defined(ESP8266) // ESP8266 specific
#include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#include "Arduino.h"
#else // error
#error Platform not defined
#endif // end IDE


#include <Wire.h>
#include "InfraredReceiver.h"
#define PWM_pin 8
int key;//pressed key
int ledPin = 9;
int analogInPin =A0;
int sensorValue = 0;
const int PIRSensorPin = 3;     // the number of the Sensor pin

// variables will change:
int sensorState = 0;         // variable for reading the Sensor status

int flag = 0; //use the flag to prevent continuous outputing when no moving people
const int relay = 7;

int pulsewidth = 0;		//高电平时间
int pos=0;

void pulse(int angle);
//The code uses interrupt 0, and it works only when the module is connected to D2 of Arduino
InfraredReceiver infraredReceiver;
void setup() {
    infraredReceiver.begin();
    Serial.begin(9600);
    Serial.println("Infrared Receiver is waiting for the control signal");
    Serial.println("Make sure the Infrared Module is connected to D2 of Arduino");
    pinMode(PWM_pin,OUTPUT);
    pinMode(PIRSensorPin, INPUT);
    digitalWrite(PIRSensorPin, LOW);
    pinMode(relay, OUTPUT);
    
}

void loop() {
    key = infraredReceiver.read();// get the pressed key information
    if (key >= 0)
    {
        Serial.print("Key=");//Display the key information
        Serial.println(key);
        Serial.println();
    }
    
    sensorValue = analogRead(analogInPin);
    int gapValue = map(sensorValue,10,1024,2,30);
    for(int fadeValue = 0; fadeValue <= 255; fadeValue +=5 ){
        analogWrite(ledPin, fadeValue);
        delay(gapValue);
    }
    for(int fadeValue = 255; fadeValue <= 0; fadeValue -=5 ){
        analogWrite(ledPin, fadeValue);
        delay(gapValue);
    }
    
    
    for(pos = 0; pos < 180; pos += 1)  // goes from 0 degrees to 180 degrees
    {                           // in steps of 1 degree
        pulse(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                  // waits 15ms for the servo to reach the position
    }
    for(pos = 180; pos>=1; pos-=1)     // goes from 180 degrees to 0 degrees
    {
        pulse(pos);               // tell servo to go to position in variable 'pos'
        delay(15);               // waits 15ms for the servo to reach the position
    }
    Serial.print("pos=");
    Serial.println(pos);
    
    // read the state of the Sensor value:
    sensorState = digitalRead(PIRSensorPin);
    
    // check if the moving people sensed.
    // if it is, the sensorState is HIGH:
    if (sensorState == HIGH) {
        // turn Buzzer on:
        Serial.println("Moving people detected");
        flag = 0;
    }
    else {
        if(flag == 0){
            Serial.println("No moving people");
            flag = 1;
        }
    }
    
    digitalWrite(relay, HIGH);   // turn the Relay on (HIGH is the voltage level)
    delay(1000);               // wait for a second
    digitalWrite(relay, LOW);    // turn the Relay off by making the voltage LOW
    delay(1000);
    
    
}
void pulse(int angle)			//设置舵机角度为angle
{
    pulsewidth=int ((angle*11)+500);	//计算高电平时间
    digitalWrite(PWM_pin,HIGH);				//设置高电平
    delayMicroseconds(pulsewidth);		//延时pulsewidth （us）
    digitalWrite(PWM_pin,LOW);				//设置低电平
    delay(20-pulsewidth/1000);				//延时20-pulsewidth/1000 （ms）
}
