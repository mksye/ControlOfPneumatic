/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************
  Note: This only works on Android!
        iOS does not support Bluetooth 2.0 Serial Port Profile
        You may need to pair the module with your smartphone
        via Bluetooth settings. Default pairing password is 1234

 *************************************************************/
#define BLYNK_PRINT Serial

#include <SoftwareSerial.h>
SoftwareSerial SwSerial(11, 12); // RX, TX
    
#include <BlynkSimpleSerialBLE.h>
#include <SoftwareSerial.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "2dad3047c43147f4a1e8463baf84184c";

SoftwareSerial SerialBLE(11, 12); // RX, TX


//  DEFINE PRESSURE PERCENTAGE (potN)
    float pot1=0;
    float pot2=0;
    float pot3=0;
    float pot4=0;      
    const int Vcc=5.0;

// This function will be called every time Slider Widget
// in Blynk app writes values to the Virtual Pins V1, V2, V3, and V4
BLYNK_WRITE(V1)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  Serial.println(pinValue);
  // process received value
  pot1 = pinValue;
}

BLYNK_WRITE(V2)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  Serial.println(pinValue);
  // process received value
  pot2 = pinValue;
}

BLYNK_WRITE(V3)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  Serial.println(pinValue);
  // process received value
  pot3 = pinValue;
}

BLYNK_WRITE(V4)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  Serial.println(pinValue);
  // process received value
  pot4 = pinValue;
}

void setup()
{

  // Debug console
  Serial.begin(9600);

  SerialBLE.begin(9600);
  Blynk.begin(SerialBLE, auth);

  Serial.println("Waiting for connections...");

  // SET UP PWMs (initialised at 0%) 
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
  TCCR0B = _BV(CS02) | _BV(CS00); 
  OCR0A = 0;
  OCR0B = 0;

  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);  
   TCCR1A = _BV(COM1A1)|_BV(COM1B1);
   TCCR1B = _BV(CS12)|_BV(CS10)|_BV(WGM13);
   ICR1=255;
   OCR1A=0;
   OCR1B=0;
}

void loop()
{

// Changes the duty cycle according to PotN inputs
                          OCR0A=(255*(pot3/100))/1;   //duty cycle adjustment (3 from left - D6)
                          OCR1A=(255*(pot2/100))/1;   //duty cycle adjustment (2  from left - D9)
                          OCR0B=(255*(pot4/100))/1;   //duty cycle adjustment (4 from left - D5)
                          OCR1B=(255*(pot1/100))/1;   //duty cycle adjustment (1  from left - D10)

  
  Blynk.run();

  Serial.print("pot1, ");
  Serial.println(pot1); 
  Serial.print("pot2, ");
  Serial.println(pot2); 
  Serial.print("pot3, ");
  Serial.println(pot3); 
  Serial.print("pot4, ");
  Serial.println(pot4); 

}


