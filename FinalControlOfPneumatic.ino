//Name: Mohammad Kazim Syed
//Description: This code is used to control the opening and closing of solenoid valves via phase-shift PWM.
// The inputs are controlled manually using potentiometers and wirelessly using a HC-05 module.
// Blynk library (http://www.blynk.cc) is utilised to simplify the wireless control

// Includes the required libraries in following code
#define BLYNK_PRINT Serial
#include <SoftwareSerial.h>
#include <BlynkSimpleSerialBLE.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
LiquidCrystal_PCF8574 lcd(0x27);  // sets the LCD address to 0x27 for a 16 chars and 2 line display
SoftwareSerial SerialBLE(0, 1); // RX, TX pins for HC-05 module (wireless transmission)

// This unique Auth Token is for the Blynk App.
char auth[] = "2dad3047c43147f4a1e8463baf84184c"; //for wireless connection to HC05

// Defining all functions
    void CalcP1OffSet();
    void CalcP2OffSet();
    void CalcP3OffSet();
    void CalcP4OffSet();
    void SetNegZero();
    float CalcP1();
    float CalcP2();
    float CalcP3();
    float CalcP4();

//Defining all global variables
    const int Vcc=5.0;
    float pot1=100;        //initiliation of potentiometer 1 out of 100          
    float pot2=100;        //initiliation of potentiometer 2 out of 100           
    float pot3=100;        //initiliation of potentiometer 3 out of 100         
    float pot4=100;        //initiliation of potentiometer 4 out of 100    
    
    float p1OffSet=0;       //pressure offset of channel 1
    float p2OffSet=0;       //pressure offset of channel 2
    float p3OffSet=0;       //pressure offset of channel 3
    float p4OffSet=0;       //pressure offset of channel 4

    float p1;               //pressure sensor readings from channel 1
    float p2;               //pressure sensor readings from channel 2
    float p3;               //pressure sensor readings from channel 3
    float p4;               //pressure sensor readings from channel 4
    
    int N=255;              //PWM period divided into 255 segments
    float DL = 2/N;         //Delay for a sampling frequency of ~125kHz

    int bluetooth=0;        //switch to initiate wireless control
    int bluetoothSetup=0;   
    int PID=0;              //Switch to initiate PID control

//PID constants
//////////////////////////////////////////////////////////
int kp = 72;   int ki = 5;   int kd = 8;
//////////////////////////////////////////////////////////
int PID_p1 = 0;    int PID_i1 = 0;    int PID_d1 = 0;
float last_kp1 = 0;
float last_ki1 = 0;
float last_kd1 = 0;
int PID_p2 = 0;    int PID_i2 = 0;    int PID_d2 = 0;
float last_kp2 = 0;
float last_ki2 = 0;
float last_kd2 = 0;
int PID_p3 = 0;    int PID_i3 = 0;    int PID_d3 = 0;
float last_kp3 = 0;
float last_ki3 = 0;
float last_kd3 = 0;
int PID_p4 = 0;    int PID_i4 = 0;    int PID_d4 = 0;
float last_kp4 = 0;
float last_ki4 = 0;
float last_kd4 = 0;

float set_pressure1, set_pressure2, set_pressure3, set_pressure4;
float PID_error1 = 0, PID_error2 = 0, PID_error3 = 0, PID_error4 = 0;
float previous_error1 = 0,previous_error2 = 0,previous_error3 = 0,previous_error4 = 0 ;
float elapsedTime, Time, timePrev;

//This code is run only once at the start
  void setup() {
//LCD Setup  
  Wire.begin();
  Wire.beginTransmission(0x27);
  lcd.begin(16, 2); // initialize the lcd

//switches setup
  pinMode(7, INPUT);  //bluetooth switch
  pinMode(8, INPUT);  //PID switch
  
// SET UP PWMs through timer interrupts
  pinMode(50, INPUT);
  pinMode(51, INPUT);
  pinMode(52, INPUT);
  pinMode(53, INPUT);

  // output pins for valve PWM
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  int eightOnes = 255;  // this is 11111111 in binary
  TCCR3A &= ~eightOnes;   // this operation (AND plus NOT), set the eight bits in TCCR registers to 0 
  TCCR3B &= ~eightOnes;
  TCCR4A &= ~eightOnes;
  TCCR4B &= ~eightOnes;

  // set waveform generation to frequency and phase correct, non-inverting PWM output
  TCCR3A = _BV(COM3A1);
  TCCR3B = _BV(WGM33) | _BV(CS32);
  
  TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1);
  TCCR4B = _BV(WGM43) | _BV(CS42);

  //initialise duty cycle to zero for calibration
  OCR3A=0;
  OCR3B=0;
  OCR4A=0;
  OCR4B=0;

//Calculate offset for calibration
   CalcP1OffSet();
   CalcP2OffSet();
   CalcP3OffSet();
   CalcP4OffSet();

  Serial.begin(9600);

 }
  


// NEEDS TO BE IN LOOP BECAUSE pot1 IS CONTINUOUSLY CHANGING/UPDATING
void loop() {
  
  //Check whether Bluetooth mode and/or PID mode is on
  bluetooth = digitalRead(7);
  PID = digitalRead(8);

  //Code for using potentiometers (knob)
  if(bluetooth==0){
                      // READING FROM POTENTIOMETER
                         pot1 = (analogRead(A4))*(100.0/1024.0);
                         pot2 = (analogRead(A5))*(100.0/1024.0);
                         pot3 = (analogRead(A6))*(100.0/1024.0);
                         pot4 = (analogRead(A7))*(100.0/1024.0);
                          
                      //CHANGE THE DUTY CYCLE OF PWMs
                          OCR3A=(255*(pot3/100))/1;   //duty cycle adjustment (3 from left - D6)
                          OCR4A=(255*(pot2/100))/1;   //duty cycle adjustment (2  from left - D9)
                          OCR3B=(255*(pot4/100))/1;   //duty cycle adjustment (4 from left - D5)
                          OCR4B=(255*(pot1/100))/1;   //duty cycle adjustment (1  from left - D10)
                      
                      ////PRESSURE SENSOR READINGS
                         p1=CalcP1();
                         p2=CalcP2();
                         p3=CalcP3();
                         p4=CalcP4();

                      //ensure lowest pressure point is zero
                       SetNegZero();

    //round pressure to closest integer
    int pdisplay1=p1/1;
    int pdisplay2=p2/1;
    int pdisplay3=p3/1;
    int pdisplay4=p4/1;

    //Display pressure on LCD screen
    lcd.setBacklight(255);
    lcd.clear();
    lcd.print(pdisplay1);
    lcd.display();
    lcd.setCursor(10, 0);
    lcd.print(pdisplay2);
    lcd.display();

    lcd.setCursor(0, 2);
    lcd.print(pdisplay3);
    lcd.display();
    lcd.setCursor(10, 2);
    lcd.print(pdisplay4);
    lcd.display();
    
    //If PID control is initiated
    if(PID==1)
    {
      //lock the set temperature (after switch detection)
      set_pressure1=pdisplay1;
      set_pressure2=pdisplay2;
      set_pressure3=pdisplay3;
      set_pressure4=pdisplay4;
       
       //we calculate the error between the setpoint and the real value
        PID_error1 = set_pressure1 - pdisplay1;
        PID_error2 = set_pressure2 - pdisplay2;
        PID_error3 = set_pressure3 - pdisplay3;
        PID_error4 = set_pressure4 - pdisplay4;

        //Calculate the P value
        PID_p1 = 0.01*kp * PID_error1;
        PID_p2 = 0.01*kp * PID_error2;
        PID_p3 = 0.01*kp * PID_error3;
        PID_p4 = 0.01*kp * PID_error4;
        //Calculate the I value
        PID_i1 = 0.01*PID_i1 + (ki * PID_error1);
        PID_i2 = 0.01*PID_i2 + (ki * PID_error2);
        PID_i3 = 0.01*PID_i3 + (ki * PID_error3);
        PID_i4 = 0.01*PID_i4 + (ki * PID_error4);
        //For D value, we need real time to calculate speed change rate
          timePrev = Time;                            // the previous time is stored before the actual time read
          Time = millis();                            // actual time read
          elapsedTime = (Time - timePrev) / 1000; 
          //calculate the D calue
          PID_d1 = 0.01*kd*((PID_error1 - previous_error1)/elapsedTime);
          PID_d2 = 0.01*kd*((PID_error2 - previous_error2)/elapsedTime);
          PID_d3 = 0.01*kd*((PID_error3 - previous_error3)/elapsedTime);
          PID_d4 = 0.01*kd*((PID_error4 - previous_error4)/elapsedTime);
          //Final total PID value is the sum of P + I + D
          float PID_value1 = PID_p1 + PID_i1 + PID_d1;
          float PID_value2 = PID_p2 + PID_i2 + PID_d2;
          float PID_value3 = PID_p3 + PID_i3 + PID_d3;
          float PID_value4 = PID_p4 + PID_i4 + PID_d4;
        
          //PWM range between 0 and 255
          if(PID_value1 < 0)
          {    PID_value1 = 0;    }
          if(PID_value1 > 255)  
          {    PID_value1 = 255;  }
          if(PID_value2 < 0)
          {    PID_value2 = 0;    }
          if(PID_value2 > 255)  
          {    PID_value2 = 255;  }
                    if(PID_value2 < 0)
          {    PID_value3 = 0;    }
          if(PID_value3 > 255)  
          {    PID_value3 = 255;  }
                    if(PID_value3 < 0)
          {    PID_value3 = 0;    }
          if(PID_value4 > 255)  
          {    PID_value2 = 255;  }
          if(PID_value4 < 0)
          {    PID_value4 = 0;    }

          //To make adjustments in duty cycle based on PID calculations
          OCR4B=255-PID_value1;
          OCR4A=255-PID_value2;
          OCR3A=255-PID_value3;
          OCR3B=255-PID_value4; 
        
          previous_error1 = PID_error1;
          previous_error2 = PID_error2;
          previous_error3 = PID_error3;
          previous_error4 = PID_error4;  
          delay(14);
    }
    
    delay(500);
    }

    //If wireless control is initiated
        else if(bluetooth==1)
        {
           if(bluetoothSetup==0)
           {
                   SerialBLE.begin(9600);
                    Blynk.begin(SerialBLE, auth);
                    bluetoothSetup=1;
           }
          
          OCR3A = (255*(pot3/100))/1;    //duty cycle adjustment (3 from left - D6)
          OCR4A=(255*(pot2/100))/1;   //duty cycle adjustment (2  from left - D9)
          OCR3B=(255*(pot4/100))/1;   //duty cycle adjustment (4 from left - D5)
          OCR4B=(255*(pot1/100))/1;   //duty cycle adjustment (1  from left - D10)

           Blynk.run();
        }
}

// pressure sensor has high frequency, but output fed is not as responsive to the high frequyency - average

void CalcP1OffSet()
{
      float Vin;
      float Vave;
      float Vout;
      
      for(int jj=0;jj<10;jj++){
  
            for(int i=0;i<N;i++){
                     Vin= (analogRead(A0))*(Vcc/1024.0);  //reads pressure meter (feedback) - a3 = pressure meter (10 but of quantisation - need to quantise Vcc from analogue to digital)
                     Vave=(Vave+Vin);
                    delay(DL);
            }
            Vave=Vave/N;
             Vout=+Vave;
            delay(2);
    }
  
    
   p1OffSet= (((Vout/Vcc)-0.04)/(0.0012858));
}

void CalcP2OffSet()
{
      float Vin;
      float Vave;
      float Vout;
      
      for(int jj=0;jj<10;jj++){
  
            for(int i=0;i<N;i++){
                     Vin= (analogRead(A1))*(Vcc/1024.0);  //reads pressure meter (feedback) - a3 = pressure meter (10 but of quantisation - need to quantise Vcc from analogue to digital)
                     Vave=(Vave+Vin);
                    delay(DL);
            }
            Vave=Vave/N;
             Vout=+Vave;
            delay(2);
    }
  
    
   p2OffSet= (((Vout/Vcc)-0.04)/(0.0012858));
}

void CalcP3OffSet()
{
      float Vin;
      float Vave;
      float Vout;
      
      for(int jj=0;jj<10;jj++){
  
            for(int i=0;i<N;i++){
                     Vin= (analogRead(A2))*(Vcc/1024.0);  //reads pressure meter (feedback) - a3 = pressure meter (10 but of quantisation - need to quantise Vcc from analogue to digital)
                     Vave=(Vave+Vin);
                    delay(DL);
            }
            Vave=Vave/N;
             Vout=+Vave;
            delay(2);
    }
  
    
   p3OffSet= (((Vout/Vcc)-0.04)/(0.0012858));
}


void CalcP4OffSet()
{
      float Vin;
      float Vave;
      float Vout;
      
      for(int jj=0;jj<10;jj++){
  
            for(int i=0;i<N;i++){
                     Vin= (analogRead(A3))*(Vcc/1024.0);  //reads pressure meter (feedback) - a3 = pressure meter (10 but of quantisation - need to quantise Vcc from analogue to digital)
                     Vave=(Vave+Vin);
                    delay(DL);
            }
            Vave=Vave/N;
             Vout=+Vave;
            delay(2);
    }
  
    
   p4OffSet= (((Vout/Vcc)-0.04)/(0.0012858));
}

float CalcP1()
{
      float Vin;
      float Vave;
      float Vout;
      float p1;
      
      for(int jj=0;jj<10;jj++){
  
            for(int i=0;i<N;i++){
                     Vin= (analogRead(A0))*(Vcc/1024.0);  //reads pressure meter (feedback) - a3 = pressure meter (10 but of quantisation - need to quantise Vcc from analogue to digital)
                     Vave=(Vave+Vin);
                    delay(DL);
            }
            Vave=Vave/N;
             Vout=+Vave;
            delay(2);
    }
  
    
    p1= (((Vout/Vcc)-0.04)/(0.0012858))- p1OffSet;
    return p1;
}

float CalcP2()
{
      float Vin;
      float Vave;
      float Vout;
      float p2;
      
      for(int jj=0;jj<10;jj++){
  
            for(int i=0;i<N;i++){
                     Vin= (analogRead(A1))*(Vcc/1024.0);  //reads pressure meter (feedback) - a3 = pressure meter (10 but of quantisation - need to quantise Vcc from analogue to digital)
                     Vave=(Vave+Vin);
                    delay(DL);
            }
            Vave=Vave/N;
             Vout=+Vave;
            delay(2);
    }
  
    
    p2= (((Vout/Vcc)-0.04)/(0.0012858))- p2OffSet;
    return p2;
}

float CalcP3()
{
      float Vin;
      float Vave;
      float Vout;
      float p3;
      
      for(int jj=0;jj<10;jj++){
  
            for(int i=0;i<N;i++){
                     Vin= (analogRead(A2))*(Vcc/1024.0);  //reads pressure meter (feedback) - a3 = pressure meter (10 but of quantisation - need to quantise Vcc from analogue to digital)
                     Vave=(Vave+Vin);
                    delay(DL);
            }
            Vave=Vave/N;
             Vout=+Vave;
            delay(2);
    }
  
    
    p3= (((Vout/Vcc)-0.04)/(0.0012858))- p3OffSet;
    return p3;
}

float CalcP4()
{
      float Vin;
      float Vave;
      float Vout;
      float p4;
      
      for(int jj=0;jj<10;jj++){
  
            for(int i=0;i<N;i++){
                     Vin= (analogRead(A3))*(Vcc/1024.0);  //reads pressure meter (feedback) - a3 = pressure meter (10 but of quantisation - need to quantise Vcc from analogue to digital)
                     Vave=(Vave+Vin);
                    delay(DL);
            }
            Vave=Vave/N;
             Vout=+Vave;
            delay(2);
    }
  
    
    p4= (((Vout/Vcc)-0.04)/(0.0012858))- p4OffSet;
    return p4;
}

void SetNegZero(){
    if(p1<0)
  {
    p1=0;
  }
    if(p2<0)
  {
    p2=0;
  }
    if(p3<0)
  {
    p3=0;
  }
    if(p4<0)
  {
    p4=0;
  }
}

// This function will be called every time Slider Widget
// in Blynk app writes values to the Virtual Pin V1
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
