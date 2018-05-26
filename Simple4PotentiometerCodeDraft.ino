#include <SoftwareSerial.h>


    const int Vcc=5.0;
    float pot1=100;                   //change this input (D6)
    float pot2=100;                   //change this input (D9)
    float pot3=100;                 //change this input (D5)
    float pot4=100;                 //change this input (D10)
    int DutyCyAd1;
    int DutyCyAd2;
    int DutyCyAd3;
    int DutyCyAd4;
    float p1OffSet=0;
    float p2OffSet=0;
        float p3OffSet=0;
                float p4OffSet=0;

  void setup() {
// SET UP PWMs  
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


//OFFSET STUFF
  float Vave1=0;
  float Vout1=0;
  float Vave2=0;
  float Vout2=0;
    float Vave3=0;
  float Vout3=0;
      float Vave4=0;
  float Vout4=0;
  
  int N=255; //N=2/0.25;
  float DL = 2/N;


  for(int jj=0;jj<10;jj++){

          for(int i=0;i<N;i++){
                  float Vin1= (analogRead(A2))*(Vcc/1024.0);  //reads pressure meter (feedback) - a3 = pressure meter (10 but of quantisation - need to quantise Vcc from analogue to digital)
                  Vave1=(Vave1+Vin1);
                  delay(DL);
          }
          Vave1=Vave1/N;
          Vout1=+Vave1;
          delay(2);
  }

  
 p1OffSet= (((Vout1/Vcc)-0.04)/(0.0012858));


  for(int jj=0;jj<10;jj++){

            for(int i=0;i<N;i++){
                    float Vin2= (analogRead(A3))*(Vcc/1024.0);  //reads pressure meter (feedback) - a3 = pressure meter (10 but of quantisation - need to quantise Vcc from analogue to digital)
                    Vave2=(Vave2+Vin2);
                    delay(DL);
            }
            Vave2=Vave2/N;
            Vout2=+Vave2;
          delay(2);
  }

 p2OffSet= (((Vout2/Vcc)-0.04)/(0.0012858));

  for(int jj=0;jj<10;jj++){

            for(int i=0;i<N;i++){
                    float Vin3= (analogRead(A5))*(Vcc/1024.0);  //reads pressure meter (feedback) - a3 = pressure meter (10 but of quantisation - need to quantise Vcc from analogue to digital)
                    Vave3=(Vave3+Vin3);
                    delay(DL);
            }
            Vave3=Vave3/N;
            Vout3=+Vave3;
          delay(2);
  }

 p3OffSet= (((Vout3/Vcc)-0.04)/(0.0012858));


   for(int jj=0;jj<10;jj++){

            for(int i=0;i<N;i++){
                    float Vin4= (analogRead(A4))*(Vcc/1024.0);  //reads pressure meter (feedback) - a3 = pressure meter (10 but of quantisation - need to quantise Vcc from analogue to digital)
                    Vave4=(Vave4+Vin4);
                    delay(DL);
            }
            Vave4=Vave4/N;
            Vout4=+Vave4;
          delay(2);
  }

 p4OffSet= (((Vout4/Vcc)-0.04)/(0.0012858));

  Serial.begin(9600);
 }
  


// NEEDS TO BE IN LOOP BECAUSE pot1 IS CONTINUOUSLY CHANGING/UPDATING
void loop() {
// READING FROM POTENTIOMETER
    //pot1 = (analogRead(A0))*(100.0/1024.0);
    //pot2 = (analogRead(A1))*(100.0/1024.0);
//    THIS IS WHERE YOU CAN CHANGE THE DUTY CYCLE
    DutyCyAd1=(255*(pot3/100))/1;
    DutyCyAd2=(255*(pot2/100))/1;
    DutyCyAd3=(255*(pot4/100))/1;
    DutyCyAd4=(255*(pot1/100))/1;
    
    
    OCR0A = DutyCyAd1;    //duty cycle adjustment (3 from left - D6)
    OCR1A = DutyCyAd2;    //duty cycle adjustment (2  from left - D9)
    OCR0B = DutyCyAd3;    //duty cycle adjustment (4 from left - D5)
    OCR1B = DutyCyAd4;    //duty cycle adjustment (1  from left - D10)




//PRESSURE SENSOR READINGS STUFF
  float Vave1=0;
  float Vout1=0;
  float Vave2=0;
  float Vout2=0;
    float Vave3=0;
  float Vout3=0;
      float Vave4=0;
  float Vout4=0;
  
  int N=255; //N=2/0.25;
  float DL = 2/N;


  for(int jj=0;jj<10;jj++){

          for(int i=0;i<N;i++){
                  float Vin1= (analogRead(A2))*(Vcc/1024.0);  //reads pressure meter (feedback) - a3 = pressure meter (10 but of quantisation - need to quantise Vcc from analogue to digital)
                  Vave1=(Vave1+Vin1);
                  delay(DL);
          }
          Vave1=Vave1/N;
          Vout1=+Vave1;
          delay(2);
  }

  
float p1= (((Vout1/Vcc)-0.04)/(0.0012858))- p1OffSet;//+2.25 + 0.8 -3.75 +2.76 -3.5-130+25.5-0.28;  //function from data sheet (255)

// SERIAL PRINTING STUFF
  Serial.print("Pot1,");
  Serial.println(p1);
//  Serial.print(",");
//  Serial.println(p1); 

  for(int jj=0;jj<10;jj++){

            for(int i=0;i<N;i++){
                    float Vin2= (analogRead(A3))*(Vcc/1024.0);  //reads pressure meter (feedback) - a3 = pressure meter (10 but of quantisation - need to quantise Vcc from analogue to digital)
                    Vave2=(Vave2+Vin2);
                    delay(DL);
            }
            Vave2=Vave2/N;
            Vout2=+Vave2;
          delay(2);
  }

float p2= (((Vout2/Vcc)-0.04)/(0.0012858))- p2OffSet;//+2.25 + 0.8 -3.75 +2.76 -3.5-130+25.5+0.8;  //function from data sheet (255)
// SERIAL PRINTING STUFF
    Serial.print("Pot2,");
  Serial.println(p2);


  for(int jj=0;jj<10;jj++){

            for(int i=0;i<N;i++){
                    float Vin3= (analogRead(A5))*(Vcc/1024.0);  //reads pressure meter (feedback) - a3 = pressure meter (10 but of quantisation - need to quantise Vcc from analogue to digital)
                    Vave3=(Vave3+Vin3);
                    delay(DL);
            }
            Vave3=Vave3/N;
            Vout3=+Vave3;
          delay(2);
  }


float p3= (((Vout3/Vcc)-0.04)/(0.0012858))- p3OffSet;//+2.25 + 0.8 -3.75 +2.76 -3.5-130+25.5+0.8;  //function from data sheet (255)
// SERIAL PRINTING STUFF
    Serial.print("Pot3,");
  Serial.println(p3);

  for(int jj=0;jj<10;jj++){

            for(int i=0;i<N;i++){
                    float Vin4= (analogRead(A4))*(Vcc/1024.0);  //reads pressure meter (feedback) - a3 = pressure meter (10 but of quantisation - need to quantise Vcc from analogue to digital)
                    Vave4=(Vave4+Vin4);
                    delay(DL);
            }
            Vave4=Vave4/N;
            Vout4=+Vave4;
          delay(2);
  }

  float p4= (((Vout4/Vcc)-0.04)/(0.0012858))- p4OffSet;//+2.25 + 0.8 -3.75 +2.76 -3.5-130+25.5+0.8;  //function from data sheet (255)
// SERIAL PRINTING STUFF
    Serial.print("Pot4,");
  Serial.println(p4);
delay(120);
}

// pressure sensor has high frequency, but output fed is not as responsive to the high frequyency - average
