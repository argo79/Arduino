#include <TrueRandom.h>

/*
 Fade

 This example shows how to fade an LED on pin 9
 using the analogWrite() function.

 This example code is in the public domain.

 */

/************************Hardware Related Macros************************************/
#define         MQ_PIN                       (0)     //define which analog input channel you are going to use
#define         RL_VALUE                     (20)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (4.5)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                     //which is derived from the chart in datasheet

/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
                                                     //normal operation

/**********************Application Related Macros**********************************/
#define         GAS_LPG                      (0)
/*****************************Globals***********************************************/
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms

int brightness = 0;    // how bright the LED is
int fadeAmount = 10;    // how many points to fade the LED by
int triggerPort = 7;
int echoPort = 8;


void setup()  { 
  // declare pin 9 to be an output:
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode( triggerPort, OUTPUT );
  pinMode( echoPort, INPUT );
  Ro = 265;
} 

void loop()  { 
  // set the brightness of pin 9:
  analogWrite(10, brightness);    
  digitalWrite(11, HIGH);
  // change the brightness for next time through the loop:
  brightness = brightness + fadeAmount;

  // reverse the direction of the fading at the ends of the fade: 
  if (brightness == 0 || brightness == 255) {
    fadeAmount = -fadeAmount ; 
  }     
  // wait for 30 milliseconds to see the dimming effect    
  
  delay(50); 
  
  Serial.begin(230400);
  Serial.print("I threw a random die and got ");
  Serial.print(random(1,99));
  Serial.print(". Then I threw a TrueRandom die and got ");
  Serial.print(TrueRandom.random(1,99));

//porta bassa l'uscita del trigger
digitalWrite( triggerPort, LOW );
//invia un impulso di 10microsec su trigger
digitalWrite( triggerPort, HIGH );
delayMicroseconds( 10 );
digitalWrite( triggerPort, LOW );
long duration = pulseIn( echoPort, HIGH );
long r = 0.034 * duration / 2;
Serial.print( " durata: " );
Serial.print( duration );
Serial.print( " , " );
Serial.print( "distanza: " );
 
//dopo 38ms è fuori dalla portata del sensore
if( duration > 38000 ) Serial.println( "fuori portata");
else { Serial.print( r ); Serial.print( "cm " );}

 float ratio1 = (MQRead(MQ_PIN)/Ro);
 float ppm1 = pow(((ratio1)/12.414199), -2.768556);
   Serial.print(ratio1);
   Serial.print("\t"); 
   Serial.print (analogRead(MQ_PIN));
   Serial.print("\t"); 
   Serial.print(ppm1);
   Serial.println(" PPM");
//aspetta 1.5 secondi
delay( 1500 );
  
 digitalWrite(11, LOW);  
 delay(1000);                         
}


/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/ 
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
float MQRead(int mq_pin)
{
  int i;
  float rs=0;

  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs/READ_SAMPLE_TIMES;

  return rs;  
}
