/*
* Telecomando a radiofrequenza che utilizza arduino nano e il modulo nRF24l01
* Arg0 15 12 2024
*
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/

*   *   ////////////////////// RF24 Adaptor or use this pinout :) /
    __________________________________
    || 3V  ||  10 ||   11  ||  NC   ||           NC - not connected.
    || VCC-||-CSN-||- MOSI-||- IRQ  ||
    ||_____||_____||_______||_______||
    || GND-||-CE -||- SCK -||- MISO ||
    || GND ||  9  ||  13   ||   12  ||
    ||_____||_____||_______||_______||

*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Per display:
#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED

// Modulo nRF24l01 
RF24 radio(48, 49); // CE, CSN

const byte address[6] = "00001";

const int pot6Pin = 10;        // define R6
const int pot5Pin = 9;        // define R1
const int led1Pin = 2;        //era il 6 define pin for LED1 which is close to NRF24L01 and used to indicate the state of NRF24L01
const int led2Pin = 4;        // define pin for LED2 which is the mode is displayed in the robotic arm remote control mode  
const int led3Pin = 6;        // define pin for LED3 which is the mode is displayed in the robotic arm auto mode
const int led4Pin = 8;        //era il 6 define pin for LED1 which is close to NRF24L01 and used to indicate the state of NRF24L01

const int APin = 9;           // define pin for D2  mode robot/braccio
const int BPin = 3;           // define pin for D3  fanali on/off
const int CPin = 7;           // define pin for D4  abbaglianti on/off
const int DPin = 5;           // define pin for D5  camera on/off

bool statoFanale=0;        // stato del fanale SPENTO
const int u1XPin = 0;      // define pin for direction X of joystick U1
const int u1YPin = 1;      // define pin for direction Y of joystick U1
const int u2XPin = 7;      // define pin for direction X of joystick U2
const int u2YPin = 6;      // define pin for direction Y of joystick U2

const int u3XPin = 3;      // define pin for direction X of joystick U3
const int u3YPin = 5;      // define pin for direction Y of joystick U3
const int u4XPin = 4;      // define pin for direction X of joystick U4
const int u4YPin = 2;      // define pin for direction Y of joystick U4

uint16_t counter=0;
unsigned long tempoInizio;
int timer=100;


struct MyData 
{
  uint16_t X;   // dx sx  U1X
  uint16_t Y;   // av in  U1Y
  uint16_t Z;   // cam dx sx  U2X
  uint16_t W;   // cam up dw  U2Y
  uint16_t K;   // pot sx pot1
  uint16_t J;   // pot dx pot2

  uint16_t G;   // pot sx U3X
  uint16_t H;   // pot dx U3Y
  uint16_t I;   // pot sx U4X
  uint16_t L;   // pot dx U4Y

  uint8_t A;    // pulsante A
  uint8_t B;    // pulsante B
  uint8_t C;    // pulsante C
  uint8_t D;    // pulsante D
  uint16_t counter;
};

MyData dati;

int mode = 0;
bool statoRover, pinAbba;
bool statoAbba=0;
bool pinFanale=0;

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW,0);
  radio.stopListening();

  pinMode(led1Pin, OUTPUT);           // set led1Pin to output mode
  pinMode(led2Pin, OUTPUT);           // set led2Pin to output mode
  pinMode(led3Pin, OUTPUT);  
  pinMode(led4Pin, OUTPUT);           // set led3Pin to output mode         // set led3Pin to output mode
  pinMode(u1XPin, INPUT_PULLUP);        // set APin to input mode
  pinMode(u1YPin, INPUT_PULLUP);        // set BPin to input mode
  pinMode(u2YPin, INPUT_PULLUP);        // set CPin to input mode
  pinMode(u2XPin, INPUT_PULLUP);        // set DPin to input mode  

  pinMode(u3XPin, INPUT_PULLUP);        // set APin to input mode
  pinMode(u3YPin, INPUT_PULLUP);        // set BPin to input mode
  pinMode(u4YPin, INPUT_PULLUP);        // set CPin to input mode
  pinMode(u4XPin, INPUT_PULLUP);        // set DPin to input mode  

  pinMode(pot5Pin, INPUT_PULLUP);        // set SX pot to input mode
  pinMode(pot6Pin, INPUT_PULLUP);        // set DX pot to input mode

  pinMode(APin, INPUT_PULLUP);        // set APin to input mode
  pinMode(BPin, INPUT_PULLUP);        // set BPin to input mode
  pinMode(CPin, INPUT_PULLUP);        // set CPin to input mode
  pinMode(DPin, INPUT_PULLUP);        // set DPin to input mode  
  
  digitalWrite(led1Pin,HIGH);
  delay(250); 
  digitalWrite(led2Pin,HIGH);
  delay(250); 
  digitalWrite(led3Pin,HIGH);
  delay(250); 
  digitalWrite(led3Pin,LOW);
  delay(250); 
  digitalWrite(led2Pin,LOW);
  delay(250);   
  digitalWrite(led1Pin,LOW);   
  digitalWrite(led4Pin,HIGH);
  delay(500); 
  digitalWrite(led4Pin,LOW); 

  u8g2.begin();
  u8g2.setFlipMode(1);

  tempoInizio=millis();
}

void loop() {

  leggiMisure();
  if (millis()>tempoInizio+timer) {
    mostraValori();
    tempoInizio=millis();
  }  
  trasmetti();
  //delay(500);
  if(mode==0){//LED display status
    digitalWrite(led2Pin,LOW);
    //digitalWrite(led3Pin,LOW);
  }
  if(mode==1){//LED display status
      digitalWrite(led2Pin,HIGH);
      //digitalWrite(led3Pin,HIGH);
  }
  if(statoAbba==1) {
    digitalWrite(led3Pin,HIGH);
  } 
  else digitalWrite(led3Pin,LOW);
}

void leggiMisure() {

  dati.X = 1023-(analogRead(u1XPin));
  //delay(1);
  dati.Y = analogRead(u1YPin);
  //delay(1);
  dati.Z = 1023-(analogRead(u2XPin));
  dati.W = analogRead(u2YPin);
  dati.K = analogRead(pot5Pin);
  dati.J = analogRead(pot6Pin); 

  dati.G = analogRead(u3XPin);
  //delay(1);
  dati.H = analogRead(u3YPin);
  //delay(1);
  dati.I = 1023-(analogRead(u4XPin));
  dati.L = analogRead(u4YPin);


  statoRover = digitalRead(APin); 
  delay(10); 
  if (statoRover==0 && mode==1) {
    dati.A=0;
    mode = 0;
  }
  else if (statoRover==0 && mode==0) {
    dati.A=1;
    mode = 1;
  }
  //delay(1);
  //Serial.println(statoRover);
  //Serial.println(mode);

  pinFanale = digitalRead(BPin);
  delay(10); 
  if (pinFanale==0 && statoFanale==0) {
    dati.B=0; // accendo le luci
    statoFanale=1;
  }
  else if (pinFanale==0 && statoFanale==1) {
    dati.B=1;  // spengo lev luci
    statoFanale=0;
  }

  pinAbba=digitalRead(CPin);
  delay(10); 
  if (pinAbba==0 && statoAbba==0) {
    dati.C=0; // accendo gli abbaglianti
    statoAbba=1;
  }
  else if (pinAbba==0 && statoAbba==1) {
    dati.C=1;  // spengo gli abbaglianti
    statoAbba=0;
  }

  // Attivo la camera!
  dati.D = digitalRead(DPin);  
  delay(10); 

  dati.counter = counter++;
  
  //delay(1);
}


void trasmetti() {    
  // send array data. If the sending succeeds, open signal LED
  if (radio.write(&dati, sizeof(MyData))) {
    digitalWrite(led1Pin,HIGH);
    //Serial.println("Ok, mandato");
    //Serial.print("Contatore: ");
    //Serial.println(sizeof(MyData));
  }  
  else {
    Serial.println("Non ho scritto un casso...");
  }
  // delay for a period of time, then turn off the signal LED for next sending
  delay(1);
  digitalWrite(led1Pin,LOW);  
  //delay(1);
}

void mostraValori()
{
  u8g2.clearBuffer();  

  u8g2.setFontMode(1);
  u8g2.setFont(u8g2_font_luRS08_tf);		
  //u8g2.setFontMode(0);
  //u8g2.setFont(u8g2_font_helvR08_tf);		
  u8g2.setCursor(0,8);
  u8g2.print("Modalita': ");
  if(mode==0) {
    u8g2.print(" Libero");
  }
  else u8g2.print(" Evita");
  //u8g2.sendBuffer();
  if (statoFanale==1) {
    u8g2.print(" **");
  }
  else u8g2.print(" ");

  u8g2.setCursor(0,19);
  u8g2.print("JP1 X: ");
  u8g2.print(dati.X);
  u8g2.setCursor(64,19);
  u8g2.print("JP1 Y: ");
  u8g2.print(dati.Y);

  u8g2.setCursor(0, 30);
  u8g2.print("JP2 X: ");
  u8g2.print(dati.Z);
  u8g2.setCursor(64, 30);
  u8g2.print("JP2 Y: ");
  u8g2.print(dati.W);

  u8g2.sendBuffer();
  //delay(INFO_SCREEN_DELAY);
  
  u8g2.setFontMode(0);
  u8g2.setFont(u8g2_font_cu12_hr);		
  u8g2.clearBuffer();					// clear the internal memory once
}

