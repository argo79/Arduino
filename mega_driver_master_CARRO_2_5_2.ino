/*
 * Driver mega per auto robot
 * 
 * versione 2.5.1
 *  
 * arg0
 * 
 * 15 12 2024
 *   
 */

#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <time.h>
#include <EEPROM.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Ultrasonic.h>


// SENSORI VARI

const int triggerPortSx=12;
const int echoPortSx=13;
const int servoRotorePin=44;

Servo ServoRotore; //  Il nome del servo è ServoRotore
int angoloPartenza=85;
// INFRAROSSI SX E DX
const int FIRSX=43;
const int FIRDX=42;

// MOTORI: 5,6,7,8,9,10 !!!
// MOTOR ONE
const int enDx=3; // (10)Accendi PWM
const int dxIn=32;  //SXAvanti
const int dxAv=33;  //SXIndietro
// MOTOR TWO
const int enSx=4;  // (11)Accendi PWM
const int sxAv=31;  //DXAvanti
const int sxIn=30;  //DXIndietro

// LUCI
const int pinLuci=2;
const int pinRed=11;
const int pinCam=23;

//const int pinBuiltin=13;
int forzaLuci=16;
boolean luceB,luceR;

// pin joystick
//const int xPin=6;
//const int yPin=7;

const int pinBraccio=34;

// Soglie valori joystick
const int xSogliaUp=350;
const int xSogliaDw=750;
const int ySogliaSx=750;
const int ySogliaRx=350;

// Limiti massimi singoli servo motori
const int baseMax=180;
const int baseMin=0;
const int unoMax=180;
const int unoMin=50;
const int dueMax=180;
const int dueMin=0;
const int treMax=180;
const int treMin=0;
const int rotoMax=180;
const int rotoMin=0;
const int pinzaMax=64;
const int pinzaMin=0;

int xValue,yValue,zValue,wValue,kValue,jValue,gValue,hValue,iValue,lValue;
int basePos=175;
int unoPos=100;
int duePos=0;
int trePos=45;
int rotoPos=95;
int pinzaPos=0;

int potenzaMotSx;
int potenzaMotDx;
int tSvolta=15;
bool ultimaSvolta;

int detectionSX;
int detectionDX;
int maxDiff=280;

String distanzaS;
String detectionDXS;
String detectionSXS;
String serieVal;

float distanzaTot;
int rip=3; // Ripetizioni scansioni sensore ultrasuoni. (media di rip)
int distanza;
float distanzaM;
float durata;

bool statoCamera=0;

// RADIO NRF24L01 

RF24 radio(48, 49);                // CE CSN pins - define the object to control NRF24L01
byte address[6] = "00001";      // define communication address which should correspond to remote control
//int dati[9];  // define array used to save the communication data
int mode[1];
int automatic = 1;
int x=0;

int YlimitUP=573;
int YlimitDW=450;
int XlimitDX=450;
int XlimitSX=573;
int velAv, velIn, velSX, velDX;
int velMin=100;
int velMax=255;
int velMinSX=80;
int velMaxSX=250;
int velMinDX=80;
int velMaxDX=250;

unsigned long inizioMisura,inizioMisuraN;
int ritardoMisura=1000;
int TXdelay=5;    // frequenza trasmissione
int TXdelayN=10;    // frequenza trasmissione
int ritardo=2;
bool statoBraccio;
bool statoPinBraccio=0;
bool statoEvita=1;
bool micro=0;

// creo la struttura che conterrà i valori da ricevere
struct MyData 
{
  uint16_t X;   // dx sx  U1X
  uint16_t Y;   // av in  U1Y
  uint16_t Z;   // cam dx sx  U2X
  uint16_t W;   // cam up dw  U2Y
  uint16_t K;   // pot sx
  uint16_t J;   // pot dx

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

Servo servoBase;
Servo servoUno;
Servo servoDue;
Servo servoTre;
Servo servoRoto;
Servo servoPinza;

Ultrasonic ultrasonic(12, 13); //(trig, echo)


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(enSx, OUTPUT);
  pinMode(enDx, OUTPUT);
  analogWrite(enSx,0);
  analogWrite(enDx,0);
  pinMode(sxAv, OUTPUT);
  pinMode(sxIn, OUTPUT);
  pinMode(dxAv, OUTPUT);
  pinMode(dxIn, OUTPUT);  
  //pinMode(triggerPortSx, OUTPUT);
  //pinMode(echoPortSx, INPUT_PULLUP);
  pinMode(FIRSX, INPUT);
  pinMode(FIRDX, INPUT);
  pinMode(pinLuci, OUTPUT);
  pinMode(pinRed, OUTPUT);
  //pinMode(pinBuiltin, OUTPUT);
  pinMode(pinBraccio, OUTPUT);
  pinMode(pinCam, OUTPUT);

  radio.begin();                      // initialize RF24
  //radio.setRetries(0, 15);            // set retries times
  radio.openReadingPipe(0, address);// open delivery channel
  radio.setPALevel(RF24_PA_MAX,1);      // set power
  //radio.setDataRate( RF24_250KBPS );   
//  radio.setDataRate(RF24_1MBPS);
  
  radio.startListening();             // start monitoring
  //Serial.println("Radio avviata!");
  //pinMode(controlLed,OUTPUT);
  
  //Serial.println("Si parte!!!");
  inizioMisura=millis();
  luceB=0;
  luceR=0;
  analogWrite(pinLuci,luceB);
  analogWrite(pinRed,luceR);
  digitalWrite(pinBraccio,1);  
  delay(100);

  servoBase.attach(5);
  servoBase.write(basePos);
  delay(900);
  servoUno.attach(6);  
  servoUno.write(unoPos);
  delay(1000);
  servoDue.attach(7);
  servoDue.write(duePos);
  delay(1000);
  servoTre.attach(8);
  servoTre.write(trePos);
  delay(1000);
  servoRoto.attach(9);
  servoRoto.write(rotoPos);
  delay(1000);
  servoPinza.attach(10);
  servoPinza.write(pinzaPos);
  delay(1000);
  
  digitalWrite(pinBraccio,0);

  statoBraccio=0;
}


void loop() {
  // put your main code here, to run repeatedly:

  if (millis()>inizioMisura+TXdelay) {
    
    digitalWrite(pinBraccio,1);
    delay(1);
    receiveData();
    //delay(1);    
    delay(1);
    checkQ();
    delay(1);
    moveX();  
    delay(1);
    checkB();
    delay(1);
    braccioRobot();
    inizioMisura=millis();

    /*
    //digitalWrite(pinBuiltin,0);
    if (statoPinBraccio==1) {
      digitalWrite(pinBraccio,1);
      statoPinBraccio=0;
    }
    */

    }
  
    
  if (millis()>inizioMisuraN+TXdelayN) {   
      Serial.print("Distance in cm: ");
      //Serial.println(ultrasonic.distanceRead());
      //inizioMisuraN=millis();
    }  

  if (dati.B==0) {
    micro=1;
    luceB=1;
    luceR=1;  
    //digitalWrite(pinLuci,luceB);
  } 
  else {
    luceB=0;
    luceR=0;  
  }

  if (luceR==1 && luceB==1) {
    analogWrite(pinLuci,forzaLuci);  
    analogWrite(pinRed,dati.J/4);
  }
  else {
    analogWrite(pinRed,0);
    analogWrite(pinLuci,0);
  //neuronMake();
  }
  if (dati.A==0) {
    //statoEvita=0;  
    micro=0;
  }
  else {
    micro=1;
    //statoEvita=1;
  }

  if (dati.C==0) {
    forzaLuci=128;  
  }
  else forzaLuci=16;

  if (dati.D==0) {
    statoCamera=!statoCamera;  
  }
  digitalWrite(pinCam,statoCamera);
}



void receiveData(){
if ( radio.available()) {             // if receive the data
 
    radio.read(&dati,sizeof(dati));  // read data
    
    Serial.println(dati.X);
    Serial.println(dati.Y);
    Serial.println(dati.Z);
    Serial.println(dati.W);    
    Serial.println(dati.K); // potenziometro di sinistra R1
    Serial.println(dati.J); // potenziometro di destra R6

    Serial.println(dati.G);
    Serial.println(dati.H);
    Serial.println(dati.I);
    Serial.println(dati.L);    

    Serial.println(dati.A);
    Serial.println(dati.B);
    Serial.println(dati.C);
    Serial.println(dati.D);
    Serial.println(dati.counter);
    Serial.println(sizeof(dati));
    automatic=0; 
  }     
  else {
    //Serial.println("Nessun telecomando collegato...");
    automatic=1;
  }  
  //Serial.print("Dati ricevuti: ");
  //Serial.println(automatic);   
  //return(automatic);
}



void checkQ() {

  //Serial.println(automatic);
  if (automatic==0) {
    velAv=map(dati.Y,YlimitUP,1023,velMin,velMax);
    velAv=constrain(velAv,velMin,velMax);
    velIn=map(dati.Y,0,YlimitDW,velMax,velMin);
    velIn=constrain(velIn,velMin,velMax);
    velDX=map(dati.X,0,XlimitDX,velMaxDX,velMinDX);
    velDX=constrain(velDX,velMinDX,velMaxDX);
    velSX=map(dati.X,XlimitSX,1023,velMinSX,velMaxSX);
    velSX=constrain(velSX,velMinSX,velMaxSX);
    /*
    Serial.println("Auto");  
    Serial.println(velAv);  
    Serial.println(velIn);  
    Serial.println(velDX);  
    Serial.println(velSX);     
    */     
  }      
   
  else if (automatic==1) {  
    x=2; // Automatico - giroTuristico  
    //Serial.print("Dati -automatic- trasmessi :");
    //Serial.println(x);
    delay(TXdelay);     
  }      
}

void checkB() {

  //Serial.println(automatic);
  
  //  yValue=dati.Y;
  //  xValue=dati.X;
    zValue=dati.Z;
    wValue=dati.W;

    kValue=dati.K;  // pot1
    jValue=dati.J;  // pot2

    gValue=dati.G;
    hValue=dati.H;
    iValue=dati.I;
    lValue=dati.L;


    Serial.println("Braccio");  
  //  Serial.println(yValue);  
  //  Serial.println(xValue);  
  /*
    Serial.println(zValue);  
    Serial.println(wValue);          
    Serial.println(kValue);  
    Serial.println(jValue);  

    Serial.println(gValue);  
    Serial.println(hValue);  
    Serial.println(iValue);  
    Serial.println(lValue);    
*/

}
  ///////////////\\\\\\\\\\\\\\\\
 // CONTROLLO MOVIMENTO MOTORI \\\
///////////////    \\\\\\\\\\\\\\\\

void neuronMake() {
  
  /*
  MDistanza();   
  Serial.println("Distanza: ");
  Serial.println(distanza);
  if (distanza>15) {
    distanzaS="1";
  }
  else {
    distanzaS="0";
  }    
  */
  detectionDX = digitalRead(FIRDX);
  detectionSX = digitalRead(FIRSX);    
  detectionSXS=String(detectionSX);
  detectionDXS=String(detectionDX);  
  Serial.println(detectionSXS);
  //Serial.println(distanzaS);
  Serial.println(detectionDXS);
  serieVal=String(detectionDXS+detectionSXS);
  Serial.println(serieVal);
  //serieVal.toCharArray(serieChar,3);
  //dtostrf(yournumber, TotalStringLength, NumberOfDecimals, TheTargetArray)  
  //Serial.println(serieVal);
  //Serial.println(serieChar);
//  return(serieChar);
  return(serieVal);  
}


void moveX() {

    
    //Serial.print("Distanza frontale: ");
    //Serial.println(distanza);
    if(statoEvita==1){
      evita();
    }

    if(velAv>velMin&&velSX<velMinSX*2&&velDX<velMinDX*2) {    
      avanti();
    }
    else if(velIn>velMin&&velSX<velMinSX*2&&velDX<velMinDX*2) {
      indietro();
    }
    else if(dati.Y>YlimitUP&&dati.X>XlimitSX) {    
      avantiSX();
    }
    else if(dati.Y>YlimitUP&&dati.X<XlimitDX) {
      avantiDX();
    }
    else if(dati.X>XlimitSX&&dati.Y<YlimitUP&&dati.Y>YlimitDW) {
      sinistra();
    }
    else if(dati.X<XlimitDX&&dati.Y<YlimitUP&&dati.Y>YlimitDW) {
      destra();
    }
    else {
      fermo();
    }
}


void fermo()
{
  analogWrite(enSx,0);
  analogWrite(enDx,0);
}

void indietro()
{
  //Serial.print("Vado indietro a: ");  
  //Serial.println(velIn);  
  digitalWrite(sxIn,HIGH);
  digitalWrite(sxAv,LOW);
  digitalWrite(dxIn,HIGH);
  digitalWrite(dxAv,LOW);   
  analogWrite(enSx,velIn);
  analogWrite(enDx,velIn);
  aspetta(tSvolta);
  fermo();  
}
 
void avanti()
{   
    //ServoRotore.write(angoloPartenza);
  //Serial.print("Vado avanti a: ");  
  //Serial.println(velAv);  
  //evita();
  digitalWrite(sxAv,HIGH);
  digitalWrite(sxIn,LOW);
  digitalWrite(dxAv,HIGH);
  digitalWrite(dxIn,LOW);     
  analogWrite(enSx,velAv);
  analogWrite(enDx,velAv);   
  //Serial.println("Fatto!");      
  aspetta(tSvolta);
  fermo();
  /*
  do {
  digitalWrite(sxAv,HIGH);
  digitalWrite(sxIn,LOW);
  digitalWrite(dxAv,HIGH);
  digitalWrite(dxIn,LOW);     
  analogWrite(enSx,veloDritto);
  analogWrite(enDx,veloDritto);
  // Correggi la rotta e la potenza al volo!
  FIRDDist(tSvolta);    
  FIRSDist(tSvolta);
  //acceleraMotori();
  //MDistanza();
  }
  //while (distanza>35); 
  //avvicinamento();    
  */        
}   

void avantiDX()
{   
  /*ServoRotore.write(angoloPartenza);
  Serial.print("Vado avanti a destra: ");  
  Serial.print(velAv);  
  Serial.print(" - ");  
  Serial.println(maxDiff-velDX);  
  //evita();
  Serial.println(velAv);  
  */
  digitalWrite(sxAv,HIGH);
  digitalWrite(sxIn,LOW);
  digitalWrite(dxAv,HIGH);
  digitalWrite(dxIn,LOW);     
  analogWrite(enDx,maxDiff-velDX);
  analogWrite(enSx,velMax);   
  //Serial.println("Fatto!");      
  aspetta(tSvolta);
  fermo();  
}   

void avantiSX()
{   
  /*ServoRotore.write(angoloPartenza);
  Serial.print("Vado avanti a sinistra: ");  
  Serial.print(velAv);  
  Serial.print(" - ");  
  Serial.println(maxDiff-velSX);  
  Serial.println(velAv);  
  //evita();
  */
  digitalWrite(sxAv,HIGH);
  digitalWrite(sxIn,LOW);
  digitalWrite(dxAv,HIGH);
  digitalWrite(dxIn,LOW);     
  analogWrite(enDx,velMax);
  analogWrite(enSx,maxDiff-velSX);   
  //Serial.println("Fatto!");      
  aspetta(tSvolta);
  fermo();   
}   



/*
void avvicinamento() {
  do {
    digitalWrite(sxAv,HIGH);
    digitalWrite(sxIn,LOW);
    digitalWrite(dxAv,HIGH);
    digitalWrite(dxIn,LOW);     
    analogWrite(enSx,veloDritto/2);
    analogWrite(enDx,veloDritto/2);
    aspetta(100);
    FIRDDist(tSvolta);    
    FIRSDist(tSvolta);    
    MDistanza();
  }
  while (distanza>15);
  fermo();  
}
 */

void destra()
{
  //FIRSDist();    
  digitalWrite(sxIn,HIGH);
  digitalWrite(sxAv,LOW);  
  digitalWrite(dxAv,HIGH);  
  digitalWrite(dxIn,LOW);  
  analogWrite(enDx,velDX);
  analogWrite(enSx,velDX);    
  //Serial.print("Vado a destra: ");  
  //Serial.println(velDX);  
  aspetta(tSvolta);  
  fermo();    
}
 
void sinistra()
{
  //FIRDDist();
  digitalWrite(sxIn,LOW);
  digitalWrite(sxAv,HIGH);  
  digitalWrite(dxAv,LOW);  
  digitalWrite(dxIn,HIGH);  
  analogWrite(enDx,velSX);
  analogWrite(enSx,velSX);  
  //Serial.print("Vado a sinistra: ");  
  //Serial.println(velSX);  
  aspetta(tSvolta);  
  fermo();    
}



void evita() {
  
  MDistanza();
  detectionSX = digitalRead(FIRSX);
  detectionDX = digitalRead(FIRDX);
  Serial.print("Distanza frontale ");
  Serial.print(distanza);
  Serial.print(" Distanza destra ");
  Serial.print(detectionDX);
  Serial.print(" Distanza sinistra ");
  Serial.println(detectionSX);
  if(detectionDX==1&&detectionSX==0) {
    digitalWrite(sxIn,HIGH);
    digitalWrite(sxAv,LOW);  
    digitalWrite(dxAv,HIGH);  
    digitalWrite(dxIn,LOW);  
    analogWrite(enDx,200);
    analogWrite(enSx,200);    
    //Serial.print("Vado a destra: ");  
    //Serial.println(velDX);  
    aspetta(tSvolta*3);  
    fermo();    
  }
  else if(detectionSX==1&&detectionDX==0) {
    digitalWrite(sxIn,LOW);
    digitalWrite(sxAv,HIGH);  
    digitalWrite(dxAv,LOW);  
    digitalWrite(dxIn,HIGH);  
    analogWrite(enDx,200);
    analogWrite(enSx,200);  
    //Serial.print("Vado a sinistra: ");  
    //Serial.println(velSX);  
    aspetta(tSvolta);  
    fermo();    
  }
  else if(detectionSX==0&&detectionDX==0) {
    digitalWrite(sxIn,HIGH);
    digitalWrite(sxAv,LOW);
    digitalWrite(dxIn,HIGH);
    digitalWrite(dxAv,LOW);   
    analogWrite(enSx,200);
    analogWrite(enDx,200);
    aspetta(tSvolta);
    fermo();  
  }
  else {
    fermo();
  }

  if (distanza<35 && distanza>=15) {
    fermo();
    Serial.println("Mi devo fermare!!!!");
  }
  else if (distanza<15) {
    digitalWrite(sxIn,HIGH);
    digitalWrite(sxAv,LOW);
    digitalWrite(dxIn,HIGH);
    digitalWrite(dxAv,LOW);   
    analogWrite(enSx,125);
    analogWrite(enDx,125);
    aspetta(tSvolta*2);
    Serial.println("Devo indietreggiare immediatamente!!!");
    fermo();  
  }
}



void aspetta(int tempoX)
{
  unsigned long tempoVolta=millis();
  while(millis()-tempoVolta < tempoX)
  {     
    //digitalWrite(controlLed,HIGH);
  }
    //digitalWrite(controlLed,LOW);
    fermo();  
}



void MDistanza()
{  
  /*
  distanzaTot=0;
  for (int i=0; i<rip;i++) {
    //porta bassa l'uscita del trigger
    
    digitalWrite( triggerPortSx, LOW );
    delayMicroseconds( 2 );
    //invia un impulso di 10microsec su trigger
    digitalWrite( triggerPortSx, HIGH );
    delayMicroseconds( 10 );
    digitalWrite( triggerPortSx, LOW );   
    durata = pulseIn( echoPortSx, HIGH );   
    distanzaM = 0.0343 * durata / 2;
    distanzaTot=distanzaTot+distanzaM;    
  

  }
  distanza=distanzaTot/rip;
*/
  distanza=ultrasonic.distanceRead();
  delay(5);
  return (distanza);
}


void braccioRobot() {
  // Servo della base

  if (micro==1) {
    ritardo=250;
  }
  else {
    ritardo=1;
  }

  if (zValue>ySogliaSx) {
    servoBase.write(basePos++); 
    delay(ritardo);   
  }
  else if (zValue<ySogliaRx) {
    servoBase.write(basePos--);
    delay(ritardo);    
  }
  if (basePos>=baseMax) basePos=baseMax;
  if (basePos<baseMin) basePos=baseMin;


// Servo numero uno
  if (wValue>xSogliaDw) {
    servoUno.write(unoPos++); 
    delay(ritardo);
    servoDue.write(duePos++); 
    delay(ritardo);   
  }
  else if (wValue<xSogliaUp) {
    servoUno.write(unoPos--);
    delay(ritardo);
    servoDue.write(duePos--);
    delay(ritardo);    
  }
  if (unoPos>=unoMax) unoPos=unoMax;
  if (unoPos<=unoMin) unoPos=unoMin;
  if (duePos>=dueMax) duePos=dueMax;
  if (duePos<=dueMin) duePos=dueMin;
  

  // Servo numero due
  if (gValue>xSogliaDw) {
    servoDue.write(duePos--); 
    delay(ritardo*2);
    servoTre.write(trePos--); 
    delay(ritardo*2);      
  }
  else if (gValue<xSogliaUp) {
    servoDue.write(duePos++);
    delay(ritardo*2);    
    servoTre.write(trePos++); 
    delay(ritardo*2);   
  }
  if (duePos>=dueMax) duePos=dueMax;
  if (duePos<=dueMin) duePos=dueMin;
  if (trePos>=treMax) trePos=treMax;
  if (trePos<=treMin) trePos=treMin;
  
   // Servo numero tre
  if (hValue>ySogliaSx) {
      servoTre.write(trePos++); 
      delay(ritardo);   
    }
  else if (hValue<ySogliaRx) {
    servoTre.write(trePos--);
    delay(ritardo);    
  }
  if (trePos>=treMax) trePos=treMax;
  if (trePos<=treMin) trePos=treMin;
  

  // Servo numero roto
  if (iValue>ySogliaSx) {
    servoRoto.write(rotoPos--); 
    delay(ritardo);   
  }
  else if (iValue<ySogliaRx) {
    servoRoto.write(rotoPos++);
    delay(ritardo);    
  }
  if (rotoPos>=rotoMax) rotoPos=rotoMax;
  if (rotoPos<=rotoMin) rotoPos=rotoMin;

  // Servo pinza finale
  if (lValue>ySogliaSx) {
    servoPinza.write(pinzaPos++); 
    delay(ritardo);   
  }
  else if (lValue<ySogliaRx) {
    servoPinza.write(pinzaPos--);
    delay(ritardo);    
  }
  if (pinzaPos>=pinzaMax) pinzaPos=pinzaMax;
  if (pinzaPos<=pinzaMin) pinzaPos=pinzaMin;  
}