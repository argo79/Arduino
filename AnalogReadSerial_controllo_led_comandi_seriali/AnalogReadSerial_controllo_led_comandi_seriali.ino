/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/

const byte outputPin = 9; //pin di output
byte statoLed = 0; //contiene il valore sul pin 9

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
Serial.println(F("Pronto")); //sono pronto
pinMode (13, OUTPUT);

}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
 // int sensorValue = analogRead(A0);
  // print out the value you read:
//  Serial.println(sensorValue);

 int lettura; //contiene il valore di PWM letto dalla seriale
   unsigned long tempMillis; //serve per il timeout sulla ricezione del valore di PWM
   byte caratteri; //contiene il numero di caratteri ricevuti
   byte tempChar; //contiene il carattere letto

  if (Serial.available()) {
     byte command = Serial.read(); //leggo il primo byte
     switch (command) { //controllo che sia un comando valido
       case 'l': //lettura
         //invio la lettura dell'ingresso
         Serial.print(F("Lettura ingresso: "));
         Serial.println(analogRead(A0));
         break;
       case 'a': //accendo il LED
         statoLed = 255;
         analogWrite(outputPin, statoLed);
         Serial.println(F("LED acceso"));
         break;
       case 's': //spengo il led
         statoLed = 0;
         analogWrite(outputPin, statoLed);
         Serial.println(F("LED spento"));
         break;
       case 'q': //l'utente vuole lo stato del LED
         Serial.print(F("LED "));
         if (statoLed == 0) {
           Serial.println(F("spento"));
         } else if (statoLed == 255) {
           Serial.println(F("acceso"));
         } else {
           Serial.print(F("pilotato con PWM impostato a "));
           Serial.println(statoLed, DEC);
         }
         break;
       case 'w': //imposto il PWM sul LED
         lettura = 0;
         tempMillis = millis();
         caratteri = 0;
         //mi servono altri caratteri
         do {
           if (Serial.available()) {
             tempChar = Serial.read();
             caratteri++;
             if ((tempChar >= 48) && (tempChar <= 57)) { //è un numero? ok
               lettura = (lettura * 10) + (tempChar - 48);
             } else if ((tempChar == 10) || (tempChar == 13)) {
               //con invio e/o nuova linea esco
               break;
             }
           }
         //esco per timeout, per un valore di PWM maggiore di 255
         //oppure per troppi caratteri senza senso ricevuti
         } while((millis() - tempMillis < 500) && (lettura <= 255) && (caratteri < 10));
         //se il valore di PWM è valido, lo imposto sul pin
         if (lettura <= 255) {
           statoLed = lettura;
           analogWrite(outputPin, statoLed);
           Serial.print(F("LED acceso con PWM impostato a "));
           Serial.println(statoLed, DEC);
         }
         break;
     }
     //svuoto il buffer da eventuali altri caratteri che non mi servono più
     while (Serial.available()) {
       byte a = Serial.read();
     }
   }
  
  digitalWrite(13, HIGH);
  delay(250);
  digitalWrite(13, LOW);
  delay(250);
  delay(1);        // delay in between reads for stability
}
