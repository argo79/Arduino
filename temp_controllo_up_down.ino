int const redLEDPin = 11;
int const blueLEDPin = 10;

const int potenziometro = A0;
int potValue;
int redValue;
int blueValue;

int ledBLUE = 8;
int ledRED = 7;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(redLEDPin, OUTPUT);
  pinMode(blueLEDPin, OUTPUT);
  pinMode(ledBLUE, OUTPUT);
  pinMode(ledRED, OUTPUT);
  

}

void loop() {
  
  int temp = analogRead(potenziometro);
  float voltage = (temp / 1023.0) * 5.0;
  float temperat = (voltage - 0.5) * 100;

int tempo = map(temp, 20, 400, 0, 255);

if (temp < 60) {
  tempo = 0;
}
if (temp > 350) {
  tempo = 255;
}
    redValue = tempo;
    blueValue = 255 - tempo;
  
  
  Serial.print("Temperatura: ");
  Serial.print(temperat);
  Serial.print("\t Volts: ");
  Serial.print(voltage);
  Serial.print("\t Valore temp(0-1024): ");
  Serial.print(temp);
  Serial.print("\t Valore tempo(0-255): ");
  Serial.print(tempo);
  Serial.print("\t Valore led rosso: ");
  Serial.print(redValue);
  Serial.print("\t Valore led blue: ");
  Serial.println(blueValue);
  analogWrite(redLEDPin, redValue);
  analogWrite(blueLEDPin, blueValue);
  delay(50);
  
  digitalWrite(ledBLUE, LOW);
  digitalWrite(ledRED, LOW);
  
  int temp2 = analogRead(potenziometro);
  float voltage2 = (temp2 / 1023.0) * 5.0;
  float temperat2 = (voltage2 - 0.5) * 100;
  
  if (temperat2 > temperat + 0.5) {
    digitalWrite(ledRED, HIGH);
    digitalWrite(ledBLUE, LOW);
  }
  if (temperat2 < temperat - 0.5) {
    digitalWrite(ledBLUE, HIGH);
    digitalWrite(ledRED, LOW);
  }
}
