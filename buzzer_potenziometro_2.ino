int buzzer = 12;
int potenziometro = A0;

int numNote = 10;
int note[] = {261, 277, 294, 311, 330, 349, 370, 216, 432, 440};

int valoreletto = 0;
int notascelta = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  for (int i = 0; i < numNote; i++)
  {
    tone(buzzer, note[i]);
    delay(200);
  }
  tone(buzzer, note[10]);
  delay(1000);
  tone(buzzer, note[9]);
  delay(200);
  tone(buzzer, note[10]);
  delay(1000);
  tone(buzzer, note[8]);
  delay(200);
  tone(buzzer, note[9]);
  delay(100);
  noTone(buzzer);

}

void loop() {
  // put your main code here, to run repeatedly:
  valoreletto = analogRead(potenziometro);
  Serial.print("Valore=");
  Serial.println(valoreletto);
  Serial.println(notascelta);
  notascelta = map (valoreletto, 0, 1023, 0, 10);
  if (notascelta==10) {
    noTone(buzzer);
  }
  else
  {
    tone(buzzer, note[notascelta]);    
  }
  delay(200);
  
}

