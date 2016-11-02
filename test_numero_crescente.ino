int i=1;
int p=0;
float per;
void setup() {
  // put your setup code here, to run once:

Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
for (i; i<=8;i++) {
  p=p+i;
  if (i==8) {
Serial.print(i);
Serial.print("\t totale ");
Serial.print(p);
delay(1000);
float per=100.0/p;
float var = per;
Serial.print("\t un punto percentuale vale ");
Serial.print(per);
Serial.print(", il massimo sara': ");
Serial.print(i*var);
  }
}
}

