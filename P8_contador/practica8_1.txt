int state = 0;
int vivo = 1;
int contador = 100;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(10, INPUT);
  pinMode(2, OUTPUT);
  Serial.println(contador);
}

void loop() {
  // put your main code here, to run repeatedly:
  state = digitalRead(10);
  // Serial.println(state);

  if (state == 1 && contador > 0) {
    contador -= 1;
    delay(50);
    Serial.println(contador);
  }

  delay(100);

  // if (state == 1) {
  //   digitalWrite(2, LOW);
  //   delay(250);
  //   digitalWrite(2, HIGH);
  //   delay(250);
  //   digitalWrite(2, LOW);
  //   delay(250);
  //   digitalWrite(2, HIGH);
  //   delay(250);
  //   digitalWrite(2, LOW);
  // }  

  if (contador == 0 && vivo == 1) {
    vivo = 0;
    Serial.println("GAME OVER !!! :(");
  }


}
