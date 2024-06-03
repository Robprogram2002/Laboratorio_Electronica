int ventilador = 3;
int pot = A0;

void setup() {
  pinMode(ventilador, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(A0);
  float voltaje = sensorValue / 4;  
  Serial.println(voltaje);
  delay(20);

  analogWrite(ventilador, voltaje);
  delay(50);
}
