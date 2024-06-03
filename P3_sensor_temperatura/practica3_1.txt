int lm35 = A0;
int Led = 3;

void setup() {
  Serial.begin(9600);
  pinMode(Led, OUTPUT);
}

void loop() {
  int sensorValue = analogRead(A0);
  float x = (sensorValue * 5000.0)/1023;  // voltaje 0 - 5000 mV
  float temperatura = x/10;  
  float voltaje = x/4;  // voltaje de 0 - 255
  Serial.print("Temperatura: ");
  Serial.print(temperatura);
  Serial.print("\t Voltaje (0-255): ");
  Serial.println(voltaje);
  delay(20);
  
  if(temperatura > 27.0) {
  	digitalWrite(Led, HIGH);
  }else{
  	digitalWrite(Led, LOW);
  }
}
