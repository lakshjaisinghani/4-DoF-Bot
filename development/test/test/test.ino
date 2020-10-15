const int outG = 6; // green
const int outR = 3; // red
const int outB = 5; // 

void setup() {
  Serial.begin(9600);
}

void loop() {
  int reading;
  
  analogWrite(outB,255);
  
  reading = analogRead(A0);
  Serial.println(reading);
}
