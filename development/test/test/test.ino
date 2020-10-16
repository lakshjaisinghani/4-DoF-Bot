const int outG = 6; // green
const int outR = 3; // red
const int outB = 5; // 

void setup() {
  Serial.begin(9600);
  pinMode(2, OUTPUT);
}

void loop() 
{
  digitalWrite(2, 1);
  delay(1000);
  digitalWrite(2, 0);
  delay(1000);
}
