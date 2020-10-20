int pin = A0; 

void setup() {
  Serial.begin(9600);
}

void loop() 
{
  int val = analogRead(pin);

  if(val < 950)
  {
    Serial.println("object below");
  }
}
