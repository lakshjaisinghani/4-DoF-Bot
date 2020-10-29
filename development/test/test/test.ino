int pin = A0; 

void setup() {
  Serial.begin(9600);
}

void loop() 
{
  int val  = analogRead(pin);

  Serial.println(val);
}
