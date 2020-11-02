int pin = A1; 

void setup() {
  Serial.begin(9600);
}

void loop() 
{
  int val  = analogRead(pin);

  Serial.println(val);
}
