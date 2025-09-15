#define PIN_LED 7

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 0); // turn off LED.
}

void loop() {
  delay(5000);
  for(int i=0;i<5;i++){
    digitalWrite(PIN_LED,1);
    delay(1000);
    digitalWrite(PIN_LED, 0);
    delay(1000);
  }
  digitalWrite(PIN_LED,1);
  while(1){}
}

