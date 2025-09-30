#define LED_PIN    9  
#define STEP_MS    5     

int duty = 0;        
int period=0;         
bool up = true;       
unsigned long lastUpdate = 0;
int onTime=0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  set_period();
}

void loop() {

  if (millis() - lastUpdate >= STEP_MS) {
    lastUpdate = millis();
    set_duty();   
    onTime = (period * duty) / 100;  
  }
  digitalWrite(LED_PIN, HIGH);
  delayMicroseconds(onTime);          
  digitalWrite(LED_PIN, LOW);
  delayMicroseconds(period- onTime); 
}

void set_duty() {
  if (up) duty++;
  else duty--;

  if (duty > 99) { duty = 100; up = false; }
  if (duty < 1)   { duty = 0;   up = true;  }
}

void set_period() {
  period = 100;
}

