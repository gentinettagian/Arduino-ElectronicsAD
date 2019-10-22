#define INTERVAL_MICROS         1000000 //update time
#define OVERFLOW             4000000000 //at this time we reset to 0

unsigned long last_time = 0; //last timestamp 

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(A2,OUTPUT);
  digitalWrite(A2, HIGH);
}

void loop() {
  // Turn off light after 1 second and on again after 2 seonds.
  unsigned long now = micros();
  
  if (now > (last_time + 2*INTERVAL_MICROS)) {
    digitalWrite(LED_BUILTIN, HIGH);
    if (now > OVERFLOW) { //make sure not to overflow
      now = 0;
    }
    last_time = now;
  } else if (now > (last_time + INTERVAL_MICROS)) {
    digitalWrite(LED_BUILTIN, LOW);
    
  }
}
