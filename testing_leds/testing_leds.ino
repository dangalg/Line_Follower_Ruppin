#define LED 12  // The pin the LED is connected to
void setup() {
  pinMode(LED, OUTPUT); // Declare the LED as an output
}

void loop() {
  digitalWrite(LED, HIGH); // Turn the LED on
  digitalWrite(13, LOW); // Turn the LED on
}
