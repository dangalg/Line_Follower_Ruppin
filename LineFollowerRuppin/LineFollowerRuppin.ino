// motor data
int motorADirection = 12;
int motorABrake = 9;
int motorASpeed = 3;
int motorBDirection = 13;
int motorBBrake = 8;
int motorBSpeed = 11;


void setup() {
  //Setup Motor A
  pinMode(motorADirection, OUTPUT); //Initiates Motor Channel A pin
  pinMode(motorABrake, OUTPUT); //Initiates Brake Channel A pin

  //Setup Motor B
  pinMode(motorBDirection, OUTPUT); //Initiates Motor Channel A pin
  pinMode(motorBBrake, OUTPUT);  //Initiates Brake Channel A pin
}

void loop() {
  // put your main code here, to run repeatedly:
  //Motor A forward @ full speed
  digitalWrite(motorADirection, HIGH); //Establishes forward direction of Channel A
  digitalWrite(motorABrake, LOW);   //Disengage the Brake for Channel A
  analogWrite(motorASpeed, 255);   //Spins the motor on Channel A at full speed

  //Motor B backward @ half speed
  digitalWrite(motorBDirection, HIGH);  //Establishes backward direction of Channel B
  digitalWrite(motorBBrake, LOW);   //Disengage the Brake for Channel B
  analogWrite(motorBSpeed, 255);    //Spins the motor on Channel B at half speed
}
