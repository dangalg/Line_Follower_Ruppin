//APDS color sensor libaries
#include <Wire.h>
#include <SparkFun_APDS9960.h>

// Global Variables for APDS color sensor
SparkFun_APDS9960 apds = SparkFun_APDS9960();
uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;
uint8_t proximity_data = 0;

// ir sensor data
#define R_R_S 2 //ir sensor Right
#define R_S 4 //ir sensor Right
#define L_S A2 //ir sensor Left
#define L_L_S A3 //ir sensor Left

float R_R_Sval;
float R_Sval;
float L_Sval;
float L_L_Sval;

float minR_R_Sval = 9999;
float minR_Sval = 9999;
float minL_Sval = 9999;
float minL_L_Sval = 9999;

float maxR_R_Sval = 0;
float maxR_Sval = 0;
float maxL_Sval = 0;
float maxL_L_Sval = 0;

float sensorValueR_R_S;
float sensorValueR_S;
float sensorValueL_S;
float sensorValueL_L_S;

int lastDirection = 1; // 0 left 1 right
int lastSesorStatus[4] = {0};

// motor data
int motorADirection = 12;
int motorABrake = 9;
int motorASpeed = 3;
int motorBDirection = 13;
int motorBBrake = 8;
int motorBSpeed = 11;

// RGB Led data
#define RED 90
#define GREEN 91
#define BLUE 92

int ledRed = 5; 
int ledGreen = 6;  
int ledBlue = 10;

int debug = 1; // set to 1 if serial debug output needed
int debug2 = 0;


void setup() {
  if(debug){
    Serial.begin(9600);
  }

  manual_calibration();
  setupAPDS();
}

void loop() {
  getDataFromSensors();
  steerCar();

  checkProximity();

  checkColor();
}

void checkProximity()
{
  // Read the proximity value
  if ( !apds.readProximity(proximity_data) ) {
    if(debug){Serial.println("Error reading proximity value");}
  } else {
    if(debug){Serial.print("Proximity: ");}
    if(debug){Serial.println(proximity_data);}
  }
}

void checkColor()
{
  // Read the light levels (ambient, red, green, blue)
  apds.readRedLight(red_light);
  apds.readGreenLight(green_light);
  apds.readGreenLight(blue_light);
  if(debug){Serial.print("red_light: ");Serial.print(red_light);
  Serial.print(" green_light: ");Serial.print(green_light);
  Serial.print(" blue_light: ");Serial.print(blue_light);
  Serial.println();}
  if (  red_light > green_light && red_light > blue_light)
  {
    activateRGB(RED);
  }
  else if(green_light > blue_light)
  {
    activateRGB(GREEN);
  }
  else
  {
    activateRGB(BLUE);
  } 
}

void getDataFromSensors()
{
  // put your main code here, to run repeatedly:
  R_R_Sval = digitalRead(R_R_S);
  R_Sval = digitalRead(R_S);
  L_Sval = digitalRead(L_S);
  L_L_Sval = digitalRead(L_L_S);

  sensorValueR_R_S = map(R_R_Sval, minR_R_Sval, maxR_R_Sval,0,1000);
  sensorValueR_S = map(R_Sval, minR_Sval, maxR_Sval,0,1000);
  sensorValueL_S = map(L_Sval, minL_Sval, maxL_Sval,0,1000);
  sensorValueL_L_S = map(L_L_Sval, minL_L_Sval, maxL_L_Sval,0,1000);
}

void manual_calibration()
{
  int i;
  for (i = 0; i < 50; i++)  // make the calibration take about 5 seconds
  {
    R_R_Sval = digitalRead(R_R_S);
    if(R_R_Sval < minR_R_Sval){minR_R_Sval = R_R_Sval;}
    if(R_R_Sval > maxR_R_Sval){maxR_R_Sval = R_R_Sval;}
    R_Sval = digitalRead(R_S);
    if(R_Sval < minR_Sval){minR_Sval = R_Sval;}
    if(R_Sval > maxR_Sval){maxR_Sval = R_Sval;}
    L_Sval = digitalRead(L_S);
    if(L_Sval < minL_Sval){minL_Sval = L_Sval;}
    if(L_Sval > maxL_Sval){maxL_Sval = L_Sval;}
    L_L_Sval = digitalRead(L_L_S);
    if(L_L_Sval < minL_L_Sval){minL_L_Sval = L_L_Sval;}
    if(L_L_Sval > maxL_L_Sval){maxL_L_Sval = L_L_Sval;}

    if(debug){
      Serial.print("calib R_R_Sval min: "); Serial.print(minR_R_Sval);  Serial.print(" max: "); Serial.print(maxR_R_Sval); Serial.println();
      Serial.print(" calib R_Sval min: "); Serial.print(minR_Sval);    Serial.print(" max: "); Serial.print(maxR_Sval); Serial.println();
      Serial.print(" calib L_Sval min: "); Serial.print(minL_Sval);  Serial.print(" max: ");   Serial.print(maxL_Sval); Serial.println();
      Serial.print(" calib L_L_Sval min: "); Serial.print(minL_L_Sval);  Serial.print(" max: ");   Serial.print(maxL_L_Sval); Serial.println();
      Serial.println();
    }
    delay(20);
    
  }
}

void steerCar()
{

  if(proximity_data > 200){
    if(debug){Serial.println("proximity close stop");}
    Stop();
  }
  else
  {
    //Serial.print(sensorValueR_S); Serial.print(" "); Serial.print(sensorValueL_S); Serial.println();

    if(!sensorValueR_S && !sensorValueL_S && !sensorValueR_R_S && !sensorValueL_L_S)
    { 
      // reached empty space
      if(lastDirection > 3)
      {
        if(debug){Serial.println("go forward");}
        forward();
      }
      setSensorStatus(sensorValueL_L_S, sensorValueL_S, sensorValueR_S, sensorValueR_R_S);
    }
    // LLS and RRS have priority. If they see something they must turn towards it
    else if(sensorValueR_R_S || sensorValueL_L_S)
    {
      if((sensorValueR_R_S)&&(sensorValueL_L_S)){
        forwardStrong(600); 
        if(debug){Serial.println("forward intersection");}
        
        lastDirection = 1;
        setSensorStatus(sensorValueL_L_S, sensorValueL_S, sensorValueR_S, sensorValueR_R_S);
        
      }   //if Right Sensor and Left Sensor are at Black color then it will call forword function
      if((!sensorValueR_R_S)&&(sensorValueL_L_S)){
        turn90Left(); 
        if(debug){Serial.println("turn 90 Left");}
        
        lastDirection = 2;
        setSensorStatus(sensorValueL_L_S, sensorValueL_S, sensorValueR_S, sensorValueR_R_S);
        
        
      } //if Right Sensor is Black and Left Sensor is White then it will call turn Right function  
      if((sensorValueR_R_S)&&(!sensorValueL_L_S)){
        turn90Right(); 
        if(debug){Serial.println("turn 90 Right");}
        
        lastDirection = 3;
        setSensorStatus(sensorValueL_L_S, sensorValueL_S, sensorValueR_S, sensorValueR_R_S);
        
      }  //if Right Sensor is White and Left Sensor is Black then it will call turn Left function
    }
    else{
      if((sensorValueR_S)&&(sensorValueL_S)){
        forward(); 
        if(debug){Serial.println("forward");}
        
        lastDirection = 4;
        setSensorStatus(sensorValueL_L_S, sensorValueL_S, sensorValueR_S, sensorValueR_R_S);
        
      }   //if Right Sensor and Left Sensor are at Black color then it will call forword function
      if((!sensorValueR_S)&&(sensorValueL_S)){
        turnLeft(); 
        
        lastDirection = 5;
        setSensorStatus(sensorValueL_L_S, sensorValueL_S, sensorValueR_S, sensorValueR_R_S);
        
        if(debug){Serial.println("turnLeft");}
      } //if Right Sensor is Black and Left Sensor is White then it will call turn Right function  
      if((sensorValueR_S)&&(!sensorValueL_S)){
        turnRight(); 
        if(debug){Serial.println("turnRight");}
        
        lastDirection = 6;
        setSensorStatus(sensorValueL_L_S, sensorValueL_S, sensorValueR_S, sensorValueR_R_S);
        
      }  //if Right Sensor is White and Left Sensor is Black then it will call turn Left function
    }
  }
}

void forward(){  //forword

  //Motor A
  digitalWrite(motorADirection, HIGH); //Establishes forward direction of Channel A
  digitalWrite(motorABrake, LOW);   //Disengage the Brake for Channel A
  analogWrite(motorASpeed, 125);   //Spins the motor on Channel A at full speed

  //Motor B
  digitalWrite(motorBDirection, HIGH);  //Establishes backward direction of Channel B
  digitalWrite(motorBBrake, LOW);   //Disengage the Brake for Channel B
  analogWrite(motorBSpeed, 125);    //Spins the motor on Channel B at half speed
  
}

void forwardStrong(int millis)
{
  forward();
  delay(millis);
}

void turnRight(){ //turnRight
  //Motor A
  digitalWrite(motorADirection, HIGH); //Establishes forward direction of Channel A
  digitalWrite(motorABrake, LOW);   //Disengage the Brake for Channel A
  analogWrite(motorASpeed, 30);   //Spins the motor on Channel A at full speed

  //Motor B
  digitalWrite(motorBDirection, HIGH);  //Establishes backward direction of Channel B
  digitalWrite(motorBBrake, LOW);   //Disengage the Brake for Channel B
  analogWrite(motorBSpeed, 125);    //Spins the motor on Channel B at half speed
}

void turn90Right(){ //turn90Right
  //Motor A
  digitalWrite(motorADirection, LOW); //Establishes forward direction of Channel A
  digitalWrite(motorABrake, LOW);   //Disengage the Brake for Channel A
  analogWrite(motorASpeed, 30);   //Spins the motor on Channel A at full speed

  //Motor B
  digitalWrite(motorBDirection, HIGH);  //Establishes backward direction of Channel B
  digitalWrite(motorBBrake, LOW);   //Disengage the Brake for Channel B
  analogWrite(motorBSpeed, 125);    //Spins the motor on Channel B at half speed
}

void turnWideRight(){ //turnWideRight
  //Motor A
  digitalWrite(motorADirection, HIGH); //Establishes forward direction of Channel A
  digitalWrite(motorABrake, LOW);   //Disengage the Brake for Channel A
  analogWrite(motorASpeed, 50);   //Spins the motor on Channel A at full speed

  //Motor B
  digitalWrite(motorBDirection, HIGH);  //Establishes backward direction of Channel B
  digitalWrite(motorBBrake, LOW);   //Disengage the Brake for Channel B
  analogWrite(motorBSpeed, 125);    //Spins the motor on Channel B at half speed
}
void turnLeft(){ //turnLeft
  //Motor A
  digitalWrite(motorADirection, HIGH); //Establishes forward direction of Channel A
  digitalWrite(motorABrake, LOW);   //Disengage the Brake for Channel A
  analogWrite(motorASpeed, 125);   //Spins the motor on Channel A at full speed

  //Motor B
  digitalWrite(motorBDirection, HIGH);  //Establishes backward direction of Channel B
  digitalWrite(motorBBrake, LOW);   //Disengage the Brake for Channel B
  analogWrite(motorBSpeed, 30);    //Spins the motor on Channel B at half speed
}

void turn90Left(){ //turn90Left
  //Motor A
  digitalWrite(motorADirection, HIGH); //Establishes forward direction of Channel A
  digitalWrite(motorABrake, LOW);   //Disengage the Brake for Channel A
  analogWrite(motorASpeed, 125);   //Spins the motor on Channel A at full speed

  //Motor B
  digitalWrite(motorBDirection, LOW);  //Establishes backward direction of Channel B
  digitalWrite(motorBBrake, LOW);   //Disengage the Brake for Channel B
  analogWrite(motorBSpeed, 30);    //Spins the motor on Channel B at half speed
}

void turnWideLeft(){ //turnWideLeft
  //Motor A
  digitalWrite(motorADirection, HIGH); //Establishes forward direction of Channel A
  digitalWrite(motorABrake, LOW);   //Disengage the Brake for Channel A
  analogWrite(motorASpeed, 50);   //Spins the motor on Channel A at full speed

  //Motor B
  digitalWrite(motorBDirection, HIGH);  //Establishes backward direction of Channel B
  digitalWrite(motorBBrake, LOW);   //Disengage the Brake for Channel B
  analogWrite(motorBSpeed, 125);    //Spins the motor on Channel B at half speed
}
void Stop(){ //stop
  //Motor A
  digitalWrite(motorADirection, HIGH); //Establishes forward direction of Channel A
  digitalWrite(motorABrake, HIGH);   //Disengage the Brake for Channel A
  analogWrite(motorASpeed, 0);   //Spins the motor on Channel A at full speed

  //Motor B
  digitalWrite(motorBDirection, HIGH);  //Establishes backward direction of Channel B
  digitalWrite(motorBBrake, HIGH);   //Disengage the Brake for Channel B
  analogWrite(motorBSpeed, 0);    //Spins the motor on Channel B at half speed
}

void setupAPDS()
{
  if(debug){Serial.println(F("Initializing APDS-9960"));}
  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init() ) {
    if(debug){Serial.println(F("APDS-9960 initialization complete"));}
  } else {
    if(debug){Serial.println(F("Something went wrong during APDS-9960 init!"));}
  }
  // Start running the APDS-9960 light sensor (no interrupts)
  if ( apds.enableLightSensor(false) ) {
    if(debug){Serial.println(F("Light sensor is now running"));}
  } else {
    if(debug){Serial.println(F("Something went wrong during light sensor init!"));}
  }

  // Adjust the Proximity sensor gain
  if ( !apds.setProximityGain(PGAIN_2X) ) {
    if(debug){Serial.println(F("Something went wrong trying to set PGAIN"));}
  }
  
  // Start running the APDS-9960 proximity sensor (no interrupts)
  if ( apds.enableProximitySensor(false) ) {
    if(debug){Serial.println(F("Proximity sensor is now running"));}
  } else {
    if(debug){Serial.println(F("Something went wrong during sensor init!"));}
  }
}

void activateRGB(int color)
{

  if(color == RED)
  {
    analogWrite(ledRed,255); 
    analogWrite(ledGreen,0);  
    analogWrite(ledBlue,0);
  }
  else if(color == GREEN)
  {
    analogWrite(ledRed,0); 
    analogWrite(ledGreen,255);  
    analogWrite(ledBlue,0);
  }
  else if(color == BLUE)
  {
    analogWrite(ledRed,0); 
    analogWrite(ledGreen,0);  
    analogWrite(ledBlue,255);
  }
}

void setSensorStatus(int lls, int ls, int rs, int rrs)
{
  lastSesorStatus[0] = sensorValueR_R_S;
  lastSesorStatus[1] = sensorValueR_S;
  lastSesorStatus[2] = sensorValueL_S;
  lastSesorStatus[3] = sensorValueL_L_S;
}
