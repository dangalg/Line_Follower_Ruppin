//APDS color sensor libaries
#include <Wire.h>
#include <SparkFun_APDS9960.h>

// Global Variables for APDS color sensor
SparkFun_APDS9960 apds = SparkFun_APDS9960();
uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;

// ir sensor data
#define R_R_S 2 //ir sensor Right
#define R_S 3 //ir sensor Right
#define L_S 4 //ir sensor Left
#define L_L_S 5 //ir sensor Left

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


void setup() {
  if(debug){
    Serial.begin(9600);
  }

  manual_calibration();
  setupAPDS();
}

void loop() {
  //getDataFromSensors();
  //steerCar();

  activateRGB(RED);
  delay(3000);
  activateRGB(GREEN);
  delay(3000);
  activateRGB(BLUE);
  delay(3000);
}

void getDataFromSensors()
{
  // put your main code here, to run repeatedly:
  R_R_Sval = analogRead(R_R_S);
  R_Sval = analogRead(R_S);
  L_Sval = analogRead(L_S);
  L_L_Sval = analogRead(L_L_S);

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
    R_R_Sval = analogRead(R_R_S);
    if(R_R_Sval < minR_R_Sval){minR_R_Sval = R_R_Sval;}
    if(R_R_Sval > maxR_R_Sval){maxR_R_Sval = R_R_Sval;}
    R_Sval = analogRead(R_S);
    if(R_Sval < minR_Sval){minR_Sval = R_Sval;}
    if(R_Sval > maxR_Sval){maxR_Sval = R_Sval;}
    L_Sval = analogRead(L_S);
    if(L_Sval < minL_Sval){minL_Sval = L_Sval;}
    if(L_Sval > maxL_Sval){maxL_Sval = L_Sval;}
    L_L_Sval = analogRead(L_L_S);
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
  //Serial.print(sensorValueR_S); Serial.print(" "); Serial.print(sensorValueL_S); Serial.println();

  // LLS and RRS have priority. If they see something they must turn towards it
  if(sensorValueR_R_S>400 || sensorValueL_L_S > 400)
  {
    if((sensorValueR_R_S >400)&&(sensorValueL_L_S >400)){
      forwardStrong(); 
      if(debug){Serial.println("forward intersection");}
    }   //if Right Sensor and Left Sensor are at Black color then it will call forword function
    if((sensorValueR_R_S <400)&&(sensorValueL_L_S >400)){
      turn90Left(); 
      lastDirection = 0;
      if(debug){Serial.println("turn 90 Left");}
    } //if Right Sensor is Black and Left Sensor is White then it will call turn Right function  
    if((sensorValueR_R_S >400)&&(sensorValueL_L_S <400)){
      turn90Right(); 
      if(debug){Serial.println("turn 90 Right");}
      lastDirection = 1;
    }  //if Right Sensor is White and Left Sensor is Black then it will call turn Left function
  }
  else{
    if((sensorValueR_S >300)&&(sensorValueL_S >300)){
      forward(); 
      if(debug){Serial.println("forward");}
      
    }   //if Right Sensor and Left Sensor are at Black color then it will call forword function
    if((sensorValueR_S <300)&&(sensorValueL_S >300)){
      turnLeft(); 
      lastDirection = 0;
      if(debug){Serial.println("turnLeft");}
    } //if Right Sensor is Black and Left Sensor is White then it will call turn Right function  
    if((sensorValueR_S >300)&&(sensorValueL_S <300)){
      turnRight(); 
      if(debug){Serial.println("turnRight");}
      lastDirection = 1;
    }  //if Right Sensor is White and Left Sensor is Black then it will call turn Left function
    if((sensorValueR_S <300)&&(sensorValueL_S <300)){
      if(lastDirection == 0)
      {
        turnWideLeft();
        if(debug){Serial.println("turn to left after left line");}
      }
      else
      {
        turnWideRight();
        if(debug){Serial.println("turn to right after left line");}
      }
      
    } //if Right Sensor and Left Sensor are at White color then it will call Stop function

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

void forwardStrong()
{
  forward();
  delay(900);
}

void turnRight(){ //turnRight
  //Motor A
  digitalWrite(motorADirection, HIGH); //Establishes forward direction of Channel A
  digitalWrite(motorABrake, LOW);   //Disengage the Brake for Channel A
  analogWrite(motorASpeed, 10);   //Spins the motor on Channel A at full speed

  //Motor B
  digitalWrite(motorBDirection, HIGH);  //Establishes backward direction of Channel B
  digitalWrite(motorBBrake, LOW);   //Disengage the Brake for Channel B
  analogWrite(motorBSpeed, 125);    //Spins the motor on Channel B at half speed
}

void turn90Right(){ //turnRight
  //Motor A
  digitalWrite(motorADirection, HIGH); //Establishes forward direction of Channel A
  digitalWrite(motorABrake, LOW);   //Disengage the Brake for Channel A
  analogWrite(motorASpeed, 30);   //Spins the motor on Channel A at full speed

  //Motor B
  digitalWrite(motorBDirection, HIGH);  //Establishes backward direction of Channel B
  digitalWrite(motorBBrake, LOW);   //Disengage the Brake for Channel B
  analogWrite(motorBSpeed, 125);    //Spins the motor on Channel B at half speed
}

void turnWideRight(){ //turnRight
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
  analogWrite(motorBSpeed, 10);    //Spins the motor on Channel B at half speed
}

void turn90Left(){ //turnLeft
  //Motor A
  digitalWrite(motorADirection, HIGH); //Establishes forward direction of Channel A
  digitalWrite(motorABrake, LOW);   //Disengage the Brake for Channel A
  analogWrite(motorASpeed, 125);   //Spins the motor on Channel A at full speed

  //Motor B
  digitalWrite(motorBDirection, LOW);  //Establishes backward direction of Channel B
  digitalWrite(motorBBrake, LOW);   //Disengage the Brake for Channel B
  analogWrite(motorBSpeed, 30);    //Spins the motor on Channel B at half speed
}

void turnWideLeft(){ //turnLeft
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
  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init() ) {
    if(debug){Serial.println(F("APDS-9960 initialization complete"));}
  } else {
    if(debug){Serial.println(F("Something went wrong during APDS-9960 init!"));}
  }
  // Start running the APDS-9960 light sensor (no interrupts)
//  if ( apds.enableLightSensor(false) ) {
//    if(debug){Serial.println(F("Light sensor is now running"));}
//  } else {
//    if(debug){Serial.println(F("Something went wrong during light sensor init!"));}
//  }

  // Adjust the Proximity sensor gain
//  if ( !apds.setProximityGain(PGAIN_2X) ) {
//    if(debug){Serial.println(F("Something went wrong trying to set PGAIN"));}
//  }
  
  // Start running the APDS-9960 proximity sensor (no interrupts)
//  if ( apds.enableProximitySensor(false) ) {
//    if(debug){Serial.println(F("Proximity sensor is now running"));}
//  } else {
//    if(debug){Serial.println(F("Something went wrong during sensor init!"));}
//  }
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
