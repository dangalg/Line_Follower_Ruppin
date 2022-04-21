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
#define R_R_S A2 //ir sensor Right
#define R_S 2 //ir sensor Right
#define L_S 4 //ir sensor Left
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

#define SENSOR_MID 380 // sensor on edge of black line
#define SENSOR_MAX 1000 // sensor in the middle of black

#define PRX_SENSOR_DIST 180 // distance to stop proximity sensor

#define FORWARD_INTERSECTION 1 // all four sensors on black line
#define TURN_LEFT_90 2 // RRS sensor on black line
#define TURN_RIGHT_90 3 // LLS sensor on black line
#define FORWARD 4 //  Two center sensors on black line
#define TURN_LEFT 5 // Left sensor on black line
#define TURN_RIGHT 6 // right sensor on black line

#define RRS 0 // LLS sensor on black line
#define RS 1 //  Two center sensors on black line
#define LS 2 // Left sensor on black line
#define LLS 3 // right sensor on black line

#define CALIB_TIME 250 // sensor on edge of black line

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
#define WHITE 93
#define BLACK 94

int ledRed = 5; 
int ledGreen = 6;  
int ledBlue = 10;

int debug = 1; // set to 1 if serial debug output needed
int debugDrv = 1;
int debugSens = 0;
int debugLit = 0;
int debugPrx = 0;
int debugClb = 0;

// time
unsigned long StartTime;
unsigned long CurrentTime;
unsigned long ElapsedTime;


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
    if(debugPrx){Serial.println("Error reading proximity value");}
  } else {
    if(debugPrx){Serial.print("Proximity: ");}
    if(debugPrx){Serial.println(proximity_data);}
  }
}

void checkColor()
{
  // Read the light levels (ambient, red, green, blue)
  apds.readRedLight(red_light);
  apds.readGreenLight(green_light);
  apds.readGreenLight(blue_light);
  if(debugLit){Serial.print("red_light: ");Serial.print(red_light);
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
  R_R_Sval = analogRead(R_R_S);
  R_Sval = digitalRead(R_S);
  L_Sval = digitalRead(L_S);
  L_L_Sval = analogRead(L_L_S);

  sensorValueR_R_S = R_R_Sval;
  sensorValueR_S = R_Sval;
  sensorValueL_S = L_Sval;
  sensorValueL_L_S = L_L_Sval;
  
  sensorValueR_R_S = map(R_R_Sval, minR_R_Sval, maxR_R_Sval,0,SENSOR_MAX);
  //sensorValueR_S = map(R_Sval, minR_Sval, maxR_Sval,0,SENSOR_MAX);
  //sensorValueL_S = map(L_Sval, minL_Sval, maxL_Sval,0,SENSOR_MAX);
  sensorValueL_L_S = map(L_L_Sval, minL_L_Sval, maxL_L_Sval,0,SENSOR_MAX);

  if(sensorValueR_R_S < 0){sensorValueR_R_S = 0;}
  if(sensorValueR_R_S > SENSOR_MAX) {sensorValueR_R_S = SENSOR_MAX;}
  if(sensorValueL_L_S < 0){sensorValueL_L_S = 0;}
  if(sensorValueL_L_S > SENSOR_MAX){sensorValueL_L_S = SENSOR_MAX;}
}

void manual_calibration()
{
  activateRGB(BLACK);
  delay(1000);
  
  for(int i = 0; i<5; i++){
    activateRGB(RED);
    delay(100);
    activateRGB(GREEN);
    delay(100);
    activateRGB(BLUE);
    delay(100);
  }
  
  minR_Sval = 0;
  maxR_Sval = 1;
  minL_Sval = 0;
  maxL_Sval = 1;
  
  
  int i;
  for (i = 0; i < CALIB_TIME; i++)  // make the calibration take about 5 seconds
  {
    R_R_Sval = analogRead(R_R_S);
    if(R_R_Sval < minR_R_Sval){minR_R_Sval = R_R_Sval;}
    if(R_R_Sval > maxR_R_Sval){maxR_R_Sval = R_R_Sval;}
//    R_Sval = digitalRead(R_S);
//    if(R_Sval < minR_Sval){minR_Sval = R_Sval;}
//    if(R_Sval > maxR_Sval){maxR_Sval = R_Sval;}
//    L_Sval = digitalRead(L_S);
//    if(L_Sval < minL_Sval){minL_Sval = L_Sval;}
//    if(L_Sval > maxL_Sval){maxL_Sval = L_Sval;}
    L_L_Sval = analogRead(L_L_S);
    if(L_L_Sval < minL_L_Sval){minL_L_Sval = L_L_Sval;}
    if(L_L_Sval > maxL_L_Sval){maxL_L_Sval = L_L_Sval;}

    if(debugClb){
      Serial.print("calib R_R_Sval min: "); Serial.print(minR_R_Sval);  Serial.print(" max: "); Serial.print(maxR_R_Sval); Serial.println();
      Serial.print(" calib R_Sval min: "); Serial.print(minR_Sval);    Serial.print(" max: "); Serial.print(maxR_Sval); Serial.println();
      Serial.print(" calib L_Sval min: "); Serial.print(minL_Sval);  Serial.print(" max: ");   Serial.print(maxL_Sval); Serial.println();
      Serial.print(" calib L_L_Sval min: "); Serial.print(minL_L_Sval);  Serial.print(" max: ");   Serial.print(maxL_L_Sval); Serial.println();
      Serial.println();
    }
    delay(20);
    
  }

  for(int i = 0; i<5; i++){
    activateRGB(BLUE);
    delay(100);
    activateRGB(GREEN);
    delay(100);
    activateRGB(RED);
    delay(100);
  }
  
  activateRGB(WHITE);
}

void steerCar()
{

  if(proximity_data > PRX_SENSOR_DIST){
    if(debugDrv){Serial.println("proximity close stop");}
    Stop();
  }
  else
  {
    if(debugSens){Serial.print(" RRS: "); Serial.print(sensorValueR_R_S); Serial.print(" LLS: "); Serial.print(sensorValueL_L_S); Serial.println();}

    if(!sensorValueR_S && !sensorValueL_S && sensorValueR_R_S < SENSOR_MID && sensorValueL_L_S < SENSOR_MID)
    { 
      // before empty space only center sensors were on black line
      // so go forward
      if(lastDirection > TURN_RIGHT_90)
      {
        if(debugDrv){Serial.println("go forward");}
        forward();
      }
      else{
        if(lastDirection == TURN_LEFT_90)
        {
          if(!lastSesorStatus[LS])
          {
            if(debugDrv){Serial.println("turn wide left");}
            turnWideLeft();
          }
        }
        else if(lastDirection == TURN_RIGHT_90)
        {
          if(!lastSesorStatus[RS])
          {
            if(debugDrv){Serial.println("turn wide right");}
            turnWideRight();
          }
        }
      }
    }
    // LLS and RRS have priority. If they see something they must turn towards it
    else if(sensorValueR_R_S > SENSOR_MID || sensorValueL_L_S > SENSOR_MID)
    {
      if((sensorValueR_R_S > SENSOR_MID)&&(sensorValueL_L_S > SENSOR_MID)){
        forward(); 
        if(debugDrv){Serial.println("forward intersection");}
        
        lastDirection = FORWARD_INTERSECTION;
        setSensorStatus(sensorValueL_L_S, sensorValueL_S, sensorValueR_S, sensorValueR_R_S);
        
      }   //if Right Sensor and Left Sensor are at Black color then it will call forword function
      if((sensorValueR_R_S < SENSOR_MID)&&(sensorValueL_L_S > SENSOR_MID)){
        turn90Left(); 
        if(debugDrv){Serial.println("turn 90 Left");}
        
        lastDirection = TURN_LEFT_90;
        setSensorStatus(sensorValueL_L_S, sensorValueL_S, sensorValueR_S, sensorValueR_R_S);
        
        
      } //if Right Sensor is Black and Left Sensor is White then it will call turn Right function  
      if((sensorValueR_R_S > SENSOR_MID)&&(sensorValueL_L_S < SENSOR_MID)){
        turn90Right(); 
        if(debugDrv){Serial.println("turn 90 Right");}
        
        lastDirection = TURN_RIGHT_90;
        setSensorStatus(sensorValueL_L_S, sensorValueL_S, sensorValueR_S, sensorValueR_R_S);
        
      }  //if Right Sensor is White and Left Sensor is Black then it will call turn Left function
    }
    else{
      if((sensorValueR_S)&&(sensorValueL_S)){
        forward(); 
        if(debugDrv){Serial.println("forward");}
        StartTime = millis();

        lastDirection = FORWARD;
        setSensorStatus(sensorValueL_L_S, sensorValueL_S, sensorValueR_S, sensorValueR_R_S);
        
      }   //if Right Sensor and Left Sensor are at Black color then it will call forword function
      if((!sensorValueR_S)&&(sensorValueL_S)){
        turnLeft(); 
        StartTime = millis();
        lastDirection = TURN_LEFT;
        setSensorStatus(sensorValueL_L_S, sensorValueL_S, sensorValueR_S, sensorValueR_R_S);
        
        if(debugDrv){Serial.println("turnLeft");}
      } //if Right Sensor is Black and Left Sensor is White then it will call turn Right function  
      if((sensorValueR_S)&&(!sensorValueL_S)){
        turnRight(); 
        if(debugDrv){Serial.println("turnRight");}
        StartTime = millis();
        lastDirection = TURN_RIGHT;
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
  analogWrite(motorASpeed, 90);   //Spins the motor on Channel A at full speed

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
  analogWrite(motorASpeed, 125);   //Spins the motor on Channel A at full speed

  //Motor B
  digitalWrite(motorBDirection, HIGH);  //Establishes backward direction of Channel B
  digitalWrite(motorBBrake, LOW);   //Disengage the Brake for Channel B
  analogWrite(motorBSpeed, 90);    //Spins the motor on Channel B at half speed
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
  else if(color == WHITE)
  {
    analogWrite(ledRed,255); 
    analogWrite(ledGreen,255);  
    analogWrite(ledBlue,255);
  }
  else if(color == BLACK)
  {
    analogWrite(ledRed,0); 
    analogWrite(ledGreen,0);  
    analogWrite(ledBlue,0);
  }
}

void setSensorStatus(int lls, int ls, int rs, int rrs)
{
  // set status only if sensors have changed
  if(lastSesorStatus[RRS] != rrs ||
      lastSesorStatus[RS] != rs ||
      lastSesorStatus[LS] != ls ||
      lastSesorStatus[LLS] != lls)
      {
        lastSesorStatus[RRS] = rrs;
        lastSesorStatus[RS] = rs;
        lastSesorStatus[LS] = ls;
        lastSesorStatus[LLS] = lls;
      }
}
