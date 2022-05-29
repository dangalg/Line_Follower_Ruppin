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

// ir sensor port connection data
#define R_R_S A2 //ir sensor Right
#define R_S 2 //ir sensor Right
#define C_S 7 //ir sensor center (the ram)
#define L_S 4 //ir sensor Left
#define L_L_S A3 //ir sensor Left

// raw value
float R_R_Sval;
float R_Sval;
float C_Sval;
float L_Sval;
float L_L_Sval;

// adjusted value
float sensorValueR_R_S;
float sensorValueR_S;
float sensorValueC_S;
float sensorValueL_S;
float sensorValueL_L_S;

// min value
float minR_R_Sval = 9999;
float minR_Sval = 0;
float minC_Sval = 0;
float minL_Sval = 0;
float minL_L_Sval = 9999;

// max value
float maxR_R_Sval = 0;
float maxR_Sval = 1;
float maxC_Sval = 1;
float maxL_Sval = 1;
float maxL_L_Sval = 0;

// sensor definitions
#define SENSOR_MID 380 // sensor on edge of black line
#define SENSOR_MAX 1000 // sensor in the middle of black
#define PRX_SENSOR_DIST 160 // distance to stop proximity sensor

enum MoveType {
  FORWARD_INTERSECTION,
  TURN_LEFT_90,
  TURN_RIGHT_90,
  FORWARD,
  TURN_LEFT,
  TURN_RIGHT
};

#define CALIB_TIME 250 // loops of calibration

int lastDirection = 1; // 0 left 1 right : saved last direction

int lastSesorStatus[5] = {0};
enum SensorType {
  RRS,
  RS, 
  CS,
  LS,
  LLS
};

// motor port connection data
int motorADirection = 12;
int motorABrake = 9;
int motorASpeed = 3;
int motorBDirection = 13;
int motorBBrake = 8;
int motorBSpeed = 11;

// motor speeds
#define FAST_SPD 200.0 //
#define MED_SPD 150.0 //
#define SLOW_SPD 30.0 //
#define TURN_SPD 90.0 //
float speedPercent = 1.0;
#define MAX_SPD_PRECENT 1.7
#define MED_SPD_PRECENT 0.9
#define SLOW_SPD_PRECENT 0.7
bool cameFromWhite = 0;

// RGB Led data
enum ColorType {
  RED,
  GREEN, 
  BLUE,
  WHITE,
  BLACK
};

// led connection data
int ledRed = 5; 
int ledGreen = 6;  
int ledBlue = 10;

// set these to get serial print of data, on real run disable these
int debug = 0; // set to 1 if serial debug output needed
int debugDrv = 0;
int debugSens = 0;
int debugLit = 0;
int debugPrx = 0;
int debugClb = 0;
int debugLedDrv = 0;

// time
unsigned long StartForwardTime;
unsigned long CurrentForwardTime;
unsigned long ElapsedForwardTime;

unsigned long StartWhiteSpaceTime;
unsigned long CurrentWhiteSpaceTime;
unsigned long ElapsedWhiteSpaceTime;

unsigned long StartBackupTime;
unsigned long CurrentBackupTime;
unsigned long ElapsedBackupTime;

#define MAX_BACKUP_TIME 2000 //max backup time
#define MAX_FORWARD_TIME 2000 //max backup time
#define MAX_WHITESPACE_TIME 500 //max backup time


void setup() {
  if(debug){
    Serial.begin(9600);
  }

  manual_calibration();
  setupAPDS();

}

void loop() {
  checkProximity();
  checkColor();
  getDataFromSensors();
  steerCar();
  //forward();
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
  apds.readBlueLight(blue_light);
  if(debugLit){Serial.print("red_light: ");Serial.print(red_light);
                Serial.print(" green_light: ");Serial.print(green_light);
                Serial.print(" blue_light: ");Serial.print(blue_light);
                Serial.println();}

  // display the brightest color caught
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
  // read sensors
  R_R_Sval = analogRead(R_R_S);
  R_Sval = digitalRead(R_S);
  C_Sval = digitalRead(C_S);
  L_Sval = digitalRead(L_S);
  L_L_Sval = analogRead(L_L_S);

  // pass data to sensor adjusted
  sensorValueR_R_S = R_R_Sval;
  sensorValueR_S = R_Sval;
  sensorValueC_S = C_Sval;
  sensorValueL_S = L_Sval;
  sensorValueL_L_S = L_L_Sval;

  // adjust sensor data
  sensorValueR_R_S = map(R_R_Sval, minR_R_Sval, maxR_R_Sval,0,SENSOR_MAX);
  //sensorValueR_S = map(R_Sval, minR_Sval, maxR_Sval,0,SENSOR_MAX);
  //sensorValueL_S = map(L_Sval, minL_Sval, maxL_Sval,0,SENSOR_MAX);
  sensorValueL_L_S = map(L_L_Sval, minL_L_Sval, maxL_L_Sval,0,SENSOR_MAX);

  // fix if lower than 0 and greater than max
  if(sensorValueR_R_S < 0){sensorValueR_R_S = 0;}
  if(sensorValueR_R_S > SENSOR_MAX) {sensorValueR_R_S = SENSOR_MAX;}
  if(sensorValueL_L_S < 0){sensorValueL_L_S = 0;}
  if(sensorValueL_L_S > SENSOR_MAX){sensorValueL_L_S = SENSOR_MAX;}
}

void manual_calibration()
{
  activateRGB(BLACK);
  delay(1000);

  // blink lights to signify calibration start
  for(int i = 0; i<5; i++){
    activateRGB(GREEN);
    delay(100);
    activateRGB(BLUE);
    delay(100);
  }

  // set digital sensor max min
  minR_Sval = 0;
  maxR_Sval = 1;
  minL_Sval = 0;
  maxL_Sval = 1;
  

  // set analog sensor max min
  int i;
  for (i = 0; i < CALIB_TIME; i++)  // make the calibration take about 5 seconds
  {
    R_R_Sval = analogRead(R_R_S);
    if(R_R_Sval < minR_R_Sval){minR_R_Sval = R_R_Sval;}
    if(R_R_Sval > maxR_R_Sval){maxR_R_Sval = R_R_Sval;}
    L_L_Sval = analogRead(L_L_S);
    if(L_L_Sval < minL_L_Sval){minL_L_Sval = L_L_Sval;}
    if(L_L_Sval > maxL_L_Sval){maxL_L_Sval = L_L_Sval;}

    if(debugClb){
      Serial.print(" calib L_L_Sval min: "); Serial.print(minL_L_Sval);  Serial.print(" max: ");   Serial.print(maxL_L_Sval);
      Serial.print("calib R_R_Sval min: "); Serial.print(minR_R_Sval);  Serial.print(" max: "); Serial.print(maxR_R_Sval);Serial.println();
      Serial.println();
    }
    delay(20);
    
  }

  // blink lights to signify calibration end
  for(int i = 0; i<5; i++){
    activateRGB(BLUE);
    delay(100);
    activateRGB(RED);
    delay(100);
  }

  // show white light to signify end of calibration
  activateRGB(WHITE);
  delay(1000);
}

void steerCar()
{

  // if close to object stop
  if(proximity_data > PRX_SENSOR_DIST){
    if(debugDrv){Serial.print("proximity close stop: "); Serial.println(proximity_data);}
    Stop();
  }
  else
  {
    
    if(debugSens){
      Serial.print(" LLS: "); Serial.print(sensorValueL_L_S);
      Serial.print(" RS: "); Serial.print(sensorValueL_S);
      Serial.print(" RS: "); Serial.print(sensorValueR_S);
      Serial.print(" RRS: "); Serial.print(sensorValueR_R_S);
      Serial.println();
    }

    // all sensors show nothing
    if(!sensorValueR_S && !sensorValueL_S && sensorValueR_R_S < SENSOR_MID && sensorValueL_L_S < SENSOR_MID)
    { 
      cameFromWhite =0;
      handleWhiteSpace();
    }
    // LLS and RRS have priority. If they see something they must turn towards it
    
    else 
    {
      CurrentWhiteSpaceTime = millis();
      ElapsedWhiteSpaceTime = CurrentWhiteSpaceTime - StartWhiteSpaceTime;
      
      // last time sensors were on white
      if(lastSesorStatus[RRS] < SENSOR_MID ||
      lastSesorStatus[RS] == 0 ||
      lastSesorStatus[LS] == 0 ||
      lastSesorStatus[LLS] < SENSOR_MID)
      {
        cameFromWhite =1;
        StartWhiteSpaceTime = millis();
      }

      if(ElapsedWhiteSpaceTime > MAX_WHITESPACE_TIME)
      {
        cameFromWhite =0;
      }

      if(cameFromWhite == 1)
      {
        speedPercent = SLOW_SPD_PRECENT;
      }
      else
      {
        speedPercent = MAX_SPD_PRECENT;
      }

      StartForwardTime = millis();
      if(sensorValueR_R_S > SENSOR_MID || sensorValueL_L_S > SENSOR_MID)
      {
        // this is an intersection
        if((sensorValueR_R_S > SENSOR_MID)&&(sensorValueL_L_S > SENSOR_MID)){
          if(debugLedDrv){activateRGB(WHITE);}
          speedPercent = MED_SPD_PRECENT;
          moveCar(FORWARD_INTERSECTION, String("forward intersection"));
  
        }   
        // this is for 90 degree turn left
        else if((sensorValueR_R_S < SENSOR_MID)&&(sensorValueL_L_S > SENSOR_MID)){
          if(debugLedDrv){activateRGB(WHITE);}
          moveCar(TURN_LEFT_90, String("turn 90 Left"));
  
        }
        
        // this is for 90 degree turn right
        else if((sensorValueR_R_S > SENSOR_MID)&&(sensorValueL_L_S < SENSOR_MID)){
          if(debugLedDrv){activateRGB(WHITE);}
          moveCar(TURN_RIGHT_90, String("turn 90 Right"));
       
        }
      }
      else{
        // both center sennsors are on black, go forward
        if((sensorValueR_S)&&(sensorValueL_S)){
          if(debugLedDrv){activateRGB(BLACK);}
          moveCar(FORWARD, String("forward"));
          
        }   
        // left on black, turn left
        else if((!sensorValueR_S)&&(sensorValueL_S)){
          if(debugLedDrv){activateRGB(BLACK);}
          moveCar(TURN_LEFT, String("turn left"));
  
        }   
        // right on black, turn right
        else if((sensorValueR_S)&&(!sensorValueL_S)){
          if(debugLedDrv){activateRGB(BLACK);}
          moveCar(TURN_RIGHT, String("turn Right"));
          
        }
      }
    }
  }
}

void handleWhiteSpace()
{
  CurrentForwardTime = millis();
  ElapsedForwardTime = CurrentForwardTime - StartForwardTime;

  // if not enough time has passed 
  if(ElapsedForwardTime < MAX_FORWARD_TIME)
  {
    speedPercent = SLOW_SPD_PRECENT;
    if(sensorValueC_S)
    {
      if(debugLedDrv){activateRGB(RED);}
      // ram detected black so go forward
      forward();
    }
    else if(lastDirection > TURN_RIGHT_90)
    {
      if(debugLedDrv){activateRGB(GREEN);}
      // before empty space only center sensors were on black line
      // so go forward
      forward();
    }
    else{
      if(debugLedDrv){activateRGB(BLUE);}
      forward();
    }
  }
  // else go back to track
  else
  {
//    CurrentBackupTime = millis();
//    ElapsedBackupTime = CurrentBackupTime - StartBackupTime;
//    if(ElapsedTime < MAX_BACKUP_TIME)
//    {
      speedPercent = MED_SPD_PRECENT;
      backup();
//    }
  }
}

bool inspectWhiteSpace(bool goForward)
{
  // go forward to inspect until black found for 10 times
  bool foundBlack = false;
  for(int i = 0; i < 10 && !foundBlack; i++)
  {

    // go forwards or backwards
    if(goForward){forward();}
    else{backup();}
    delay(100);

    // stop 
    Stop();
    
    //check sensors
    getDataFromSensors();
    
    // if found black return info
    if(sensorValueR_S || sensorValueL_S || sensorValueR_R_S > SENSOR_MID || sensorValueL_L_S > SENSOR_MID)
    {
      foundBlack = true;
    }
  }
  return foundBlack;
}

void moveCar(MoveType mt, String name)
{
  switch (mt) {
    case FORWARD_INTERSECTION:
        forwardStrong(200);
      break;
    case TURN_LEFT_90:
        turn90Left();
      break;
    case TURN_RIGHT_90:
      turn90Right();
    break;
    case FORWARD:
      forward();
    break;
    case TURN_LEFT:
      turnLeft();
    break;
    case TURN_RIGHT:
      turnRight();
    break;
    default:
      forward();
      break;
  }
  
  if(debugDrv){
      Serial.println(name);
      Serial.println(speedPercent);
    }
  
  lastDirection = mt;
  setSensorStatus(sensorValueR_R_S, sensorValueR_S, sensorValueC_S, sensorValueL_S, sensorValueL_L_S);
}

void forward(){  //forword
  controlMotors(HIGH, LOW, MED_SPD*speedPercent,HIGH, LOW, MED_SPD*speedPercent);
}

void forwardStrong(int millis)
{
  forward();
  delay(millis);
}

void turnRight(){ //turnRight
  controlMotors(HIGH, LOW, SLOW_SPD*speedPercent,HIGH, LOW, MED_SPD*speedPercent);
}

void turn90Right(){ //turn90Right
  controlMotors(LOW, LOW, MED_SPD*speedPercent,HIGH, LOW, FAST_SPD*speedPercent);
}

void turnWideRight(){ //turnWideRight
  controlMotors(HIGH, LOW, TURN_SPD*speedPercent,HIGH, LOW, MED_SPD*speedPercent);
}

void turnLeft(){ //turnLeft
  controlMotors(HIGH, LOW, MED_SPD*speedPercent,HIGH, LOW, SLOW_SPD*speedPercent);
}

void turn90Left(){ //turn90Left
  controlMotors(HIGH, LOW, FAST_SPD*speedPercent,LOW, LOW, MED_SPD*speedPercent);
}

void turnWideLeft(){ //turnWideLeft
  controlMotors(HIGH, LOW, MED_SPD*speedPercent,HIGH, LOW, TURN_SPD*speedPercent);
}
void Stop(){ //stop
  controlMotors(HIGH, HIGH, 0,HIGH, HIGH, 0);
}
void backup(){ //backup
  controlMotors(LOW, LOW, MED_SPD*speedPercent,LOW, LOW, MED_SPD*speedPercent);
}

void controlMotors(int ADirection, int ABrake, int ASpeed,int BDirection, int BBrake, int BSpeed)
{
  //Motor A
  digitalWrite(motorADirection, ADirection); //Establishes forward direction of Channel A
  digitalWrite(motorABrake, ABrake);   //Disengage the Brake for Channel A
  analogWrite(motorASpeed, ASpeed);   //Spins the motor on Channel A at full speed

  //Motor B
  digitalWrite(motorBDirection, BDirection);  //Establishes backward direction of Channel B
  digitalWrite(motorBBrake, BBrake);   //Disengage the Brake for Channel B
  analogWrite(motorBSpeed, BSpeed);    //Spins the motor on Channel B at half speed
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

void activateRGBCode(int red, int green, int blue)
{
  analogWrite(ledRed,red); 
  analogWrite(ledGreen,green);  
  analogWrite(ledBlue,blue);
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

// save last sensor data
void setSensorStatus(int rrs, int rs, int cs, int ls, int lls)
{
  // set status only if sensors have changed
  if(lastSesorStatus[RRS] != rrs ||
      lastSesorStatus[RS] != rs ||
      lastSesorStatus[CS] != cs ||
      lastSesorStatus[LS] != ls ||
      lastSesorStatus[LLS] != lls)
      {
        lastSesorStatus[RRS] = rrs;
        lastSesorStatus[RS] = rs;
        lastSesorStatus[CS] = cs;
        lastSesorStatus[LS] = ls;
        lastSesorStatus[LLS] = lls;
      }
}
