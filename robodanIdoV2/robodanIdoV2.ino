// Arduino Line Follower Robot Code

#define rightMotor 5//Enable1 L293 Pin enA 
#define in1 3 //Motor1  L293 Pin in1 
#define in2 4 //Motor1  L293 Pin in1 
#define in3 2 //Motor2  L293 Pin in1 
#define in4 1 //Motor2  L293 Pin in1 
#define leftMotor 6 //Enable2 L293 Pin enB 
#define R_S 0//ir sensor Right
#define L_S 1 //ir sensor Left

// variables
float rightSensorMin = 9999;
float leftSensorMin = 9999;
float leftSensorMax = 0;
float rightSensorMax = 0;
int rightMotorSpeed = 50;
int leftMotorSpeed = 50;
float P = 0;
float previousError = 0;
float error = 0;
float I = 0;
float D = 0;
float PIDvalue = 0;
float Kp = 25;
float Ki = 12.5;
float Kd = 12.5;

void setup(){ 
  //  Serial.begin(9600);
  pinMode(R_S, INPUT); 
  pinMode(L_S, INPUT); 
  pinMode(rightMotor, OUTPUT); 
  pinMode(in1, OUTPUT); 
  pinMode(in2, OUTPUT); 
  pinMode(in3, OUTPUT); 
  pinMode(in4, OUTPUT); 
  pinMode(leftMotor, OUTPUT);
  analogWrite(rightMotor, 100); 
  analogWrite(leftMotor, 100); 
  delay(1000);
  analogWrite(rightMotor, 0); 
  analogWrite(leftMotor, 0); 
  delay(1000);
  calibrate();
  delay(1000);
}

void loop(){  
  if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 0)){forward(); }   //if Right Sensor and Left Sensor are at White color then it will call forword function
  if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 0)){turnRight();} //if Right Sensor is Black and Left Sensor is White then it will call turn Right function  
  if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 1)){turnLeft();}  //if Right Sensor is White and Left Sensor is Black then it will call turn Left function
  if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1)){Stop(); } //if Right Sensor and Left Sensor are at Black color then it will call Stop function
}
void forward(){  //forword
  digitalWrite(in1, HIGH); //Right Motor forword Pin 
  digitalWrite(in2, LOW);  //Right Motor backword Pin 
  digitalWrite(in3, LOW);  //Left Motor backword Pin 
  digitalWrite(in4, HIGH); //Left Motor forword Pin 
  analogWrite(rightMotor, rightMotorSpeed); 
  analogWrite(leftMotor, leftMotorSpeed); 
}
void turnRight(){ //turnRight
  digitalWrite(in1, LOW);  //Right Motor forword Pin 
  digitalWrite(in2, HIGH); //Right Motor backword Pin  
  digitalWrite(in3, LOW);  //Left Motor backword Pin 
  digitalWrite(in4, HIGH); //Left Motor forword Pin 
  analogWrite(rightMotor, rightMotorSpeed - PIDvalue); 
  analogWrite(leftMotor, leftMotorSpeed - PIDvalue); 
}
void turnLeft(){ //turnLeft
  digitalWrite(in1, HIGH); //Right Motor forword Pin 
  digitalWrite(in2, LOW);  //Right Motor backword Pin 
  digitalWrite(in3, HIGH); //Left Motor backword Pin 
  digitalWrite(in4, LOW);  //Left Motor forword Pin 
  analogWrite(rightMotor, rightMotorSpeed - PIDvalue); 
  analogWrite(leftMotor, leftMotorSpeed - PIDvalue); 
}
void Stop(){ //stop
  digitalWrite(in1, LOW); //Right Motor forword Pin 
  digitalWrite(in2, LOW); //Right Motor backword Pin 
  digitalWrite(in3, LOW); //Left Motor backword Pin 
  digitalWrite(in4, LOW); //Left Motor forword Pin 
  analogWrite(rightMotor, rightMotorSpeed); 
  analogWrite(leftMotor, leftMotorSpeed); 
}

void getSensorData()
{
  int rs = analogRead(r_S);
  int ls = analogRead(L_S);
  rs = map(rs, 0, 1023, rightSensorMin, rightSensorMax);
  ls = map(ls, 0, 1023, leftSensorMin, leftSensorMax);
  
}

void calibrate()
{
  for(int i = 0; i < 500; i++)
  {
    int rs = analogRead(r_S);
    int ls = analogRead(L_S);
    if(rs < rightSensorMin)
    {
      rightSensorMin = rs;
    }
    if(ls < leftSensorMin)
    {
      rightSensorMin = rs;
    }
    if(rs > rightSensorMax)
    {
      rightSensorMax = rs;
    }
    if(ls > leftSensorMax)
    {
      rightSensorMax = rs;
    }
    delay(10);
  }
}

void calculatePID()
{
  P = error;
  I = I + error;
  D = error-previousError;
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
  previousError = error;
}
