// Arduino Line Follower Robot Code

#define enA 5//Enable1 L293 Pin enA 
#define in1 3 //Motor1  L293 Pin in1 
#define in2 4 //Motor1  L293 Pin in1 
#define in3 8 //Motor2  L293 Pin in1 
#define in4 7 //Motor2  L293 Pin in1 
#define enB 6 //Enable2 L293 Pin enB 
#define R_S 13//ir sensor Right
#define L_S 12 //ir sensor Left

void setup(){ 
  Serial.begin(9600);
  pinMode(R_S, INPUT); 
  pinMode(L_S, INPUT); 
  pinMode(enA, OUTPUT); 
  pinMode(in1, OUTPUT); 
  pinMode(in2, OUTPUT); 
  pinMode(in3, OUTPUT); 
  pinMode(in4, OUTPUT); 
  pinMode(enB, OUTPUT);
  analogWrite(enA, 255); 
  analogWrite(enB, 255); 
  delay(1000);
}
void loop(){  
  if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1)){
    forward(); 
    //Serial.println("forward");
  }   //if Right Sensor and Left Sensor are at White color then it will call forword function
  if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 0)){
    turnLeft(); 
    //Serial.println("turnRight");
  } //if Right Sensor is Black and Left Sensor is White then it will call turn Right function  
  if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 1)){
    turnRight(); 
    //Serial.println("turnLeft");
  }  //if Right Sensor is White and Left Sensor is Black then it will call turn Left function
  if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 0)){
    Stop(); 
    //Serial.println("Stop");
  } //if Right Sensor and Left Sensor are at Black color then it will call Stop function
}
void forward(){  //forword
  digitalWrite(in1, HIGH); //Right Motor forword Pin 
  digitalWrite(in2, LOW);  //Right Motor backword Pin 
  digitalWrite(in3, LOW);  //Left Motor backword Pin 
  digitalWrite(in4, HIGH); //Left Motor forword Pin 
}
void turnRight(){ //turnRight
  digitalWrite(in1, LOW);  //Right Motor forword Pin 
  digitalWrite(in2, HIGH); //Right Motor backword Pin  
  digitalWrite(in3, LOW);  //Left Motor backword Pin 
  digitalWrite(in4, HIGH); //Left Motor forword Pin 
}
void turnLeft(){ //turnLeft
  digitalWrite(in1, HIGH); //Right Motor forword Pin 
  digitalWrite(in2, LOW);  //Right Motor backword Pin 
  digitalWrite(in3, HIGH); //Left Motor backword Pin 
  digitalWrite(in4, LOW);  //Left Motor forword Pin 
}
void Stop(){ //stop
  digitalWrite(in1, LOW); //Right Motor forword Pin 
  digitalWrite(in2, LOW); //Right Motor backword Pin 
  digitalWrite(in3, LOW); //Left Motor backword Pin 
  digitalWrite(in4, LOW); //Left Motor forword Pin 
}
