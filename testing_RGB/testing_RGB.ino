
// RGB Led data
#define RED 90
#define GREEN 91
#define BLUE 92

int ledRed = 5; 
int ledGreen = 6;  
int ledBlue = 10;

void setup() {
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledBlue, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  activateRGB(RED);
  delay(3000);
  activateRGB(GREEN);
  delay(3000);
  activateRGB(BLUE);
  delay(3000);
}

//void activateRGB(int color)
//{
//
//  if(color == RED)
//  {
//    digitalWrite(ledRed,HIGH); 
//    digitalWrite(ledGreen,LOW);  
//    digitalWrite(ledBlue,LOW);
//  }
//  else if(color == GREEN)
//  {
//    digitalWrite(ledRed,LOW); 
//    digitalWrite(ledGreen,HIGH);  
//    digitalWrite(ledBlue,LOW);
//  }
//  else if(color == BLUE)
//  {
//    digitalWrite(ledRed,LOW); 
//    digitalWrite(ledGreen,LOW);  
//    digitalWrite(ledBlue,HIGH);
//  }
//}

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
