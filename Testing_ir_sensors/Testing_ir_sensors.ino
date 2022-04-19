// ir sensor data
#define R_R_S 2 //ir sensor Right
#define R_S 3 //ir sensor Right
#define C_S 1 //ir sensor Right
#define L_S 4 //ir sensor Left
#define L_L_S 5 //ir sensor Left

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  float R_R_Sval;
  R_R_Sval = analogRead(R_R_S);
  float R_Sval;
  R_Sval = analogRead(R_S);
  float C_Sval;
  C_Sval = analogRead(C_S);
  float L_Sval;
  L_Sval = analogRead(L_S);
  float L_L_Sval;
  L_L_Sval = analogRead(L_L_S);
  Serial.print(R_R_Sval); Serial.print(" ");
  Serial.print(R_Sval); Serial.print(" ");
  Serial.print(C_Sval); Serial.print(" ");
  Serial.print(L_Sval); Serial.print(" ");
  Serial.print(L_L_Sval); Serial.print(" ");
  Serial.println();
  Serial.println();
  delay(100);
}
