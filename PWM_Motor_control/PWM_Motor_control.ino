#include <PWM.h>

enum{MOTOR_L=2,MOTOR_S=3,CO2VELVE_L=10,CO2VELVE_S=8};


void setup() {
  Serial.begin(9600);
  InitTimersSafe();
  pinMode(MOTOR_S, OUTPUT);
  digitalWrite(MOTOR_S, HIGH);
  digitalWrite(4,LOW);
  SetPinFrequencySafe(MOTOR_L,400);
}

void loop() {
  pwmWriteHR(MOTOR_L,65536);
  //analogWrite(MOTOR_L, 255);
  delay(1000);
}
