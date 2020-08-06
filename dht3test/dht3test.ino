#include "DHT.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

DHT dht0(A0, DHT11);
DHT dht1(A1, DHT11);
DHT dht2(A2, DHT11);
LiquidCrystal_I2C lcd(0x27, 16, 2);

float h0,h1,h2,t0,t1,t2;
int c0=0,c1=0,c2=0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  dht0.begin();
  dht1.begin();
  dht2.begin();
  lcd.init();
  //lcd.backlight();
}

void loop() {
  h0 = dht0.readHumidity();
  t0 = dht0.readTemperature();
  h1 = dht1.readHumidity();
  t1 = dht1.readTemperature();
  h2 = dht2.readHumidity();
  t2 = dht2.readTemperature();
  if(isnan(h0) || isnan(t0))
     c0++;
  if(isnan(h1) || isnan(t1))
     c1++;
  if(isnan(h2) || isnan(t2))
     c2++;
  lcd.setCursor(0,0);
  Serial.print("1번 온도 : ");Serial.print(t0);
  Serial.print(", 2번 온도 : ");Serial.print(t1);
  Serial.print(", 3번 온도 : ");Serial.println(t2);
  lcd.print("a:");  lcd.print((int)t0);  
  lcd.print(" b:");  lcd.print((int)t1);  
  lcd.print(" c:");  lcd.print((int)t2);  
  
  Serial.print("1번 에러 횟수 : ");Serial.print(c0);
  Serial.print(", 2번 에러 횟수 : ");Serial.print(c1);
  Serial.print(", 3번 에러 횟수 : ");Serial.println(c2);
  lcd.setCursor(0,1);
  lcd.print("e:");  lcd.print((int)c0);  
  lcd.print(" e:");  lcd.print((int)c1);  
  lcd.print(" e:");  lcd.print((int)c2); 
  delay(1000);
  lcd.clear();
}
