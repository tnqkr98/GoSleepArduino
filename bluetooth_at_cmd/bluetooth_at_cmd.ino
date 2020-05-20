//#include <SoftwareSerial.h>

//SoftwareSerial BTSerial(11, 10);   

// AT 이름설정 뒤에 버퍼껴들어가면 AT+RESET 수행 한번 해보자

void setup() {
  Serial.begin(9600);
  //BTSerial.begin(9600);
  Serial2.begin(9600);
  Serial.println("ATcommand");  //ATcommand Start
}

void loop() {
  if (Serial2.available())
    Serial.write(Serial2.read());
  if (Serial.available())
    Serial2.write(Serial.read());
  /*if (BTSerial.available())
    Serial.write(BTSerial.read());
  if (Serial.available())
    BTSerial.write(Serial.read());*/
 //Serial.println("gigi");
 //delay(1000);
}
