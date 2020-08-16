#include <SoftwareSerial.h>
/*#define UNO_TEST 0
#define RX 2
#define TX 3
SoftwareSerial BTSerial(RX, TX);*/   

// AT 이름설정 뒤에 버퍼 껴들어가면 AT+RESET 수행 한번 해보자

void setup() {
  Serial.begin(9600);
  /*if(UNO_TEST)
      BTSerial.begin(9600);
  else*/
      Serial2.begin(9600);
  Serial.println("ATcommand");  //ATcommand Start
}

void loop() {
  /*if(UNO_TEST){
    if (BTSerial.available())
        Serial.write(BTSerial.read());
    if (Serial.available())
       BTSerial.write(Serial.read());
  }
  else{*/
    if (Serial2.available())
      Serial.write(Serial2.read());
    if (Serial.available())
      Serial2.write(Serial.read());
  //}
}
