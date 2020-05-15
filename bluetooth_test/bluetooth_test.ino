#include "DHT.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_NeoPixel.h>

#define DHTPIN    A0
#define DHTTYPE   DHT11

#define BLUETOOTHWAITING  5     // 5초 이상 안드로이드로 부터 ack 받지 못하면 연결 끊긴것(송수신 범위 벗어남)
#define SENDING_TICK      3     // 3초에 한번씩 안드로이드로 센싱값 전송
#define NUM_PIXELS        4     // 네오픽셀 LED 개수 

enum{RX=2,TX,LED_PIN};          // 핀 번호
enum{STOP_MODE=1,WAIT_MODE,DIST_MODE,SLEEP_MODE,SENS_MODE,WAKE_MODE};

DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial BTserial(RX,TX);
DS3231 rtc;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_PIXELS,LED_PIN, NEO_GRBW + NEO_KHZ800);

int MODE = 2, fanS = 80, brightness = 50;

char c,buf[2],buf3[3],buf_rgb[3][4];      // 각종 읽기 버퍼
short time[4],t=0,bluetoothCount = 0;
bool SetAlramOn = false, BluetoothOn = false, GoodDistance = false;
bool FanOn = true, VelveOn = false, HeatOn = false, MoodLedOn = false;

void parseAndroidMessage();               // 안드로이드 수신 메시지 분석 후 출력 함수
void sendAndroidMessage();                // 안드로이드 발신 메시지 설정 함수
void rawMessage();                        // 안드로이드 수신 메시지 출력 함수
void printLog();                          // 로그 출력용 함수
void distanceCheck();                     // 거리 측정 모드 동작 함수
void alarmRinging();                      // 알람 시각 동작 함수
void moodLedControl(int r,int g,int b);   // 조명 제어 함수

void setup(){
  Wire.begin();
  Serial.begin(9600);
  BTserial.begin(9600);

  pixels.setBrightness(50);
  pixels.begin();
  pixels.show();
}

void loop(){
  DateTime now = rtc.now();
  if(SetAlramOn){
      if(time[0] == now.month() && time[1] == now.day() && time[2] == now.hour() && time[3] == now.minute()){
        Serial.println("알람~~~~~~~~~~~~~~~~");
        SetAlramOn = false;
      }
  }

  //rawMessage();
  parseAndroidMessage();
  sendAndroidMessage();
  printLog();
  
  switch(MODE){
      case DIST_MODE:
          break;
      case SLEEP_MODE:
          break;
  }
  
  if(MODE == 7) MODE = 1;
  if(MODE == 0) MODE = 6;

  bluetoothCount++;
  if(bluetoothCount == BLUETOOTHWAITING*1000){
    bluetoothCount = 0;
    BluetoothOn = false;
  }
  //delay(1000);
}

/*----------------------------------- 안드로이드 발신 메시지 설정 함수 */
void sendAndroidMessage(){
    static int sendTime = 0;
    sendTime++;
    if(sendTime == SENDING_TICK*1000){
      int h = dht.readHumidity();
      int t = dht.readTemperature();
      BTserial.print(h);BTserial.print(",");BTserial.print(t);      // 온습도상태 송신
      BTserial.print(",");BTserial.print(fanS);BTserial.print(","); // 팬속도 송신
      BTserial.println(MODE);   // 현재모드상태 송신
      sendTime = 0;
    }
}
/*----------------------------------- 안드로이드 실제 수신 메시지 출력 */
void rawMessage(){
  //parseAndroidMessage 와 동시사용 불가
  while(BTserial.peek()!=-1)
    Serial.write(BTserial.read());
}

/*----------------------------------- 안드로이드 수신 메시지 분석 함수 */
void parseAndroidMessage(){
  int readHead;
  if(BTserial.available())
    BluetoothOn = true;
  
  if(BTserial.peek()!=-1){
    readHead = BTserial.read();
    switch(readHead){
      case 'a':   // 안드로이드와 현재 연결 상태인가?
          BluetoothOn = true;
          bluetoothCount = 0;
          break;
      case 'm':   // 모드 변경 
          delay(5);
          c = BTserial.read();
          if(c == 'n'){
              Serial.println("From Android >> 다음 모드로 이동");
              switch(MODE){
                case DIST_MODE:
                    if(!GoodDistance)
                      Serial.println("다음 모드로 이동 불가(거리가 적당하지 않음)");
                    else
                      MODE++;
                    break;
                default :
                    MODE++;
                    break;
              }
          }
          else if(c == 'b'){
            Serial.println("From Android >> 이전 모드로 이동");
            MODE--;
          }
          break;
      case 't':   // 알람 시각 설정
          memset(buf,'\0',sizeof(buf));
          Serial.print("From Android >> 알람 설정 시간 : ");
          for(int i=0;i<8;i++){
            buf[i%2]=BTserial.read();
            delay(5);
            if(i%2 == 1)
              time[t++] = atoi(buf);
          }
          Serial.print(time[0]);Serial.print("월 ");
          Serial.print(time[1]);Serial.print("일 ");
          Serial.print(time[2]);Serial.print("시 ");
          Serial.print(time[3]);Serial.println("분 ");
          t=0;
          SetAlramOn = true;
          break;
      case 'v':   // 밸브 on/off 설정
          if(BTserial.read() == '1'){
              VelveOn = true;
              Serial.println("Velve ON");
          }
          else{
              VelveOn = false;
              Serial.println("Velve OFF");
          }
          break;
      case 'h':
          if(BTserial.read() == '1'){
              HeatOn = true;
              Serial.println("Heat ON");
          }
          else{
              HeatOn = false;
              Serial.println("Heat OFF");
          }
          break;
      case 'f':   // 팬 속도 설정
          delay(10); // 너무 빨리 읽으면 문자열이 깨짐(혹은 쓰레기값)
          if(BTserial.peek() == 's'){
            BTserial.read(); delay(10);
            for(int i=0;i<3;i++){
              buf3[i] = BTserial.read();
              delay(10);
            }
            Serial.print("팬 속도 설정 : ");
            fanS = atoi(buf3);
            Serial.println(fanS);
            memset(buf3,'\0',sizeof(buf3));
          }
          else{
            if(BTserial.read() == '1'){
                FanOn = true;
                Serial.println("Fan ON");
            }
            else {
                FanOn = false;
                Serial.println("Fan OFF");
            }
          }
          break;
      case 'z':
          Serial.println("CO2 영점 조절");
          break;
      case 'l':   //LED 조절
          if(BTserial.peek() == 'e'){
              Serial.println("MOOD LED OFF");
              pixels.fill(pixels.Color(0, 0, 0), 0, NUM_PIXELS);
              pixels.show();
              MoodLedOn = false;
          }
          else if(BTserial.peek() == 'p'){
              Serial.println("MOOD LED ON");
              BTserial.read(); delay(5);
              for(int i=0;i<3;i++){
                  for(int j=0;j<3;j++){
                      buf_rgb[i][j] = BTserial.read();
                      delay(5);
                  }
                  buf_rgb[i][3] = '\0';
              }
              MoodLedOn = true;
          }
          else if(BTserial.peek() == 'b'){
              BTserial.read();
              for(int i=0;i<3;i++){
                  buf3[i] = BTserial.read();
                  delay(5);
              }
              brightness = atoi(buf3);
              Serial.println(brightness);
          }
          if(MoodLedOn)
              moodLedControl(atoi(buf_rgb[0]),atoi(buf_rgb[1]),atoi(buf_rgb[2]));
          break;
      case 'c':   // 통신 종료 메시지 수신의 경우 종료.
          BluetoothOn = false;
          break;
    }

    while(BTserial.peek() != -1) //남은 버퍼 제거
        BTserial.read();
  }
}
/*----------------------------------- 조명 제어 함수 */
void moodLedControl(int r,int g,int b){
    Serial.print("LED R : ");
    Serial.print(r);
    Serial.print(" G : ");
    Serial.print(g);
    Serial.print(" B : ");
    Serial.print(b);
    Serial.print(" Brightness : ");
    Serial.println(brightness);
    if(brightness !=0)
        pixels.setBrightness(brightness);
    pixels.fill(pixels.Color(r, g, b), 0, NUM_PIXELS);
    pixels.show();
}

/*----------------------------------- 로그 출력용 함수 */
void printLog(){
  static int printTime = 0;
  printTime++;
  if(printTime==1*1000){
     switch(MODE){
        case STOP_MODE:Serial.print(" 현재 상태 : 절전 모드 ");break;
        case WAIT_MODE:Serial.print(" 현재 상태 : 대기 모드 ");break;
        case DIST_MODE:Serial.print(" 현재 상태 : 거리 모드 ");break;
        case SLEEP_MODE:Serial.print(" 현재 상태 : 수면 모드 ");break;
        case SENS_MODE:Serial.print(" 현재 상태 : 센싱 모드 ");break;
        case WAKE_MODE:Serial.print(" 현재 상태 : 기상 모드 ");break;
    }
    if(BluetoothOn)Serial.print(" (안드로이드와 통신 ON) ");
    DateTime now = rtc.now();
    if(SetAlramOn){
        Serial.print("  현재 시각 :");
        Serial.print(now.month()); Serial.print("월");
        Serial.print(now.day()); Serial.print("일");
        Serial.print(now.hour()); Serial.print("시");
        Serial.print(now.minute()); Serial.print("분");
        Serial.print(now.second()); Serial.print("초");
    }
    Serial.println("");
    printTime = 0;
  }
}
