#include "DHT.h"
#include "RTClib.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#define DHTPIN    A0

#define BLUETOOTHWAITING  5     // 5초 이상 안드로이드로 부터 ack 받지 못하면 연결 끊긴것(송수신 범위 벗어남)
#define SENDING_TICK      3     // 3초에 한번씩 안드로이드로 센싱값 전송
#define NUM_PIXELS       12     // 네오픽셀 LED 개수 

enum{BT_RX=17,BT_TX=16,LED_PIN=33};          // 핀 번호
enum{STOP_MODE=1,WAIT_MODE,DIST_MODE,SLEEP_MODE,SENS_MODE,WAKE_MODE};

DHT dht(DHTPIN, DHT11);
SoftwareSerial BTserial(BT_RX,BT_TX);
DS3231 rtc;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_PIXELS,LED_PIN, NEO_GRBW + NEO_KHZ800);

short MODE = 2, fanSpeed = 80, brightness = 128;

char c,buf2[2],buf3[3],buf_rgb[3][4];      // 각종 읽기 버퍼
short time[4],t=0,bluetoothCount = 0;
bool SetAlramOn = false, BluetoothOn = false, modeNextEnable = true, modeBackEnable = true;
bool LED_MOOD_ON = false, LED_STATE_ON = false;
bool ON = true, OFF = false;

void parseAndroidMessage();               // 안드로이드 수신 메시지 분석 후 출력 함수
void sendAndroidMessage(bool direct);     // 안드로이드 발신 메시지 설정 함수(0:동기, 1: 비동기)
void rawMessage();                        // 안드로이드 수신 메시지 출력 함수
void printLog(bool direct);               // 로그 출력용 함수
void moodLedControl(int r,int g,int b);   // 조명 제어 함수
void modeControl();                       // 각종 모드 제어 함수
bool distanceCheck();                     // [거리 측정 모드] 동작 함수
void sleepModeWorking();                  // [수면 모드] 동작 함수
void sensingModeWorking();                // [센싱 모드] 동작 함수
void alarmWorking();                      // [기상 모드] 동작 함수
void VELVE(bool in,bool android);         // 이하 모듈 제어(ON/OFF)
void FAN(bool in,bool android);
void HEAT(bool in,bool android);

void _printf(const char *s, ...){
  va_list args;
  va_start(args, s);
  int n = vsnprintf(NULL, 0, s, args);
  char *str = new char[n+1];
  vsprintf(str, s, args);
  va_end(args);
  Serial.print(str);
  delete [] str;
}

void setup(){
  #if defined (__AVR_ATtiny85__)
   if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif 
  Wire.begin();
  Serial.begin(9600);
  BTserial.begin(9600);
  pixels.setBrightness(128);
  pixels.begin();
  pixels.show();
}

void loop(){
  DateTime now = rtc.now();
  if(SetAlramOn)
      if(time[0] == now.month() && time[1] == now.day() && time[2] == now.hour() && time[3] == now.minute()){
        SetAlramOn = false;
        MODE = WAKE_MODE;
      }

  //rawMessage();
  parseAndroidMessage();
  sendAndroidMessage(0);
  printLog(0);
  
  modeControl();

  bluetoothCount++;
  if(bluetoothCount == BLUETOOTHWAITING*1000){
    bluetoothCount = 0;
    BluetoothOn = false;
  }
  //delay(1);
}
/*----------------------------------- 각종 모드 제어 함수 */
void modeControl(){
    if(MODE == WAIT_MODE){
       modeNextEnable = true;
       modeBackEnable = false;
    }

    if(MODE == DIST_MODE){
        modeNextEnable = distanceCheck();
        modeBackEnable = true;
    }

    if(MODE == SLEEP_MODE){
        sleepModeWorking();
        modeNextEnable = false;
    }
    
    if(MODE == SENS_MODE){
        sensingModeWorking();
        modeNextEnable = true; // 일단
        //modeBackEnable = false;
    }
        
    if(MODE == WAKE_MODE){
        alarmWorking();
        modeNextEnable = true;
        modeBackEnable = true;
    }
}
/*----------------------------------- [거리 측정 모드] 동작 함수 */
bool distanceCheck(){
    // 거리 측정 해서 적정 거리 시, true 반환
    
  
    return true;
}
/*----------------------------------- [수면 모드] 동작 함수 */
void sleepModeWorking(){
    static short save_fan_speed = fanSpeed;
    sendAndroidMessage(1);
    printLog(1);
    pixels.fill(pixels.Color(0, 0, 0), 0, NUM_PIXELS);        // 일단 불을 끔
    pixels.show();

    for(int i=0;i<25;i++){  // 수면 시나리오
        if(i==0)FAN(ON,false);
        if(i==5)VELVE(ON,false);
        if(i==20)VELVE(OFF,false);

        _printf("수면 모드 %2d 분 : ",i+1);
        if(i<5){
          fanSpeed+=50;
          if(fanSpeed > 256) fanSpeed = 255; // 최대치로
          _printf("팬속도 증가 %d\n",fanSpeed);
        }
        else if(i<20)
          Serial.println("수면 가스 분사");
        else if(i<25){
          fanSpeed-=50;
          if(fanSpeed <0) fanSpeed = 0; // 최소치로
          _printf("팬속도 감소 %d\n",fanSpeed);
        }

        if(i==24) FAN(OFF,false);
        sendAndroidMessage(1);
        parseAndroidMessage();
        
        if(MODE == SLEEP_MODE-1){      // 수면모드 강제 중단
          VELVE(OFF,false); FAN(OFF,false);
          fanSpeed = save_fan_speed;
          return;
        }
        /*if(MODE == SLEEP_MODE+1){      // 수면모드 일시 중단 (보류)
          while(
        }*/
        
        delay(1*500); // 1000 * 60 을 넣으면 분단위 수행 ( 비동기 종료를 위해선 이걸 쓰면안됨)
    }
    MODE++;
}

/*----------------------------------- [센싱 모드] 동작 함수 */
void sensingModeWorking(){
  
}

/*----------------------------------- [기상 모드] 동작 함수 */
void alarmWorking(){
    sendAndroidMessage(1);
    pixels.fill(pixels.Color(255, 255, 255), 0, NUM_PIXELS);
    pixels.setBrightness(0);
    pixels.show();

    FAN(ON,false);
    for(int i=1;i<=15;i++){            // 정확히는 기상 15분전에 동작시작
       fanSpeed+=17;
       if(fanSpeed > 256) fanSpeed = 255; // 최대치로
       _printf("기상 모드 %d 분: LED 밝기 증가 , FAN 속도 증가(%3d)\n",i,fanSpeed);
        pixels.fill(pixels.Color(255, 255, 255), 0, NUM_PIXELS); 
        pixels.setBrightness(i*17);
        pixels.show();
        delay(500);
    }
    FAN(OFF,false);
    MODE = WAIT_MODE;   // 대기 모드로 전환.
}

/*----------------------------------- 안드로이드 발신 메시지 설정 함수 */
void sendAndroidMessage(bool direct){     // 매개변수: 전송 주기 관계없이 비동기 송신(1)
    static int sendTime = 0;
    sendTime++;
    if(sendTime == SENDING_TICK*1000 || direct){
      int h = dht.readHumidity();
      int t = dht.readTemperature();
      BTserial.print(h);BTserial.print(",");BTserial.print(t);      // 온습도상태 송신
      BTserial.print(",");BTserial.print(fanSpeed);BTserial.print(","); // 팬속도 송신
      BTserial.println(MODE);   // 현재모드상태 송신
      sendTime = 0;
    }
}
/*----------------------------------- 안드로이드 실제 수신 메시지(RAW) 출력 */
void rawMessage(){
  //parseAndroidMessage 와 동시사용 불가
  while(BTserial.peek()!=-1)
      Serial.write(BTserial.read());
  /*if(BTserial.available())
      Serial.write(BTserial.read());
  if(Serial.available())
      BTserial.write(Serial.read());*/
}

/*----------------------------------- 안드로이드 수신 메시지 분석 함수 */
void parseAndroidMessage(){
  int readHead;
  if(BTserial.available())
      BluetoothOn = true;
  
  if(BTserial.peek()!=-1){
    readHead = BTserial.read();delay(10); // 너무 빨리 읽으면 문자열이 깨짐(혹은 쓰레기값)
    switch(readHead){
      case 'a':   // 안드로이드와 현재 연결 상태인가?
          BluetoothOn = true;
          bluetoothCount = 0;
          break;
      case 'm':   // 모드 변경 
          c = BTserial.read();
          if(c == 'n'){
              Serial.println("From Android >> 다음 모드로 이동");
              if(!modeNextEnable)
                   Serial.println("다음 모드로 이동 불가");
              else{
                  MODE++;
                  sendAndroidMessage(1);
              }
          }
          else if(c == 'b'){
              Serial.println("From Android >> 이전 모드로 이동");
              if(modeBackEnable)
                  MODE--;
              else
                  Serial.println("이전 모드로 이동 불가");
          }
          break;
      case 't':   // 알람 시각 설정 (오류 검사 및 정상 수신 검사 코드 필요)
          memset(buf3,'\0',sizeof(buf3));
          delay(10);
          if(BTserial.peek() == 'r'){
             _printf("From Android >> 알람 리셋\n");
             SetAlramOn = false;
          }
          else{
            for(int i=0;i<8;i++){
              buf3[i%2]=BTserial.read();delay(10);
              if(i%2 == 1){
                buf3[2] = '\0';
                time[t++] = atoi(buf3);
              }
            }
           _printf("From Android >> 알람 설정 시간 : %d월 %d일 %d시 %d분\n",time[0],time[1],time[2],time[3]);
            t=0;
            SetAlramOn = true;
          }
          break;
      case 'v':   // 밸브 on/off 설정
          if(BTserial.read() == '1') VELVE(ON,true);
          else VELVE(OFF,true);
          break;
      case 'h':   // 열선 on/off 제어
          if(BTserial.read() == '1') HEAT(ON,true);
          else HEAT(OFF,true);
          break;
      case 'f':   // 팬 속도 설정
          if(BTserial.peek() == 's'){
            BTserial.read(); delay(10);
            for(int i=0;i<3;i++){
              buf3[i] = BTserial.read();
              delay(10);
            }
            fanSpeed = atoi(buf3);
            _printf("팬 속도 설정 : %d\n",fanSpeed);
            memset(buf3,'\0',sizeof(buf3));
          }
          else // 팬 on/off 제어
            if(BTserial.read() == '1') FAN(ON,true);
            else FAN(OFF,true);
          break;
      case 'z':
          Serial.println("CO2 영점 조절");
          break;
      case 'l':   //LED 조절
          if(BTserial.peek() == 'e'){
              Serial.println("MOOD LED OFF");
              pixels.fill(pixels.Color(0, 0, 0), 0, NUM_PIXELS);
              pixels.show();
              LED_MOOD_ON = false;
          }
          else if(BTserial.peek() == 'p'){
              Serial.println("MOOD LED ON");
              BTserial.read(); delay(5);
              for(int i=0;i<3;i++){
                  for(int j=0;j<3;j++){
                      buf_rgb[i][j] = BTserial.read();delay(5);
                  }
                  buf_rgb[i][3] = '\0';
              }
              LED_MOOD_ON = true;
          }
          else if(BTserial.peek() == 'b'){
              BTserial.read();delay(5);
              for(int i=0;i<3;i++){
                  buf3[i] = BTserial.read();delay(5);
              }
              brightness = atoi(buf3);
          }
          if(LED_MOOD_ON && MODE != SLEEP_MODE)
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
    _printf("LED RGBW(%d,%d,%d,밝기:%d)\n",r,g,b,brightness);
    if(brightness !=0)
        pixels.setBrightness(brightness);
    pixels.fill(pixels.Color(r, g, b), 0, NUM_PIXELS);
    pixels.show();
}
/*----------------------------------- 모듈 제어 함수 */
void VELVE(bool in,bool android){
  if(in == ON)Serial.println("Velve ON");
  else Serial.println("Velve OFF");

  if(!android && in){
    BTserial.print("v");BTserial.println(",1");}
  else if(!android && !in){
    BTserial.print("v");BTserial.println(",0");}
}
void FAN(bool in,bool android){
  if(in == ON)Serial.println("Fan ON");
  else Serial.println("Fan OFF");
    
  if(!android && in){
    BTserial.print("f");BTserial.println(",1");}
  else if(!android && !in){
    BTserial.print("f");BTserial.println(",0"); }
}
void HEAT(bool in,bool android){
  if(in == ON)Serial.println("Heat ON");
  else Serial.println("Heat OFF"); 

  if(!android && in){
    BTserial.print("h");BTserial.println(",1");}
  else if(!android && !in){
    BTserial.print("h");BTserial.println(",0");}
}
/*----------------------------------- 로그 출력용 함수 */
void printLog(bool direct){
  static int printTime = 0;
  printTime++;
  if(printTime==1*1000 || direct){
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
          Serial.print("  현재 시각 (알람설정됨) :");
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
