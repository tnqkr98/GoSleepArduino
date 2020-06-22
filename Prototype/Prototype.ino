#include "HCMotor.h"
#include "DHT.h"
#include "RTClib.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#define DHTPIN              A0    // 온습도 아날로그
#define INFRARED_SENSOR     A1    // 적외선 아날로그
#define ILLUMINANCE_SENSOR  A2    // 조도 아날로그(CDS)

#define BLUETOOTHWAITING  5     // n초 이상 안드로이드로 부터 ack 받지 못하면 연결 끊긴것(송수신 범위 벗어남)
#define SENDING_TICK      1     // n초에 한번씩 안드로이드로 센싱값 전송
#define DIST_LOWER       20     // 거리 최소
#define DIST_UPPER       30     // 거리 최대
#define NUM_PIXELS        8     // 네오픽셀 LED 개수 

#define SLEEP_MODE_TOTAL  5     // 수면모드 진행시간(A분 = B+C+D  수식에 맞게 설정할것)  defalut : 25 분
#define INIT_WIND_TIME    2     // 초기 B분간 팬속도 증가  defalut : 5 분
#define CO2_WIND_TIME     1     // C분간 Co2 분 사       defalut : 15 분
#define FIN_WIWN_TIME     2     // D분간 팬속도 감소      defalut :  5 분

enum{MOTOR_L=2,MOTOR_S=3,CO2VELVE=22,LED_PIN=26,NEXT_BT=30,PREV_BT=28,MOOD=24,VIBE=32,SPEAKER=34};  // 핀 번호
enum{STOP_MODE=1,WAIT_MODE,DIST_MODE,SLEEP_MODE,SENS_MODE,WAKE_MODE};

DHT dht(DHTPIN, DHT11);
RTC_DS3231 rtc;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_PIXELS,LED_PIN, NEO_GRB + NEO_KHZ800);

short MODE = 2, fanSpeed = 80, brightness = 128;
short global_mood = 1;

char c,buf2[2],buf3[3],buf_rgb[3][4];      // 각종 읽기 버퍼
short time[4],t=0,bluetoothCount = 0;
bool SetAlramOn = false, BluetoothOn = false;
bool LED_MOOD_ON = false, prevLastState, nextLastState, prevMode=2;
bool ON = true, OFF = false;
int modeNextEnable, modeBackEnable;

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
void keyInterrupt(int PUSH_TIMING);       // 물리 버튼 제어 함수(매개변수 : 전역루프에서는 300, 수면루프에서는 10 넣어야. 0.3초,1초 반응속도)
void keyMoodLightControl();               // 물리 버튼 무드등 제어 함수
void VELVE(bool in,bool android);         // 이하 모듈 제어(ON/OFF), 두번째 매개변수 false: 비동기 송신
void FAN(bool in,bool android);
void HEAT(bool in,bool android);

bool rtcAvailabe();                      // RTC 모듈 예외처리

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
  dht.begin();
  Wire.begin();
  Serial.begin(9600);
  Serial1.begin(9600);  // 시리얼 1 : CO2
  Serial2.begin(9600);  // 시리얼 2 : Bluetooth

  pinMode(CO2VELVE, OUTPUT);  //OPEN
  pinMode(PREV_BT, INPUT);    //RED_BTN
  pinMode(NEXT_BT, INPUT);    //BLUE_BTN
  pinMode(VIBE,OUTPUT);       
  pinMode(SPEAKER, OUTPUT);   //SPEAKER_PIN
  pinMode(ILLUMINANCE_SENSOR, INPUT);
  
  digitalWrite(CO2VELVE, HIGH);   //OPEN
  digitalWrite(PREV_BT, LOW);    //RED_BTN
  digitalWrite(NEXT_BT, LOW);    //BLUE_BTN
  digitalWrite(VIBE,LOW);
  
  analogWrite(MOTOR_S, fanSpeed); //

  #if defined (__AVR_ATtiny85__)
   if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif 

  Serial.println("GoSleep ready");
  pixels.setBrightness(128);
  pixels.begin();
  pixels.show();
}

void loop(){
  if(SetAlramOn){
    if(rtcAvailable()){
      DateTime now = rtc.now();
        if(time[0] == now.month() && time[1] == now.day() && time[2] == now.hour() && time[3] == now.minute()){
          SetAlramOn = false;
          MODE = WAKE_MODE;
        }
    }
  }
  //rawMessage();
  parseAndroidMessage();      // android 명령 처리
  keyInterrupt(300);          // key button 명령 처리

  sendAndroidMessage(0);
  printLog(0);
  modeControl();

  bluetoothCount++;
  if(bluetoothCount == BLUETOOTHWAITING*1000){
    bluetoothCount = 0;
    BluetoothOn = false;
  }
  delay(1);   // 시간 처리 방식 변경 예정.
}
/*-------------------------------------------------------------------------------------- 각종 모드 제어 함수 */
void modeControl(){
    if(MODE == WAIT_MODE){
       modeNextEnable = true;
       modeBackEnable = false;
       if(!LED_MOOD_ON){
           pixels.setBrightness(0);
           pixels.fill(pixels.Color(255, 255, 255), 0, NUM_PIXELS);
           pixels.show();
       }
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

    prevMode = MODE;
}
/*-------------------------------------------------------------------------------------- [거리 측정 모드] 동작 함수 */
bool distanceCheck(){   // 거리 측정 해서 적정 거리 시, true 반환
    int dist = getDistance();
    static int logcount = 0;
    bool ret;

    if(LED_MOOD_ON){
      Serial.println("MOOD LED OFF");   // 무드등이 켜져있으면 끔
      pixels.fill(pixels.Color(0, 0, 0), 0, NUM_PIXELS);
      pixels.show();
      LED_MOOD_ON = false;
    }
    
    pixels.setBrightness(30);       // 거리조절모드 밝기
    if((logcount++) == 1000){
      _printf("      * 대상과의 거리 : %d\n",dist);
      logcount = 0;
    }
    if(dist < DIST_LOWER){  // 가깝
      pixels.setPixelColor(2, pixels.Color(50, 0, 0));
      ret = false;
    } else if((dist < DIST_UPPER )){ //적절
      pixels.setPixelColor(2, pixels.Color(0, 50, 0));
      ret = true;
    } else {  // 멈
      pixels.setPixelColor(2, pixels.Color(0, 0, 50));
      ret = false;
    }
    pixels.show();
    if(ret)
      return true;
    else
       return false;
}

int getDistance(){  // 적외선 모듈 이용, 거리(cm) 반환
  int volt = map(analogRead(INFRARED_SENSOR), 0, 1023, 0, 5000); 
  return (27.61 / (volt - 0.1696)) * 1000;
  //return 25;
}

/*-------------------------------------------------------------------------------------- [수면 모드] 동작 함수 */
void sleepModeWorking(){
    static short save_fan_speed = fanSpeed;
    long pastTime;
    sendAndroidMessage(1);
    printLog(1);
    pixels.fill(pixels.Color(0, 0, 0), 0, NUM_PIXELS);        // 불을 끔
    pixels.show();

    fanSpeed = 0;   // save_fan_speed 를 사용자가 설정한 속도로 쓸것.
    FAN(ON,false);
    
    int M = 600; //0.1초 X 600 = 1분
    for(unsigned int i=0;i<SLEEP_MODE_TOTAL*M;i++){  // 수면 시나리오    0.1초에 한번 루프 돌게.  25*M
      pastTime = millis();
      
        if(i==INIT_WIND_TIME*M)VELVE(ON,false);
        if(i==(INIT_WIND_TIME+CO2_WIND_TIME)*M)VELVE(OFF,false);

        if(i%10==0){    //수면모드 동작중 , 매 루프 수행해야 할 것들.
          sendAndroidMessage(1);
          _printf("수면 모드 [%5d 초] 진행중 >> 현재 수행 동작 : ",i/10);
        }
          
        if(i<INIT_WIND_TIME*M && i%10==0){
            fanSpeed = map(i/10,0,60*INIT_WIND_TIME,0,255);    //속도 조절은 1초 단위. (즉 10루프당 1회 속도조절)  여기서 255가 사용자가 설정한 값이여야.
            _printf("팬속도 증가 [속도 값 %3d]\n",fanSpeed);
            analogWrite(MOTOR_S, fanSpeed);
        }
        else if(i<(INIT_WIND_TIME+CO2_WIND_TIME)*M){
          if(i%10==0)
            Serial.println("수면 가스 분사 중..");
        }
        else if(i<SLEEP_MODE_TOTAL*M && i%10 == 0){
          fanSpeed = map(i/10,60*(INIT_WIND_TIME+CO2_WIND_TIME),60*SLEEP_MODE_TOTAL,255,0);
          _printf("팬속도 감소 [속도 값 %3d]\n",fanSpeed);
          analogWrite(MOTOR_S, fanSpeed);
        }

        if(i==SLEEP_MODE_TOTAL*M - 1) FAN(OFF,false);

        parseAndroidMessage();          // 명령 처리
        keyInterrupt(10);
        
        if(MODE == SLEEP_MODE-1){      // 수면모드 강제 중단
          VELVE(OFF,false); FAN(OFF,false);
          fanSpeed = save_fan_speed;
          return;
        }
        
        if(MODE == SLEEP_MODE+1){      // 수면모드 일시 중단 
          VELVE(OFF,false); FAN(OFF,false);
          Serial.print("수면모드 일시중단 ");
          for(int j=0;;j++){
            delay(1);
            if(j==1000){
              Serial.print(">");
              j=0;
            }
            parseAndroidMessage();          // 명령 처리
            keyInterrupt(300);
            
            if(MODE >= SLEEP_MODE+2){   // 수면모드 일시 중단 탈출.
                MODE = SLEEP_MODE;
                Serial.println("");
                if(INIT_WIND_TIME*M > i || (SLEEP_MODE_TOTAL-FIN_WIWN_TIME)*M <i)
                  FAN(ON,false);
                if(INIT_WIND_TIME*M <= i && (SLEEP_MODE_TOTAL-FIN_WIWN_TIME)*M >=i)
                  VELVE(ON,false); 
                break;
            }
          }
        }

        while(millis() - pastTime < 100)  // 루프주기 0.1초
          delay(1); 
    }
    MODE++;
}

/*-------------------------------------------------------------------------------------- [센싱 모드] 동작 함수 */
void sensingModeWorking(){
  
}

/*-------------------------------------------------------------------------------------- [기상 모드] 동작 함수 */
void alarmWorking(){
    sendAndroidMessage(1);
    pixels.fill(pixels.Color(255, 255, 255), 0, NUM_PIXELS);
    pixels.setBrightness(0);
    pixels.show();

    FAN(ON,false);
    for(int i=1;i<=15;i++){            // 정확히는 기상 15분전에 동작시작
       fanSpeed+=17;
       if(fanSpeed > 256) fanSpeed = 255; // 최대치로
       analogWrite(MOTOR_S, fanSpeed); 
       _printf("기상 모드 %d 분: LED 밝기 증가 , FAN 속도 증가(%3d)\n",i,fanSpeed);
        pixels.fill(pixels.Color(255, 255, 255), 0, NUM_PIXELS); 
        pixels.setBrightness(i*17);
        pixels.show();
        delay(500);
    }
    FAN(OFF,false);
    MODE = WAIT_MODE;   // 대기 모드로 전환.
}

/*-------------------------------------------------------------------------------------- 안드로이드 발신 메시지 설정 함수 */
void sendAndroidMessage(bool direct){     // 매개변수: 전송 주기 관계없이 비동기 송신(1)
    static int sendTime = 0;
    static float h1=0;
    static long co2=0,d=0;
    static float t1=0;

    sendTime++;
    if(sendTime == SENDING_TICK*1000 || direct){
      float h, t;
      h = dht.readHumidity();
      t = dht.readTemperature();
      if(isnan(h) || isnan(t))
        Serial.println("Failed to read from DHT sensor!");
      else{
        h1 = h;
        t1 = t;
      }
        
      if(Serial1.available()){
        long ccc = Serial1.parseInt();
        if(ccc*10>300 && ccc*10 <100000)
          co2 = ccc;
      }
      else{
        //Serial.println("Co2 Sensor Error");
      }

      d = getDistance();

      // https://www.allaboutcircuits.com/projects/design-a-luxmeter-using-a-light-dependent-resistor/
      double v = 1250000*pow(analogRead(ILLUMINANCE_SENSOR),-1.4059);
      
      Serial2.print(h1);Serial2.print(",");            // 온도 송신
      Serial2.print(t1);Serial2.print(",");            // 습도 송신     
      Serial2.print(fanSpeed);Serial2.print(",");     // 팬속도 송신      
      Serial2.print(MODE);Serial2.print(",");         // 현재모드상태 송신      
      Serial2.print(co2*10);Serial2.print(",");       // CO2 송신      
      Serial2.print(d);Serial2.print(",");            // 거리 송신     
      Serial2.println((int)v);                        // 조도 송신
      sendTime = 0;
    }
}
/*-------------------------------------------------------------------------------------- 안드로이드 실제 수신 메시지(RAW) 출력 */
void rawMessage(){
  //parseAndroidMessage 와 동시사용 불가
  
 while(Serial2.peek()!=-1)
      Serial.write(Serial2.read());
      
  /*if(Serial2.available())
      Serial.write(Serial2.read());
  if(Serial.available())
      Serial2.write(Serial.read());*/
}

/*-------------------------------------------------------------------------------------- 안드로이드 수신 메시지 분석 함수 */
void parseAndroidMessage(){
  int readHead;
  if(Serial2.available())
      BluetoothOn = true;
  
  if(Serial2.peek()!=-1){
    readHead = Serial2.read();delay(10); // 너무 빨리 읽으면 문자열이 깨짐(혹은 쓰레기값)
    //_printf("From Android >> 수신:%c%c\n",readHead,Serial2.peek());
    switch(readHead){
      case 'a':   // 안드로이드와 현재 연결 상태인가?
          BluetoothOn = true;
          bluetoothCount = 0;
          break;
      case 'm':   // 모드 변경 
          c = Serial2.read();
          if(c == 'n'){
              _printf("\nFrom Android >> Next 버튼\n");
              if(!modeNextEnable)
                   Serial.println("다음 모드로 이동 불가");
              else{
                  MODE++;
                  sendAndroidMessage(1);
              }
          }
          else if(c == 'b'){
              _printf("\nFrom Android >> Back 버튼\n");
              if(modeBackEnable)
                  MODE--;
              else
                  Serial.println("이전 모드로 이동 불가");
          }
          break;
      case 't':   // 알람 시각 설정 (오류 검사 및 정상 수신 검사 코드 필요)
          memset(buf3,'\0',sizeof(buf3));
          delay(10);
          if(Serial2.peek() == 'r'){
             _printf("From Android >> 알람 리셋\n");
             SetAlramOn = false;
          }
          else{
            for(int i=0;i<8;i++){
              buf3[i%2]=Serial2.read();delay(10);
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
          if(Serial2.read() == '1') VELVE(ON,true);
          else VELVE(OFF,true);
          break;
      case 'h':   // 열선 on/off 제어
          if(Serial2.read() == '1') HEAT(ON,true);
          else HEAT(OFF,true);
          break;
      case 'f':   // 팬 속도 설정
          if(Serial2.peek() == 's'){
            Serial2.read(); delay(5);
            for(int i=0;i<3;i++){
              buf3[i] = Serial2.read();
              delay(5);
            }
            fanSpeed = atoi(buf3);
            _printf("팬 속도 설정 : %d\n",fanSpeed);
            analogWrite(MOTOR_S, fanSpeed); 
            memset(buf3,'\0',sizeof(buf3));
          }
          else // 팬 on/off 제어
            if(Serial2.read() == '1') FAN(ON,true);
            else FAN(OFF,true);
          break;
      case 'z':
          Serial.println("CO2 영점 조절");
          break;
      case 'l':   //LED 조절
          if(Serial2.peek() == 'e'){
              Serial.println("MOOD LED OFF");
              pixels.fill(pixels.Color(0, 0, 0), 0, NUM_PIXELS);
              pixels.show();
              LED_MOOD_ON = false;
          }
          else if(Serial2.peek() == 'p'){
              Serial.println("MOOD LED ON");
              Serial2.read(); delay(5);
              for(int i=0;i<3;i++){
                  for(int j=0;j<3;j++){
                      buf_rgb[i][j] = Serial2.read();delay(1);
                  }
                  buf_rgb[i][3] = '\0';
              }
              LED_MOOD_ON = true;
          }
          else if(Serial2.peek() == 'b'){
              Serial2.read();delay(5);
              for(int i=0;i<3;i++){
                  buf3[i] = Serial2.read();delay(5);
              }
              brightness = atoi(buf3);
          }
          if(LED_MOOD_ON && MODE != SLEEP_MODE)
              moodLedControl(atoi(buf_rgb[0]),atoi(buf_rgb[1]),atoi(buf_rgb[2]));
          break;
      case 'c':   // 통신 종료 메시지 수신의 경우 종료.
          BluetoothOn = false;
          break;
      case 'r':   // 끊겼다재연결시 아두이노 상태를 안드로이드에 동기화 하기위한 안드의 요청
          if(SetAlramOn){
            Serial2.print("t,");
            Serial2.print(time[0]);Serial2.print(",");
            Serial2.print(time[1]);Serial2.print(",");
            Serial2.print(time[2]);Serial2.print(",");
            Serial2.println(time[3]);
          }
          else
            Serial2.println("t,n");
          break;
    }

    while(Serial2.peek() != -1) //남은 버퍼 제거
        Serial2.read();
  }
}
/*-------------------------------------------------------------------------------------- 조명 제어 함수 */
void moodLedControl(int r,int g,int b){
    _printf("LED RGBW(%d,%d,%d,밝기:%d)\n",r,g,b,brightness);
    if(brightness !=0)
        pixels.setBrightness(brightness);
    pixels.fill(pixels.Color(r, g, b), 0, NUM_PIXELS);
    pixels.show();
}
/*-------------------------------------------------------------------------------------- 모듈 제어 함수 */
void VELVE(bool in,bool android){
  if(in == ON){
    Serial.println("Velve ON");
    digitalWrite(CO2VELVE, LOW);
  }
  else {
    Serial.println("Velve OFF");
    digitalWrite(CO2VELVE, HIGH);   //밸브 잠금
  }

  if(!android && in){Serial2.print("v");Serial2.println(",1");}
  else if(!android && !in){Serial2.print("v");Serial2.println(",0"); }
}
void FAN(bool in,bool android){
  if(in == ON){
    Serial.println("Fan ON");
    digitalWrite(MOTOR_L, HIGH);  
    analogWrite(MOTOR_S, fanSpeed);   
  }
  else {
    Serial.println("Fan OFF");
    digitalWrite(MOTOR_L, LOW);
  }
    
  if(!android && in){
    Serial2.print("f");Serial2.println(",1");}
  else if(!android && !in){
    Serial2.print("f");Serial2.println(",0"); }
}
void HEAT(bool in,bool android){
  if(in == ON)Serial.println("Heat ON");
  else Serial.println("Heat OFF"); 

  if(!android && in){
    Serial2.print("h");Serial2.println(",1");}
  else if(!android && !in){
    Serial2.print("h");Serial2.println(",0");}
}

void VIBE_CALL(){
    digitalWrite(VIBE,HIGH);
    delay(150);
    digitalWrite(VIBE,LOW);
}
/*-------------------------------------------------------------------------------------- 물리 터치 버튼 제어 함수 */
void keyInterrupt(int PUSH_TIMING){
  static long prev_stack = 0, next_stack = 0, mood_stack = 0;

  // 이전 버튼
  if(digitalRead(PREV_BT) == HIGH){
    prev_stack++;
    if(prev_stack == PUSH_TIMING){
      VIBE_CALL();
      _printf("Key Interrupt!! : Prev\n");
      if(modeBackEnable) MODE--;
      else Serial.println("이전 모드로 이동 불가");
    }
  }else if(digitalRead(PREV_BT) == LOW)
     prev_stack = 0;

  // 다음 버튼
  if(digitalRead(NEXT_BT) == HIGH){
    next_stack++;
    if(next_stack == PUSH_TIMING){
      VIBE_CALL();
       _printf("Key Interrupt!! : Next\n");
      if(!modeNextEnable) Serial.println("다음 모드로 이동 불가");
      else{
        MODE++;
        sendAndroidMessage(1);
      }
    }
  }else if(digitalRead(NEXT_BT) == LOW)
     next_stack = 0;

  // 무드 버튼
  if(digitalRead(NEXT_BT) == HIGH && digitalRead(PREV_BT) == HIGH){
    next_stack = 0;
    prev_stack = 0;
    mood_stack++;
    if(mood_stack == PUSH_TIMING){
      _printf("Key Interrupt!! : Mood\n");
      if(MODE != DIST_MODE){ 
        VIBE_CALL();
        keyMoodLightControl();
      }
    }
  }else if(digitalRead(NEXT_BT) == LOW || digitalRead(PREV_BT) == LOW)
     mood_stack = 0;
}

void keyMoodLightControl(){
  if(LED_MOOD_ON){
    LED_MOOD_ON = false;
    Serial.println("무드등 off");
    pixels.setBrightness(0);    
  }
  else{
    LED_MOOD_ON = true;
    Serial.println("무드등 on");
    pixels.setBrightness(15);
  }
  pixels.fill(pixels.Color(255, 255, 255), 0, NUM_PIXELS);  // 네오 픽셀 적용순서 ( 밝기 -> Fill -> show )
  pixels.show();  
}

/*-------------------------------------------------------------------------------------- RTC 모듈 예외처리 */
bool rtcAvailable(){
  bool ret_value = true;
  if(!rtc.begin()){
    ret_value = false;
    Serial.println("RTC Error : The RTC module is not available");
  }
  /*if(rtc.lostPower()){  //이상함
    ret_value = false;
    Serial.println("RTC Error : The RTC module losts power");
  }*/
  return ret_value;
}

/*-------------------------------------------------------------------------------------- 로그 출력용 함수 */
void printLog(bool direct){
  static int printTime = 0;
  if((printTime++)==2000 || direct){
       switch(MODE){
          case STOP_MODE:Serial.print(" 현재 상태 : 절전 모드 ");break;
          case WAIT_MODE:Serial.print(" 현재 상태 : 대기 모드 ");break;
          case DIST_MODE:Serial.print(" 현재 상태 : 거리 모드 ");break;
          case SLEEP_MODE:Serial.print(" 현재 상태 : 수면 모드 ");break;
          case SENS_MODE:Serial.print(" 현재 상태 : 센싱 모드 ");break;
          case WAKE_MODE:Serial.print(" 현재 상태 : 기상 모드 ");break;
      }
      
      if(BluetoothOn)Serial.print(" (안드로이드와 통신 ON) ");
      
      if(rtcAvailable()){
        DateTime now = rtc.now();
        if(SetAlramOn){
           Serial.print("  현재 시각 (알람설정됨) :");
           Serial.print(now.month()); Serial.print("월");
           Serial.print(now.day()); Serial.print("일");
           Serial.print(now.hour()); Serial.print("시");
           Serial.print(now.minute()); Serial.print("분");
           Serial.print(now.second()); Serial.print("초");
        }
      }
      
      Serial.println("");
      printTime = 0;
  }
}
