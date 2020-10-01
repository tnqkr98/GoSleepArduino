/* 
 *   프로그램명 : Gosleep Prototype 0.2b
 *   작성자 : 박동한
 *   최종 수정일 : 2020.09.25
 */

#include "HCMotor.h"
#include "DHT.h"
#include "RTClib.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <SPI.h>
#include <MFRC522.h>

#define DS3231_I2C_ADDRESS 104    // RTC 모듈 주소

#define DEVELOPER_MODE    0      // <------------ 1로 변경 시, 각종 기기 환경설정 가능, 일반 기기 동작은 0으로 설정.

#define DHTPIN              A0    // 온습도 아날로그
#define INFRARED_SENSOR     A1    // 적외선 아날로그
#define ILLUMINANCE_SENSOR  A2    // 조도 아날로그(CDS)

#define BLUETOOTHWAITING  5     // n초 이상 안드로이드로 부터 a 받지 못하면 연결 끊긴것(송수신 범위 벗어남)
#define SENDING_TICK    0.5     // n초에 한번씩 안드로이드로 센싱값 전송
#define DIST_LOWER       20     // 거리 최소
#define DIST_UPPER       30     // 거리 최대
#define NUM_PIXELS       19     // 네오픽셀 LED 개수 

#define SLEEP_MODE_TOTAL 22      // 수면모드 진행시간(A분 = B+C+D  수식에 맞;게 설정할것)  defalut : 22 분
#define INIT_WIND_TIME    2      // 초기 B분간 팬속도 증가  default : 2 분
#define CO2_WIND_TIME    15      // C분간 Co2 분 사        default : 15 분
#define FIN_WIND_TIME     5      // D분간 팬속도 감소       default :  5 분

#define ALARM_LED_TIME   15      // 기상모드 LED 시작x분전(x>=y)    default : 15분
#define ALARM_FAN_TIME   15      // 기상모드 FAN 시작y분전          default : 15분
#define LONG_SLEEP       40      // 알람방식의 전환 시간(70<수면시간 : 점진적기상, 70>수면시간 : 즉각기상)

enum{MOTOR_L=2,MOTOR_S=3,CO2VALVE_L=10,CO2VALVE_S=8,LED_PIN=26,NEXT_BT=30,PREV_BT=28,MOOD=24,VIBE=32,SPEAKER=22};  // 핀 번호
enum{SS_PIN=53,RST_PIN=5};      // RFID(NFC관련) 핀번호
enum{STOP_MODE=1,WAIT_MODE,DIST_MODE,SLEEP_MODE,SENS_MODE,WAKE_MODE};

DHT dht(DHTPIN, DHT11);
RTC_DS3231 rtc;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_PIXELS,LED_PIN, NEO_GRB + NEO_KHZ800);

short MODE = 2, fanSpeed = 100, brightness = 128;    // 제품 상태
short global_mood = 1, alarmType = 1;   // type = 1 : 40분 점진적 기상,   type = 2 : 즉각 기상 (70분미만 수면시)
long int code = 0;

// User Custom Variable 
short user_fanSpeed = 100, user_Co2Concent = 255;
short alarm_fanSpeed = 255, alarm_Led_bright = 250;
bool alarmLedTone = false; // false : warm톤   true : cool톤
bool ANDROID_FAN_UI = false, ANDROID_VALVE_UI  = false;

char c,buf2[2],buf3[3],buf_rgb[3][4],co2code[36];      // 각종 읽기 버퍼
short bluetoothCount = 0;
byte time[2]={0},t=0;
bool SetAlramOn = false, BluetoothOn = false;
bool LED_MOOD_ON = false;
bool ON = true, OFF = false;
int modeNextEnable, modeBackEnable;
String product_code="NYX-";

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
void VALVE(bool in,bool android);         // 이하 모듈 제어(ON/OFF), 두번째 매개변수 false: 비동기 송신
void FAN(bool in,bool android);
void HEAT(bool in,bool android);
void setAlarmMemory(bool on);             // 알람 설정 및 알람 시각 메모리 영구저장.
void readNFC();                           // RFID 이용한 NFC 리더
bool rtcAvailabe();                       // RTC 모듈 예외처리
void developerMode();                     // 개발자 환경설정
void printTime();                         // 시간 출력

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
  Serial.begin(9600);
  Serial1.begin(9600);  // 시리얼 1 : CO2
  Serial2.begin(9600);  // 시리얼 2 : Bluetooth
  dht.begin();
  Wire.begin(); 
  SPI.begin();          // RFID
  
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));   // 컴파일 시간 동기화

  pinMode(PREV_BT, INPUT);    //RED_BTN
  pinMode(NEXT_BT, INPUT);    //BLUE_BTN
  pinMode(VIBE,OUTPUT);       
  pinMode(SPEAKER, OUTPUT);   //SPEAKER_PIN
  pinMode(ILLUMINANCE_SENSOR, INPUT);
  
  digitalWrite(PREV_BT, LOW);    //RED_BTN
  digitalWrite(NEXT_BT, LOW);    //BLUE_BTN
  digitalWrite(VIBE,LOW);

  /* 모터 컨트롤 */
  TCCR2B = (TCCR2B & 0xF8) | 0x01 ;
  TCCR3B = (TCCR3B & 0xF8) | 0x01 ;
  
  pinMode(MOTOR_L, INPUT);  
  pinMode(CO2VALVE_L, INPUT);
  pinMode(MOTOR_S,OUTPUT);
  pinMode(CO2VALVE_S,OUTPUT);
  
  digitalWrite(4,LOW);            // FAN 안쓰는 핀 고정값
  digitalWrite(9,LOW);            // VALVE 안쓰는 핀 고정값
  
  #if defined (__AVR_ATtiny85__)
   if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif 

  pixels.setBrightness(128);
  pixels.begin();
  pixels.show();
  
  byte address[7]={0};
  for(int i=0;i<7;i++)            // EEPROM 읽기(저장값)
    address[i] = EEPROM.read(i);

  if(address[0] == 1){            // 저장된 알람시간 읽기
     SetAlramOn = true;
     time[0] = address[1];
     time[1] = address[2];
  }
  Serial.println("====================================================================================================================");
  if(address[3] != 0){
    code += (int)address[3] * 1000000;
    code += (int)address[4] * 10000;
    code += (int)address[5] * 100;
    code += (int)address[6];
    product_code+=code;
    _printf(" Product Code : NYX-%ld\n", code);
  }
  else
    Serial.println(" Product Code : 지정되지 않음 ");
    
  if(DEVELOPER_MODE){
    Serial.println(" Developer Configuaraion Setting Mode ...");
    developerMode();
  }
  Serial.println(" GoSleep is ready to operation ... ");
  VALVE(OFF,false); 
  FAN(OFF,false);
}

void loop(){
  //rawMessage();
  parseAndroidMessage();      // android 명령 처리
  keyInterrupt(300);          // key button 명령 처리

  sendAndroidMessage(0);
  printLog(0);

  readNFC();
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
       alarmType = 0;
    }

    static long start = 0;
    if(MODE == DIST_MODE){
        //modeNextEnable = distanceCheck();
        modeNextEnable = false;
        modeBackEnable = true;
        
        if(distanceCheck() && start == 0)
            start = millis();
        else if(distanceCheck() && start != 0){     // 5초 동안 적정거리이면 자동으로 수면모드 전환
            if(millis()-start >=5000)
                MODE++;
        }
        else if(!distanceCheck())
            start = 0;
    }

    if(MODE == SLEEP_MODE){
        DateTime now = rtc.now();
        byte sleep[2];
        sleep[0] = now.hour();
        sleep[1] = now.minute();
        int sMin, aMin, totalSleepTime;
        sMin = sleep[0]*60 + sleep[1];
        aMin = time[0]*60 + time[1];
        if(sMin > aMin)
          totalSleepTime = 1440 - sMin + aMin;
        else
          totalSleepTime = aMin - sMin;

        if(totalSleepTime > LONG_SLEEP)
          alarmType = 1;        // 점진적 기상 타입
        else
          alarmType = 2;        // 즉각 기상 타입

        modeNextEnable = true;
        sleepModeWorking();
    }

    static int timeT1=0,timeT2=0;
    if(MODE == SENS_MODE){
        sensingModeWorking();
        if(SetAlramOn){
          if(rtcAvailable()){
            DateTime now = rtc.now();

            int alarmMin, nowMin;
            alarmMin = time[0]*60 + time[1];
            nowMin = now.hour()*60 + now.minute();

            if(alarmType == 2 && alarmMin == nowMin && now.second() == 0){
              pixels.fill(pixels.Color(255, 255, 255), 0, NUM_PIXELS); 
              pixels.setBrightness(255);
              pixels.show();
              FAN(ON,false);
              fanSpeed = 255;
              analogWrite(MOTOR_L, fanSpeed);     
              MODE = WAKE_MODE;               // 즉각 기상
            }

            if(timeT1++ == 2000 && alarmType == 2){
              _printf(" 즉각 기상 시간 로그 >>  nowMin : %d  , alarmMin : %d\n",nowMin,alarmMin);
              timeT1 = 0;
            }
            
            alarmMin = (alarmMin-ALARM_LED_TIME<0)?(alarmMin-ALARM_LED_TIME+1440):(alarmMin-ALARM_LED_TIME);

            if(timeT2++ == 2000 && alarmType == 1){
              _printf(" 점진 기상 시간 로그 >>  nowMin : %d  , alarmMin : %d\n",nowMin,alarmMin);
              timeT2 = 0;
            }
            
            if(alarmType == 1 && alarmMin == nowMin && now.second() == 0){
              MODE = WAKE_MODE;
              alarmWorking();       // 점진적 기상
            }
          }
        }
        modeNextEnable = true;
        modeBackEnable = false;
    }
        
    if(MODE == WAKE_MODE){
        modeNextEnable = true;
        modeBackEnable = false;
    }

    if(MODE > WAKE_MODE){
       MODE = WAIT_MODE;
       FAN(OFF,false);
    }
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
    
    if((logcount++) == 1000){
      _printf("      * 대상과의 거리 : %d\n",dist);
      logcount = 0;
    }
    if(dist < DIST_LOWER){  // 가깝
      pixels.setPixelColor(0, pixels.Color(24, 0, 0));
      ret = false;
    } else if((dist < DIST_UPPER )){ //적절
      pixels.setPixelColor(0, pixels.Color(0, 24, 0));
      ret = true;
    } else {  // 멈
      pixels.setPixelColor(0, pixels.Color(0, 0, 24));
      ret = false;
    }
    pixels.setBrightness(10);       // 거리조절모드 밝기
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
      
        if(i==INIT_WIND_TIME*M)VALVE(ON,false);
        if(i==(INIT_WIND_TIME+CO2_WIND_TIME)*M)VALVE(OFF,false);

        if(i%10==0){    //수면모드 동작중 , 매 루프 수행해야 할 것들.
          sendAndroidMessage(1);
          _printf("수면 모드 [%5d 초] 진행중 >> 현재 수행 동작 : ",i/10);
        }
 
        if(i<INIT_WIND_TIME*M && i%10==0){
            fanSpeed = map(i/10,0,60*INIT_WIND_TIME,40,user_fanSpeed);    //속도 조절은 1초 단위. (즉 10루프당 1회 속도조절)  여기서 255가 사용자가 설정한 값이여야.
            _printf("팬속도 증가 [속도 값 %3d]\n",fanSpeed);
            analogWrite(MOTOR_L, fanSpeed);
        }
        else if(i<(INIT_WIND_TIME+CO2_WIND_TIME)*M){
          if(i%10==0)
            Serial.println("수면 가스 분사 중..");
        }
        else if(i<SLEEP_MODE_TOTAL*M && i%10 == 0){
          fanSpeed = map(i/10,60*(INIT_WIND_TIME+CO2_WIND_TIME),60*SLEEP_MODE_TOTAL,user_fanSpeed,40);  // 최저속도 20으로(소음때매)
          _printf("팬속도 감소 [속도 값 %3d]\n",fanSpeed);
          analogWrite(MOTOR_L, fanSpeed);
        }

        if(i==SLEEP_MODE_TOTAL*M - 1) FAN(OFF,false);

        parseAndroidMessage();          // 명령 처리
        keyInterrupt(10);
        
        if(MODE == SLEEP_MODE-1){       // 수면모드 강제 중단
          VALVE(OFF,false); FAN(OFF,false);
          fanSpeed = save_fan_speed;
          MODE--;                       // 대기모드로
          return;
        }
        
        if(MODE == SLEEP_MODE+1){      // 수면모드 일시 중단 
          VALVE(OFF,false); FAN(OFF,false);
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
                if(INIT_WIND_TIME*M > i || (SLEEP_MODE_TOTAL-FIN_WIND_TIME)*M <i) // 중단됐던 시나리오에 알맞게 동작.
                  FAN(ON,false);
                if(INIT_WIND_TIME*M <= i && (SLEEP_MODE_TOTAL-FIN_WIND_TIME)*M >=i)
                  VALVE(ON,false); 
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
    int ledbright;
    long pastTime;
  
    pixels.fill(pixels.Color(255, 255, 255), 0, NUM_PIXELS);
    pixels.setBrightness(0);
    pixels.show();

    int M = 600; //0.1초 X 600 = 1분
    FAN(ON,false);
    for(int i=0;i<=ALARM_LED_TIME*M;i++){
      pastTime = millis();

      if(i%10==0){   
          sendAndroidMessage(1);
          _printf("기상 모드 [%5d 초] 진행중 >> 현재 상태 : ",i/10);
        }

      if(i%10==0){
        ledbright = map(i/10,0,ALARM_LED_TIME*60,0,255);
        pixels.fill(pixels.Color(255, 255, 255), 0, NUM_PIXELS); 
        pixels.setBrightness(ledbright);
        pixels.show(); 
        _printf("LED 동작 중[밝기 : %5d] ",ledbright);
      }

      parseAndroidMessage();          // 명령 처리
      keyInterrupt(10);

      if(MODE > WAKE_MODE){               // 사용자가 다음 누를 시
          MODE = WAIT_MODE;               // 알람 종료.
          FAN(OFF,false); 
          pixels.fill(pixels.Color(255, 255, 255), 0, NUM_PIXELS);
          pixels.setBrightness(0);
          pixels.show();
          return;
      }

      if(i%10==0 && i>(ALARM_LED_TIME-ALARM_FAN_TIME)*M){
        fanSpeed = map(i/10,ALARM_LED_TIME*60-ALARM_FAN_TIME*60,ALARM_LED_TIME*60,40,alarm_fanSpeed);
        analogWrite(MOTOR_L, fanSpeed);
        _printf("| FAN 동작 중[속도 : %5d] ",fanSpeed);
      }
       
      while(millis() - pastTime < 100)  // 루프주기 0.1초
          delay(1);

      if(i%10==0)
        Serial.println("");
    }

    MODE = WAKE_MODE;   // 최대출력인 상태로 기상 모드로 전환.
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
      if(isnan(h) || isnan(t)){
        Serial.println("Failed to read from DHT sensor!");
        h = 0; t =0;
      }
      else{
        h1 = h;
        t1 = t;
      }
        
      co2 = co2sensing()*10;   
      d = getDistance();

      // https://www.allaboutcircuits.com/projects/design-a-luxmeter-using-a-light-dependent-resistor/
      double v = 1250000*pow(analogRead(ILLUMINANCE_SENSOR),-1.4059);
      
      Serial2.print(h1);Serial2.print(",");            // 온도 송신
      Serial2.print(t1);Serial2.print(",");            // 습도 송신     
      Serial2.print(fanSpeed);Serial2.print(",");     // 팬속도 송신      
      Serial2.print(MODE);Serial2.print(",");         // 현재모드상태 송신      
      Serial2.print(co2);Serial2.print(",");       // CO2 송신 
      Serial2.print(d);Serial2.print(",");            // 거리 송신     
      Serial2.print((int)v);Serial2.print(",");       // 조도 송신
      
      if(ANDROID_FAN_UI){ //FAN 상태 송신
        Serial2.print(1);
        Serial2.print(",");  
      }
      else{
        Serial2.print(0);
        Serial2.print(",");  
      }
      if(ANDROID_VALVE_UI) //VALVE 상태 송신
        Serial2.print(1);
      else
        Serial2.print(0);
      Serial2.print(",");  
      
      sendTime = 0;

      // 제품 시간 전송
      DateTime now = rtc.now();
      Serial2.print(now.hour());Serial2.print(',');
      Serial2.println(now.minute());
    }
}

long co2sensing(){
  static bool init = true;
  static long past =0;
  static String past_str="";
  static int size_pick =0;
  static int err_count = 0;
  long current=0;
  String cur_str="";
  if(init){
    init = false;
    if(Serial1.available()){
      current = Serial1.parseInt();
      past_str+=current;
      size_pick = past_str.length();
    }
  }
  
  if(Serial1.available()){
      current = Serial1.parseInt();    
      if(current*10>1000 && current*10 <600000){
        cur_str+= current;
        //_printf("현재 길이 %d, 픽된 길이 %d  에러카운트  %d,",cur_str.length(),size_pick,err_count);

        if(err_count > 1){
            size_pick = cur_str.length();
            err_count = 0;
        }

        if(size_pick != cur_str.length()){
          err_count++;
          return past;
        }
        else{
          err_count = 0;
          past = current;
          return current;
        }
      }
  }
  else{
      Serial.println("Co2 Sensor Error");
      return 0;
  }
  return past;    
}
/*-------------------------------------------------------------------------------------- 안드로이드 실제 수신 메시지(RAW) 출력 */
void rawMessage(){
  //parseAndroidMessage 와 동시사용 불가
 while(Serial2.peek()!=-1)
      Serial.write(Serial2.read());
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
             EEPROM.write(0,0);
          }
          else{
            for(int i=0;i<4;i++){
              buf3[i%2]=Serial2.read();delay(10);
              if(i%2 == 1){
                buf3[2] = '\0';
                time[t++] = atoi(buf3);
              }
            }
           _printf("From Android >> 알람 설정 시간 : %d시 %d분\n",time[0],time[1]);
           EEPROM.write(0,1);
           EEPROM.write(1,time[0]);
           EEPROM.write(2,time[1]);
            t=0;
            SetAlramOn = true;
          }
          break;
      case 'v':   // 밸브 on/off 설정
          if(Serial2.peek() == 's'){
            Serial2.read(); delay(5);
            for(int i=0;i<3;i++){
              buf3[i] = Serial2.read();
              delay(5);
            }
            user_Co2Concent = atoi(buf3);
            _printf("농도 제어 : %d\n",user_Co2Concent);
            analogWrite(CO2VALVE_L, user_Co2Concent); 
            memset(buf3,'\0',sizeof(buf3));
          }
          else{
            if(Serial2.read() == '1') VALVE(ON,true);
            else VALVE(OFF,true);
          }
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
            user_fanSpeed = fanSpeed;
            analogWrite(MOTOR_L, fanSpeed); 
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
          if(time[0]!=-1){
            Serial2.print("t,");
            Serial2.print(time[0]);Serial2.print(",");
            Serial2.println(time[1]);
          }
          else
            Serial2.println("t,n");
            
          /* 인터넷 시간 ~ RTC 동기화 */
          mobileSyncTime();
          //Serial.println(" ");          
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
void VALVE(bool in,bool android){
  if(in == ON){
    Serial.println("Valve ON");   
    analogWrite(CO2VALVE_L, user_Co2Concent);
    digitalWrite(CO2VALVE_S, HIGH);
    //Serial2.print("v");Serial2.println(",1");
    ANDROID_VALVE_UI  = true;
  }
  else {
    Serial.println("Valve OFF");
    digitalWrite(CO2VALVE_S, LOW);
    //Serial2.print("v");Serial2.println(",0");
    ANDROID_VALVE_UI  = false;
  }
  /*if(!android && in){
    Serial2.print("v");Serial2.println(",1");
  }
  else if(!android && !in){
    Serial2.print("v");Serial2.println(",0");
  }*/
}
void FAN(bool in,bool android){
  if(in == ON){
    Serial.println("Fan ON");
    analogWrite(MOTOR_L, fanSpeed);  
    digitalWrite(MOTOR_S, HIGH);
    //Serial2.print("f");Serial2.println(",1");
    ANDROID_FAN_UI = true;
  }
  else {
    Serial.println("Fan OFF");
    digitalWrite(MOTOR_S, LOW);
    //Serial2.print("f");Serial2.println(",0");
    ANDROID_FAN_UI = false;
  }
}
void HEAT(bool in,bool android){
  if(in == ON)Serial.println("Heat ON");
  else Serial.println("Heat OFF"); 

  if(!android && in){
    //Serial2.print("h");Serial2.println(",1");
  }
  else if(!android && !in){
    //Serial2.print("h");Serial2.println(",0");
 }
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
      if(modeBackEnable) {
        MODE--;
        prev_stack = 0;
      }
      else Serial.println("이전 모드로 이동 불가");
    }
  }
  else if(digitalRead(PREV_BT) == LOW)
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
        next_stack = 0;
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
    pixels.setBrightness(255);
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
  if(rtc.lostPower()){  // 가끔 이상함
    //ret_value = false;
    Serial.println("RTC Error : The RTC module losts power");
  }
  return ret_value;
}

/*-------------------------------------------------------------------------------------- NFC 리더 */
void readNFC(){
  static int readingTime = 0;
  if((readingTime++)==2000){
      readingTime = 0;
      MFRC522 mfrc522(SS_PIN, RST_PIN); // RFID 저장 방지를 위한, 객체 매번 생성. (더 좋은 방법이 떠오르질 않는다)
      mfrc522.PCD_Init();
      
      MFRC522::MIFARE_Key key;
      MFRC522::StatusCode status;
      for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;

      int it = 0;
      byte block,len,buffer1[18],buffer2[18];

     if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
        block = 4;
        len = 18;
        
        status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 4, &key, &(mfrc522.uid)); 
        if (status != MFRC522::STATUS_OK) {
          Serial.print(F("Authentication failed: "));
          Serial.println(mfrc522.GetStatusCodeName(status));
          //mfrc522.PICC_HaltA();
          //mfrc522.PCD_StopCrypto1();
          return;
        }
      
        status = mfrc522.MIFARE_Read(block, buffer1, &len);
        if (status != MFRC522::STATUS_OK) {
          Serial.print(F("Reading failed: "));
          Serial.println(mfrc522.GetStatusCodeName(status));
          //mfrc522.PICC_HaltA();
          //mfrc522.PCD_StopCrypto1();
          return;
        }
      
        //Read CO2 bottle code
        for (uint8_t i = 0; i < 16; i++)
          if (buffer1[i] != 32)
            co2code[it++] = (char)buffer1[i];
        
        Serial.print(co2code);
        memset(co2code,0,sizeof(co2code));

        mfrc522.PICC_HaltA();
        mfrc522.PCD_StopCrypto1();
      }
      else{
       // mfrc522.PICC_HaltA();
       // mfrc522.PCD_StopCrypto1();
      }
  }
}
/*-------------------------------------------------------------------------------------- 개발자 환경설정[Configuration] */
byte seconds, minutes, hours, day, date, month, year;
char weekDay[4];
byte tMSB, tLSB;
float temp3231;

void developerMode(){
  String cmd;
  menu();
  while(1){
    cmd  = readCommand();
    Serial.print(">> 입력 : ");
    Serial.print(cmd);
    if(cmd.charAt(0) == '1'){         // 블루투스 설정 모드
       int cmd2_i =0;
       char cmd2[50];
       _printf("\n<<<<                                                  >>>>");
       _printf("\n<<<<      블루투스 환경 설정. 명령어 종류 (소문자도 가능)   >>>>");
       _printf("\n<<<<                                                  >>>>\n");
       _printf("      AT          : 블루투스 연결 상태 확인(최초 마구 입력해서 확인할것) 정상 응답 : OK\n");
       _printf("      AT+NAME     : 현재 기기명 확인                                 정상 응답 : +NAME=기기명\n");
       _printf("      AT+NAME#### : 블루투스 기기명 설정. ex) AT+NAMEgosleep         정상 응답 : OK\n");
       _printf("      AT+PIN      : 현재 핀번호 확인                                 정상 응답 : +PIN=0000\n");
       _printf("      AT+PIN####  : 핀번호 설정. ex) AT+PIN1234                     정상 응답 : OK\n");
       _printf("      exit        : 블루투스 환경 설정 종료\n");
       memset(cmd2,'\0',sizeof(cmd2)); 
       while(1){ 
          if (Serial2.available())
              Serial.write(Serial2.read());
          if (Serial.available()){
              cmd2[cmd2_i++] = Serial.peek();
              if(Serial.peek() == '\n'){
                Serial.print(">> 입력 : ");
                Serial.print(cmd2);   
                if(cmd2[0] == 'e' && cmd2[1] == 'x' && cmd2[2] == 'i'&& cmd2[3] == 't') break;       
                memset(cmd2,'\0',sizeof(cmd2));  
                cmd2_i = 0;
              }
              Serial2.write(Serial.read());
          }
       }
       Serial.println("<<<< 블루투스 환경 설정 종료 >>>>");
       menu();
    }
    else if(cmd.charAt(0) == '2'){    // 제품 코드 설정 모드
      long numCode = 0;
      String oldCode="";
      _printf("\n<<<<               제품 코드(NYX-V######) 설정           >>>>");
      _printf("\n<<<<     자주 바꾸지 않기를 권장(EEPROM의 쓰기 제한 10만번) >>>>\n\n");
      _printf("버전 입력(한 자리 숫자) : ");
      cmd  = readCommand();
      Serial.println((byte)(cmd.substring(0,1).toInt()));
      numCode += (byte)(cmd.substring(0,1).toInt())*1000000;
      EEPROM.write(3,(byte)(cmd.substring(0,1).toInt()));
      
      _printf("일련번호 입력(6자리 숫자) : ");
      cmd  = readCommand();
      Serial.print(cmd.substring(0,2));
      EEPROM.write(4,(byte)(cmd.substring(0,2).toInt()));      // 메모리 4번 주소
      numCode += (byte)(cmd.substring(0,2).toInt())*10000;
      Serial.print(cmd.substring(2,4));
      EEPROM.write(5,(byte)(cmd.substring(2,4).toInt()));      // 메모리 5번 주소
      numCode += (byte)(cmd.substring(2,4).toInt())*100;
      Serial.println(cmd.substring(4,6));
      EEPROM.write(6,(byte)(cmd.substring(4,6).toInt()));      // 메모리 6번 주소
      numCode += (byte)(cmd.substring(4,6).toInt());

      _printf("입력된 제품 코드는 < NYX-%ld >\n",numCode);
      _printf("\n<<<< 제품 코드 설정 종료 >>>>\n");
      menu();
    }
    else if(cmd.charAt(0) == '4'){      // 팬, 밸브 테스트
      char testcmd;
      bool ontest = false;
      _printf("\n<<<<                         팬, 밸브 테스트                             >>>>");
      _printf("\n<<<<           (1 : 최대 출력 작동, 2: 작동 중지, 3: 테스트 종료)          >>>>\n\n");
      while(1){
        //if(Serial1.available()){
          //long ccc = Serial1.parseInt();
          //if(ccc*10>300 && ccc*10 <650000){
            _printf("현재 분사부 CO2 농도 : %ld", co2sensing()*10);
            if(ontest)
              _printf("(최대 출력 작동 중)");
            Serial.println();
        //  }
        //}
        //else
        //  Serial.println("Co2 Sensor Error");
        delay(1000);

        testcmd = Serial.read();
        if(testcmd == '1'){
          analogWrite(MOTOR_L, 255);  
          digitalWrite(MOTOR_S, HIGH); 
          //digitalWrite(4, LOW); 

          analogWrite(CO2VALVE_L, 255);
          digitalWrite(CO2VALVE_S, HIGH);
          ontest = true;
        }
        else if(testcmd == '2'){
          digitalWrite(MOTOR_S, LOW); 
          digitalWrite(CO2VALVE_S, LOW);
          ontest = false;
        }
        else if(testcmd == '3'){
          break;
        }
        testcmd='.';
      }
      Serial.println("<<<< 팬, 밸브 테스트 종료 >>>>");
      menu();
    }
    else if(cmd.charAt(0) == '5'){     // 관리자 모드 종료 후 고슬립 동작 시작.
      return;
    }
  }
}

String readCommand(){
  char c=' ';
  String cmd="";
  while(c != '\n'){
      if (Serial.available()){
        c = Serial.read();
        cmd+=c;
      }
  }
  return cmd;
}

void mobileSyncTime(){
  int packet[7],checksum=0;
  for(int i=0;i<7;i++){
    char c = Serial2.read();
    packet[i] = (int)c;
  }
  Serial.print(" ㄴ 안드로이드와 인터넷 시간 동기화 ~ 20");
  Serial.print(packet[0]);Serial.print("/");
  Serial.print(packet[1]);Serial.print("/");
  Serial.print(packet[2]);Serial.print(" ");
  Serial.print(packet[3]);Serial.print(":");
  Serial.print(packet[4]);Serial.print(":");
  Serial.print(packet[5]);Serial.print(" ");
  Serial.print(", Checksum(Android) : ");Serial.print(packet[6]);

  // checksum
  for(int i=0;i<6;i++){
     checksum += packet[i]/10;
     checksum += packet[i]%10;
  }
  Serial.print(", Checksum(Arduino) : ");Serial.print(checksum);
  if(checksum == packet[6]) {
    Serial.println(" ChecksumTest : 오류없음");
    rtc.adjust(DateTime(2000+packet[0],packet[1] ,packet[2],packet[3],packet[4],packet[5]));
  }
  else {
    Serial.println(" ChecksumTest : 오류있음, 재전송 요청");
    Serial2.println("r");
  }
}

void printTime(){
  char daysOfTheWeek[7][12] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
  DateTime now = rtc.now();
  Serial.print(" ");
  Serial.print(now.year(), DEC);Serial.print('/');
  Serial.print(now.month(), DEC);Serial.print('/');
  Serial.print(now.day(), DEC);Serial.print(" ");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);Serial.print(" ");
  Serial.print(now.hour(), DEC);Serial.print(':');
  Serial.print(now.minute(), DEC);Serial.print(':');
  Serial.print(now.second(), DEC);Serial.print(' ');
}

void menu(){
  Serial.println("");
  _printf("<<<<                                                   >>>>\n");
  _printf("<<<<                번호 선택 후 Enter                  >>>>\n");
  _printf("<<<< 앱과 연결은 종료 하는 것을 권장 (특히 블루투스 설정 시) >>>>\n");
  _printf("<<<<                                                   >>>>\n");
  _printf("       1. 블루투스 설정\n");
  _printf("       2. 제품 코드 설정\n");
  _printf("       3. (미개발)장착된 CO2 코드 설정\n");
  _printf("       4. 팬, 밸브 테스트 (최대 출력으로 동작)\n");
  _printf("       5. 종료 후 고슬립 작동 시작\n");
}
/*-------------------------------------------------------------------------------------- 로그 출력용 함수 */
void printLog(bool direct){
  static int printT = 0;
  if((printT++)==500 || direct){
       printTime();
       switch(MODE){
          case STOP_MODE:Serial.print(" 상태 : 절전 ");break;
          case WAIT_MODE:Serial.print(" 상태 : 대기 ");break;
          case DIST_MODE:Serial.print(" 상태 : 거리 ");break;
          case SLEEP_MODE:Serial.print(" 상태 : 수면 ");break;
          case SENS_MODE:Serial.print(" 상태 : 센싱 ");break;
          case WAKE_MODE:Serial.print(" 상태 : 기상 ");break;
      }
      if(BluetoothOn)Serial.print("| Android통신 ON ");    
      if(SetAlramOn){
         _printf("| 알람시간 :%2d시 %2d분",time[0],time[1]);
         if(alarmType ==0)
            Serial.print(" | 알람방식 : Nan" );
         else if(alarmType ==1)
            Serial.print(" | 알람방식 : 점진" );
         else if(alarmType ==2)
            Serial.print(" | 알람방식 : 즉각" );
      }
      Serial.println("");
      printT = 0;
  }
}
