

#define RUN 900
#define TIM 300

int running_sec = 0,cnt = 0;
bool BTN_ON = false;

void setup(){
  Serial.begin(9600);
  pinMode(7, OUTPUT); 
  pinMode(6, INPUT);
  digitalRead(6, HIGH);
}

void loop() {
 long pastTime = millis();

 if(digitalRead(6)==LOW && !BTN_ON){
    BTN_ON = true;
    delay(1000);
 }
 else if(digitalRead(6)==LOW && BTN_ON){
    BTN_ON = false;
    delay(1000);
 }

  cnt++;
  if(cnt == 10){
    if(running_sec <= RUN && BTN_ON){
      running_sec++;
      Serial.print("작동중(밸브 열림): ");
      digitalWrite(7, HIGH);
      Serial.print(running_sec);Serial.println("초");
    }
    else if(running_sec <= RUN+TIM && BTN_ON){
      Serial.print("작동중(밸브 닫힘): ");
      digitalWrite(7, LOW);
      Serial.print(running_sec-RUN);Serial.println("초");
    }
    else if(BTN_ON)
      running_sec = 0;
    else if(!BTN_ON){
      Serial.println("미작동중 ...");
      running_sec = 0;
    }
    cnt =0;
  }

  while(millis() - pastTime < 100)  // 루프주기 0.1초
     delay(1); 
}
