#include <SoftwareSerial.h>
#include "Type4067Mux.h"
#define PER 0.3
#define MAX_ERR 3
#define AVG_COUNT 5        // MAX_ERR 보다 커야함.

SoftwareSerial mux_1(A15,A14);
SoftwareSerial mux_2(A13,A14);
SoftwareSerial head(A12,A14);
Type4067Mux mux1(A15, INPUT, ANALOG, 7, 6, 5, 4);
Type4067Mux mux2(A13, INPUT, ANALOG, 13, 12, 11, 10);

long before_H;

typedef struct nn{
  int mux;
  int channel;
  int err_count;
  long before; 
} Node;
Node co2[57]; // co2_H : 56
Node FAN[23];

void setup() {
  Serial.begin(9600);
  mux_1.begin(9600);
  mux_2.begin(9600);
  head.begin(9600);
  Serial2.begin(9600);
  Serial.println("LABEL,Time,11,12,13,14,15,21,22,23,24,25,31,32,33,34,35,41,42,43,44,45,51,52,53,54,55,H1,Fan11,Fan12,Fan21,Fan22");
  for(int i=0;i<=41;i++){
    co2[i].mux =1;
    co2[i].err_count=0;
    co2[i].channel = -1;
    co2[i].before=0;
  }
  for(int i=42;i<=56;i++){
    co2[i].mux = 2;
    co2[i].err_count=0;
    co2[i].channel = -1;
    co2[i].before=0;
  }
  for(int i=0;i<=22;i++){
    FAN[i].mux = 2;
    FAN[i].err_count=0;
    FAN[i].channel = -1;
    FAN[i].before=0;
  }

  co2[11].channel = 0;
  co2[12].channel = 1;
  co2[13].channel = 2;
  co2[14].channel = 3;
  co2[15].channel = 4;
  co2[21].channel = 5;
  co2[22].channel = 6;
  co2[23].channel = 7;
  co2[24].channel = 8;
  co2[25].channel = 9;
  co2[31].channel = 10;
  co2[32].channel = 11;
  co2[33].channel = 12;
  co2[34].channel = 13;
  co2[35].channel = 14;
  co2[41].channel = 15;
  
  co2[42].channel = 0;
  co2[43].channel = 1;
  co2[44].channel = 2;
  co2[45].channel = 3;
  co2[51].channel = 4;
  co2[52].channel = 5;
  co2[53].channel = 6;
  co2[54].channel = 7;
  co2[55].channel = 8;
  co2[56].channel = 9; // H

  FAN[11].channel = 10;
  FAN[12].channel = 11;
  FAN[21].channel = 12;
  FAN[22].channel = 13;

  //initialize
  //Serial.print("DATA,TIME,");
  for(int i=0;i<=56;i++)
    if(co2[i].channel >=0)
      co2mux2(true,i,co2[i].mux,co2[i].channel,true);
  for(int i=0;i<=22;i++)
    if(FAN[i].channel >=0)
      co2mux2(true,i,FAN[i].mux,FAN[i].channel,false);
  //Serial.println(" ");
}

void loop() {
  Serial.print("DATA,TIME,");
  for(int i=0;i<=56;i++)
    if(co2[i].channel >=0)
      co2mux2(false,i,co2[i].mux,co2[i].channel,true);
  for(int i=0;i<=22;i++)
    if(FAN[i].channel >=0)
      co2mux2(false,i,FAN[i].mux,FAN[i].channel,false);
  //co2mux2(11,co2[11].mux,co2[11].channel,true);
  Serial.println(" ");
}

void co2mux2(bool init,int num, int mux, int channel, bool co2OrFan){ 
  int i=0,err_count =0;
  long data,co2data=0,result=0,before;
  
  while(1){
    if(mux == 1){
        data = mux1.read(channel);
        delay(50); 
        mux_1.listen();
        mux_1.println("Z");
        mux_1.flush();
        mux_1.read();
        co2data = mux_1.parseInt()*10;
    }else if(mux == 2){
        data = mux2.read(channel);
        delay(50);  
        if(num != 56){
          mux_2.listen();
          mux_2.println("Z");
          mux_2.flush();
          mux_2.read();
          co2data = mux_2.parseInt()*10;
        }else if(num ==56){
          head.listen();
          head.println("Z");
          head.flush();
          mux_2.read();
          co2data = head.parseInt()*10;
        }
        if(num == 56)
          mux_2.read();
    }
    //Serial.println("");Serial.print("!");Serial.print(co2data); Serial.print(", i : ");Serial.print(i);Serial.println("  ");
    if(i==0)
      before = co2data;
      
    if(co2data!=0 && ((before*(1-PER) < co2data) && (before*(1+PER) > co2data))){
      result += co2data;
      before = co2data;
      err_count =0;
      i++;
    }else{
      err_count++;
      if(err_count ==MAX_ERR){
        err_count =0;
        before = co2data;
      }
    }
    
    if(i == AVG_COUNT) break;
  }
  
  if(init && co2OrFan){
    co2[num].before = result/AVG_COUNT;
    //Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    //Serial.print(co2[num].before);
    //Serial.print(",");
  }

  if(init && !co2OrFan){
    FAN[num].before = result/AVG_COUNT;
    //Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    //Serial.print(FAN[num].before);
    //Serial.print(",");
  }
    
  if(!init && co2OrFan){
    //Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    Serial.print((result/AVG_COUNT + co2[num].before)/2);
    Serial.print(",");
  }
  else if(!init && !co2OrFan){
    //Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    Serial.print((result/AVG_COUNT + FAN[num].before)/2);
    Serial.print(",");
  }
  
}
