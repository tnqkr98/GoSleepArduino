#include <SoftwareSerial.h>
#include "Type4067Mux.h"
#define PER 40
#define MAX_ERR 10

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
  for(int i=0;i<=56;i++)
    if(co2[i].channel >=0)
      co2mux(true,i,co2[i].mux,co2[i].channel,true);
  for(int i=0;i<=22;i++)
    if(FAN[i].channel >=0)
      co2mux(true,i,FAN[i].mux,FAN[i].channel,false);
}

void loop() {
  Serial.print("DATA,TIME,");
  for(int i=0;i<=56;i++)
    if(co2[i].channel >=0)
      co2mux(false,i,co2[i].mux,co2[i].channel,true);
  for(int i=0;i<=22;i++)
    if(FAN[i].channel >=0)
      co2mux(false,i,FAN[i].mux,FAN[i].channel,false);
  Serial.println(" ");
}

void co2mux(bool init, int num, int mux, int channel, bool co2OrFan){  //true : co2, false :fan
   long data,co2data;

  do{
    if(mux == 1){
      data = mux1.read(channel);
      delay(200); 
      mux_1.listen();
      mux_1.println("Z");
      mux_1.flush();
      mux_1.read();
      co2data = mux_1.parseInt()*10;
    }else if(mux == 2){
      data = mux2.read(channel);
      delay(200);  
      if(num != 56){
        mux_2.listen();
        mux_2.println("Z");
        mux_2.flush();
        mux_2.read();
      }else if(num ==56){
        head.listen();
        head.println("Z");
        head.flush();
        mux_2.read();
      }
      co2data = mux_2.parseInt()*10;
      if(num == 56)
        mux_2.read();
    }
  }while((co2data == 0 && num != 56));
  
  if(co2OrFan){
    if(init){
      co2[num].before = co2data;
      return;
    }
  
    if(co2data >= co2[num].before*0.6 && co2data <= co2[num].before*1.4){
      Serial.print(co2data);
      delay(10);
      Serial.print(",");
      co2[num].before = co2data;
    }else{
      Serial.print(co2[num].before);
      delay(10);
      Serial.print(",");
      co2[num].err_count++;
      if(co2[num].err_count == MAX_ERR){
        co2[num].err_count =0;
        co2[num].before = co2data;
      }
    }
  }
  else{
    if(init){
      FAN[num].before = co2data;
      return;
    }  
  
    if(co2data >= FAN[num].before*0.6 && co2data <= FAN[num].before*1.4){
      Serial.print(co2data);
      Serial.print(",");
      FAN[num].before = co2data;
    }else{
      Serial.print(FAN[num].before);
      Serial.print(",");
      FAN[num].err_count++;
      if(FAN[num].err_count == MAX_ERR){
        FAN[num].err_count =0;
        FAN[num].before = co2data;
      }
    }
  }
}
