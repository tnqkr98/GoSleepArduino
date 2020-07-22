#include <SoftwareSerial.h>
#define MAX_QUEUE_SIZE 100 
#define MAX_PACKET_SIZE 100 

SoftwareSerial Co2Serial(3, 2);  

char RxQueue[MAX_QUEUE_SIZE]; 
int QueWRPos=0; 
int QueRDPos=0;

char RxBuff[MAX_PACKET_SIZE]; 
int RxWRPos=0; 

void UARTx_ISR(void) { 
  int iTmp;
  iTmp=QueWRPos+1; 
  if(iTmp>=MAX_QUEUE_SIZE)
    iTmp=0;

  if(iTmp!=QueRDPos)  //Non-Overflow 
  {
      RxQueue[QueWRPos]=Co2Serial.read(); 
      QueWRPos=iTmp; 
  }
     //Overflow -> Discard data
}

// Packet format : [D6][D5][D4][D3][D2][D1][SP]['p']['p']['m'][0x0D][0x0A] 
int RxParser(void) { 
  int iPos,LP01; 
  char *pSrcBuff; 
  long RetPPM; //The data type is depend on MCU
  
  if(RxWRPos<7)
    return -1;
  //Minimum packet : [D1][SP]['p']['p']['m'][0x0D][0x0A] 
  
    iPos=RxWRPos-7; 
    pSrcBuff=&RxBuff[iPos]; 
    
    if((pSrcBuff[1]!=0x20)||
    (pSrcBuff[2]!='p')|| 
    (pSrcBuff[3]!='p')|| 
    (pSrcBuff[4]!='m')|| 
    (pSrcBuff[5]!=0x0D)|| 
    (pSrcBuff[6]!=0x0A)) 
      return -1; 
     
  pSrcBuff[1]=0; //Insert the termination character in [SP]
  //Search the first character of PPM string 
  pSrcBuff=NULL; 
  for(LP01=0;LP01<=iPos;LP01++) 
    if(RxBuff[LP01]!=0x20) { 
      pSrcBuff=&RxBuff[LP01];
      break; 
    }
  
  if(pSrcBuff==NULL)
    return -1;

  RetPPM=atol(pSrcBuff); //or atoi(pSrcBuff)
  Serial.println(RetPPM);
  return 0;
}

void UARTx_Process(void) { 
  char cTmp; 
  while(QueWRPos!=QueRDPos) { //Serialization 
    cTmp=RxQueue[QueRDPos]; 
    QueRDPos++; 
    if(QueRDPos>=MAX_QUEUE_SIZE)
      QueRDPos=0;
      
    RxBuff[RxWRPos]=cTmp;
    RxWRPos++;

    if(cTmp==0x0A) {  //0x0A=End of Packet 
      RxParser(); 
      RxWRPos=0; 
      return; 
    } 
    else if(RxWRPos>=MAX_PACKET_SIZE) //Discard data on overflow 
      RxWRPos=0; 
  }
}


void setup() {
  Serial.begin(38400);
  Co2Serial.begin(38400);
}

void loop() {

 // UARTx_ISR();
  //UARTx_Process();
  //Serial.println(RxBuff);
  if (Co2Serial.available())
        Serial.write(Co2Serial.read());
  if (Serial.available())
       Co2Serial.write(Serial.read());
  //delay(1000);
}
