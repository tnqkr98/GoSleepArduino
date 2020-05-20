
void setup() 
{
  Serial.begin(9600);   
  Serial2.begin(9600); 
}
void loop()
{
  if (Serial2.available()) {       
    Serial.write(Serial2.read());  
  }
  if (Serial.available()) {         
    Serial2.write(Serial.read()); 
  }

  //while(true){
    //Serial.println("hi");
  //  delay(1000);
  //}
}
