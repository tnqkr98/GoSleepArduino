


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial2.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial2.available()){
    char c = Serial2.read();
    Serial.print(c);Serial.print(" ");
  }
  Serial.println("");
  delay(1000);
}