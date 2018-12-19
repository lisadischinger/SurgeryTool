void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);      // open the serial port at 9600 bps:
  unsigned long StartTime = millis();
  Serial.print("StartTime: ");
  Serial.print (StartTime);
  delay(1000);                  // waits for a second
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentTime = millis();
  Serial.print("currentTime: ");
  Serial.print (currentTime);
  delay(1000);                  // waits for a second
  
}
