void setup() {
  Serial.begin(115200);

  // NOT PRESSED  HIGH 0
  // PRESSED      LOW  1  
  pinMode(2, INPUT_PULLUP); // Internal pull-up resistor, the pin reads HIGH and LOW when pressed

}

void loop() {
  int voltageRead = analogRead(A0); // 0-1023V Reading
  int drone_go = (digitalRead(2) == LOW);   // button pressed = 1

  //Uncomment and open serial monitor to test out
  Serial.print(voltageRead); // left: 0 | right: 1023 | middle: 500
  Serial.print(",");
  Serial.println(drone_go);
  delay(50);
}