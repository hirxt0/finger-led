int leds[] = {2, 3, 4, 5, 6}; 

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 5; ++i) pinMode(leds[i], OUTPUT);
}

void loop() {
    if (Serial.available() > 0) {
      String line = Serial.readStringUntil('\n');
      line.trim();
      int count = line.toInt();
      if (count < 0) count = 0;
      if (count > 5) count = 5;
      for (int i = 0; i < 5; ++i) {
        digitalWrite(leds[i], i < count ? HIGH : LOW);
    }
  }
  // put your main code here, to run repeatedly:

}
