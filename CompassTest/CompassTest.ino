#include <Wire.h>
#include <LSM303.h>

LSM303 compass;


float headingOffset, adjHeading;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  initCompass();

}

void loop() {
  

  getHeading();
  
  Serial.println(adjHeading);
  delay(100);
}

void initCompass () {  
  compass.init();
  compass.enableDefault();

  /*
      Calibration values obtained from running calibration example.
  */
  compass.m_min = (LSM303::vector<int16_t>){-398, -374, -489};
  compass.m_max = (LSM303::vector<int16_t>){+240, +250, +0};
  
  float _heading = 0;
  for (int i = 0; i < 100; i++) {
    delay(10);
    compass.read();
    _heading = _heading + compass.heading();
  }
  headingOffset = _heading / 100;
  
}

void getHeading() {  
  compass.read();
  adjHeading = compass.heading() - headingOffset;
  if (adjHeading < 0) adjHeading += 360;
  else if (adjHeading > 360) adjHeading -= 360;
  
  if (adjHeading < 180) adjHeading = -adjHeading;
  else if (adjHeading > 180) adjHeading = -(adjHeading - 360);
}
