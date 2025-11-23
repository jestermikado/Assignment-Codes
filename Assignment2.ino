// Pseudo-code to initialise + read IMU @100 Hz

#include <Wire.h>
#include "MPU6050.h"

MPU6050 sensor;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  sensor.initialize();                
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;

  sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.printf("A:%d %d %d  G:%d %d %d\n",
                ax, ay, az, gx, gy, gz);

  delay(10);   
}
