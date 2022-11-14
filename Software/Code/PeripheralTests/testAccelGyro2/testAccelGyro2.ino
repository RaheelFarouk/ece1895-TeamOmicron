// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup() {
    //Setting up the Gyro?/Accel
  // Try to initialize MPU6050!
  Serial.begin(9600);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_5_HZ);


  Serial.println("");
  delay(100);
}

void loop() {

      /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    // Serial.println(a.acceleration.z);
    if((a.acceleration.z)<-15.0){ //-10.0 is the acceleration threshold
      Serial.println(a.acceleration.z);
    }
    delay(100);
}