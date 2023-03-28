#define MPU6050_SPI_CS         PB9
#define MPU6050_SPI_MOSI       PC3
#define MPU6050_SPI_MISO       PC2
#define MPU6050_SPI_SCLK       PB10
#define MPU6050_RESET          PB8

#include <SPI.h>
#include "gyro.h"

void setup() {
  Serial.begin(115200);
  pinMode(MPU6050_RESET, OUTPUT);
  digitalWrite(MPU6050_RESET, LOW);
  delay(3000);
  digitalWrite(MPU6050_RESET, HIGH);
  delay(1000);
  bool gyroStatus = gyro_Init();
}

void loop() {
  gyroscope.getZGyroAngle();
  Serial.println(gyroscope.ZAngle);
  delay(100);
}
