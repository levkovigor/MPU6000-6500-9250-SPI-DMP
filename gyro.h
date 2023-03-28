#include "src/MPU6050_6Axis_MotionApps612.h"

class GyroObject {
  public:
    SPIClass mpu6050SPI;
    MPU6050 mpu = MPU6050(MPU6050_SPI_CS, &mpu6050SPI);
    int autoCalibrationQuality = 1;
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    float ZAngle;
    float ZAngleR = 0;
    float uartZAngle = 0.0;
    int16_t gyroXOffset = 0;
    int16_t gyroYOffset = 0;
    int16_t gyroZOffset = 0;
    int16_t accXOffset = 0;
    int16_t accYOffset = 0;
    int16_t accZOffset = 0;
    unsigned long gyroTimerStatus = 0;
    bool isInited = false;
    
    bool initialization(){
      pinMode(MPU6050_SPI_CS, OUTPUT);
      digitalWrite(MPU6050_SPI_CS, HIGH);
      mpu6050SPI.setMOSI(MPU6050_SPI_MOSI);
      mpu6050SPI.setMISO(MPU6050_SPI_MISO);
      mpu6050SPI.setSCLK(MPU6050_SPI_SCLK);
      mpu6050SPI.begin();
      mpu.initialize();
      if(mpu.testConnection() != true){
        return false;
      }
      uint8_t devStatus = mpu.dmpInitialize();   
      if (devStatus == 0) {
        for (int i = 0; i < autoCalibrationQuality; i++){
          mpu.CalibrateAccel(15);
          mpu.CalibrateGyro(15);
        }
        //mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        packetSize = mpu.dmpGetFIFOPacketSize();
      } else {
        return false;
      }
       gyroXOffset = mpu.getXGyroOffset();
       gyroYOffset = mpu.getYGyroOffset();
       gyroZOffset = mpu.getZGyroOffset();
       accXOffset = mpu.getXAccelOffset();
       accYOffset = mpu.getYAccelOffset();
       accZOffset = mpu.getZAccelOffset();
       isInited = true;
       gyroTimerStatus = millis();
       return true;
    }

    void gyroRestart(){
      mpu6050SPI.setMOSI(MPU6050_SPI_MOSI);
      mpu6050SPI.setMISO(MPU6050_SPI_MISO);
      mpu6050SPI.setSCLK(MPU6050_SPI_SCLK);
      mpu6050SPI.begin();
      if (isInited){
        mpu.initialize();
        uint8_t devStatus = mpu.dmpInitialize();   
        if (devStatus == 0) {
          mpu.setXGyroOffset(gyroXOffset);
          mpu.setYGyroOffset(gyroYOffset);
          mpu.setZGyroOffset(gyroZOffset);
          mpu.setXAccelOffset(accXOffset);
          mpu.setYAccelOffset(accYOffset);
          mpu.setZAccelOffset(accZOffset);
          mpu.setDMPEnabled(true);
          mpuIntStatus = mpu.getIntStatus();
          packetSize = mpu.dmpGetFIFOPacketSize();
          while (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { }
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          ZAngleR = ypr[0] * 180.0/M_PI + 0.0042 * millis() / 1000.0 - ZAngle; 
        }
      } else {
        initialization();
      }
      gyroTimerStatus = millis();   
    }

    bool getZGyroAngle(void){
      if(mpu.testConnection()){
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);           
          ZAngle = ypr[0] * 180.0/M_PI + 0.0042 * millis() / 1000.0 - ZAngleR;
          gyroTimerStatus = millis();
        }
      } else {
        digitalWrite(MPU6050_RESET, LOW);
        mpu6050SPI.end();
        Serial.println("gyro_drop"); 
        delay(100);
        digitalWrite(MPU6050_RESET, HIGH);  
        gyroRestart();
        return false;
      }

      if (gyroTimerStatus + 200 < millis()){
        digitalWrite(MPU6050_RESET, LOW);
        mpu6050SPI.end();
        Serial.println("gyro_drop"); 
        delay(100);
        digitalWrite(MPU6050_RESET, HIGH);  
        gyroRestart();
        return false;
      }

      return true;
    }
};

GyroObject gyroscope;

bool gyro_Init(void){
   return gyroscope.initialization(); 
}
