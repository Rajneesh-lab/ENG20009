/*
   Adresses for devices are as follows:
   0x23 - BH1750
   0x50 - RTC Memory
   0x68 - RTC
   0x69 - IMU (NEEDS TO BE UPDATED IN THE BMX160 HEADER[.h] FILE)
   0x76 - BME680 (NEED TO CHECK HEADER [.h] ADDRESS IS CORRECT)
   
*/
//RTC
#include <RTClib.h>
//IMU
#include <DFRobot_BMX160.h>
//BH1750
#include <hp_BH1750.h>
//BME BME680
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#define SEALEVELPRESSURE_HPA (1013.25);

class ENG_Sensors{
  public:
  ENG_Sensors(){
    RTC_DS1307 rtc;
    DFRobot_BMX160 IMU;
    hp_BH1750 DLDR;
    Adafruit_BME680 AQS;

    Serial.println();
    Serial.println("--SENSOR OBJECT INIT-- --STARTING SENSOR COMMS ON I2C--");
    if (rtc.begin() == false) {
      Serial.println("RTC NOT CONNECTED");
    }
    else{
      Serial.println("RTC CONNECTED");
    }

    if (IMU.begin() == false) {
      Serial.println("IMU NOT CONNECTED");
    }
    else{
      Serial.println("IMU CONNECTED");
    }

    if (DLDR.begin(BH1750_TO_GROUND) == false) {
      Serial.println("DLDR NOT CONNECTED");
    }
    else{
      Serial.println("DLDR CONNECTED");
    }

    if (AQS.begin(0x76) == false) {
      Serial.println("AQS NOT CONNECTED");
    }
    else{
      Serial.println("AQS CONNECTED");
      AQS.setTemperatureOversampling(BME680_OS_8X);
      AQS.setHumidityOversampling(BME680_OS_2X);
      AQS.setPressureOversampling(BME680_OS_4X);
      AQS.setIIRFilterSize(BME680_FILTER_SIZE_3);
      AQS.setGasHeater(320, 150);  // 320*C for 150 ms
    }
  }
};
