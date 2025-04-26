
//RTC
#include <RTClib.h>
RTC_DS1307 rtc;
DateTime now;

//IMU
#include <DFRobot_BMX160.h>
DFRobot_BMX160 IMU;
sBmx160SensorData_t Omagn, Ogyro, Oaccel;

//BH1750
#include <hp_BH1750.h>
hp_BH1750 DLDR;

//BME BME680
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#define SEALEVELPRESSURE_HPA (1013.25);
Adafruit_BME680 AQS;

//Button Pin allocation:
void DefineButtons(){
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  Serial.println("BUTTON PINS ALLOCATED");
}

  
  

bool SetUpErrorFlag = false;

void setup(){
  Serial.begin(9600);
  Serial.println();
  Serial.println("--STARTING SENSOR COMMS ON I2C--");
  if (rtc.begin() == false) {
    Serial.println("RTC NOT CONNECTED");
    SetUpErrorFlag = true;
  }
  else{
    Serial.println("RTC CONNECTED");
    rtc.adjust(DateTime(2025,4,26,0,0,0));
  }
  if (IMU.begin() == false) {  
    Serial.println("IMU NOT CONNECTED");    
    SetUpErrorFlag = true;
  }
  else{
    Serial.println("IMU CONNECTED");
  }
  if (DLDR.begin(BH1750_TO_GROUND) == false) {
    Serial.println("DLDR NOT CONNECTED");
    SetUpErrorFlag = true;
  }
  else{
    Serial.println("DLDR CONNECTED");
  }
  if (AQS.begin(0x76) == false) {
    Serial.println("AQS NOT CONNECTED");
    SetUpErrorFlag = true;
  }
  else{
    Serial.println("AQS CONNECTED");
    AQS.setTemperatureOversampling(BME680_OS_8X);
    AQS.setHumidityOversampling(BME680_OS_2X);
    AQS.setPressureOversampling(BME680_OS_4X);
    AQS.setIIRFilterSize(BME680_FILTER_SIZE_3);
    AQS.setGasHeater(320, 150);  // 320*C for 150 ms
  }
  DefineButtons();
  while(SetUpErrorFlag == true){
    Serial.println("Setup Error");
    delay(1000);
  }
}


void loop(){
  now =rtc.now();
  IMU.getAllData(&Omagn, &Ogyro, &Oaccel);
  if(ButtonPressed(1)){
    Serial.println("You pressed button 1");
  }
}
//Functions

//Function that checks if a button is pressed and released before returning True. buttonN corrosponds to button 1-4 on the board.
//BLOCKING
bool ButtonPressed(int buttonN){
  buttonN++;
  if(buttonN>5 || buttonN<2){
    Serial.print("ButtonError Out of range");
    return false;
  }
  else{
    if((digitalRead(buttonN)==LOW) && (buttonN != 0)){
    while(true){
      //Serial.println("button " + String(buttonNf) + " pressed");
      if(digitalRead(buttonN)==HIGH){
        //Serial.println("button " + String(buttonNf) + " released");
        return true;
      }
    }
    }
    else{
    return false;
    }
  }
}
