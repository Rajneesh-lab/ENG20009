//Scanner for diagnostics
#include "I2CScanner.h"
I2CScanner scanner;

//LCD PINS
#define TFT_CS    10
#define TFT_RST   6 
#define TFT_DC    7 
#define TFT_SCLK 13   
#define TFT_MOSI 11   
//Adafruit_ST7735 LCD SETUP
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);


//I2C bus
#include <Wire.h>

//RTC
#include <RTClib.h>
RTC_DS1307 rtc;
DateTime now;

//IMU
#include <DFRobot_BMX160.h>
DFRobot_BMX160 IMU;
sBmx160SensorData_t Omagn, Ogyro, Oaccel;

//BH1750 Christopher Laws
#include <hp_BH1750.h> 
hp_BH1750 DLDR;

//BME BME280
#include <Adafruit_BME280.h>
Adafruit_BME280 BME;
#define SeaLevel_HPA (1013.25)

//Button Pin allocation:
void DefineButtons(){
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  Serial.println("--Button Pins Allocated--");
}

  
int DIRO = 9;//pin that controls the input/output state SDI adaptor. HIGH receives and low sends from arduino

bool SetUpErrorFlag = false;

void setup(){
  Serial.begin(9600);
  Wire.begin();
  SetupSensors();
  SetupSDI12();
  DefineButtons();
  SetupLCD();
  Serial.println("--SetUp Complete");
}


void loop(){
  if(ButtonPressed(1)){
    ReadAll();
  }
  if(ButtonPressed(2)){
    Serial.println("You pressed button 2");
    Serial.println(SDI12receive());
  }
}
//Functions

//Function that checks if a button is pressed and released before returning True. buttonN corrosponds to button 1-4 on the board.
//BLOCKING!!
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

void SDI12send(String str){
  digitalWrite(DIRO, LOW);//Send
  Serial1.println(str);
  delay(200);
  Serial1.flush();
  SDI12receive();
}
String SDI12receive(){
  digitalWrite(DIRO, HIGH);//Receive
  String output;
  Serial1.read(); // Remove first dead char
  while(Serial1.available()){
    char c = Serial1.read();
    output+=c;
  }
  return output;
}
void SetupSensors(){
  scanner.Init();
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
  if (BME.begin(0x76) == false) {
    Serial.println("BME NOT CONNECTED");
    SetUpErrorFlag = true;
  }
  else{
    Serial.println("BME CONNECTED");
  }
  while(SetUpErrorFlag == true){
    scanner.Scan(); //Debugging Loop
    delay(1000);
  }

}

//BME Enviromental sensor
float ReadTemperature(){
  return BME.readTemperature();
}
float ReadPressure(){
  return (BME.readPressure() / 100.0F);
}
float ReadAltitude(){
  return BME.readAltitude(SeaLevel_HPA);
}
float ReadHumidity(){
  return BME.readHumidity();
}

//Acceleration
float ReadAccelerationX(){
  IMU.getAllData(&Omagn, &Ogyro, &Oaccel);
  return Oaccel.x;
}
float ReadAccelerationY(){
  IMU.getAllData(&Omagn, &Ogyro, &Oaccel);
  return Oaccel.y;
}
float ReadAccelerationZ(){
  IMU.getAllData(&Omagn, &Ogyro, &Oaccel);
  return Oaccel.z;
}
//Magnetometer
float ReadMagnetometerX(){
  IMU.getAllData(&Omagn, &Ogyro, &Oaccel);
  return Omagn.x;
}
float ReadMagnetometerY(){
  IMU.getAllData(&Omagn, &Ogyro, &Oaccel);
  return Omagn.y;
}
float ReadMagnetometerZ(){
  IMU.getAllData(&Omagn, &Ogyro, &Oaccel);
  return Omagn.z;
}
//gyrometer 
float ReadGyrometerX(){
  IMU.getAllData(&Omagn, &Ogyro, &Oaccel);
  return Ogyro.x;
}
float ReadGyrometerY(){
  IMU.getAllData(&Omagn, &Ogyro, &Oaccel);
  return Ogyro.y;
}
float ReadGyrometerZ(){
  IMU.getAllData(&Omagn, &Ogyro, &Oaccel);
  return Ogyro.z;
}

//Light Sensor
float ReadLux(){
  DLDR.start();
  //Wait for measurement to be ready
  while(!DLDR.hasValue()){
  }
  return DLDR.getLux();
}

//Diagnostic to check all sensors
void ReadAll(){
  Serial.println("###BME###");
  delay(300);
  Serial.println("ReadTemperature " + String(ReadTemperature()));
  delay(300);
  Serial.println("ReadPressure " + String(ReadPressure()));
  delay(300);
  Serial.println("ReadAltitude " + String(ReadAltitude()));
  delay(300);
  Serial.println("ReadHumidity " + String(ReadHumidity()));

  Serial.println("###IMU###");
  delay(300);
  Serial.println("ReadAccelerationX " + String(ReadAccelerationX()));
  Serial.println("ReadAccelerationY " + String(ReadAccelerationY()));
  Serial.println("ReadAccelerationZ " + String(ReadAccelerationZ()));
  delay(300);
  Serial.println("ReadMagnetometerX " + String(ReadMagnetometerX()));
  Serial.println("ReadMagnetometerY " + String(ReadMagnetometerY()));
  Serial.println("ReadMagnetometerZ " + String(ReadMagnetometerZ()));
  delay(300);
  Serial.println("ReadGyrometerX " + String(ReadGyrometerX()));
  Serial.println("ReadGyrometerY " + String(ReadGyrometerY()));
  Serial.println("ReadGyrometerZ " + String(ReadGyrometerZ()));


  Serial.println("###DLDR###");
  Serial.println("DLDR " + String(ReadLux()));
}

void SetupSDI12(){
  //SDI-12
  Serial1.begin(1200, SERIAL_7E1);//SDI-12 UART, configures serial port for 7 data bits, even parity, and 1 stop bit
  pinMode(DIRO, OUTPUT);//DIRO Pin
  digitalWrite(DIRO, HIGH);//lsiten
}

void SetupLCD(){
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  Serial.println("--LCD SetUp Complete--");
}
