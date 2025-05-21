//Scanner for diagnostics
#include "I2CScanner.h"
I2CScanner scanner;
  //scanner.Init();
  //scanner.Scan();

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
  Serial.println("BUTTON PINS ALLOCATED");
}

  
int DIRO = 7;//pin that controls the input/output state SDI adaptor. HIGH receives and low sends from arduino

bool SetUpErrorFlag = false;

void setup(){
  SetupSensors();
  SetupSDI12();
  DefineButtons();
}


void loop(){
  now =rtc.now();
  IMU.getAllData(&Omagn, &Ogyro, &Oaccel);
  if(ButtonPressed(1)){
    Serial.println("You pressed button 1");
    SDI12send("abc");
  }
  if(ButtonPressed(2)){
    Serial.println("You pressed button 2");
    Serial.println(SDI12receive());
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
void SetupSDI12(){
  //SDI-12
  Serial1.begin(1200, SERIAL_7E1);//SDI-12 UART, configures serial port for 7 data bits, even parity, and 1 stop bit
  pinMode(DIRO, OUTPUT);//DIRO Pin
  digitalWrite(DIRO, HIGH);//lsiten
}
