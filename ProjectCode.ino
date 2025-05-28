#include <SPI.h>
#include <Wire.h>
//Stepper
#include <AccelStepper.h> //Include the AccelStepper library
#define MP1  51 // IN1 on the ULN2003
#define MP2  49 // IN2 on the ULN2003
#define MP3  47 // IN3 on the ULN2003
#define MP4  45 // IN4 on the ULN2003
#define MotorInterfaceType 8 // Define the interface type as 8 = 4 wires * step factor (2 for half step)
AccelStepper stepper = AccelStepper(MotorInterfaceType, MP1, MP3, MP2, MP4);//Define the pin sequence (IN1-IN3-IN2-IN4)
const int SPR = 2048;//Steps per revolution

//RTC
#include "RTClib.h" 

//SD
#include "SdFat.h"
//define file system for sd card
SdFs sd;
FsFile file;
// Chip select may be constant or RAM variable.
const uint8_t SD_CS_PIN = A3;
// Pin numbers in templates must be constants.
const uint8_t SOFT_MISO_PIN = 12;
const uint8_t SOFT_MOSI_PIN = 11;
const uint8_t SOFT_SCK_PIN  = 13;
// SdFat software SPI template
SoftSpiDriver<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> softSpi;
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(0), &softSpi)
String Datatype;

//Scanner for diagnostics
#include "I2CScanner.h"
I2CScanner scanner;

//SDI
String command;
String DeviceAddress = "0"; // default address for project
String deviceIdentification = "014ENG20009123456789";
int DIRO = 9;//pin that controls the input/output state SDI adaptor. HIGH receives and low sends from arduino


String T = "0.0"; //Temp
String P = "0.0"; //pressure
String A = "0.0"; //Altitude
String H = "0.0"; //humidity
String L = "0.0"; //LUX
String Amag = "0.0"; //Accel Magnitude
String Gmag = "0.0"; //gyro Magnitude
String Mmag = "0.0"; //Mag Magnitude


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

//Menu Variable
int currentMenu = 0;
int selectedMenu = 1;
bool menuInitialized[8] = {false};
bool mainMenuInitialized = false;

int graphX = 0;
unsigned long lastGraphUpdate = 0;
const unsigned long graphInterval = 50;

String MenuText[] = {
  "",
  "Accelerometer",
  "Gyroscope",
  "Magnetometer",
  "Light",
  "Temperature",
  "Pressure",
  "Altitude",
  "Humidity"
};


//I2C bus
#include <Wire.h>

//RTC
#include <RTClib.h>
RTC_DS1307 rtc;
DateTime now;

//Stepper
bool TimerFlag = true;

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
void SetupButtons(){
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  Serial.println("--Button Pins Allocated--");
  pinMode(53, INPUT_PULLUP);//Logger vs SDI mode switch
  
}

bool SetUpErrorFlag = false;

void setup(){
  Serial.begin(9600);
  Wire.begin();
  SetupSensors();
  SetupSDI12();
  SetupButtons();
  SetupStepper();
  SetupLCD();
  SetupSD();
  SetupInterupt();

  Serial.println("|--SetUp Complete--|");
}


void loop(){
  delay(100);
  TimerFlagHandler();
  if(DataLoggerMode()){
    PollMenu();
  }
  else{
    PollSDI();
    }
}
//Functions
void SetupSD(){
  if (!sd.begin(SD_CONFIG)) {
    Serial.println("SD card initialization failed!");
    sd.initErrorHalt();
  }
  else{
    file.open("LoggerFile.txt", O_RDWR | O_CREAT);
    file.close();
    Serial.print("--SD Setup--");
  }
}
void LogVariable(int type, float Value1, float Value2, float Value3) {  
  now = rtc.now();
  switch(type){
    case 1:
      Datatype = "Accelerometer ";
      break;
    case 2:
      Datatype = "Gyroscope ";
      break;
    case 3:
      Datatype = "Magnetometer ";
      break;
    case 4:
      Datatype = "Light ";
      break;
    case 5:
      Datatype = "Temperature ";
      break;
    case 6:
      Datatype = "Pressure ";
      break;
    case 7:
      Datatype = "Altitude ";
      break;
    case 8:
      Datatype = "Humidity ";
      break;
    default:
      Datatype = "UNKNOWN ";
      break;

  }
  file.open("LoggerFile.txt", O_RDWR);
  file.seekEnd();
  file.println("<"+ String(now.year()) + "/" +String(now.month())+ "/" + String(now.day())+ "/" + String(now.hour())+ "/" + String(now.minute())+ "/" + String(now.second()) + "> " + Datatype + String(Value1) + "|" + String(Value2)+ "|" + String(Value3));
  file.close();
}




void TC3_Handler(){
  TC_GetStatus(TC1, 0);
  TimerFlag = true;
}
void TimerFlagHandler(){
  if(TimerFlag){
    TimerFlag = false;
    if(ReadHumidity()<60){
      OpenStepper();
    }
    else{
      CloseStepper();
    }
  }
}
void OpenStepper(){
  stepper.moveTo(-0.5*SPR); //Set the open motor position (i.e. turn motor for 3 full revolutions)
  stepper.runToPosition(); // Run the motor to the target position
}

void CloseStepper(){
  stepper.moveTo(0*SPR); //Set the close motor position (i.e. turn motor for 3 full revolutions)
  stepper.runToPosition(); // Run the motor to the target position
}

void SetupInterupt(){
  float freq = 2; //frequency hz
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(TC3_IRQn);
  TC_Configure(TC1, 0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4); //Sets up Timer1 Channel 0
  uint32_t rc = VARIANT_MCK / 128 / freq;
  TC_SetRA(TC1, 0, rc >> 1); // 50% duty cycle square wave
  TC_SetRC(TC1, 0, rc); //Overflow and Interrupt
  TC_Start(TC1, 0);
  TC1->TC_CHANNEL[0].TC_IER=  TC_IER_CPCS | TC_IER_CPAS; //interupt enable register
  TC1->TC_CHANNEL[0].TC_IDR=~(TC_IER_CPCS | TC_IER_CPAS); //interupt disable register
  NVIC_EnableIRQ(TC3_IRQn); //enable innterupt handler
  Serial.println("--ISR Setup--");
}
void SetupStepper(){
  stepper.setMaxSpeed(1000);//Set the maximum motor speed in steps per second
  stepper.setAcceleration(200);//Set the maximum acceleration in steps per second^2
  Serial.println("--Stepper SetUp--");
}

bool DataLoggerMode(){
  return digitalRead(53);
}

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
  /*
  while(SetUpErrorFlag == true){
    scanner.Scan(); //Debugging Loop
    delay(1000);
  }
  */

}

//###BME### Enviromental sensor
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

//###MCU###
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

//###Light Sensor###
float ReadLux(){
  DLDR.start();
  //Wait for measurement to be ready
  while(!DLDR.hasValue()){
  }
  return DLDR.getLux();
}

//Diagnostic to check all sensors
/*
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
*/

void SetupLCD(){
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  Serial.println("--LCD SetUp Complete--");
}

void PollMenu(){
  if (currentMenu == 0){
    showMainMenu();
    checkMainMenuButtons();
  }
  else{
    if (millis() - lastGraphUpdate >= graphInterval) {
      lastGraphUpdate = millis();
      showSensorGraph(currentMenu);
    }
  }
}


void showSensorGraph(int type){
  int x = 0, y = 0, z = 0;
  //maps raw readings and transforms them into dispalayable coordiantes
  if (type == 1) {
    x = map(ReadAccelerationX(), -14, 14, 120, 10);//set graph coords
    y = map(ReadAccelerationY(), -14, 14, 120, 10);//set graph coords
    z = map(ReadAccelerationZ(), -14, 14, 120, 10);//set graph coords
  } 
  else if (type == 2) {
    x = map(ReadGyrometerX(), -50, 50, 120, 10);//set graph coords
    y = map(ReadGyrometerY(), -50, 50, 120, 10);//set graph coords
    z = map(ReadGyrometerZ(), -50, 50, 120, 10);//set graph coords
  } 
  else if (type == 3) {
    x = map(ReadMagnetometerX(), -350, 350, 120, 10);//set graph coords
    y = map(ReadMagnetometerY(), -350, 350, 120, 10);//set graph coords
    z = map(ReadMagnetometerZ(), -350, 350, 120, 10);//set graph coords
  } 
  else if (type == 4) {
    int lux = map(ReadLux(), 0, 1000, 120, 20);
    x = lux;//set graph coords
    y = lux;
    z = lux;
  } 
  else if (type == 5) {
    int temp = map(ReadTemperature(), -50, 50, 120, 20);//set graph coords
    x = temp;
    y = temp;
    z = temp;
  } 
  else if (type == 6) {
    int press = map(ReadPressure(), 1000, 1100, 120, 20);//set graph coords
    x = press;
    y = press;
    z = press;
  } 
  else if (type == 7) {
    int alt = map(ReadAltitude(), -200, 200, 120, 20);//set graph coords
    x = alt;
    y = alt;
    z = alt;
  } 
  else if (type == 8) {
    int humid = map(ReadHumidity(), 0, 100, 120, 20);
    x = 60;
    y = 0;
    z = humid;
  }
  
  LogVariable(type,x,y,z);

  if (!menuInitialized[type]) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(10, 5);
    tft.setTextColor(ST77XX_CYAN);
    tft.setTextSize(1);
    switch (type) {
      case 1: tft.print("Accelerometer Graph"); break;
      case 2: tft.print("Gyroscope Graph"); break;
      case 3: tft.print("Magnetometer Graph"); break;
      case 4: tft.print("Light Graph"); break;
      case 5: tft.print("Temperature Graph"); break;
      case 6: tft.print("Pressure Graph"); break;
      case 7: tft.print("Altitude Graph"); break;
      case 8: tft.print("Humidity Graph"); break;
    }
    menuInitialized[type] = true;
  }
  //print labels 
  if (graphX >= 160) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(10, 5);
    tft.setTextColor(ST77XX_CYAN);
    tft.setTextSize(1);
    switch (type) {
      case 1: tft.print("Accelerometer Graph"); break;
      case 2: tft.print("Gyroscope Graph"); break;
      case 3: tft.print("Magnetometer Graph"); break;
      case 4: tft.print("Light Graph"); break;
      case 5: tft.print("Temperature Graph"); break;
      case 6: tft.print("Pressure Graph"); break;
      case 7: tft.print("Altitude Graph"); break;
      case 8: tft.print("Humidity Graph"); break;
    }
    graphX = 0;
  }

  tft.drawPixel(graphX, x, ST77XX_RED);
  tft.drawPixel(graphX, y, ST77XX_GREEN);
  tft.drawPixel(graphX, z, ST77XX_BLUE);
  graphX++;

  if (ButtonPressed(4)) {
    currentMenu = 0;
    resetMenus();
    tft.fillScreen(ST77XX_BLACK);
  }
}  
void checkMainMenuButtons() {
  if (ButtonPressed(2)) {
    selectedMenu--; //goes down a menu
    if (selectedMenu < 1) selectedMenu = 8; //loops back around
    mainMenuInitialized = false; //forced redraws
  } else if (ButtonPressed(1)) {
    selectedMenu++;
    if (selectedMenu > 8) selectedMenu = 1;
    mainMenuInitialized = false;
  }
  else if (ButtonPressed(3)) {
    currentMenu = selectedMenu;
    resetMenus();
    tft.fillScreen(ST77XX_BLACK);
  }
}


void showMainMenu(){
  if (!mainMenuInitialized) {
    //Only blacks out the text making it faster than fill
    tft.setTextSize(1);
    tft.setCursor(3, 30);
    for (int i = 1; i <= 8; i++) {
      tft.setCursor(3, tft.getCursorY()); // offsets all new lines by 3ppx
      if (i == selectedMenu + 1) {
        tft.setTextColor(ST77XX_WHITE);
        tft.println(MenuText[i]); // cycles through predetermined names of the menus stored in a array
        tft.setCursor(3, tft.getCursorY()+3); // adds padding in between lines
      }
      else if (i == selectedMenu - 1) {
        tft.setTextColor(ST77XX_WHITE);
        tft.println(MenuText[i]); // cycles through predetermined names of the menus stored in a array
        tft.setCursor(3, tft.getCursorY()+3); // adds padding in between lines
      } 
      else {
        tft.setTextColor(ST77XX_WHITE);
        tft.println(""); // cycles through predetermined names of the menus stored in a array
        tft.setCursor(3, tft.getCursorY()+3); // adds padding in between lines
      }
      
    }



    tft.setCursor(10, 10);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(2);
    tft.println("Main Menu");

    tft.setTextSize(1);
    tft.setCursor(3, 30);
    for (int i = 1; i <= 8; i++) {
      tft.setCursor(3, tft.getCursorY()); // offsets all new lines by 3ppx
      if (i == selectedMenu) {
        tft.setTextColor(ST77XX_GREEN);
      } else {
        tft.setTextColor(ST77XX_WHITE);
      }
      tft.println(MenuText[i]); // cycles through predetermined names of the menus stored in a array
      tft.setCursor(3, tft.getCursorY()+3); // adds padding in between lines
    }
    mainMenuInitialized = true;
  }
}

void resetMenus(){
  mainMenuInitialized = false;
  for (int i = 0; i < 8; i++) {
    menuInitialized[i] = false;
  }
}



//SDI -12

void SetupSDI12(){
  //SDI-12
  Serial1.begin(1200, SERIAL_7E1);//SDI-12 UART, configures serial port for 7 data bits, even parity, and 1 stop bit
  pinMode(DIRO, OUTPUT);//DIRO Pin
  digitalWrite(DIRO, HIGH);//lsiten
}


void PollSDI(){
  int byte;
  //Receive SDI-12 over UART and then print to Serial Monitor
  if(Serial1.available()) {
    byte = Serial1.read();        //Reads incoming communication in bytes
    //Serial.println(byte);
    if (byte == 33) {             //If byte is command terminator (!)
      SDI12Receive(command);
      command = "";               //reset command string
    } else {
      if (byte != 0) {            //do not add start bit (0)
      command += char(byte);      //append byte to command string
      }
    }
  }
}

void SDI12Receive(String input) {
  Serial.println("IN : " + input);
  if((String(input.charAt(0)) == "?")){      //Determines if the command is addressed for this device
    SDI12Send("");
  }                                 //address query command
  if (String(input.charAt(0)) == DeviceAddress) {  
    if((String(input.charAt(1)) == "A")){   // change address command
      DeviceAddress = String(input.charAt(2)); 
      SDI12Send("");
    }
    if((String(input.charAt(1)) == "M")){  // start measurement command
    SDI12Send("0003"); // 000 seconds for data collection. 3 Parameters.
    StartMeasurement();
    }

    if((String(input.charAt(1)) == "D")){  // send data command
      switch(input.charAt(2)){
        case '0':
          SDI12Send(Amag + "+" + Mmag + "+" + Gmag);
          break;
        case '1':
          SDI12Send(T + "+" + P + "+" + A + "+" + H);
          break;
        case '2':
          SDI12Send(L);
          break;
        default:
          SDI12Send("");
        break;
      }
    }
  }
  if(String(input.charAt(1)) == "I"){   // Send Identification command
    SDI12Send(deviceIdentification);
  }
  
  if((String(input.charAt(1)) == "R")){       // Continuous Measurements 
    switch(input.charAt(2)){
        case '0': 
          continuous_measurements_mcu();
          break;
        case '1':
          continuous_measurements_bme();
          break;
        case '2':
          continuous_measurements_lux();
          break;
        default:
          SDI12Send("");
          break;
      }
    }
}  

void SDI12Send(String message) {
  String address = String(DeviceAddress);
  digitalWrite(DIRO, LOW);
  delay(100);
  Serial.println("OUT: " + address + message + String("\r\n"));
  Serial1.print(address + message + String("\r\n"));
  Serial1.flush();    //wait for print to finish
  Serial1.end();
  Serial1.begin(1200, SERIAL_7E1);
  digitalWrite(DIRO, HIGH);
}

void continuous_measurements_mcu(){
  StartMeasurement();
  SDI12Send(Amag + "+" + Mmag + "+" + Gmag);
}

void continuous_measurements_bme(){
  StartMeasurement();
  SDI12Send(T + "+" + P + "+" + A + "+" + H);
}

void continuous_measurements_lux(){
  StartMeasurement();
  SDI12Send(L);
}

void StartMeasurement(){
  float Ax = ReadAccelerationX();
  float Ay = ReadAccelerationY();
  float Az = ReadAccelerationZ();
  Amag = String(sqrt(Ax*Ax+Ay*Ay+Az*Az));
  
  float Gx = ReadGyrometerX();
  float Gy = ReadGyrometerY();
  float Gz = ReadGyrometerZ();
  Gmag =String(sqrt(Gx*Gx+Gy*Gy+Gz*Gz));

  float Mx = ReadMagnetometerX();
  float My = ReadMagnetometerY();
  float Mz = ReadMagnetometerZ();
  Mmag =String(sqrt(Mx*Mx+My*My+Mz*Mz));

  T = String(ReadTemperature());
  P = String(ReadPressure());
  H = String(ReadHumidity());
  A = String(ReadAltitude());

  L = String(ReadLux());
}
