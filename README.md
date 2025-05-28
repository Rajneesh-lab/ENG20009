//Add this at the top (global variables)://
bool alertLogged = false;
unsigned long lastAlertTime = 0;
const unsigned long alertCooldown = 30000; // 30 seconds cooldown


//Then update TimerFlagHandler() like this://
void TimerFlagHandler(){
  if (TimerFlag){
    TimerFlag = false;

    float humidity = ReadHumidity();
    float temperature = ReadTemperature();
    float lux = ReadLux();

    if(humidity < 60){
      OpenStepper();
    } else {
      CloseStepper();
    }

    if ((temperature > 35.0 || lux > 800.0) && (millis() - lastAlertTime > alertCooldown)) {
      LogVariable(4, lux, 0, 0);
      LogVariable(5, temperature, 0, 0);
      LogVariable(6, ReadPressure(), 0, 0);
      LogVariable(8, humidity, 0, 0);
      lastAlertTime = millis();
      Serial.println("** Auto Environmental Alert Logged **");
    }
  }
}
