//For cable connections, follow
//https://learn.adafruit.com/adafruit-bme280-humidity-barometric-pressure-temperature-sensor-breakout/arduino-test 

#include <TheThingsNetwork.h>
#define loraSerial Serial1
#define debugSerial Serial
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <OneWire.h> //For external temperature sensor
#include <DallasTemperature.h>
//#include "Adafruit_SI1145.h"

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan TTN_FP_US915
TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);
// Set your AppEUI and AppKey

const char *appEui = "70B3D57ED001884C";
const char *appKey = "BC9A50133F4524CC1AF90A2CB8C7460A";

#define SEALEVELPRESSURE_HPA (1013.25)
#define PRESSURE_BASE 40000

#define VIS_SENSOR_INTERVAL 61000 //Warm up time required by the VIS sensor 
//#define TX_INTERVAL 1800000 //ms =30 min
#define TX_INTERVAL 121000 //ms =2 min


//For BME280 sensor
Adafruit_BME280 bme; // I2C
bool statusBME=false;
//********** DIGITAL SENSORS PIN *************//
int externalTemp = 7;

//********** POWER CONTROLLER PIN *************//
int POWER_PIN = 8; //5 v
int POWER_PIN12=9; //12V

OneWire oneWirePin(externalTemp);
DallasTemperature sensors(&oneWirePin);

//********** ANALOG SENSORS PIN *************//
const int aLightPin=A0;
const int aLiquidLevelPin=A1;
const int aBattery=A2;
const int aVisPin=A3;
const int aWindPin=A4;

const float a=21.01;
const  float b=-0.1411;

void setup()
{
  loraSerial.begin(57600);
  debugSerial.begin(9600);
  //POWER SENSORS
  pinMode(POWER_PIN,OUTPUT);
  pinMode(POWER_PIN12,OUTPUT);
 
  //Warming up VIS sensor
  digitalWrite(POWER_PIN12,HIGH);
  debugSerial.println("Warming up VIS optical sensor");
  delay(VIS_SENSOR_INTERVAL);
  digitalWrite(POWER_PIN,HIGH);
  delay(5000);
  
  // INIT TTN INTERFACE
  while (!debugSerial && millis() < 10000)
    ;
  debugSerial.println("-- STATUS");
  ttn.wake();
  ttn.showStatus();
  //ttn.reset(false);
  debugSerial.println("-- JOIN");
  ttn.join(appEui, appKey);

  //INIT BME SENSOR
  // default settings
   statusBME = bme.begin();  
    if (!statusBME) {
        debugSerial.println("Could not find a valid BME280 sensor, check wiring!");
        
    }
    debugSerial.println("-- Default Test --");
    debugSerial.println();
    sensors.begin();   //INIT UV SENSOR
    delay(1000);
    Serial.println("OK!");
}

void loop()
{
 // uint32_t intTemperature=getIntTemperature()*100;
 // debugSerial.print("Internal temperature: ");
 // debugSerial.println(intTemperature);
  uint32_t intHumidity=getIntHumidity()*100;
  debugSerial.print("Humidity: ");
  debugSerial.println(intHumidity);
  uint32_t intAirPressure=getAirPressure()*100;
  debugSerial.print("Air pressure: ");
  debugSerial.println(intAirPressure); 
  //To be able to send airpresure in 2 bytes
  intAirPressure=intAirPressure-PRESSURE_BASE;  
  uint32_t extTemperature=getExtTemperature()*100;
  debugSerial.print("External Temperature: ");
  debugSerial.println(extTemperature);
  uint32_t liquidLevel=getAnalogLiquidLevel()*100;
  debugSerial.print("liquid level: ");
  debugSerial.println(liquidLevel);
  uint32_t vis=getVis()*100;
  debugSerial.print("Vis (Km): ");
  debugSerial.println(vis);
  
  uint32_t light=getIntLight();
  debugSerial.print("Light: ");
  debugSerial.println(light);  
  //uint32_t uvIndex=getUvIndex();
 // debugSerial.print("UV Index: ");
 // debugSerial.println(uvIndex);
  uint32_t battery=getBattery()*100;
  debugSerial.print("Battery: ");
  debugSerial.println(battery);
  uint32_t windSpeed=getWindSpeed()*100;
  debugSerial.print("Wind Speed: ");
  debugSerial.println(windSpeed);
 

  byte payload[15];   
  //Floats
  payload[0]=highByte(intHumidity);
  payload[1]=lowByte(intHumidity);
  payload[2]=highByte(intAirPressure);
  payload[3]=lowByte(intAirPressure);
  payload[4]=highByte(extTemperature);
  payload[5]=lowByte(extTemperature);
  payload[6]=highByte(liquidLevel);
  payload[7]=lowByte(liquidLevel);
  payload[8]=highByte(vis);
  payload[9]=lowByte(vis);
  //Int analog readings
  payload[10]=light; //from 0 to 255
  payload[11]=highByte(battery); //voltage
  payload[12]=lowByte(battery); //
  payload[13]=highByte(windSpeed); //WindSpeed
  payload[14]=lowByte(windSpeed); //
  
  ttn.sendBytes(payload,sizeof(payload));
 
  

  //******** SLEEP THE NODE ***************//
  //Sleepin LORA module
    ttn.sleep(TX_INTERVAL);
    Serial.print("****** SLEEPING:");
    Serial.print(TX_INTERVAL);
    Serial.println(" mili seconds");
  //Sleeping sensors connected to POWER_PIN
    digitalWrite(POWER_PIN12,LOW);
    digitalWrite(POWER_PIN,LOW);    
    Serial.println("Before delay");
    delay(TX_INTERVAL-VIS_SENSOR_INTERVAL);

   //******** WAKING UP THE NODE ***********//
   //Wake up VIS sensor VIS_SENSOR_INTERVAL before the whole node
    Serial.println("*****Waking up VIS sensor! ******** ");
    digitalWrite(POWER_PIN12,HIGH);
    delay(VIS_SENSOR_INTERVAL);
    //Waking up LoRa module
    ttn.wake();
    digitalWrite(POWER_PIN,HIGH);
    delay(2000);
    //************** Start again BME sensor//
    statusBME=bme.begin();
    if(!statusBME){
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      }
     
    delay(1000);
    Serial.println("***** Just wake up! ******** ");
  
}

//Get Internal sensors
float getIntTemperature(){
   if (statusBME){
    return bme.readTemperature(); //*C
   }else{
    return 0.0; //sensor could not be init
    }
}
float getIntHumidity(){
  if (statusBME){
    return bme.readHumidity(); //%
  }
  else{
    return 0.0;
    }
}
float getAirPressure(){
  if (statusBME){
    return (bme.readPressure() / 100.0F); //hPa
  }
  else
  {
    return 0.0;
    }
}
float getAltitude(){
  if(statusBME){
    return (bme.readAltitude(SEALEVELPRESSURE_HPA)); //m
  }
  else{
    return 0.0;
  }
  
}

//Analog light
int getIntLight(){
  //Measured from the analog pin
  int sensorVal=analogRead(aLightPin);
  int light=map(sensorVal,0,1023,0,255);
  return light;
}
//External sensors
float getExtTemperature(){
    sensors.requestTemperatures(); 
    return sensors.getTempCByIndex(0); //*C
}

//Analog liquid level
float getAnalogLiquidLevel(){
  //Measured from the analog pin
  float sensorVal=analogRead(aLiquidLevelPin);
  sensorVal=sensorVal/100;
  return sensorVal; 
  //return (a+b*pow(sensorVal,3)); 
}

//Get Battery
float getBattery(){
  int sensorVal=analogRead(aBattery);
  float batteryVoltage=sensorVal*(5.0/1023.0);
  return batteryVoltage;
 }
float getVis(){
  int visVal=analogRead(aVisPin);
  float visVal_Voltage=visVal*(5.0/1023.0);
  return visVal_Voltage;
  
  }


float getWindSpeed(){
  float voltageMax=2.0;
  float voltageMin=0.45;
  float windSpeedMin=0;
  float windSpeedMax=32.4;
  int windSensorDelay=500; //mili seconds
  int nWindReadings=20;
  
  float windSpeed=0.0;
  int windSensorValue=0;
  float windSpeedVoltage=0.0;
  float totalWindSpeed=0.0;
  float avrWindSpeed=0.0;
  for(int i=0;i<nWindReadings;i++){
      //0. Init windSpeed to 0
      windSpeed=0.0;
      //1. take value from analog pin
      windSensorValue=analogRead(aWindPin);
      //2convert to speed
      //2.1 Convert to voltage
      windSpeedVoltage =windSensorValue*(5.0/1023);
      //2.2 Convert to speed
      if (windSpeedVoltage > voltageMin){
        windSpeed=((windSpeedVoltage -voltageMin)*windSpeedMax/(voltageMax-voltageMin))*2.232694; 
        
      }
      //Add value to total
      totalWindSpeed=totalWindSpeed+windSpeed;
      delay(windSensorDelay);
    }
    //Calculate average speed
    avrWindSpeed=totalWindSpeed/nWindReadings;
    return avrWindSpeed;
  
}  
 
