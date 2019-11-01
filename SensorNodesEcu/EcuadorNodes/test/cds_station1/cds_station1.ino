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
const char *appKey = "0E2C062A2324A49D308F637D9CD720C8";

#define SEALEVELPRESSURE_HPA (1013.25)
#define PRESSURE_BASE 40000

//#define TX_INTERVAL 1200000 //ms =20 min
//#define TX_INTERVAL 300000 //ms =5 min
#define TX_INTERVAL 121000 //ms =2 min



//For BME280 sensor
Adafruit_BME280 bme; // I2C
//********** DIGITAL SENSORS PIN *************//
int externalTemp = 7;
int rainGauge = 6;

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


const float a=21.01;
const  float b=-0.1411;

void setup()
{
  loraSerial.begin(57600);
  debugSerial.begin(9600);
  //POWER SENSORS
  pinMode(POWER_PIN,OUTPUT);
  digitalWrite(POWER_PIN,HIGH);

  //pinMode(POWER_PIN12,OUTPUT);
  //digitalWrite(POWER_PIN12,HIGH);
  delay(5000);
  
  // INIT TTN INTERFACE
  while (!debugSerial && millis() < 10000)
    ;
  debugSerial.println("-- STATUS");
  //ttn.wake();
  ttn.showStatus();
  debugSerial.println("-- JOIN");
  ttn.join(appEui, appKey);

  //INIT BME SENSOR
   bool status;
   // default settings
   status = bme.begin();  
    if (!status) {
        debugSerial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
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
  debugSerial.print("VIS (Km): ");
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
 

  byte payload[13];   
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
  payload[12]=lowByte(battery); 
 
    
  ttn.sendBytes(payload,sizeof(payload));
 
  

  //**** SLEEP THE NODE
  //Sleepin LORA module
    ttn.sleep(TX_INTERVAL);
    Serial.print("****** SLEEPING:");
    Serial.print(TX_INTERVAL);
    Serial.println(" mili seconds");
  //Sleeping sensors connected to POWER_PIN
    digitalWrite(POWER_PIN,LOW);
    delay(TX_INTERVAL);

   //****** WAKING UP THE NODE
    //Waking up LoRa module
    ttn.wake();
    digitalWrite(POWER_PIN,HIGH);
    delay(2000);
    //************** Start again BME sensor//
    bool status=bme.begin();
     if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
     }
    delay(1000);
    Serial.println("***** Just wake up! ******** ");
  
}

//Get Internal sensors
float getIntTemperature(){
   return bme.readTemperature(); //*C
}
float getIntHumidity(){
  return bme.readHumidity(); //%
}
float getAirPressure(){
  return (bme.readPressure() / 100.0F); //hPa
}
float getAltitude(){
  return (bme.readAltitude(SEALEVELPRESSURE_HPA)); //m
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
  return (a+b*pow(sensorVal,3)); 
}

float getFogLevel(){
    return 88.88F; //*C
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

  
