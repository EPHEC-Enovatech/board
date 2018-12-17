
// Select your preferred method of sending data
#define CONTAINERS

/***************************************************************************/



#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ATT_LoRaWAN.h>
#include "keys.h"
#include <MicrochipLoRaModem.h>

#define SERIAL_BAUD 57600

#define debugSerial Serial
#define loraSerial Serial1

#define LightSensorPin A4
#define AirSensorPin A0
#define MoistureSensorPin A2

#define SEND_EVERY 120000

MicrochipLoRaModem modem(&loraSerial, &debugSerial);
ATTDevice device(&modem, &debugSerial, false, 7000);  // minimum time between 2 messages set at 7000 milliseconds

#ifdef CONTAINERS
  #include <Container.h>
  Container container(device);
#endif

Adafruit_BME280 tph; // I2C
int digitalSensor = 20;  // Digital sensor is connected to pin D20/21
int relay = 4;

float soundValue;
float lightValue;
float airValue;
int moistureValue;
float temp;
float hum;
float pres;

void setup() 
{
  
  pinMode(digitalSensor, INPUT);  // Initialize the digital pin as an input
  pinMode(relay, OUTPUT);
  delay(3000);

  debugSerial.begin(SERIAL_BAUD);
  debugSerial.begin(SERIAL_BAUD);
  while((!debugSerial) && (millis()) < 10000){}  // wait until the serial bus is available

  loraSerial.begin(modem.getDefaultBaudRate());  // set baud rate of the serial connection to match the modem
  while((!loraSerial) && (millis()) < 10000){}   // wait until the serial bus is available

  while(!device.initABP(DEV_ADDR, APPSKEY, NWKSKEY))
  debugSerial.println("Ready to send data");
  
  debugSerial.println();
  debugSerial.println("-- Environmental Sensing LoRa experiment --");
  debugSerial.println();

  initSensors();
}

void loop() 
{
    pinMode(GROVEPWR, OUTPUT);  // turn on the power for the secondary row of grove connectors
    digitalWrite(GROVEPWR, HIGH);
    readSensors();
    digitalWrite(GROVEPWR, LOW);
    //displaySensorValues();
    sendSensorValues();
}

void initSensors()
{
  debugSerial.println("Initializing sensors, this can take a few seconds...");
  pinMode(LightSensorPin, INPUT);

  pinMode(AirSensorPin, INPUT);

  pinMode(MoistureSensorPin, INPUT);

  tph.begin();
  // give some time to the sensors to init
  delay(500);
  debugSerial.println("Done");
}

void readSensors()
{
    debugSerial.println("Start reading sensors");
    debugSerial.println("---------------------");
    
    lightValue = analogRead(LightSensorPin);
    lightValue = lightValue * 3.3 / 1023;  // convert to lux based on the voltage that the sensor receives
    lightValue = pow(10, lightValue);

    //la valeur est a déterminer selon la luminosité solaire.
    if(lightValue < 10){
      digitalWrite(relay, HIGH);
      delay(100);
    }else {
      digitalWrite(relay, LOW);
      delay(100);
    }

    airValue = analogRead(AirSensorPin);

    
    moistureValue = analogRead(MoistureSensorPin);
    
    temp = tph.readTemperature();
    pres = tph.readPressure()/100;
    hum = tph.readHumidity();    
}

void process()
{
  while(device.processQueue() > 0)
  {
    debugSerial.print("QueueCount: ");
    debugSerial.println(device.queueCount());
    delay(10000);
  }
}

void sendSensorValues()
{
  
  #ifdef CONTAINERS
  debugSerial.println("Start sending data to the Proximus EnCo platform");
  debugSerial.println("--------------------------------------------");
  container.addToQueue(lightValue, LIGHT_SENSOR, false); process();
  container.addToQueue(airValue, AIR_QUALITY_SENSOR, false); process();
  container.addToQueue(moistureValue, INTEGER_SENSOR, false); process();
  container.addToQueue(temp, TEMPERATURE_SENSOR, false); process();
  container.addToQueue(pres, PRESSURE_SENSOR, false); process();
  container.addToQueue(hum, HUMIDITY_SENSOR, false); process();
  #endif
}

void displaySensorValues()
{
  debugSerial.print("Light intensity: ");
  debugSerial.print(lightValue);
  debugSerial.println(" Lux");
      
  debugSerial.print("Temperature: ");
  debugSerial.print(temp);
  debugSerial.println(" °C");

  debugSerial.print("Pression: ");
  debugSerial.print(pres);
  debugSerial.println(" hPa");
      
  debugSerial.print("Humidity: ");
  debugSerial.print(hum);
	debugSerial.println(" %");

  debugSerial.print("Air quality: ");
  debugSerial.print(airValue);
  debugSerial.println(" AQI");

  debugSerial.print("Moisture: ");
  debugSerial.print(moistureValue);
  debugSerial.println(" UM");
    
}
