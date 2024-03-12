#include "DFRobot_AHT20.h"
DFRobot_AHT20 aht20;
// AUthor: Dan Pancamo
// Features
//      1. Hall Sensor
//      2. AHT20 Sensor

//includes
#include <WiFi.h>
#include <WiFiUDP.h>
#include <map>
#include <algorithm>
#include <string.h>
#include <Wire.h>



String Version = "WindMan DFrobot Firebeetle 2 ESP32-E Gravity IO Shield";                       // Version 
String BoardId = "windman.ktxcypress-100";         
const uint64_t sleepTime = 120e6; // 5 minutes in microseconds

uint64_t lastLogTime=millis();





//DFRobot HALL sensor SEN0185
#define hallSensorPin D2 // Digital pin connected to the hall sensor
#define sampleTime 1000 // Sample time in milliseconds (1 second)



volatile int rotations = 0; // Counter for rotations (volatile for interrupt safety)
unsigned long lastTime = 0; // Milliseconds since last measurement


const int batteryPin = A2; // Analog pin connected to the battery voltage

//MY WIFI

const char* ssid     = "cam24";
const char* password = "olivia15";

//UDP
int port = 8089;
const char *influxDNS = "bi.pancamo.com";
IPAddress influxIP;
WiFiUDP udp;


//OTA
#include <ElegantOTA.h> 
#include <WiFiClient.h>
#include <WebServer.h>
WebServer webserver(80);

//WIFIMAN
uint8_t DisconnectReason=0;
unsigned long wifiUptime = millis();
unsigned long wifiDowntime = millis();

String line;



//WIFIMAN
void wifiSetup()
{

  // WIFI RECONNECT
  WiFi.disconnect(true);
  wifiUptime=millis();
  wifiDowntime=millis();
  
  delay(2000);

  WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);


  WiFi.begin(ssid, password);
    
  Serial.println();
  Serial.println();
  Serial.println("Wait for WiFi... ");
  
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  DisconnectReason = info.wifi_sta_disconnected.reason;

  wifiDowntime=millis();

  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("Trying to Reconnect");
  WiFi.begin(ssid, password);
}

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("WiFi connected OTA");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Version: " + Version);

  if (WiFi.hostByName(influxDNS, influxIP)) {
      Serial.print("Influx IP: ");
      Serial.println(influxIP);
    } else {
      Serial.println("DNS lookup failed for " + String(influxDNS));
    }
  
  wifiDowntime=millis();
  
  // reason 0 on 1st status is ok
  line = String(BoardId + ".wifi.disreason value=" + String(DisconnectReason));
  toInflux(line);  
    
  
  //line = String(BoardId + ".wifi.localip." + dot2dash(String(WiFi.localIP()))  + " value=" + String(DisconnectReason));
  //toInflux(line);

}

void logWifiStatus()
{
  line = String(BoardId + ".wifi.rssi value=" + String(WiFi.RSSI()));
  toInflux(line);

  wifiUptime=millis()-wifiDowntime; 
  line = String(BoardId + ".wifi.uptime value=" + String(wifiUptime/1000));
  toInflux(line);

}

//OTA 
void handle_OnConnect() 
{
  webserver.send(200, "text/plain", "Hello from " + Version + " " + BoardId);
}

void otaSetup()
{
  webserver.on("/", handle_OnConnect);
  ElegantOTA.begin(&webserver);    // Start ElegantOTA
  webserver.begin();
}




//UDP
void toInflux (String line)
{

      Serial.println(line);

      udp.beginPacket(influxIP, port);
      udp.print(line);
      udp.endPacket();

}












void countRotation() {
  rotations++;
  //Serial.print("windSpeed sample" );
}

void logWind() {
  if (millis() - lastTime >= sampleTime) {
    lastTime = millis();
    
    // Calculate revolutions per second (RPS)
    float rps = (float)rotations / sampleTime * 1000; // Convert milliseconds to seconds

    // Define factors based on your anemometer design
    //  * Circumference of the anemometer cups' rotation path (cm)
    //  * Number of magnet rotations per anemometer cup rotation
    const float circumference = 125; // Replace with your anemometer's circumference
    const int magnetRotationsPerCupRotation = 1; // Replace with your sensor setup

    // Calculate linear velocity (m/s)
    float linearVelocity = rps * circumference * magnetRotationsPerCupRotation / 100; // Convert cm to meters

    // Convert linear velocity to km/h for better readability
    float windSpeed = linearVelocity * 3.6; 

    // **Instead of printing here, store the value for later access**
    toInflux(BoardId + ".wind.mph value=" + String(windSpeed));
    toInflux(BoardId + ".wind.rps value=" + String(rps));
    toInflux(BoardId + ".wind.linearVelocity value=" + String(linearVelocity));




    rotations = 0; // Reset counter for next sample
  }
}



void sensorSetup() {
  pinMode(hallSensorPin, INPUT);
  pinMode(batteryPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(hallSensorPin), countRotation, FALLING); // Interrupt on falling edge


    uint8_t status;
    while((status = aht20.begin()) != 0)
    {
      Serial.print("AHT20 sensor initialization failed. error status : ");
      Serial.println(status);
      delay(1000);
    }


}

/*
void temperature()
{
  if(aht20.startMeasurementReady(true))
  {
    Serial.println("temp = " + String (aht20.getTemperature_F()));
    Serial.println("RH = " + String (aht20.getHumidity_RH()));
  }
}
*/


float logBatteryLevel() {
  // Read the battery voltage (assuming a 3.7V LiPo battery)
  float voltage = analogRead(batteryPin) * (3.3 / 4095.0);  

  Serial.println("Voltage=" + String(voltage));


  // Map the voltage to a percentage (replace with your specific battery curve if known)
  float percentage = map(voltage, 3.3, 0, 100, 0);

  // Constrain the percentage to be between 0 and 100
  percentage = constrain(percentage, 0, 100);


  toInflux(BoardId + ".battery.level value=" + String(percentage));
  toInflux(BoardId + ".battery.voltage value=" + String(voltage));



  return percentage;
}







void logTemperature()
{


  if(aht20.startMeasurementReady(true))
  {

      //AHT20
      float temperature = aht20.getTemperature_F();
      float humidity = aht20.getHumidity_RH();

      //Serial.printf("Temp = %2.2f\n", temperature);   

      line = String(BoardId + ".aht20.temperature value=" + String(temperature));
      toInflux(line);
      line = String(BoardId + ".aht20.humidity value=" + String(humidity));
      toInflux(line);
  }

}



void setup() 
{

    Serial.begin(9600);


    //WIFI SETUP  
    wifiSetup();

    //OTA Setup
    //otaSetup();

    sensorSetup();
}

void loop() {


  //webserver.handleClient();


  // No rotation counting happens here, handled by the interrupt
  // You can call displayWindSpeed() here or from other parts of your code
  // if you want to print the wind speed at specific points


logTemperature();
logBatteryLevel();
logWind();





delay(1000);


}
