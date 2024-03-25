#include "DFRobot_AHT20.h"
DFRobot_AHT20 aht20;
// Dan Pancamo V3

//includes
#include <WiFi.h>
#include <WiFiUDP.h>
#include <map>
#include <algorithm>
#include <string.h>
#include <Wire.h>

//AS5600
#include "AS5600.h"
AS5600L as5600;   //  use default Wire


String Version = "WindMan DFrobot Firebeetle 2 ESP32-E Gravity IO Shield";                       // Version 
String BoardId = "windman.ktxcypress-200";         
const uint64_t sleepTime = 120e6; // 5 minutes in microseconds

uint64_t lastLoopTime=millis();



// REED SWITCH
int switchPin=D6;
int switchState;
float revs = 0.0;
unsigned long  reedLastTime = 0;
int lastSwitchState = HIGH;
int elapsed=0;
float rps = 0.0;


//DFRobot HALL sensor SEN0185
#define hallSensorPin D2 // Digital pin connected to the hall sensor
#define PI 3.14159  // Define pi constant



volatile int rotations = 0; // Counter for rotations (volatile for interrupt safety)
unsigned long lastTime = millis(); // Milliseconds since last measurement

unsigned long sampleTime = 1000;   // Sample time in milliseconds (1 second)

const int batteryPin = A2; // Analog pin connected to the battery voltage

//MY WIFI

//const char* ssid     = "cam24";
//const char* password = "olivia15";

const char* ssid     = "YoDaddy";
const char* password = "abcd1234";

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
  
}













void setupAHT20(){

    uint8_t status;
    while((status = aht20.begin()) != 0)
    {
      Serial.print("AHT20 sensor initialization failed. error status : ");
      Serial.println(status);
      delay(1000);
    }


}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



float logBatteryLevel() {
  // Read the battery voltage (assuming a 3.7V LiPo battery)
  float voltage = analogRead(batteryPin) * (3.3 / 4095.0);  

  Serial.println("Voltage=" + String(voltage));


  // Map the voltage to a percentage (replace with your specific battery curve if known)
  float percentage = mapFloat(voltage, 1.92, 1.1, 100, 0);

  // Constrain the percentage to be between 0 and 100
 // percentage = constrain(percentage, 0, 100);


  toInflux(BoardId + ".battery.level value=" + String(percentage));
  toInflux(BoardId + ".battery.voltage value=" + String(voltage));



  return percentage;
}



void setupAS5600()
{
  Wire.begin();

  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  int b = as5600.isConnected();
  Serial.print("AS5600 Connect: ");
  Serial.println(b);

  Serial.print("ADDR: ");
  Serial.println(as5600.getAddress());

  as5600.setAddress(0x36);

  Serial.print("ADDR: ");
  Serial.println(as5600.getAddress());

}



#define PI 3.14159  // Define pi for circumference calculation

float calculateWindSpeedMph(int circleDiameterMM, int rotations, float timeInterval) {
  // Convert mm to meters
  float circleDiameterM = circleDiameterMM / 1000.0;

  // Distance traveled per cup per rotation depends on circle diameter
  float circumference = PI * circleDiameterM;
  float distancePerCup = circumference;

  // Total distance traveled is calculated based on a single cup (assuming all cups move together)
  float totalDistance = distancePerCup * rotations;

  // Calculate wind speed in meters per second
  float windSpeedMps = totalDistance / timeInterval;

  // Convert wind speed to miles per hour
  float windSpeedMph = windSpeedMps * 2.23694;

  return windSpeedMph;
}



void logWind(float rps)
{
      line = String(BoardId + ".as5600.getAngularSpeed value=" + String(as5600.getAngularSpeed(AS5600_MODE_RPM)));
      toInflux(line);

      line = String(BoardId + ".as5600.degrees value=" + String(as5600.rawAngle() * AS5600_RAW_TO_DEGREES));
      toInflux(line);

      float windspeed=calculateWindSpeedMph(275,rps,1);

      line = String(BoardId + ".wind wind_direction=" + String(as5600.rawAngle() * AS5600_RAW_TO_DEGREES) + ",wind_speed=" + String(windspeed) + ",rps=" + String(rps));
      toInflux(line);

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



float readRead() {
  
  switchState = digitalRead(switchPin);


  if (switchState == LOW && lastSwitchState == HIGH ) {

    lastSwitchState = switchState;
    revs++;
  }

  lastSwitchState = switchState;
  return (revs);

}

// Count the Reveloutions per Seconds of the Magnet across the reed
float reedRPS(){


  revs = readRead();
  
  int sampletime = 1;    // Sample each X seconds

  int elapsed = millis() - reedLastTime;

  if (elapsed > sampletime) {
    rps = (float) revs/((float)elapsed/1000.0);
    reedLastTime = millis();
    revs=0;
  }
  return (rps);

}







void setup() 
{

    Serial.begin(115200);


    //WIFI SETUP  
    wifiSetup();

    //Setup BatteryPin
    pinMode(batteryPin, INPUT);


    //OTA Setup
    //otaSetup();

    setupAHT20();  //setup TEMP sensor

    //AS5600 Setup Magnet Sensor
    setupAS5600();

    //SETUP REED SWITCH
    pinMode(switchPin, INPUT_PULLUP); // Set the switch pin as input with internal pullup

}




void loop() {


  //webserver.handleClient();


  int  elapsed = millis() - lastLoopTime;

  float newrps = reedRPS();


  if (elapsed > 3000){
    logTemperature();
    logBatteryLevel();
    logWind(5);
    lastLoopTime = millis();

  }

}
