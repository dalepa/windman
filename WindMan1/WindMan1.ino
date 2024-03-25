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
String BoardId = "windman.ktxcypress-100";         
const uint64_t sleepTime = 120e6; // 5 minutes in microseconds

uint64_t lastLogTime=millis();



// REED SWITCH
const int switchPin = D6; // Replace with the actual switch pin on the IO Shield
const int debounceTime = 50; // Debounce time in milliseconds (adjust as needed)

volatile int stateChangeCount = 0; // Volatile for interrupt safety
volatile int lastSwitchState = HIGH; // Initial state assumed HIGH (adjust if needed)
volatile unsigned long lastStateChangeTime = 0; // Last time state change occurred (milliseconds)
unsigned long lastRotationTime = 0;




//DFRobot HALL sensor SEN0185
#define hallSensorPin D2 // Digital pin connected to the hall sensor
#define PI 3.14159  // Define pi constant



volatile int rotations = 0; // Counter for rotations (volatile for interrupt safety)
unsigned long lastTime = millis(); // Milliseconds since last measurement

unsigned long sampleTime = 1000;   // Sample time in milliseconds (1 second)

const int batteryPin = A2; // Analog pin connected to the battery voltage

//MY WIFI

const char* ssid     = "cam24";
const char* password = "olivia15";

//const char* ssid     = "YoDaddy";
//const char* password = "abcd1234";

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


float getWindSpeed() {
  
  int  elapsed = millis() - lastTime;

  //if (elapsed >= sampleTime) {


   // Serial.println ("rotations=" + String(rotations));
    //Serial.println ("elapsed=" + String(elapsed/1000));
    Serial.println ("rotation/sec=" + String((float) rotations/(elapsed/1000)));
  /*

    Serial.println ("sampleTime=" + String(sampleTime));
    Serial.println ("lastTime=" + String(lastTime));
    Serial.println ("elapsed=" + String(elapsed));
    Serial.println ("millis() - lastTime=" + String(millis() - lastTime));
*/


    float windSpeed = calculateWindSpeedMph(275, rotations, elapsed/1000);


    // **Instead of printing here, store the value for later access**
  //  toInflux(BoardId + ".wind.mph value=" + String(windSpeed));
  //  toInflux(BoardId + ".wind.rps value=" + String(rotations/(elapsed/1000)));
  //   toInflux(BoardId + ".wind.linearVelocity value=" + String(windSpeed));

    rotations = 0; // Reset counter for next sample
    lastTime = millis();

    return windSpeed;

 // }

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

void logWind()
{
      line = String(BoardId + ".as5600.getAngularSpeed value=" + String(as5600.getAngularSpeed(AS5600_MODE_RPM)));
      toInflux(line);

      line = String(BoardId + ".as5600.degrees value=" + String(as5600.rawAngle() * AS5600_RAW_TO_DEGREES));
      toInflux(line);

      float windspeed=getWindSpeed();

      line = String(BoardId + ".wind wind_direction=" + String(as5600.rawAngle() * AS5600_RAW_TO_DEGREES) + ",wind_speed=" + String(windspeed));
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




void reedSwitchISR() {
  unsigned long currentMillis = millis();

  // Debounce: Ignore interrupts within debounceTime of the last change
  if (currentMillis - lastStateChangeTime >= debounceTime) {
    int newSwitchState = digitalRead(switchPin);
    if (newSwitchState != lastSwitchState) {
      // Only count transition from HIGH to LOW (magnet passing the reed switch)
      if (lastSwitchState == HIGH && newSwitchState == LOW) {
        stateChangeCount++;
      }
      lastSwitchState = newSwitchState;
    }
    lastStateChangeTime = currentMillis;
  }
}



float rotationsPerSecond()
{
    int elapsed = millis() - lastRotationTime;

    float  rotationPerInterval = (float) stateChangeCount/(float)elapsed * 1000.0;

    stateChangeCount = 0;
    lastRotationTime = millis();

    return (rotationPerInterval);

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
    //setupAS5600();

    //SETUP REED SWITCH
    pinMode(switchPin, INPUT_PULLUP); // Set the switch pin as input with internal pullup
    attachInterrupt(digitalPinToInterrupt(switchPin), reedSwitchISR, CHANGE); // Attach interrupt on change

}




void loop() {


  //webserver.handleClient();


  // No rotation counting happens here, handled by the interrupt
  // You can call displayWindSpeed() here or from other parts of your code
  // if you want to print the wind speed at specific points


  int  elapsed = millis() - lastLogTime;

  if (elapsed > 1000){
    //logTemperature();
    //logBatteryLevel();
    //logWind();


    float rps = rotationsPerSecond();
    Serial.println("rps=" + String (rps));

    String line = String(BoardId + ".wind.rps value=" + rps);
    toInflux(line);



    lastLogTime = millis();
    
  }




}
