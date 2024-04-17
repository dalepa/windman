// Dan Pancamo V5
// 
// Latest:  Adding NVS to store highest and lowest batterylevel after boot



//LED
#include <FastLED.h>
#define NUM_LEDS 1     //Number of RGB LED beads
#define DATA_PIN D8    //The pin for controlling RGB LED
#define LED_TYPE NEOPIXEL    //RGB LED strip type
CRGB leds[NUM_LEDS];    //Instantiate RGB LED



#include <esp_err.h>
#include <nvs_flash.h>

#include "DFRobot_AHT20.h"
DFRobot_AHT20 aht20;


#include "Arduino.h"

//BME680
#include "bme68xLibrary.h"

#ifndef PIN_CS
#define PIN_CS 15
#endif

#ifndef ADD_I2C
#define ADD_I2C 0x77
#endif

Bme68x bme;


//LTR390
#include <Wire.h>
#include <LTR390.h>

#define LTR_I2C_ADDRESS 0x53
LTR390 ltr390(LTR_I2C_ADDRESS);





//ICP10111 Air Pressure
#include <DFRobot_ICP10111.h>
DFRobot_ICP10111 icp;
int ICPstatus=0;


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
      
const uint64_t sleepTime = 120e6; // 5 minutes in microseconds

uint64_t lastLoopTime=millis();


//BATTERY CALC  SCALE over time.
float highestVoltage=0.0;
float lowestVoltage=100.0;

#define PI 3.14159



//CODE
String Version = "WindMan DFrobot Firebeetle 2 ESP32-E Gravity IO Shield LTR380-BME680-AS5600 2024-04-17 v3";                       // Version 
String BoardId = "windman.ktxcypress-300";   









// REED SWITCH
int switchPin=D6;
int switchState;
float revs = 0.0;
unsigned long  reedLastTime = 0;
int lastSwitchState = HIGH;
//int elapsed=0;
float rps = 0.0;

//Bucket
int bucketSwitchPin=D5;
float tips = 0.0;
float tipsLastMinute=0;
float tipsLastHour=0; 
float tipsLast24Hours=0;

unsigned long  bucketLastTipTime = 0;
const int debounceTime = 500; // Debounce time in milliseconds (adjust as needed)

int tipsLastMinuteTimer =0;
int tipsLastLastHourTimer =0;
int tipsLastLast24HourTimer =0;





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
  Serial.print ("IP address: ");
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

 
  String ipAddressString = WiFi.localIP().toString();

    
 
  //line = String(BoardId + ".wifi.localip, tag1=value1, ip_address=") + String(ipAddressString) + String(" ") + String(" value=10");

  // line = String("testip,tag1=value1 ip_address=123 value=20") ;
  // toInflux(line);

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
  ElegantOTA. setAutoReboot(true);
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



//NVS FLASH
// Function to write a float value to NVS flash
esp_err_t writeNVSFloat(const char* var_name, float value) {
  nvs_handle_t my_handle;
  esp_err_t err;

  // Open NVS handle
  err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    return err;
  }

  // Write float to NVS
  err = nvs_set_blob(my_handle, var_name, &value, sizeof(value));

  // Commit updates and close handle
  if (err == ESP_OK) {
    err = nvs_commit(my_handle);
  }
  nvs_close(my_handle);

  return err;
}

// Function to read a float value from NVS flash
float readNVSFloat(const char* var_name) {
  nvs_handle_t my_handle;
  esp_err_t err;
  float value = -1.0f;  // Default value in case of error

  // Open NVS handle
  err = nvs_open("storage", NVS_READONLY, &my_handle);
  if (err != ESP_OK) {
    return value;  // Return default value on error
  }

  // Read float from NVS
  size_t data_size = sizeof(value);
  err = nvs_get_blob(my_handle, var_name, &value, &data_size);

  // Close handle
  nvs_close(my_handle);

  // Check for errors or data size mismatch
  if (err != ESP_OK || data_size != sizeof(value)) {
    return value;  // Return default value on error
  }

  return value;
}


void setupNVS() {
  

  // Initialize NVS flash
  int err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
    // If no free pages, perform NVS erase (optional)
    Serial.println("NVS flash full, erasing...");
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);  // Check for other errors

  Serial.println("ESP32 NVS flash ready");

  // Read initial battery level from NVS (optional)
  highestVoltage = readNVSFloat("highestVoltage");
  if (highestVoltage != -1.0f) {
    Serial.println("highestVoltage Saved battery level: " + String(highestVoltage));
  } else {
    Serial.println("No highestVoltage battery level stored in NVS");
  }

  lowestVoltage = readNVSFloat("lowestVoltage");
  if (lowestVoltage != -1.0f) {
    Serial.print("lowestVoltage Saved battery level: "+ String(lowestVoltage));
  } else {
    Serial.println("No lowestVoltage battery level stored in NVS");
  }


}






void countRotation() {
  rotations++;
  
}

void logICPressure(){

    if (ICPstatus == 1) {
      line = String(BoardId + ".icp.temperature value=" + String((icp.getTemperature() * 9/5) + 32));
      toInflux(line);

      line = String(BoardId + ".icp.airpressure value=" + String(icp.getAirPressure()/100));
      toInflux(line);

      line = String(BoardId + ".icp.altitude value=" + String(icp.getElevation() * 3.28084));
      toInflux(line);
    }


      /*
          Serial.println("------------------------------");
        Serial.print("Read air pressure:");
        Serial.print(icp.getAirPressure()/100);
        Serial.println("mb");
        Serial.print("Read temperature:");
        Serial.print((icp.getTemperature() * 9/5) + 32);
        Serial.println("F");
        Serial.print("Read altitude:");
        Serial.print(icp.getElevation() * 3.28084);
        Serial.println("ft");
      */

}


void setupICP(){

    int failCnt=0;

    while(icp.begin() != 0){
      
      Serial.println("setupICP Failed to initialize the ICP pressure sensor.  Retrying...");
      delay(500);
      failCnt++;
      if (failCnt > 3) { break;}

      }

    if (icp.begin() != 0) {
          Serial.println("setupICP FAILED to initialize the pressure sensor");
          ICPstatus = 0;
    }
    else{
      ICPstatus = 1;
      Serial.println("setupICP SUCCESS to initialize the pressure sensor");
    }

    delay(500);
    
     /**
      * @brief Set work mode
      * |------------------|-----------|-------------------|----------------------|
      * |       api        |   mode    |Conversion Time(ms)|Pressure RMS Noise(Pa)|
      * |icp.eLowPower     |  Low Power   |      1.8          |        3.2           |
      * |icp.eNormal       |  Normal      |      6.3          |        1.6           |
      * |icp.eLowNoise     |  Low Noise   |      23.8         |        0.8           |
      * |icp.eUltraLowNoise|  Ultra-low Noise |      94.5         |        0.4           |
      */
     icp.setWorkPattern(icp.eNormal);

}




void setupAHT20(){

    uint8_t status;
    int failCnt=0;
    while((status = aht20.begin()) != 0)
    {
      Serial.print("AHT20 sensor initialization failed. error status : ");
      Serial.println(status);
      delay(500);
      failCnt++;
      if (failCnt > 3) {break;}
    }


}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  // 
  //  (X - 1.5) * (100-0) / (1.9-1.5) + 0 
}



float logBatteryLevel() {
  // Read the battery voltage (assuming a 3.7V LiPo battery)
  float voltage = analogRead(batteryPin) * (3.26 / 4095.0);  

  //Serial.println("Voltage=" + String(voltage));


  //UPDATE NVS with highest and lowest measured voltages

  if (voltage > highestVoltage) {
    highestVoltage=voltage;
    writeNVSFloat("highestVoltage", highestVoltage);
    }

  if (voltage < lowestVoltage && voltage > 0) {
    lowestVoltage=voltage;
    writeNVSFloat("lowestVoltage", lowestVoltage);
    }

  // Map the voltage to a percentage (replace with your specific battery curve if known)
  float percentage = mapFloat(voltage, lowestVoltage, highestVoltage, 0, 100);

  // Constrain the percentage to be between 0 and 100
  // percentage = constrain(percentage, 0, 100);


  toInflux(BoardId + ".battery.level value=" + String(percentage));
  toInflux(BoardId + ".battery.voltage value=" + String(voltage));
  toInflux(BoardId + ".battery.highestVoltage value=" + String(highestVoltage));
  toInflux(BoardId + ".battery.lowestVoltage value=" + String(lowestVoltage));



  return percentage;
}



void setupAS5600()
{
  Wire.begin();

  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  int b = as5600.isConnected();
  Serial.print("AS5600 Connect Code: ");
  Serial.println(b);

  Serial.print("Current AS5600 ADDR: ");
  Serial.println(as5600.getAddress());

  as5600.setAddress(0x36);

  Serial.print("New AS5600 ADDR: ");
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
  
  int sampletime = 1000;    // Sample each X milliseconds

  int elapsed = millis() - reedLastTime;

  if (elapsed > sampletime) {
    rps = (float) revs/((float)elapsed/1000.0);
    reedLastTime = millis();
    revs=0;
  }
  return (rps);

}


void tipping() {
    if (millis() - bucketLastTipTime >= debounceTime) {
      tips++;
      bucketLastTipTime = millis();
      
      tipsLastMinute++;
      tipsLastHour++; 
      tipsLast24Hours++;
      //Serial.println("TIP");
  }
}




float bucketTips(){
  return(tips);
}





float calculateRainfall(float diameter_mm, float tips, float tip_volume_ml) {
  // Convert diameter to cm
  float diameter_cm = diameter_mm * (1.0 / 10.0);

  // Calculate collector area in cm²
  float collector_area = PI * pow(diameter_cm / 2.0, 2);

  // Convert tip volume from ml to cm³ (corrected line)
  float tip_volume_cm3 = tip_volume_ml * (1.0 / 1.0);

  // Calculate total rainfall volume collected (cm³)
  float rainfall_volume = tips * tip_volume_cm3;

  // Convert rainfall volume from cm³ to mm (millimeters of rain)
  float rainfall_mm = rainfall_volume / collector_area * 10;

  return rainfall_mm / 25.4;
}


void setupBME680()
{
  Wire.begin();     //I2C mode
	//SPI.begin();    //SPI mode
	
	while (!Serial)
		delay(10);
		
	/* initializes the sensor based on SPI library */
	//bme.begin(PIN_CS, SPI);     //SPI mode
  bme.begin(ADD_I2C, Wire);     //I2C mode

	if(bme.checkStatus())
	{
		if (bme.checkStatus() == BME68X_ERROR)
		{
			Serial.println("Sensor error:" + bme.statusString());
			return;
		}
		else if (bme.checkStatus() == BME68X_WARNING)
		{
			Serial.println("Sensor Warning:" + bme.statusString());
		}
	}
	
	/* Set the default configuration for temperature, pressure and humidity */
	bme.setTPH();

	/* Set the heater configuration to 300 deg C for 100ms for Forced mode */
	bme.setHeaterProf(300, 100);

	//Serial.println("TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%), Gas resistance(ohm), Status");
}

void logBME680(void)
{
	bme68xData data;

	bme.setOpMode(BME68X_FORCED_MODE);
	//delay(500+bme.getMeasDur()/200);

	if (bme.fetchData())
	{
		bme.getData(data);

      line = String(BoardId + ".bme680.temperature value=" + String((data.temperature * 9/5) + 32));
      toInflux(line);
      line = String(BoardId + ".bme680.humidity value=" + String(data.humidity));
      toInflux(line);
      line = String(BoardId + ".bme680.pressure value=" + String(data.pressure));
      toInflux(line);
      line = String(BoardId + ".bme680.gas value=" + String(data.gas_resistance));
      toInflux(line);
      


/*
		Serial.print(String(millis()) + ", ");
		Serial.print(String(data.temperature * 9/5 + 32) + ", ");
		Serial.print(String(data.pressure) + ", ");
		Serial.print(String(data.humidity) + ", ");
		Serial.print(String(data.gas_resistance) + ", ");
		Serial.println(data.status, HEX);
*/	
  
  }
}

void setupLTR390() {

  Wire.begin();
  if(!ltr390.init()){
    Serial.println("LTR390 not connected!");
  }

  ltr390.setMode(LTR390_MODE_ALS);

  ltr390.setGain(LTR390_GAIN_3);
  Serial.print("LTR390 Gain : ");
  switch (ltr390.getGain()) {
    case LTR390_GAIN_1: Serial.println(1); break;
    case LTR390_GAIN_3: Serial.println(3); break;
    case LTR390_GAIN_6: Serial.println(6); break;
    case LTR390_GAIN_9: Serial.println(9); break;
    case LTR390_GAIN_18: Serial.println(18); break;
  }
  
  ltr390.setResolution(LTR390_RESOLUTION_18BIT);
  Serial.print("LTR390 Resolution : ");
  switch (ltr390.getResolution()) {
    case LTR390_RESOLUTION_13BIT: Serial.println(13); break;
    case LTR390_RESOLUTION_16BIT: Serial.println(16); break;
    case LTR390_RESOLUTION_17BIT: Serial.println(17); break;
    case LTR390_RESOLUTION_18BIT: Serial.println(18); break;
    case LTR390_RESOLUTION_19BIT: Serial.println(19); break;
    case LTR390_RESOLUTION_20BIT: Serial.println(20); break;
  }

  //ltr390.setThresholds(100, 1000);
  //ltr390.configInterrupt(true, LTR390_MODE_UVS);

}


void logLTR390() {
  if (ltr390.newDataAvailable()) {
      if (ltr390.getMode() == LTR390_MODE_ALS) {
        line = String(BoardId + ".ltr380.lux value=" + String(ltr390.getLux()) );
        toInflux(line);


        //Serial.print("Ambient Light Lux: "); 
        // Serial.println(ltr390.getLux());
         ltr390.setGain(LTR390_GAIN_18);                  //Recommended for UVI - x18
         ltr390.setResolution(LTR390_RESOLUTION_20BIT);   //Recommended for UVI - 20-bit
         ltr390.setMode(LTR390_MODE_UVS); 

      } else if (ltr390.getMode() == LTR390_MODE_UVS) {

        line = String(BoardId + ".ltr380.uvi value=" + String(ltr390.getUVI()) );
        toInflux(line);

         //Serial.print("UV Index: "); 
         //Serial.println(ltr390.getUVI());
         
         ltr390.setGain(LTR390_GAIN_3);                   //Recommended for Lux - x3
         ltr390.setResolution(LTR390_RESOLUTION_18BIT);   //Recommended for Lux - 18-bit
         ltr390.setMode(LTR390_MODE_ALS);
      }
  }
}



void setup() 
{

    Serial.begin(115200);

    FastLED.addLeds<LED_TYPE, DATA_PIN>(leds, NUM_LEDS);     

    pinMode(batteryPin, INPUT);   // READ BATTERY 

    setupNVS();

    //voltageReset();  //reset when needed.


    //WIFI SETUP  
    wifiSetup();

    //Setup BatteryPin
    pinMode(batteryPin, INPUT);


    //OTA Setup
    otaSetup();


    setupBME680();  // BME680  temp-hum-press-air

    setupLTR390();  //LTR390 light and UV

    //setupAHT20();  //setup TEMP sensor

    //setup Pressure Sensor
    //setupICP();

    //AS5600 Setup Magnet Sensor
    setupAS5600();

    //SETUP REED CUPS SWITCH
    pinMode(switchPin, INPUT_PULLUP); // Set the switch pin as input with internal pullup
    
    //SETUP RAIN SWITCH
    pinMode(bucketSwitchPin, INPUT_PULLUP); // Set the switch pin as input with internal pullup
    attachInterrupt(digitalPinToInterrupt(bucketSwitchPin), tipping, FALLING); // Attach interrupt on falling edge



  //works line = String("cpu_temp2,location=server_room,sensor_type=102.168.0.1 value=75") ;
  
  line = String(BoardId + ".wifi.ip" + ",ipaddress=" + WiFi.localIP().toString() + " value=75") ;

  toInflux(line);

}



void logRain(int tips, int elapsed) {

    int diameter = 160;
    float bucketsize=3.3;
    float rainfall;

    tipsLastMinuteTimer += elapsed;
    tipsLastLastHourTimer += elapsed;
    tipsLastLast24HourTimer += elapsed;

    if (tipsLastMinuteTimer > 60){
      tipsLastMinute = 0;
      tipsLastMinuteTimer = 0;
    }
    if (tipsLastLastHourTimer > 60*60){
      tipsLastHour = 0;
      tipsLastLastHourTimer = 0;
    }
    if (tipsLastLast24HourTimer > 60*60*24){
      tipsLast24Hours = 0;
      tipsLastLast24HourTimer=0;
    }


    //Serial.printf("elapsed = %d\n", elapsed);   

   

    rainfall = calculateRainfall(diameter,tipsLastMinute,bucketsize);
    line = String(BoardId + ".rain.lastminute value=" + String(rainfall));
    toInflux(line);

    rainfall = calculateRainfall(diameter,tipsLastHour,bucketsize);
    line = String(BoardId + ".rain.lasthour value=" + String(rainfall));
    toInflux(line);

    rainfall = calculateRainfall(diameter,tipsLast24Hours,bucketsize);
    line = String(BoardId + ".rain.last24hours value=" + String(rainfall));
    toInflux(line);

    line = String(BoardId + ".rain.tips value=" + String(tips));
    toInflux(line);

}

void voltageReset() {
  writeNVSFloat("highestVoltage", 0);
  writeNVSFloat("lowestVoltage", 100);
}

void OTAloop(){

     //OTA
    webserver.handleClient();
    ElegantOTA.loop();
}

void loop() {


  OTAloop();

  int  elapsed = millis() - lastLoopTime;

  float newrps = reedRPS();
  float newtips = bucketTips();

  

  leds[0] = CRGB::Green;     //LED shows red light
  FastLED.show();

  if (elapsed > 3000){

    leds[0] = CRGB::Blue;     // LED shows blue light
    FastLED.show();

    //logTemperature();
    //logICPressure();

    logBME680();
    logLTR390();
    logBatteryLevel();
    logWind(newrps);
    logRain(newtips,elapsed/1000);

    logWifiStatus();

    lastLoopTime = millis();
    
  }

}
