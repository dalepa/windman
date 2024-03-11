#include "DFRobot_AHT20.h"
DFRobot_AHT20 aht20;



#define hallSensorPin D2 // Digital pin connected to the hall sensor
#define sampleTime 1000 // Sample time in milliseconds (1 second)

volatile int rotations = 0; // Counter for rotations (volatile for interrupt safety)
unsigned long lastTime = 0; // Milliseconds since last measurement


const int batteryPin = A2; // Analog pin connected to the battery voltage

float getBatteryLevel() {
  // Read the battery voltage (assuming a 3.7V LiPo battery)
  float voltage = analogRead(batteryPin) * (3.3 / 4095.0);  

  Serial.println("Voltage=" + String(voltage));


  // Map the voltage to a percentage (replace with your specific battery curve if known)
  float percentage = map(voltage, 3.3, 0, 100, 0);

  // Constrain the percentage to be between 0 and 100
  percentage = constrain(percentage, 0, 100);

  return percentage;
}

void batterylevel()
{
    float batteryLevel = getBatteryLevel();
  Serial.print("Battery Level: ");
  Serial.print(batteryLevel);
  Serial.println("%");


}



void countRotation() {
  rotations++;
  //Serial.print("windSpeed sample" );
}

void calculateWindSpeed() {
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
    displayWindSpeed(windSpeed);

    rotations = 0; // Reset counter for next sample
  }
}

void displayWindSpeed(float windSpeed) {
  Serial.print("Wind Speed: ");
  Serial.print(windSpeed);
  Serial.println(" km/h");
}

void setup() {
  Serial.begin(9600); // Start serial communication
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

void temperature()
{
  if(aht20.startMeasurementReady(true))
  {
    Serial.println("temp = " + String (aht20.getTemperature_F()));
    Serial.println("RH = " + String (aht20.getHumidity_RH()));
  }
}

void loop() {
  // No rotation counting happens here, handled by the interrupt
  // You can call displayWindSpeed() here or from other parts of your code
  // if you want to print the wind speed at specific points

calculateWindSpeed();
temperature();
batterylevel();



delay(3000);


}
