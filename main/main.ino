#include <BluetoothSerial.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "PIDLoop.h"
#include <ESP32Servo.h>
#include <esp_bt_main.h>
#include <esp_gap_bt_api.h>
#include <esp_bt_device.h>

#include "BluetoothSerial.h"

#define ZUMO_FAST        255

Servo motorServo;
Servo steeringServo;
Adafruit_INA219 ina219;
BluetoothSerial SerialBT;
byte BTData;
HUSKYLENS huskylens;

PIDLoop headingLoop(2000, 0, 0, false);
//HUSKYLENS green line >> SDA; blue line >> SCL
int ID1 = 1;
void printResult(HUSKYLENSResult result);

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;
unsigned long SetupTime = 0;
unsigned long measuredT = 0;
unsigned long currentT = 0;
const char* deviceName = "too evil";

int steeringPin = 32;
int motorPin = 33;
int left = 0, right = 0;
int direction = 90;

// 0 = run, 1= stop
int running = 0;


/* Check if Bluetooth configurations are enabled in the SDK */
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

void setup()
{

  ina219.begin();
  Serial.begin(115200);

  SerialBT.begin("vehicle connect");
  SerialBT.println("####################");
  SerialBT.println("# Startup Complete #");
  SerialBT.println("####################");
  Serial.println("Bluetooth Started! Ready to pair...");

  servoSetup();

  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  wireSetup();
  huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING); //Switch the algorithm to line tracking.
  motorServo.write(75);

  SetupTime = millis(); //measuring real time it takes to setup
}

void loop()
{

  int32_t error; 
  int newDirection = 0;
  int diff = 0;
  if(running = 1){
    if (!huskylens.request(ID1)) {Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));left = 0; right = 0;}
  else if(!huskylens.isLearned()) {Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));left = 0; right = 0;}
  else if(!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
  else {
     HUSKYLENSResult result = huskylens.read();

     // Calculate the error:
     error = (int32_t)result.xTarget - (int32_t)160;
     
     // Perform PID algorithm.
     headingLoop.update(error);

        // separate heading into left and right wheel velocities.
        //Serial.println(String()+"Heading "+headingLoop.m_command);  // -300..0..300

        newDirection = (headingLoop.m_command + 300);  // 0..600
        newDirection = (newDirection * 180) / 600;  // 0..180

        if( newDirection < 0 )
          newDirection = 0;
        if( newDirection > 180 )
          newDirection = 180;

        // reverse left/right
        newDirection = abs(newDirection - 180);

        diff = newDirection - direction;  // 0, 90 = 0 - 90 = -90
        diff = diff / 5; // 10%
        direction = direction + diff;
        
        if( direction < 0 )
          direction = 0;
        if( direction > 180 )
          direction = 180;

 }
 steeringServo.write(direction);

  }else{

  }
  
  //Reading output from bluetooth
  if(SerialBT.available())
  {
    BTData = SerialBT.read();
    Serial.write(BTData);
  }
  //send the BT signal every t second where t > 0.1s
  if(millis() - measuredT > 100){
    getPower();
  }
  //after running for 2 min, the car stops
  if(millis()-SetupTime > 120000){
    running = 0;
  }
  

}

void calculateAngle(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
      double angle = tan((result.yCenter/result.xCenter));
      //Serial.println(angle);
      //if the line is straight, drive faster
      if(angle > 80 || angle < 100) {
        Serial.println("I want to drive faster!");
        motorServo.write(80);
      }
      delay(250);
    }
    else if (result.command == COMMAND_RETURN_ARROW){
      double angle = tan((result.yOrigin/result.xOrigin));
      //Serial.println(angle);
      //if the line is straight, drive faster
      if(angle > 80 || angle < 100) {
        Serial.println(angle);
        motorServo.write(80);
      }
      delay(250);
    }
    else{
        Serial.println("Object unknown!");
    }
}

void printResult(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
        Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
    }
    else if (result.command == COMMAND_RETURN_ARROW){
        Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
    }
    else{
        Serial.println("Object unknown!");
    }
}

void getPower(){
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  measuredT = millis();
 /* Serial.print("time:     "); Serial.print(measuredT - SetupTime); Serial.println(" ms");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");
  */
  SerialBT.print(measuredT - SetupTime);SerialBT.print(", ");SerialBT.print(loadvoltage);
  SerialBT.print(", ");SerialBT.print(current_mA);SerialBT.print(", ");SerialBT.println(power_mW);
}

void servoSetup(){
  motorServo.attach(motorPin, 800, 2000);
  steeringServo.attach(steeringPin, 800, 2000);
  
  delay(1000);  // Wait for the motor driver to recognize the servo attachment
  motorServo.write(0); //// Set the motor to 0 speed to calibrate the driver
  steeringServo.write(90); 
  delay(3000); // Wait for the driver to calibrate to 0 speed
  motorServo.write(100);
  delay(500);
}

void wireSetup(){
  Wire.begin();
    while (!huskylens.begin(Wire)){
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protol Type>>I2C)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }
}
