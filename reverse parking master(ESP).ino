/* This is a code for an autonomous parking system. `this system is supposed to make a system that will
▪ Implement an IMU based system to control the amount of vehicle rotation
▪ Use both systems to:
▪ Drive forwards for 1 second
▪ Turn 180 degrees
▪ Reverse up to an obstacle, stopping 10cm away
▪ Then either...
a) Turn 90 degrees clockwise and reverse up to another obstacle, again s topping 10cm away
or
b) Turn 90 degrees clockwise and reverse a further 30cm*/



#include <Wire.h>
#include <Ultrasonic.h>
#include "MPU9250.h"
MPU9250 mpu;
#define SLAVE_ADDRESS 0x04  // I2C address of the Arduino Nano
#define GO_FORWARDS 'F'    // Command to make the Nano go forwards
#define GO_CLOCKWISE 'C'   // Command to make the Nano go clockwise
#define GO_ANTICLOCKWISE 'A' // Command to make the Nano go anticlockwise
#define GO_BACKWARDS 'B'   // Command to make the Nano go backwards
#define STOP_MOTORS 'S'    // Command to make the Nano stop the motors
#define MOVE_STEERING 'M'  //Command to adjust the steering angle

float prevYaw = 0;
const int TRIG_PIN = 26;  // define the trigger pin
const int ECHO_PIN = 17;  // define the echo pin
double distance;

Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN);  // create an Ultrasonic object

void setup() {
  Serial.begin(115200);
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
  Wire.begin();
  ultrasonic.setTimeout(500);  // set a timeout of 500ms
}

void loop() {
      if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            print_roll_pitch_yaw();
            prev_ms = millis();
        }
    }
  Wire.requestFrom(SLAVE_ADDRESS, 1); // request 1 byte of data from the slave
  if (Wire.available()) { // if data is available
    distance = Wire.read(); // read the distance variable from the I2C bus and store it in the distance variable
  }
  long ultrasonic_distance = ultrasonic.read();  // read the distance in cm
  // Send the command to Adjust the steering
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(MOVE_STEERING);
  Wire.endTransmission();
  delay(100);

// Send the command to go forwards to the Nano
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(GO_FORWARDS);
  delay(1000); // go forwards for 1 second
  Wire.endTransmission();

  // Send the command to stop the motors to the Nano
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(STOP_MOTORS);
  Wire.endTransmission();
  delay(5000);

  // Send the command to go clockwise to the Nano
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(GO_CLOCKWISE);
  Wire.endTransmission();
  prevYaw = mpu.getYaw();
  if (mpu.getYaw >= 180)
  {
   Wire.beginTransmission(SLAVE_ADDRESS);
   Wire.write(STOP_MOTORS);
   Wire.endTransmission();
  

 // Send the command to go backwards to the Nano
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(GO_BACKWARDS);
  Wire.endTransmission();
  if (ultrasonic_distance <= 10)
  {
     Wire.beginTransmission(SLAVE_ADDRESS);
     Wire.write(STOP_MOTORS);
     Wire.endTransmission();
  

  // Send the command to go clockwise to the Nano
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(GO_CLOCKWISE);
  Wire.endTransmission();
  prevYaw = mpu.getYaw();
  if(currentYaw - prevYaw >= 90)
  {
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write(STOP_MOTORS);
    Wire.endTransmission();    
  
  distance = 0;

 // Send the command to go backwards to the Nano
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(GO_BACKWARDS);
  Wire.endTransmission();

  if (distance >= 30)
  {
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write(STOP_MOTORS);
    Wire.endTransmission(); 
  }
  }
  }
  }
}

void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
}

  
  

