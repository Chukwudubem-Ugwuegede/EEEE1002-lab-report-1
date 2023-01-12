#include <Wire.h>
#include <Encoder.h>  // include the Encoder library 
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal
#include <Servo.h>    //include the servo library
#define servoPin 4
Servo myservo;        // create servo object to control a servo
float steeringAngle;  // variable to store the servo position

#define enA 5   //EnableA command line - should be a PWM pin
#define enB 6   //EnableB command line - should be a PWM pin

//name the motor control pins - replace the CHANGEME with your pin number, digital pins do not need the 'D' prefix whereas analogue pins need the 'A' prefix
#define INa A0  //Channel A direction 
#define INb A1  //Channel A direction 
#define INc A2 //Channel B direction 
#define INd A3  //Channel B direction 

byte speedSetting = 0;  //initial speed = 0

#define encoderPinA1 11 
#define encoderPinB1 2 
#define encoderPinA2 12 
#define encoderPinB2 3

// create two Encoder objects for the two wheels  

Encoder encoder1(encoderPinA1, encoderPinB1);  
Encoder encoder2(encoderPinA2, encoderPinB2);  

// define the number of pulses per revolution for the encoders  

const int PPR = 24;  // 21 pulses per revolution  
double prevDistance1 = 0;  

double prevDistance2 = 0;  
long distance1;
long distance2;


void setup() {
  // put your setup code here, to run once:
  Wire.begin(I2C_SLAVE_ADDR);   // join i2c bus #4 - on the Arduino NANO the default I2C pins are A4 (SDA), A5 (SCL)
  Wire.onReceive(receiveEvent); // register event
   Wire.onRequest(sendDistance); // register the sendDistance function to be called when the master requests data
  myservo.attach(servoPin);  //attach our servo object to pin D4
  //the Servo library takes care of defining the PinMode declaration (libraries/Servo/src/avr/Servo.cpp line 240)

  //configure the motor control pins as outputs
  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);   

  //initialise serial communication
  Serial.begin(9600);
  Serial.println("Arduino Nano is Running"); //sanity check

  speedSetting = 100;
  motors(speedSetting, speedSetting); //make a call to the "motors" function and provide it with a value for each of the 2 motors - can be different for each motor - using same value here for expedience
  Serial.print("Motor Speeds: ");
  Serial.println(speedSetting); 
}


void loop() {
  
  // get the number of pulses counted by the encoders  

  long pulses1 = encoder1.read();  

  long pulses2 = encoder2.read();  

  

  // calculate the distance moved by each wheel in meters  

  distance1 = pulses1 * (M_PI * 6) / PPR;  // 6 is the diameter in centimeters  

   distance2 = pulses2 * (M_PI * 6) / PPR;  // 6 is the diameter in centimeters  

}

// This function will be called when the Nano receives a command from the ESP32
void receiveEvent(int numBytes) {
  // Read the command from the ESP32
  char command = Wire.read();

  // Act on the command
  switch (command) {
    case 'F':
      // Go forwards
    goForwards();
      break;

    case 'C':
      // Go clockwise
      goClockwise();
      break;

    case 'A':
      // Go anticlockwise
     goAntiClockwise();
      break;
    case 'B':
      // Go backwards
      
      break;
    case 'S':
      // Stop the motors
      goBackwards();
      break;

      case 'M':
      //Adjust the steering angle
      moveSteering();
      break;
  }
}


void sendDistance(){
  Wire.write(distance1);
}


//for each of the below function, two of the 'IN' variables must be HIGH, and two LOW in order to move the wheels - use a trial and error approach to determine the correct combination for your EEEBot
void goForwards() {
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
}

void goBackwards() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, HIGH);
  digitalWrite(INc, LOW);
  digitalWrite(INd, HIGH);
}

void goClockwise() {
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, HIGH);
}

void goAntiClockwise() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, HIGH);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
}

void stopMotors() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, LOW);
}
void moveSteering() {
  //you may need to change the maximum and minimum servo angle to have the largest steering motion
  int maxAngle = 180;
  int minAngle = 90;
  myservo.write(136);
  
    delay(15);                      //waits 15 ms for the servo to reach the position
  }

void motors(int leftSpeed, int rightSpeed) {
  //set individual motor speed
  //direction is set separately

  analogWrite(enA, leftSpeed);
  analogWrite(enB, rightSpeed);
}

