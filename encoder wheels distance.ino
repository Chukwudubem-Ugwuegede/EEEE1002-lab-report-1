#include <Encoder.h>  // include the Encoder library  

// define the pins for the two encoders  
#define encoderPinA1 11 
#define encoderPinB1 2 
#define encoderPinA2 12 
#define encoderPinB2 3

// create two Encoder objects for the two wheels  
Encoder encoder1(encoderPinA1, encoderPinB1);  
Encoder encoder2(encoderPinA2, encoderPinB2);  

// define the number of pulses per revolution for the encoders  
const int PPR = 23;  // 23 pulses per revolution  

// define variables to store the previous distance traveled by each wheel  
double prevDistance1 = 0;  
double prevDistance2 = 0;  

void setup() {  
  // initialize serial communication  
  Serial.begin(9600);  
}  

void loop() {  
  // get the number of pulses counted by the encoders  
  long pulses1 = encoder1.read();  
  long pulses2 = encoder2.read();  

  // calculate the distance moved by each wheel in centimeters 
  double distance1 = pulses1 * (M_PI * 6) / PPR;
  double distance2 = pulses2 * (M_PI * 6) / PPR; 

  // print the distance moved by each wheel to the serial monitor if it has changed  
  if (distance1 != prevDistance1) {  
    Serial.print("Wheel 1: ");  // specify the wheel  
    Serial.print(distance1, 2);  // print distance1 with 2 decimal places  
     Serial.println(" cm");  // print the unit of distance
    prevDistance1 = distance1;  // update the previous distance traveled by the first wheel  
  }  
  if (distance2 != prevDistance2) {  
    Serial.print("Wheel 2 : ");  // specify the wheel  
    Serial.print(distance2 , 2);  // print distance2 with 2 decimal places  
     Serial.println(" cm");  // print the unit of distance
    prevDistance2 = distance2;  // update the previous distance traveled by the second wheel  
  }  
} 
