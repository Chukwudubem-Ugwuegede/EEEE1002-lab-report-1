#include <Wire.h> // Include the Wire library

const int MPU9250_ADDRESS = 0x68; // MPU9250 address (can be 0x68 or 0x69 depending on the AD0 pin)

void setup() {
  Wire.begin(); // Start the I2C bus
  Wire.beginTransmission(MPU9250_ADDRESS); // Connect to the MPU9250
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // Set to zero to wake up the MPU9250
  Wire.endTransmission(true); // Stop transmission

  // Configure the accelerometer range
  Wire.beginTransmission(MPU9250_ADDRESS); // Connect to the MPU9250
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x10); // Set the range to 16g
  Wire.endTransmission(true); // Stop transmission
  
  // Configure the gyroscope range
  Wire.beginTransmission(MPU9250_ADDRESS); // Connect to the MPU9250
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0x10); // Set the range to 2000 deg/s
  Wire.endTransmission(true); // Stop transmission

}

void loop() {
  Wire.beginTransmission(MPU9250_ADDRESS); // Connect to the MPU9250
  Wire.write(0x3B); // Starting register address for Accelerometer
  Wire.endTransmission(false); // Send data to I2C dev with option for a repeated start.
  Wire.requestFrom(MPU9250_ADDRESS, 6); // Request 6 bytes of data
  while(Wire.available() < 6); // Wait for the data to be received
  
  // Read and store the accelerometer data
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  
   // Print the accelerometer data
  Serial.print("Accelerometer X: ");
  Serial.print(ax);
  Serial.print("  Y: ");
  Serial.print(ay);
  Serial.print("  Z: ");
  Serial.println(az);

  Wire.beginTransmission(MPU9250_ADDRESS); // Connect to the MPU9250
  Wire.write(0x43); // Starting register address for Gyroscope
  Wire.endTransmission(false); // Send data to I2C dev with option for a repeated start.
  Wire.requestFrom(MPU9250_ADDRESS, 6); // Request 6 bytes of data
  while(Wire.available() < 6); // Wait for data to be received 
    // Read and store the gyroscope data
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();

  // Print the gyroscope data
  Serial.print("Gyroscope X: ");
  Serial.print(gx);
  Serial.print("  Y: ");
  Serial.print(gy);
  Serial.print("  Z: ");
  Serial.println(gz);

  delay(250); // Wait before reading the next set of data
}

