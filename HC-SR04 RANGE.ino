const int TRIG_PIN = 13;  // define the trigger pin
const int ECHO_PIN = 12;  // define the echo pin

void setup() {
  Serial.begin(9600);  // initialize serial communication
  while (!Serial) {;}  // wait for serial connection

  pinMode(TRIG_PIN, OUTPUT);  // set the trigger pin as an output
  pinMode(ECHO_PIN, INPUT);   // set the echo pin as an input
}

void loop() {
  long duration, distance;

  digitalWrite(TRIG_PIN, LOW); // ensure the trigger pin is low
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); // trigger a measurement by setting the trigger pin high
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW); // stop the trigger pulse
  
  // calculate the duration of the echo pulse
  duration = pulseIn(ECHO_PIN, HIGH);
  
  // convert the duration to distance using the speed of sound
  distance = (duration / 2) / 29.1;

  Serial.print("Distance: ");  // print "Distance: "
  Serial.println(distance);  // print the distance
  delay(1000);  // delay for 1 second
}
