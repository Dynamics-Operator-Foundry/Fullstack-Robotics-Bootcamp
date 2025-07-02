// Define the PWM pin
const int pwmPin0 = 6;
const int pwmPin1 = 9;
const int pwmPin2 = 10;
const int pwmPin3 = 11;
const int pwmPin = 9;



#include <Servo.h>

Servo myServo;  // Create a servo object

void setup() {
  myServo.attach(pwmPin1); // Attach servo to the pin
  Serial.begin(9600);
}

void loop() {
  myServo.write(0);  // Move servo to 0 degrees
  Serial.println("Servo at 0 degrees");
  delay(1000);

  myServo.write(180); // Move servo to 180 degrees
  Serial.println("Servo at 180 degrees");
  delay(1000);
}


