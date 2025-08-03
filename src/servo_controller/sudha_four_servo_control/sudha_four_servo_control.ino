// Angle check/calibration

#include <Servo.h>

Servo myServo;
const int servoPin = 8; // Adjust if using a different pin

void setup() {
  myServo.attach(servoPin);
}

void loop() {
  // Move to 110 degrees
  myServo.write(110);
  delay(2000);

  // Move to 10 degrees
  myServo.write(10);
  delay(2000);

  // Move to 155 degrees
  myServo.write(155);
  delay(2000);
  
  // Move to 110 degrees
  myServo.write(110);
  delay(2000);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Servo.h>

Servo myServo;
const int servoPin = 8;

const int neutralPos = 110;     // Preset positions (angles)
const int pullDownPos = 10;     
const int resetPos = 135;      //+45deg angular
const int zero = 0;           // 0 deg servo
const int angle180 = 180;           // 180 deg servo

void setup() {
  Serial.begin(9600);           // Initialize Serial communication
  myServo.attach(servoPin);
  myServo.write(neutralPos);    // Start at neutral position
  Serial.println("Enter state: 0, n, pd, r or 180");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "n") {
      myServo.write(neutralPos);
      Serial.println("Moved to neutral position");
    }
    else if (input == "0") {
      myServo.write(zero);
      Serial.println("zero angle Servo");
    } 
    else if (input == "pd") {
      myServo.write(pullDownPos);
      Serial.println("Moved to pull down position");
    }
     else if (input == "180") {
      myServo.write(angle180);
      Serial.println("Moved to 180 deg position");
    } 
    else if (input == "r") {
      myServo.write(resetPos);
      Serial.println("Moved to reset position");
    } 
    else {
      Serial.println("Invalid state. Use: neutral, pulldown, or reset.");
    }
  }
}
