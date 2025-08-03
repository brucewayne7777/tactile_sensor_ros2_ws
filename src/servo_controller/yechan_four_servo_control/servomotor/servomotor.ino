#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

void setup() {
  servo1.attach(8);   servo1.write(110);  // Servo 1 to neutral
  servo2.attach(9);   servo2.write(110);  // Servo 2 to neutral
  servo3.attach(10);  servo3.write(110);  // Servo 3 to neutral
  servo4.attach(11);  servo4.write(110);  // Servo 4 to neutral
  
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() >= 2) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // servo1 control
    if (command == "1n") {
      servo1.write(110);
      Serial.println("Servo 1: Neutral (110 degrees)");
    } else if (command == "1p") {
      servo1.write(20);
      Serial.println("Servo 1: Pull (20 degrees)");
    } else if (command == "1u") {
      servo1.write(155);
      Serial.println("Servo 1: Push (155 degrees)");
    } else if (command == "1z") {
      servo1.write(0);
      Serial.println("Servo 1: Zero (0 degrees)");
    }
    
    // servo2 control
    else if (command == "2n") {
      servo2.write(110);
      Serial.println("Servo 2: Neutral (110 degrees)");
    } else if (command == "2p") {
      servo2.write(10);
      Serial.println("Servo 2: Pull (20 degrees)");
    } else if (command == "2u") {
      servo2.write(155);
      Serial.println("Servo 2: Push (155 degrees)");
    } else if (command == "2z") {
      servo2.write(0);
      Serial.println("Servo 2: Zero (0 degrees)");
    }
    
    // servo3 control
    else if (command == "3n") {
      servo3.write(110);
      Serial.println("Servo 3: Neutral (110 degrees)");
    } else if (command == "3p") {
      servo3.write(20);
      Serial.println("Servo 3: Pull (20 degrees)");
    } else if (command == "3u") {
      servo3.write(155);
      Serial.println("Servo 3: Push (155 degrees)");
    } else if (command == "3z") {
      servo3.write(0);
      Serial.println("Servo 3: Zero (0 degrees)");
    }
    
    // servo4 control
    else if (command == "4n") {
      servo4.write(110);
      Serial.println("Servo 4: Neutral (110 degrees)");
    } else if (command == "4p") {
      servo4.write(20);
      Serial.println("Servo 4: Pull (20 degrees)");
    } else if (command == "4u") {
      servo4.write(155);
      Serial.println("Servo 4: Push (155 degrees)");
    } else if (command == "4z") {
      servo4.write(0);
      Serial.println("Servo 4: Zero (0 degrees)");
    }
  }
}
