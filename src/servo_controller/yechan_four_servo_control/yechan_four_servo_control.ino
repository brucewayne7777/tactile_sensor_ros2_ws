// Controlling four servos with arduino. Adapted from Yechan Kwon
// Baudrate: 9600
// Serial Monitor commands: 1p(pull), 1u(push), 1n(neutral) 
// Example: 1p - pull command for Servo 1
//        : 2u - pushcommand for Servo 2



#include <Servo.h>
// Declare 4 servo motor objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
void setup() {
 // Attach servos to digital pins and set initial angle to 110 degrees
 servo1.attach(8);   servo1.write(110);  // Servo 1 to neutral
 servo2.attach(9);   servo2.write(110);  // Servo 2 to neutral
 servo3.attach(10);  servo3.write(110);  // Servo 3 to neutral
 servo4.attach(11);  servo4.write(110);  // Servo 4 to neutral
 // Start serial communication at 9600 baud rate
 Serial.begin(9600);
}
void loop() {
 if (Serial.available() >= 2) {  // Wait for at least 2-character command
   String command = Serial.readStringUntil('\n');  // Read command until newline
   command.trim();  // Remove whitespace
   // Control Servo 1
   if (command == "1n") {
     servo1.write(110);  // Neutral (110 degrees)
     Serial.println("Servo 1: Neutral (110 degrees)");
   } else if (command == "1p") {
     servo1.write(20);   // Pull (20 degrees)
     Serial.println("Servo 1: Pull (20 degrees)");
   } else if (command == "1u") {
     servo1.write(155);  // Push (155 degrees)
     Serial.println("Servo 1: Push (155 degrees)");
   }
   // Control Servo 2
   else if (command == "2n") {
     servo2.write(110);  // Neutral (110 degrees)
     Serial.println("Servo 2: Neutral (110 degrees)");
   } else if (command == "2p") {
     servo2.write(20);   // Pull (20 degrees)
     Serial.println("Servo 2: Pull (20 degrees)");
   } else if (command == "2u") {
     servo2.write(155);  // Push (155 degrees)
     Serial.println("Servo 2: Push (155 degrees)");
   }
   // Control Servo 3
   else if (command == "3n") {
     servo3.write(110);  // Neutral (110 degrees)
     Serial.println("Servo 3: Neutral (110 degrees)");
   } else if (command == "3p") {
     servo3.write(20);   // Pull (20 degrees)
     Serial.println("Servo 3: Pull (20 degrees)");
   } else if (command == "3u") {
     servo3.write(155);  // Push (155 degrees)
     Serial.println("Servo 3: Push (155 degrees)");
   }
   // Control Servo 4
   else if (command == "4n") {
     servo4.write(110);  // Neutral (110 degrees)
     Serial.println("Servo 4: Neutral (110 degrees)");
   } else if (command == "4p") {
     servo4.write(20);   // Pull (20 degrees)
     Serial.println("Servo 4: Pull (20 degrees)");
   } else if (command == "4u") {
     servo4.write(155);  // Push (155 degrees)
     Serial.println("Servo 4: Push (155 degrees)");
   }
 }
}
