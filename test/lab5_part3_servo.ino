#include <Servo.h>

Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position

String cmdStr;
float msg_val;

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);
  Serial.println("Started!");
}

void loop() {
  if (Serial.available() > 0) {
    cmdStr = Serial.readString();
    Serial.println(cmdStr);

    // drive commands eg. '[d2:90]'
    if (cmdStr.charAt(0) == '[') {
      //Remove characters to get only the inch distance value
      cmdStr.remove(cmdStr.length() - 1,1);  // remove last bracket
      cmdStr.remove(0,1);                    // remove first bracket
      if (cmdStr.charAt(0) != 'd') {
        Serial.println("Error: command not recognized");
        return;
      }
      cmdStr.remove(0,1);
      if (cmdStr.charAt(0) != '2') {
        Serial.println("Error: drive system not found");
        return;
      }
      cmdStr.remove(0, 2);
      msg_val = cmdStr.toFloat();
      if (msg_val > 180 || msg_val < 0) {
        Serial.println("Error: desired angle out of range");
        return;
      }
      myservo.write(msg_val);
      Serial.println("d2");
      Serial.println(msg_val);
    }
  }
  delay(1);
}