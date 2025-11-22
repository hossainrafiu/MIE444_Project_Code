// #include <VL53L1X.h>

// VL53L1X sensor;

// void setup() {
//   Serial.begin(115200);
//   Wire.begin();
//   sensor.setTimeout(500);
//   if (!sensor.init())
//   {
//     Serial.println("Failed to detect and initialize sensor!");
//     while (1);
//   }
//   sensor.setDistanceMode(VL53L1X::DistanceMode::Short);
//   Serial.println("ROI Center: " + String(sensor.getROICenter()));
//   sensor.setROISize(4, 4);
//   uint8_t roiwidth, roiheight;
//   sensor.getROISize(&roiwidth, &roiheight);
//   Serial.println("ROI Width: " + String(roiwidth) + " Height: " + String(roiheight));
//   sensor.setMeasurementTimingBudget(50000);
//   sensor.startContinuous(100);
// }

// void loop() {
//   Serial.print("Distance (mm): ");
//   Serial.println(sensor.read());
//   delay(100);
// }

#include <main.h>

bool verboseConsole = false;
bool verboseSensors = false;
bool remoteControl = true;

void setup() {
  Serial1.begin(115200);
  if (!remoteControl) Serial1.println("Starting....");
  Serial.begin(115200);
  Serial.println("Starting....");
  pinMode(In1A, OUTPUT);
  pinMode(In2A, OUTPUT);
  pinMode(EnM1A, OUTPUT);
  pinMode(In3A, OUTPUT);
  pinMode(In4A, OUTPUT);
  pinMode(EnM2A, OUTPUT);
  pinMode(In1B, OUTPUT);
  pinMode(In2B, OUTPUT);
  pinMode(EnM3B, OUTPUT);
  pinMode(In3B, OUTPUT);
  pinMode(In4B, OUTPUT);
  pinMode(EnM4B, OUTPUT);
  pinMode(LEDPin[0], OUTPUT);
  pinMode(LEDPin[1], OUTPUT);
  pinMode(LEDPin[2], OUTPUT);
  pinMode(LEDPin[3], OUTPUT);
  
  pinMode(switchPin, INPUT);

  // initialize servos (0 - base, 1 - gripper)
  servos[0].attach(servoPins[0]);
  servos[1].attach(servoPins[1]);
  servos[0].write(120); // Gripper Up position
  servos[1].write(5); // Gripper Open position

  resetToF(false);
  digitalWrite(LEDPin[0], HIGH);
}

void resetToF(bool initializeLoadSensors){
  // Blink LEDs to indicate initialization
  Wire.end();
  
  for (int i=0; i<4; i++){
    digitalWrite(LEDPin[0], HIGH);
    delay(10);
    digitalWrite(LEDPin[0], LOW);
  }
  for (int i=0; i<4 && !initializeLoadSensors; i++){
    pinMode(tofPins[i], OUTPUT);
    digitalWrite(tofPins[i], LOW); // Disable all sensors
  }
  for (int i=0; i<2; i++){
    pinMode(loadToFPins[i], OUTPUT);
    digitalWrite(loadToFPins[i], LOW); // Disable all load sensors
  }

  delay(500);
  Wire.begin();
  delay(500);

  for (int i=0; i<4 && !initializeLoadSensors; i++){
    if (!remoteControl) Serial1.println("Initializing sensor " + String(i));
    Serial.println("Initializing sensor " + String(i));
    pinMode(tofPins[i], INPUT);
    delay(100); // Wait for sensor to boot
    sensors[i].setTimeout(500);
    sensors[i].init();
    sensors[i].setMeasurementTimingBudget(20000);
    sensors[i].startContinuous();
    sensors[i].setAddress(0x30 + i);
  }
  if (initializeLoadSensors) {
    if (!remoteControl) Serial1.println("Initializing Load sensor Top");
    Serial.println("Initializing Load sensor Top");
    pinMode(loadToFPins[0], INPUT);
    delay(100); // Wait for sensor to boot
    loadSensorTop.setTimeout(500);
    loadSensorTop.init();
    loadSensorTop.setMeasurementTimingBudget(50000);
    loadSensorTop.startContinuous();
    loadSensorTop.setAddress(0x35);

    if (!remoteControl) Serial1.println("Initializing Load sensor Bottom");
    Serial.println("Initializing Load sensor Bottom");
    pinMode(loadToFPins[1], INPUT);
    delay(100); // Wait for sensor to boot
    loadSensorBottom.setTimeout(500);
    loadSensorBottom.init();
    loadSensorBottom.setMeasurementTimingBudget(50000);
    loadSensorBottom.setROISize(4, 4);
    loadSensorBottom.startContinuous(100);
    loadSensorBottom.setAddress(0x36);
  }
}

void loop(){
  controlFromSerial();
}

void commandTimeoutCheck()
{
  // check for command timeout
  if ((commandTimeout > 0 && (millis() - lastCommandTime >= commandTimeout)) ||
      (commandTimeout > 0 && tofDistancesReal[frontDirection] < 100)){
    halt();
    commandTimeout = 0;
  }
}

String command = "";
int start = 0;
int end = 0;
void controlFromSerial()
{
  commandTimeoutCheck();
  if (Serial1.available())
  {
    command = Serial1.readStringUntil('\n');

    if (!remoteControl) Serial1.println("Received command: " + command);
    Serial.println("Received command: " + command);
    
    // COMMAND PARSING
    start = command.indexOf('[');
    end = command.indexOf(']');
    if (start == -1 || end == -1)
    {
      Serial1.println("Invalid command format.");
      return;
    }
    val = command.charAt(start + 1);
    commandDuration = command.substring(start + 2, end).toInt();
    if (commandDuration <= 0){
      commandDuration = 300; // Default duration 300 milliseconds
    }

    // FORWARD
    if (val == 'f')
    {
      forward();
      lastCommandTime = millis();
      commandTimeout = commandDuration;
    }
    // BACKWARDS
    else if (val == 's')
    {
      backwards();
      lastCommandTime = millis();
      commandTimeout = commandDuration;
    }
    // LEFT
    else if (val == 'a')
    {
      left();
      lastCommandTime = millis();
      commandTimeout = commandDuration;
    }
    // RIGHT
    else if (val == 'd')
    {
      right();
      lastCommandTime = millis();
      commandTimeout = commandDuration;
    }
    // CW ROTATION
    else if (val == 'e'){
      rotateCW();
      lastCommandTime = millis();
      commandTimeout = commandDuration;
    }
    // CCW ROTATION
    else if (val == 'q'){
      rotateCCW();
      lastCommandTime = millis();
      commandTimeout = commandDuration;
    }
    // CHANGE FRONT DIRECTION
    else if (val == 'r')
    {
      int direction = command.charAt(start + 2) - '0';
      if (direction < 0 || direction > 3){
        Serial1.println("Invalid direction for 'r' command.");
        return;
      }
      changeFrontDirection(direction);
    }

    // TRUE FORWARDS and BACKWARDS
    else if (val == 'i'){
      trueForward();
      lastCommandTime = millis();
      commandTimeout = commandDuration;
    }
    else if (val == 'k'){
      trueBackwards();
      lastCommandTime = millis();
      commandTimeout = commandDuration;
    }

    // PING TOF SENSORS
    else if (val == 'p')
    {
      int numTimes = command.substring(start + 2, end).toInt();
      if (start + 2 >= end){
        pingToF();
      }
      else{
        pingToF(numTimes);
      }
      transmitToFData();
    }

    // HALT
    else if (val == 'h')
    {
      halt();
    }

    // SERVOS
    else if (val == 'l'){
      int servo = command.charAt(start + 2) - '0';
      int pwnPosition = command.substring(start+3,end).toInt();
      actuateServo(servo, pwnPosition);
    }

    // LOAD SENSORS
    else if (val == 'v'){
      Serial.println("Ping Load TOF");
      pingLoadToF();
      Serial.println("Transmit Load TOF");
      transmitLoadToFData();
    }
    else if (val == 'n'){
      int switchState = digitalRead(switchPin);
      Serial1.println("[" + String(switchState) + "]");
    }

    // SPEED ADJUSTMENTS
    else if (val == 'z'){
      for (int i = 0; i<4 ; i++){
        speeds[i] += 10;
      }
      Serial1.println("Speeds increased to: " + String(speeds[0]) + ", " + String(speeds[1]) + ", " + String(speeds[2]) + ", " + String(speeds[3]));
    }
    else if (val == 'x'){
      for (int i = 0; i<4 ; i++){
        speeds[i] -= 10;
      }
      Serial1.println("Speeds decreased to: " + String(speeds[0]) + ", " + String(speeds[1]) + ", " + String(speeds[2]) + ", " + String(speeds[3]));
    }
    else if (val == 'm'){
      int motor = command.charAt(start + 2) - '0';
      float newSpeed = command.substring(start+3,end).toFloat();
      if (motor < 1 || motor > 4){
        Serial1.println("Invalid motor index for 'm' command.");
        return;
      }
      speeds[motor - 1] = newSpeed;
    }
    else if (val == 'c'){
      Serial1.println("[" + String(speeds[0]) + ", " + String(speeds[1]) + ", " + String(speeds[2]) + ", " + String(speeds[3]) + "]");
    }

    // RESET ARDUINO
    else if (val == 'b'){
      pinMode(resetPin, OUTPUT);
      digitalWrite(resetPin, HIGH);
    }
    else
    {
      Serial1.println("Unknown command: " + String(val));
    }
  if (val != 0 && val != 'p' && val != 'u' && val != 'v' && val != 'c')
    {
      Serial1.println("[+]");
    }
  }
}

// Assuming:
//         Forward
//       (M1     M2)
// Left               Right
//       (M3     M4)
//        Backwards

// Assuming:
// In1:HIGH and In2:LOW -> CCW

void trueForward(float speedDivisor)
{
  digitalWrite(In1A, HIGH); 
  digitalWrite(In2A, LOW); 
  analogWrite(EnM1A, speeds[0]);

  digitalWrite(In3A, LOW); 
  digitalWrite(In4A, HIGH); 
  analogWrite(EnM2A, speeds[1]);

  digitalWrite(In1B, HIGH); 
  digitalWrite(In2B, LOW); 
  analogWrite(EnM3B, speeds[2]);

  digitalWrite(In3B, LOW); 
  digitalWrite(In4B, HIGH); 
  analogWrite(EnM4B, speeds[3]);
}

void trueBackwards(float speedDivisor)
{
  digitalWrite(In1A, LOW); 
  digitalWrite(In2A, HIGH); 
  analogWrite(EnM1A, speeds[0]);

  digitalWrite(In3A, HIGH); 
  digitalWrite(In4A, LOW); 
  analogWrite(EnM2A, speeds[1]);

  digitalWrite(In1B, LOW); 
  digitalWrite(In2B, HIGH); 
  analogWrite(EnM3B, speeds[2]);

  digitalWrite(In3B, HIGH); 
  digitalWrite(In4B, LOW); 
  analogWrite(EnM4B, speeds[3]);
}

void forward(float speedDivisor)
{
  switch (frontDirection)
  {
  case 0: // front
    frontLeft(speedDivisor);
    break;

  case 1: // right
    frontRight(speedDivisor);
    break;

  case 2: // back
    backRight(speedDivisor);
    break;

  case 3: // left
    backLeft(speedDivisor);
    break;

  default:
    break;
  }
}
void backwards(float speedDivisor)
{
  switch (frontDirection)
  {
  case 0: // front
    backRight(speedDivisor);
    break;
  case 1: // right
    backLeft(speedDivisor);
    break;
  case 2: // back
    frontLeft(speedDivisor);
    break;
  case 3: // left
    frontRight(speedDivisor);
    break;
  default:
    break;
  }
}
void left(float speedDivisor)
{
  switch (frontDirection)
  {
  case 0: // front
    backLeft(speedDivisor);
    break;
  case 1: // right
    frontLeft(speedDivisor);
    break;
  case 2: // back
    frontRight(speedDivisor);
    break;
  case 3: // left
    backRight(speedDivisor);
    break;
  default:
    break;
  }
}
void right(float speedDivisor)
{
  switch (frontDirection)
  {
  case 0: // front
    frontRight(speedDivisor);
    break;
  case 1: // right
    backRight(speedDivisor);
    break;
  case 2: // back
    backLeft(speedDivisor);
    break;
  case 3: // left
    frontLeft(speedDivisor);
    break;
  default:
    break;
  }
}
void frontRight(float speedDivisor)
{
  digitalWrite(In1A, HIGH); 
  digitalWrite(In2A, LOW); 
  analogWrite(EnM1A, speeds[0]/speedDivisor);

  digitalWrite(In3A, LOW); 
  digitalWrite(In4A, LOW); 
  analogWrite(EnM2A, 0);

  digitalWrite(In1B, LOW); 
  digitalWrite(In2B, LOW); 
  analogWrite(EnM3B, 0);

  digitalWrite(In3B, LOW); 
  digitalWrite(In4B, HIGH); 
  analogWrite(EnM4B, speeds[3]/speedDivisor);
}
void frontLeft(float speedDivisor)
{
  digitalWrite(In1A, LOW); 
  digitalWrite(In2A, LOW); 
  analogWrite(EnM1A, 0);

  digitalWrite(In3A, LOW); 
  digitalWrite(In4A, HIGH); 
  analogWrite(EnM2A, speeds[1]/speedDivisor);

  digitalWrite(In1B, HIGH); 
  digitalWrite(In2B, LOW); 
  analogWrite(EnM3B, speeds[2]/speedDivisor);

  digitalWrite(In3B, LOW); 
  digitalWrite(In4B, LOW); 
  analogWrite(EnM4B, 0);
}
void backRight(float speedDivisor)
{
  digitalWrite(In1A, LOW); 
  digitalWrite(In2A, LOW); 
  analogWrite(EnM1A, 0);

  digitalWrite(In3A, HIGH); 
  digitalWrite(In4A, LOW); 
  analogWrite(EnM2A, speeds[1]/speedDivisor);

  digitalWrite(In1B, LOW); 
  digitalWrite(In2B, HIGH); 
  analogWrite(EnM3B, speeds[2]/speedDivisor);

  digitalWrite(In3B, LOW); 
  digitalWrite(In4B, LOW); 
  analogWrite(EnM4B, 0);
}
void backLeft(float speedDivisor)
{
  digitalWrite(In1A, LOW); 
  digitalWrite(In2A, HIGH); 
  analogWrite(EnM1A, speeds[0]/speedDivisor);

  digitalWrite(In3A, LOW); 
  digitalWrite(In4A, LOW); 
  analogWrite(EnM2A, 0);

  digitalWrite(In1B, LOW); 
  digitalWrite(In2B, LOW); 
  analogWrite(EnM3B, 0);

  digitalWrite(In3B, HIGH); 
  digitalWrite(In4B, LOW); 
  analogWrite(EnM4B, speeds[3]/speedDivisor);
}
void rotateCW(float speedDivisor)
{
  digitalWrite(In1A, HIGH); 
  digitalWrite(In2A, LOW); 
  analogWrite(EnM1A, speeds[0]/speedDivisor);

  digitalWrite(In3A, HIGH); 
  digitalWrite(In4A, LOW); 
  analogWrite(EnM2A, speeds[1]/speedDivisor);

  digitalWrite(In1B, HIGH); 
  digitalWrite(In2B, LOW); 
  analogWrite(EnM3B, speeds[2]/speedDivisor);

  digitalWrite(In3B, HIGH); 
  digitalWrite(In4B, LOW); 
  analogWrite(EnM4B, speeds[3]/speedDivisor);
}
void rotateCCW(float speedDivisor)
{
  digitalWrite(In1A, LOW); 
  digitalWrite(In2A, HIGH); 
  analogWrite(EnM1A, speeds[0]/speedDivisor);

  digitalWrite(In3A, LOW); 
  digitalWrite(In4A, HIGH); 
  analogWrite(EnM2A, speeds[1]/speedDivisor);

  digitalWrite(In1B, LOW); 
  digitalWrite(In2B, HIGH); 
  analogWrite(EnM3B, speeds[2]/speedDivisor);

  digitalWrite(In3B, LOW); 
  digitalWrite(In4B, HIGH); 
  analogWrite(EnM4B, speeds[3]/speedDivisor);
}
void halt()
{
  digitalWrite(In1A, HIGH); 
  digitalWrite(In2A, HIGH); 
  analogWrite(EnM1A, 255);

  digitalWrite(In3A, HIGH); 
  digitalWrite(In4A, HIGH); 
  analogWrite(EnM2A, 255);

  digitalWrite(In1B, HIGH); 
  digitalWrite(In2B, HIGH); 
  analogWrite(EnM3B, 255);

  digitalWrite(In3B, HIGH); 
  digitalWrite(In4B, HIGH); 
  analogWrite(EnM4B, 255);
}

void pingToF(int numTimes)
{
  int calibration[4] = {20, 10, 10, 0};
  for (int i=0; i<4; i++){
    long measurement = sensors[i].readRangeContinuousMillimeters();
    tofDistancesReal[i] = measurement;
    for (int j=1; j<numTimes; j++){
      measurement = sensors[i].readRangeContinuousMillimeters();
      tofDistancesReal[i] += measurement;
    }
    tofDistancesReal[i] /= (numTimes);
    tofDistancesReal[i] -= calibration[i];
  }

  bool CAN_RESET_TOF_ON_TIMEOUT = true;
  // Reset ToFs if any sensor times out (> 60000)
  if (CAN_RESET_TOF_ON_TIMEOUT){
    if (tofDistancesReal[0] > 60000 || tofDistancesReal[1] > 60000 || tofDistancesReal[2] > 60000 || tofDistancesReal[3] > 60000){
      Serial1.println("One or more ToF sensors timed out. Resetting sensors...");
      resetToF(false);
      loadSensorsInitialized = false;
    }
  }

  for (int i = 0 ; i < 4 ; i++){
    lastToFDistances[i] = tofDistances[i];
    tofDistances[i] = tofDistancesReal[(i + frontDirection) % 4];
  }
}

void pingLoadToF(int numTimes)
{
  if (loadSensorsInitialized == false){
    resetToF(true);
    loadSensorsInitialized = true;
  }
  for (int i = 0 ; i < 2 ; i++){
    lastLoadToFDistances[i] = loadToFDistances[i];
  }
  int calibration[2] = {0, 0};

  loadToFDistances[0] = loadSensorTop.readRangeContinuousMillimeters();
  for (int j=1; j<numTimes; j++){
    loadToFDistances[0] += loadSensorTop.readRangeContinuousMillimeters();
  }
  loadToFDistances[0] /= (numTimes);
  loadToFDistances[0] -= calibration[0];

  loadToFDistances[1] = loadSensorBottom.readRangeContinuousMillimeters();
  for (int j=1; j<numTimes; j++){
    loadToFDistances[1] += loadSensorBottom.readRangeContinuousMillimeters();
  }
  loadToFDistances[1] /= (numTimes);
  loadToFDistances[1] -= calibration[1];

  bool CAN_RESET_LOAD_TOF_ON_TIMEOUT = true;
  // Reset Load ToFs if any sensor times out (> 60000)
  if (CAN_RESET_LOAD_TOF_ON_TIMEOUT){
    if (loadToFDistances[0] > 60000 || loadToFDistances[1] == 0){
      Serial1.println("One or more Load ToF sensors timed out. Resetting sensors...");
      resetToF(true);
    }
  }
}

void changeFrontDirection(int newFront){
  frontDirection = newFront % 4;
  if (verboseConsole) Serial1.println("Front direction changed to: " + String(frontDirection));
  pingToF();
  delay(100);
  
  for (int i = 0 ; i < 4 ; i++){
    int write = 0;
    if (i==newFront){
      write = 1;
    }
    digitalWrite(LEDPin[i], write);
  }
}

void transmitToFData(){
  // Transmit sensor distances over Serial1 in a comma-separated format
  String buffer = "[";
  for (int i = 0; i < 4; i++){
    buffer += String(tofDistancesReal[i]);
    buffer += ",";
  }
  buffer += String(frontDirection);
  buffer += "]";
  Serial.println(buffer);
  Serial1.println(buffer);
}

void transmitLoadToFData(){
  // Transmit load sensor distances over Serial1 in a comma-separated format
  String buffer = "[";
  buffer += String(loadToFDistances[0]);
  buffer += ",";
  buffer += String(loadToFDistances[1]);
  buffer += "]";
  Serial1.println(buffer);
}

void actuateServo(int servo, int angle){
  if (servo < 0 || servo >= 2){
    Serial1.println("Invalid servo index: " + String(servo));
    return;
  }
  if (angle < MIN_SERVO_ANGLE[servo] || angle > MAX_SERVO_ANGLE[servo]){
    Serial1.println("Invalid servo angle: " + String(angle) + " limit: [" + String(MIN_SERVO_ANGLE[servo]) + ", " + String(MAX_SERVO_ANGLE[servo]) + "]" );
    return;
  }
  servos[servo].write(angle);
  if (verboseConsole) Serial1.println("Servo " + String(servo) + " moved to angle " + String(angle));
}