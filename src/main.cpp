#include <main.h>

bool verboseConsole = false;
bool verboseSensors = false;
bool remoteControl = true;

void setup() {
  Serial1.begin(9600);
  if (!remoteControl) Serial1.println("Starting....");
  Serial.begin(9600);
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

  // initialize servos (0 - base, 1 - gripper)
  servos[0].attach(servoPins[0]);
  servos[1].attach(servoPins[1]);
  servos[0].write(120); // Gripper Up position
  servos[1].write(5); // Gripper Open position

  Wire.begin();
  resetToF(false);
  digitalWrite(LEDPin[0], HIGH);
}

void resetToF(bool initializeLoadSensors){
  // Blink LEDs to indicate initialization
  for (int i=0; i<4; i++){
    digitalWrite(LEDPin[0], HIGH);
    delay(10);
    digitalWrite(LEDPin[0], LOW);
  }
  for (int i=0; i<4; i++){
    pinMode(tofPins[i], OUTPUT);
    digitalWrite(tofPins[i], LOW); // Disable all sensors
  }
  for (int i=0; i<2; i++){
    pinMode(loadToFPins[i], OUTPUT);
    digitalWrite(loadToFPins[i], LOW); // Disable all load sensors
  }
  delay(1000);
  for (int i=0; i<4; i++){
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
    if (!remoteControl) Serial1.println("Initializing load sensor on top");
    Serial.println("Initializing load sensor on top");
    pinMode(loadToFPins[0], INPUT);
    delay(100); // Wait for sensor to boot
    loadSensorTop.setTimeout(500);
    loadSensorTop.init();
    loadSensorTop.setMeasurementTimingBudget(20000);
    loadSensorTop.startContinuous();
    loadSensorTop.setAddress(0x35);

    if (!remoteControl) Serial1.println("Initializing load sensor on bottom");
    Serial.println("Initializing load sensor on bottom");
    pinMode(loadToFPins[1], INPUT);
    delay(100); // Wait for sensor to boot
    loadSensorBottom.setTimeout(500);
    loadSensorBottom.init();
    loadSensorBottom.setMeasurementTimingBudget(20000);
    loadSensorBottom.startContinuous(10);
    loadSensorBottom.setAddress(0x36);
  }
}

void loop(){
  controlFromSerial();
}

void serialTest(){
  if (Serial1.available()){
    char command = Serial1.read();
    pingToF(1);
    transmitToFData();
  }
}

void checks()
{
  // check for command timeout
  pingToF(3);
  if ((commandTimeout > 0 && (millis() - lastCommandTime >= commandTimeout))
  // || (tofDistances[0] < 70 && carefulForward)
  // || (tofDistances[1] < 30)
  // || (tofDistances[3] < 30)
    )
  {
    // transmitToFData();
    halt();
    commandTimeout = 0;
  }
}

String command = "";
int start = 0;
int end = 0;
void controlFromSerial()
{
  // Serial.println("Checking serial...");
  // Serial.println("Checking serial...");
  checks();
  if (Serial1.available())
  {
    command = Serial1.readStringUntil('\n');

    if (!remoteControl) Serial1.println("Received command: " + command);
    Serial.println("Received command: " + command);
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

    // PING TOF SENSORS
    else if (val == 'p')
    {
      int numTimes = command.substring(start + 2, end).toInt();
      if (start + 2 >= end){
        transmitToFData();
      }
      else{
        pingToF(numTimes);
        transmitToFData();
      }
    }
    // PING ULTRASONIC SENSORS
    else if (val == 'u')
    {
      int numTimes = command.substring(start + 2, end).toInt();
      if (start + 2 >= end){
        numTimes = 3; // Default to 3 times
      }
      pingSensors(numTimes); // number of times to ping
      transmitSensorData();
    }

    // HALT
    else if (val == 'h')
    {
      halt();
    }
    // CENTERING
    else if (val == 'c')
    {
      centering();
    }
    // ORIENT
    else if (val == 'o')
    {
      orient();
    }
    
    // PARALLEL SENSOR ADJUSTMENT #DEPRECATED
    else if (val == 'j')
    {
      int direction = command.charAt(start + 2) - '0';
      if (direction < 0 || direction > 3){
        Serial1.println("Invalid direction for 'j' command.");
        return;
      }
      parallelSensorAdjustment(direction);
    }

    else if (val == 'l'){
      int servo = command.charAt(start + 2) - '0';
      int pwnPosition = command.substring(start+3,end).toInt();
      actuateServo(servo, pwnPosition);
    }
    else if (val == 'v'){
      pinMode(31, OUTPUT);
      digitalWrite(31, HIGH);
      // Serial.println("Ping Load TOF");
      // pingLoadToF();
      // Serial.println("Transmit Load TOF");
      // transmitLoadToFData();
    }

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
    else if (val == 'b'){
      carefulForward = !carefulForward;
    }
    else if (val == '<')
    {
      MOVELEFTWHENPOSSIBLE = true;
      MOVERIGHTWHENPOSSIBLE = false;
    }
    else if (val == '>')
    {
      MOVERIGHTWHENPOSSIBLE = true;
      MOVELEFTWHENPOSSIBLE = false;
    }
    else
    {
      Serial1.println("Unknown command: " + String(val));
    }
  if (val != 0 && val != 'p' && val != 'u' && val != 'v')
  if (val != 0 && val != 'p' && val != 'u' && val != 'v')
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

void forward(float speedDivisor = 1.0)
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
  // frontLeft(sensorReading);

  // digitalWrite(In1A, HIGH); 
  // digitalWrite(In2A, LOW); 
  // analogWrite(EnM1A, speeds[0+shift]);

  // digitalWrite(In3A, LOW); 
  // digitalWrite(In4A, HIGH); 
  // analogWrite(EnM2A, speeds[1+shift]);

  // digitalWrite(In1B, HIGH); 
  // digitalWrite(In2B, LOW); 
  // analogWrite(EnM3B, speeds[2+shift]);

  // digitalWrite(In3B, LOW); 
  // digitalWrite(In4B, HIGH); 
  // analogWrite(EnM4B, speeds[3+shift]);

}
void backwards(float speedDivisor = 1.0)
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
  // backRight();

  // digitalWrite(In1A, LOW); 
  // digitalWrite(In2A, HIGH); 
  // analogWrite(EnM1A, speeds[0+shift]);

  // digitalWrite(In3A, HIGH); 
  // digitalWrite(In4A, LOW); 
  // analogWrite(EnM2A, speeds[1+shift]);

  // digitalWrite(In1B, LOW); 
  // digitalWrite(In2B, HIGH); 
  // analogWrite(EnM3B, speeds[2+shift]);

  // digitalWrite(In3B, HIGH); 
  // digitalWrite(In4B, LOW); 
  // analogWrite(EnM4B, speeds[3+shift]);
}
void left(float speedDivisor = 1.0)
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
  // backLeft();

  // digitalWrite(In1A, LOW); 
  // digitalWrite(In2A, HIGH); 
  // analogWrite(EnM1A, speeds[0+shift]);

  // digitalWrite(In3A, LOW); 
  // digitalWrite(In4A, HIGH); 
  // analogWrite(EnM2A, speeds[1+shift]);

  // digitalWrite(In1B, HIGH); 
  // digitalWrite(In2B, LOW); 
  // analogWrite(EnM3B, speeds[2+shift]);

  // digitalWrite(In3B, HIGH); 
  // digitalWrite(In4B, LOW); 
  // analogWrite(EnM4B, speeds[3+shift]);
}
void right(float speedDivisor = 1.0)
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
  // frontRight();

  // digitalWrite(In1A, HIGH); 
  // digitalWrite(In2A, LOW); 
  // analogWrite(EnM1A, speeds[0+shift]);

  // digitalWrite(In3A, HIGH); 
  // digitalWrite(In4A, LOW); 
  // analogWrite(EnM2A, speeds[1+shift]);

  // digitalWrite(In1B, LOW); 
  // digitalWrite(In2B, HIGH); 
  // analogWrite(EnM3B, speeds[2+shift]);

  // digitalWrite(In3B, LOW); 
  // digitalWrite(In4B, HIGH); 
  // analogWrite(EnM4B, speeds[3+shift]);
}
void frontRight(float speedDivisor = 1.0)
{
  digitalWrite(In1A, HIGH); 
  digitalWrite(In2A, LOW); 
  analogWrite(EnM1A, speeds[0+shift]/speedDivisor);

  digitalWrite(In3A, LOW); 
  digitalWrite(In4A, LOW); 
  analogWrite(EnM2A, 0);

  digitalWrite(In1B, LOW); 
  digitalWrite(In2B, LOW); 
  analogWrite(EnM3B, 0);

  digitalWrite(In3B, LOW); 
  digitalWrite(In4B, HIGH); 
  analogWrite(EnM4B, speeds[3+shift]/speedDivisor);
}
void frontLeft(float speedDivisor = 1.0)
{
  digitalWrite(In1A, LOW); 
  digitalWrite(In2A, LOW); 
  analogWrite(EnM1A, 0);

  digitalWrite(In3A, LOW); 
  digitalWrite(In4A, HIGH); 
  analogWrite(EnM2A, speeds[1+shift]/speedDivisor);

  digitalWrite(In1B, HIGH); 
  digitalWrite(In2B, LOW); 
  analogWrite(EnM3B, speeds[2+shift]/speedDivisor);

  digitalWrite(In3B, LOW); 
  digitalWrite(In4B, LOW); 
  analogWrite(EnM4B, 0);
}
void backRight(float speedDivisor = 1.0)
{
  digitalWrite(In1A, LOW); 
  digitalWrite(In2A, LOW); 
  analogWrite(EnM1A, 0);

  digitalWrite(In3A, HIGH); 
  digitalWrite(In4A, LOW); 
  analogWrite(EnM2A, speeds[1+shift]/speedDivisor);

  digitalWrite(In1B, LOW); 
  digitalWrite(In2B, HIGH); 
  analogWrite(EnM3B, speeds[2+shift]/speedDivisor);

  digitalWrite(In3B, LOW); 
  digitalWrite(In4B, LOW); 
  analogWrite(EnM4B, 0);
}
void backLeft(float speedDivisor = 1.0)
{
  digitalWrite(In1A, LOW); 
  digitalWrite(In2A, HIGH); 
  analogWrite(EnM1A, speeds[0+shift]/speedDivisor);

  digitalWrite(In3A, LOW); 
  digitalWrite(In4A, LOW); 
  analogWrite(EnM2A, 0);

  digitalWrite(In1B, LOW); 
  digitalWrite(In2B, LOW); 
  analogWrite(EnM3B, 0);

  digitalWrite(In3B, HIGH); 
  digitalWrite(In4B, LOW); 
  analogWrite(EnM4B, speeds[3+shift]/speedDivisor);
}
void rotateCW(float speedDivisor = 1.0)
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
void rotateCCW(float speedDivisor = 1.0)
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

void pingSensors(int numTimes = 5)
{
    for (int i = 0; i < 8; i++)
    {
        pingTimes[i] = sonars[i].ping_median(numTimes);
        sensorDistancesReal[i] = pingTimes[i] / 5.73; // Convert to mm
    }

    for (int i = 0 ; i < 8 ; i++){
      sensorDistances[i] = sensorDistancesReal[(i + frontDirection*2) % 8];
    }
}

void pingToF(int numTimes = 5)
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
    if (sensors[i].timeoutOccurred()) {
      Serial1.print("TIMEOUT");
      tofDistancesReal[i] = 8000;
    }
  }

  for (int i = 0 ; i < 4 ; i++){
    lastToFDistances[i] = tofDistances[i];
    tofDistances[i] = tofDistancesReal[(i + frontDirection) % 4];
  }
}

void pingFrontToF()
{
  tofDistancesReal[frontDirection] = sensors[frontDirection].readRangeContinuousMillimeters();
  tofDistancesReal[frontDirection] += sensors[frontDirection].readRangeContinuousMillimeters();
  tofDistancesReal[frontDirection] += sensors[frontDirection].readRangeContinuousMillimeters();
  tofDistancesReal[frontDirection] /= 3;
  if (sensors[frontDirection].timeoutOccurred()) {
    Serial1.print("TIMEOUT");
    tofDistancesReal[frontDirection] = 8000;
  }
  tofDistances[0] = tofDistancesReal[frontDirection];
}

void pingLoadToF(int numTimes = 5)
{
  for (int i = 0 ; i < 2 ; i++){
    lastLoadToFDistances[i] = loadToFDistances[i];
  }
  
  
  int calibration[2] = {0, 0};

  long measurement = loadSensorTop.readRangeContinuousMillimeters();
  Serial.println("Load ToF Top Measurement: " + String(measurement));
  loadToFDistances[0] = measurement;
  for (int j=1; j<numTimes; j++){
    measurement = loadSensorTop.readRangeContinuousMillimeters();
    Serial.println("Load ToF Top Measurement: " + String(measurement));
    loadToFDistances[0] += measurement;
  }
  loadToFDistances[0] /= (numTimes);
  loadToFDistances[0] -= calibration[0];

  measurement = loadSensorBottom.readRangeContinuousMillimeters();
  Serial.println("Load ToF Bottom Measurement: " + String(measurement));
  loadToFDistances[1] = measurement;
  for (int j=1; j<numTimes; j++){
    long measurement = loadSensorBottom.readRangeContinuousMillimeters();
    Serial.println("Load ToF Bottom Measurement: " + String(measurement));
    loadToFDistances[1] += measurement;
  }
  loadToFDistances[1] /= (numTimes);
  loadToFDistances[1] -= calibration[1];
}

void centering(){
  if (verboseConsole) Serial1.println("Centering rover...");
  int lastMeasure1 = 8000;
  int lastMeasure2 = 8000;
  int lastMeasure3 = 8000;
  for (int count = 0; count < 40; count++){
    halt();
    rotateCW(1.3);
    delay(200);
    halt();
    delay(200);
    pingToF();
    lastMeasure3 = lastMeasure2;
    lastMeasure2 = lastMeasure1;
    lastMeasure1 = tofDistances[1];
    if (verboseConsole) Serial1.println("M1: " + String(lastMeasure1) + " M2: " + String(lastMeasure2) + " M3: " + String(lastMeasure3));
    if (tofDistances[0] > 300 and tofDistances[1] < 150){
      if (lastMeasure1 > lastMeasure2 && lastMeasure3 > lastMeasure2){
        halt();
        rotateCCW(1.3);
        delay(200);
        halt();
        return;
      }
    }
  }
  // If centering fails, rotate until two corners are detected (so we are facing a corner)
  // rotate 45 degrees so we are facing a corridor
  // if (verboseConsole) Serial1.println("Centering with corners...");
  // while(true){
  //   halt();
  //   rotateCW();
  //   delay(150);
  //   halt();
  //   delay(200);
  //   pingToF();
  //   if (tofDistances[0] < 200 && tofDistances[1] < 200 && (tofDistances[0] - tofDistances[1]) < 50){
  //     halt();
  //     rotateCW();
  //     delay(500);
  //     halt();
  //     return;
  //   }
  // }
}

void orient() {
  rotateCW();
  int max = 0;
  for (int i = 0; i <= 26; i++) {
    pingToF();
    if (sensorDistances[0] > max) {
      max = sensorDistances[0];
    }
    delay(100);
  }
  for (int i = 0; i <= 26; i++) {
    pingToF();
    if (max * 0.95 < sensorDistances[0]) {
      halt();
      break;
    }
    delay(100);
  }
}

void sensorChangeRotation(int sensorIndex, unsigned long delayTime=100){
  // using lastSensorDistances and sensorDistances
  // check if they are close to a wall and rotate based on change (in the delay)

  if (lastSensorDistances[sensorIndex] < 150 && sensorDistances[sensorIndex] < 150){

    int change = sensorDistances[sensorIndex] - lastSensorDistances[sensorIndex];
    if (verboseSensors) Serial1.println("Sensor " + String(sensorIndex) + " change: " + String(change));

    if ((change < 0 && sensorIndex==2) || (change > 0 && sensorIndex==6)){
      // getting closer to wall, rotate CCW
      if (verboseSensors) Serial1.println("Rotating CCW to avoid wall.");
      halt();
      rotateCCW();
      delay(delayTime);
      halt();
    }

    else if ((change > 0 && sensorIndex==2) || (change < 0 && sensorIndex==6)){
      // getting away from wall, rotate CW
      if (verboseSensors) Serial1.println("Rotating CW to avoid wall.");
      halt();
      rotateCW();
      delay(delayTime);
      halt();
    }
  }

}

void parallelSensorAdjustment(int direction){
  // direction: 0 = front, 1 = right, 2 = back, 3 = left
  // Ultrasonic sensor orientations
  //       [0][1]
  //   [7]        [2]
  //   [6]        [3]
  //       [5][4]
  int sensor1, sensor2;
  if (direction == 0){
    sensor1 = 7;
    sensor2 = 2;
  }
  else if (direction == 1){
    sensor1 = 1;
    sensor2 = 4;
  }
  else if (direction == 2){
    sensor1 = 3;
    sensor2 = 6;
  }
  else if (direction == 3){
    sensor1 = 5;
    sensor2 = 0;
  }
  int diff = sensorDistances[sensor1] - sensorDistances[sensor2];
  if (verboseConsole) Serial1.println("Parallel adjustment diff: " + String(diff));
  if (abs(diff) > 0 && abs(diff) < 100 && tofDistances[direction] < 100){
    unsigned long delayTime = abs(diff) * 5; // adjust delay time based on difference
    if (diff > 0){
      // sensor1 is farther than sensor2, rotate CW
      if (verboseConsole) Serial1.println("Rotating CW for parallel adjustment.");
      halt();
      rotateCW(1.2);
      delay(delayTime);
      halt();
    }
    else{
      // sensor2 is farther than sensor1, rotate CCW
      if (verboseConsole) Serial1.println("Rotating CCW for parallel adjustment.");
      halt();
      rotateCCW(1.2);
      delay(delayTime);
      halt();
    }
    pingSensors(); // update readings after adjustment
    delay(100);
    if (abs(sensorDistances[sensor1] - sensorDistances[sensor2]) > 20 &&
        abs(sensorDistances[sensor1] - sensorDistances[sensor2]) < 100){
      if (verboseConsole) Serial1.println("Further parallel adjustment needed.");
      parallelSensorAdjustment(direction); // recursive call if still not parallel
    }
  }
}

void changeFrontDirection(int newFront){
  if (tofDistancesReal[newFront] < 100) {
    Serial1.println("Cannot change front direction to " + String(newFront) + " due to obstacle.");
    return;
  }
  frontDirection = newFront % 4;
  if (verboseConsole) Serial1.println("Front direction changed to: " + String(frontDirection));
  pingSensors();
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
  buffer += String(loadToFDistances[0]);
  buffer += ",";
  buffer += String(loadToFDistances[1]);
  buffer += "]";
  Serial1.println(buffer);
}

void transmitSensorData(){
  // Transmit sensor distances over Serial1 in a comma-separated format
  String buffer = "[";
  for (int i = 0; i < 8; i++){
    buffer += String(sensorDistancesReal[i]);
    buffer += ",";
  }
  buffer += String(frontDirection);
  buffer += "]";
  Serial.println(buffer);
  Serial.println(buffer);
  Serial1.println(buffer);
}

void blinkLED(int times, int delayTime){
  for (int i=0; i<times; i++){
    digitalWrite(LEDPin[0], HIGH);
    delay(delayTime);
    digitalWrite(LEDPin[0], LOW);
    delay(delayTime);
  }
}

void actuateServo(int servo, int angle){
  if (servo < 0 || servo >= 2){
    Serial1.println("Invalid servo index: " + String(servo));
    return;
  }
  if (angle < MIN_SERVO_ANGLE[servo] || angle > MAX_SERVO_ANGLE[servo]){
    Serial1.println("Invalid servo angle: " + String(angle));
    return;
  }
  servos[servo].write(angle);
  if (verboseConsole) Serial1.println("Servo " + String(servo) + " moved to angle " + String(angle));
}