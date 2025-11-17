#include <NewPing.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <Wire.h>
#include <Servo.h>

#define MAX_DISTANCE 200

void resetToF(bool initializeLoadSensors);
void additionalSetup();
void manualControl();
void serialTest();
void controlFromSerial();
void obstacleAvoidance();
void forward(float speedDivisor = 1.0);
void backwards(float speedDivisor = 1.0);
void left(float speedDivisor = 1.0);
void right(float speedDivisor = 1.0);
void frontRight(float speedDivisor = 1.0);
void frontLeft(float speedDivisor = 1.0);
void backRight(float speedDivisor = 1.0);
void backLeft(float speedDivisor = 1.0);
void rotateCW(float speedDivisor = 1.0);
void rotateCCW(float speedDivisor = 1.0);
void halt();
void pingSensors(int numTimes = 5);
void pingToF(int numTimes = 5);
void pingLoadToF(int numTimes = 5);
void pingFrontToF();
void centering();
void sensorChangeRotation(int sensorIndex, unsigned long delayTime=100);
void parallelSensorAdjustment(int sensorIndex);
void changeFrontDirection(int newDirection);
void transmitSensorData();
void transmitToFData();
void transmitLoadToFData();
void blinkLED(int times, int delayTime);
void orient();
void actuateServo(int servo, int angle);

// Starting from front left, going clockwise
// Ultrasonic Sensor 1 pins
int TRIG_PIN1 = 22; //Digital
int ECHO_PIN1 = 24;  //Digital

// Ultrasonic Sensor 2 pins
int TRIG_PIN2 = 26;  //Digital
int ECHO_PIN2 = 28;  //Digital

// Ultrasonic Sensor 3 pins
int TRIG_PIN3 = 30;  //Digital
int ECHO_PIN3 = 32;  //Digital

// Ultrasonic Sensor 4 pins
int TRIG_PIN4 = 34;  //Digital
int ECHO_PIN4 = 36;  //Digital

// Ultrasonic Sensor 5 pins
int TRIG_PIN5 = 38;  //Digital
int ECHO_PIN5 = 40;  //Digital

// Ultrasonic Sensor 6 pins
int TRIG_PIN6 = 42;  //Digital
int ECHO_PIN6 = 44; //Digital

// Ultrasonic Sensor 7 pins
int TRIG_PIN7 = 46; //Digital
int ECHO_PIN7 = 48; //Digital

// Ultrasonic Sensor 8 pins
int TRIG_PIN8 = 50; //Digital
int ECHO_PIN8 = 52; //Digital

// sets up the TRIG_PIN and ECHO_PIN
// Starting from front left, going clockwise
NewPing sonar1(TRIG_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing sonar2(TRIG_PIN2, ECHO_PIN2, MAX_DISTANCE);
NewPing sonar3(TRIG_PIN3, ECHO_PIN3, MAX_DISTANCE);
NewPing sonar4(TRIG_PIN4, ECHO_PIN4, MAX_DISTANCE);
NewPing sonar5(TRIG_PIN5, ECHO_PIN5, MAX_DISTANCE);
NewPing sonar6(TRIG_PIN6, ECHO_PIN6, MAX_DISTANCE);
NewPing sonar7(TRIG_PIN7, ECHO_PIN7, MAX_DISTANCE);
NewPing sonar8(TRIG_PIN8, ECHO_PIN8, MAX_DISTANCE);
NewPing sonars[8] = {sonar1, sonar2, sonar3, sonar4, sonar5, sonar6, sonar7, sonar8};
// Ultrasonic sensor orientations
//       [0][1]
//   [7]        [2]
//   [6]        [3]
//       [5][4]

unsigned long pingTimes[8];
unsigned long lastSensorDistances[8];
unsigned long sensorDistances[8];
int sensorDistancesReal[8]; // Used for omniwheel drive where the 'front' is rotated

int LEDPin[4] = {24,26,28,30};

// Motor 1 pins
int In1A=43; //Digital
int In2A=41; //Digital
int EnM1A=13; //PWM

// Motor 2 pins
int In3A=39; //Digital
int In4A=37; //Digital
int EnM2A=12; //PWM

// Motor 3 pins
int In1B=51; //Digital
int In2B=49; //Digital
int EnM3B=11; //PWM

// Motor 4 pins
int In3B=47; //Digital
int In4B=45; //Digital
int EnM4B=10; //PWM

// int speeds[12]={93,86,77,75,159,154,150,153,225,225,225,225};
float speeds[12]={75.0,50.0,55.0,70.0,159.0,154.0,150.0,153.0,225.0,225.0,225.0,225.0};
int shift=0;

VL53L0X sensors[4];
int tofPins[4] = {2, 3, 4, 5}; // XSHUT pins for the 4 ToF sensors
long lastToFDistances[4];
long tofDistances[4]; // distances from ToF sensors
long tofDistancesReal[4]; // Used for omniwheel drive where the 'front' is rotated
// Sensors orientation:
//     [0]
// [3]     [1]
//     [2]
VL53L0X loadSensorTop; // Pin 6
VL53L1X loadSensorBottom; // Pin 7
int loadToFPins[2] = {6, 7}; // XSHUT pins for the 2 Load ToF sensors
long lastLoadToFDistances[2];
long loadToFDistances[2]; // distances from Load ToF sensors
bool loadSensorsInitialized = false;

Servo servos[2];
int servoPins[2] = {8, 9}; // 8 - base, 9 - gripper
int MAX_SERVO_ANGLE[2] = {120, 120};
int MIN_SERVO_ANGLE[2] = {10, 0};

char val = 0;
unsigned long commandDuration = 0;
bool carefulForward = true;

bool MOVELEFTWHENPOSSIBLE = false;
bool MOVERIGHTWHENPOSSIBLE = false;
bool OMNIWHEELDRIVE = true;
int frontDirection = 0; // 0: front, 1: right, 2: back, 3: left

// For control over serial
long lastCommandTime = 0;
long commandTimeout = 5000; // milliseconds