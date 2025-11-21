#include <NewPing.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <Wire.h>
#include <Servo.h>

void resetToF(bool initializeLoadSensors);
void additionalSetup();
void commandTimeoutCheck();
void controlFromSerial();
void trueForward(float speedDivisor = 1.0);
void trueBackwards(float speedDivisor = 1.0);
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
void pingToF(int numTimes = 5);
void pingLoadToF(int numTimes = 5);
void changeFrontDirection(int newDirection);
void transmitToFData();
void transmitLoadToFData();
void actuateServo(int servo, int angle);

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
float speeds[4]={75.0,50.0,55.0,70.0};

// Sensors orientation:
//     [0]
// [3]     [1]
//     [2]
VL53L0X sensors[4];
int tofPins[4] = {2, 3, 4, 5}; // XSHUT pins for the 4 ToF sensors
long lastToFDistances[4];
long tofDistances[4]; // distances from ToF sensors
long tofDistancesReal[4]; // Used for omniwheel drive where the 'front' is rotated

VL53L0X loadSensorTop; // Pin 6
VL53L1X loadSensorBottom; // Pin 7
int loadToFPins[2] = {6, 7}; // XSHUT pins for the 2 Load ToF sensors
long lastLoadToFDistances[2];
long loadToFDistances[2]; // distances from Load ToF sensors
bool loadSensorsInitialized = false;

int switchPin = 33; // Digital Input pin for Load Detection Switch
int resetPin = 31;  // Digital Input pin for Reset Switch

Servo servos[2];
int servoPins[2] = {8, 9}; // 8 - base, 9 - gripper
int MAX_SERVO_ANGLE[2] = {120, 120};
int MIN_SERVO_ANGLE[2] = {10, 0};

int frontDirection = 0; // 0: front, 1: right, 2: back, 3: left

// For control over serial
char val = 0; 
unsigned long commandDuration = 0;
unsigned long lastCommandTime = 0;
unsigned long commandTimeout = 5000; // milliseconds