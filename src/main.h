#include <NewPing.h>
#include <VL53L0X.h>
#include <Wire.h>

#define MAX_DISTANCE 200

void additionalSetup();
void obstacleAvoidance();
void forward(long sensorReading = 8000);
void backwards();
void left();
void right();
void frontRight();
void frontLeft(long sensorReading = 8000);
void backRight();
void backLeft();
void rotateCW();
void rotateCCW();
void halt();
void saveLastReadings();
void pingSensors();
void pingToF();
void centering();
void sensorChangeRotation(int sensorIndex, unsigned long delayTime=100);
void parallelSensorAdjustment(int sensorIndex);
void changeFrontDirection(int newDirection);
void transmitSensorData();

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
// Ultrasonic sensor orientations
//       [0][1]
//   [7]        [2]
//   [6]        [3]
//       [5][4]

bool nicePrint = true;
unsigned long pingTimes[8];
unsigned long lastSensorDistances[8];
unsigned long sensorDistances[8];
int sensorDistancesReal[8]; // Used for omniwheel drive where the 'front' is rotated

int LEDPin = 7;

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
float speeds[12]={63,63,70,70,159,154,150,153,225,225,225,225};
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

char val = 0;
// variables to store the number of encoder pulses for each motor

volatile long motCount = 0;

long driveTimeout = 1000; // milliseconds
long lastDriveCommand = 0;

bool MOVELEFTWHENPOSSIBLE = false;
bool MOVERIGHTWHENPOSSIBLE = false;
bool OMNIWHEELDRIVE = true;
int frontDirection = 0; // 0: front, 1: right, 2: back, 3: left

// For control over serial
long lastCommandTime = 0;
long commandTimeout = 5000; // milliseconds