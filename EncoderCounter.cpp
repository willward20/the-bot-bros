/*
Speed measuring uses hardware timer 20Hz, to measure the count per second in encoders.
Refer to "Define encoder pins" section for wiring.
Install "TimerInterrupt_Generic" library to run this code. Library website: https://gitssh ubuntu@192.168.0.145hub.com/khoih-prog/TimerInterrupt_Generic 
*/

// These define's must be placed at the beginning before #include "megaAVR_TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#include <Arduino.h>
#include <math.h>

#define TIMER_INTERRUPT_DEBUG 0
#define _TIMERINTERRUPT_LOGLEVEL_ 0
#define USING_16MHZ true // ATMega4809
#define USING_8MHZ false
#define USING_250KHZ false

#define USE_TIMER_0 false
#define USE_TIMER_1 true // enable ITimer1
#define USE_TIMER_2 false
#define USE_TIMER_3 false

#define TIMER1_INTERVAL_MS 50 // 1s = 1000ms

#include "TimerInterrupt_Generic.h"

// Define encoder pins
const byte leftEncA = 2;  // D2 reads left encoder's Ch.A (yellow)
const byte leftEncB = 3;  // D3 reads left encoder's Ch.B (white)
const byte rightEncA = 8; // D8 reads right encoder's Ch.A (yellow)
const byte rightEncB = 9; // D9 reads right encoder's Ch.B (white)
const byte leftVcc = A5;  // A5 serves as Vcc for left encoder
const byte rightVcc = A6;  // A6 serves as Vcc for right encoder

// Define constans for robot
const byte COUNTS_PER_REV = 64; 
const float GEAR_RATIO = 70; 
const float WHEEL_RADIUS = 0.04215; // m
const float WHEEL_SEPARATION = 0.3885; // m

// Define variables
int8_t leftMotorDir = 0; // forward: 1; backward: -1         //this line was commented out
int8_t rightMotorDir = 0; // forward: 1; backward: -1        //this line was commented out
int32_t leftCounter = 0;
int32_t rightCounter = 0;
float leftCPS = 0.0;
float rightCPS = 0.0;
float linear = 0.0; 
float angular = 0.0; 

void TimerHandler1(void)
{
  leftCPS = leftMotorDir * leftCounter * 1000 / TIMER1_INTERVAL_MS; // counts per second
  rightCPS = rightMotorDir * rightCounter * 1000 / TIMER1_INTERVAL_MS;
  linear = (leftCPS + rightCPS) / (COUNTS_PER_REV * GEAR_RATIO) * (2 * M_PI) * WHEEL_RADIUS / 2; // meters per second
  angular = (rightCPS - leftCPS) / (COUNTS_PER_REV * GEAR_RATIO) * (2 * M_PI) * WHEEL_RADIUS / WHEEL_SEPARATION; // radians per second
  if (TIMER_INTERRUPT_DEBUG > 1)
  {
    Serial.print(leftCPS);
    Serial.print(",");
    Serial.println(rightCPS);
  }
  leftCounter = 0;
  rightCounter = 0;
}

void ISR_countLeftEncA(void)
{
  leftCounter++;
}

void ISR_countLeftEncB(void)
{
  leftCounter++;
}

void ISR_countRightEncA(void)
{
  rightCounter++;
}

void ISR_countRightEncB(void)
{
  rightCounter++;
}

void setup()
{
  // Set pins to power up encoders
  pinMode(leftVcc, OUTPUT);
  pinMode(rightVcc, OUTPUT);
  digitalWrite(leftVcc, HIGH); // sets the analog pin A5 on   ***I added these two lines. Otherwise, the encoders aren't turned on
  digitalWrite(rightVcc, HIGH); // sets the analog pin A6 on
  
  // Set all encoder pins to inputs
  pinMode(leftEncA, INPUT);
  pinMode(leftEncB, INPUT);
  pinMode(rightEncA, INPUT);
  pinMode(rightEncB, INPUT);

  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.print(F("\nStarting ISR_RPM_Measure on "));
  Serial.println(BOARD_NAME);
  Serial.println(MEGA_AVR_TIMER_INTERRUPT_VERSION);
  Serial.println(TIMER_INTERRUPT_GENERIC_VERSION);
  Serial.print(F("CPU Frequency = "));
  Serial.print(F_CPU / 1000000);
  Serial.println(F(" MHz"));
  Serial.print(F("TCB Clock Frequency = "));

  ITimer1.init(); // set timer for 1sec
  if (ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, TimerHandler1))
  {
    Serial.print(F("Starting  ITimer1 OK, millis() = "));
    Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer1. Select another freq. or timer"));

  // Assumming the interruptPin will go LOW
  attachInterrupt(digitalPinToInterrupt(leftEncA), ISR_countLeftEncA, CHANGE); // Increase left counter A when speed sensor pin changes
  attachInterrupt(digitalPinToInterrupt(leftEncB), ISR_countLeftEncB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncA), ISR_countRightEncA, CHANGE); // Increase left counter A when speed sensor pin changes
  attachInterrupt(digitalPinToInterrupt(rightEncB), ISR_countRightEncB, CHANGE);

  // Initialize counter
  leftCounter = 0;
  rightCounter = 0;
  leftMotorDir = 1;  // CAREFUL!!! NEED GET ACTUAL DIR 
  rightMotorDir = 1;  //CAREFUL!!! NEED GET ACTUAL DIR
}

void loop()
{
  // Print speed
  // Serial.print(leftCPS);
  // Serial.print(",");
  // Serial.println(rightCPS);
  // Serial.print("left cps: ");
  // Serial.print(leftCPS);
  // Serial.print(", right cps: ");
  // Serial.print(rightCPS);
  Serial.print("linear: ");
  Serial.print(linear);
  Serial.print("angular: ");
  Serial.print(",");
  Serial.println(angular);
  delay(10);
}
