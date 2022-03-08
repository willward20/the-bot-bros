/*
Speed measuring uses high frequency hardware timer 1Hz == 1ms) to measure the time from of one rotation, in ms
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
const byte leftEncA = 2;  // D2 reads left encoder's Ch.A
const byte leftEncB = 3;  // D3 reads left encoder's Ch.B
const byte rightEncA = 8; // D8 reads right encoder's Ch.A
const byte rightEncB = 9; // D9 reads right encoder's Ch.B
const byte leftVcc = A5;  // A5 serves as Vcc for left encoder
const byte rightVcc = A6;  // A6 serves as Vcc for right encoder

// Define constans for robot
const byte COUNTS_PER_REV = 64;
const float GEAR_RATIO = 70;
const float WHEEL_RADIUS = 0.04215; // m
const float WHEEL_SEPARATION = 0.3885; // m

// Define variables
int32_t leftCounter = 0;
int32_t rightCounter = 0;
float leftCPS = 0.0;  // counts per second
float rightCPS = 0.0;
float leftWhlSpd = 0.0; // radians per second
float rightWhlSpd = 0.0;

void TimerHandler1(void)
{
  leftCPS = leftCounter * 1000 / TIMER1_INTERVAL_MS; // counts per second
  rightCPS = rightCounter * 1000 / TIMER1_INTERVAL_MS;
  leftWhlSpd = leftCPS / (COUNTS_PER_REV * GEAR_RATIO) * (2 * M_PI); // WHEEL: radians per second
  rightWhlSpd = rightCPS / (COUNTS_PER_REV * GEAR_RATIO) * (2 * M_PI);
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
  // Set all encoder pins to inputs
  pinMode(leftEncA, INPUT);
  pinMode(leftEncB, INPUT);
  pinMode(rightEncA, INPUT);
  pinMode(rightEncB, INPUT);
  pinMode(leftVcc, OUTPUT);
  pinMode(rightVcc, OUTPUT);
  digitalWrite(leftVcc, HIGH);
  digitalWrite(rightVcc, HIGH);

  Serial.begin(9600);
  while (!Serial)
    ;
  ITimer1.init(); // set timer for 1sec
  ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, TimerHandler1);

  // Attach interrupt pins for reading encoders
  attachInterrupt(digitalPinToInterrupt(leftEncA), ISR_countLeftEncA, CHANGE); // Increase left counter A when speed sensor pin changes
  attachInterrupt(digitalPinToInterrupt(leftEncB), ISR_countLeftEncB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncA), ISR_countRightEncA, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(rightEncB), ISR_countRightEncB, CHANGE);

  // Initialize counter
  leftCounter = 0;
  rightCounter = 0;
}

void loop()
{
  Serial.print(leftWhlSpd); // radians per second
  Serial.print(",");
  Serial.println(rightWhlSpd);

  delay(10);
}
