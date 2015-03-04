// ---------------------------------------------------------------------------
// Created by Breazu Adrian - breazuadrian@gmail.com
// using NewPing library and the work of arduino.cc user "Krodal"- june 2012 for Adafruit motor shield
// Copyright 2015 License: GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
// ---------------------------------------------------------------------------

#include <NewPing.h>
#include <Servo.h>

// ultrasonic sensor data
#define trigger  30 
#define echo     31
#define max_distance 400
#define stop_distance 20

// Arduino pins for the shift register
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

// These are used to set the direction of the bridge driver.
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4

// Arduino pins for the PWM signals.
#define MOTOR1_PWM 11
#define MOTOR2_PWM 3
#define SERVO1_PWM 10

// Codes for the motor function.
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4


//define distance variable 
int distance;
int first_distance;
int right_distance;
int left_distance;




// setup ultrasonic sensor using the NewPing library
NewPing sonar(trigger, echo, max_distance);

// Declare classes for Servo connectors of the MotorShield.
Servo servo_1;

// initialize components
void setup() {
	Serial.begin(9600);
  servo_1.attach(SERVO1_PWM);
}

void loop() {
  //stop engine & reset servo
  servo_1.write(90);
  delay(300);

  scan();
  first_distance = distance;
  if  (first_distance > stop_distance) {
    Serial.println("Just go ahead. Tra..lalala...");
    moveForward();
  }
  else {
    Serial.println("Object detected STOP.");
    //if obstacole ahead go back and stop
    moveBack();
    delay(700);
    stopEngines();
  
    //make the read on right using the servo
    Serial.println("Move servo to the right");
    //move servo to the right
    servo_1.write(180);  
    delay(300);
    // make another read on right side
    scan();
    right_distance = distance;

    //make the read on left using the servo
    Serial.println("Move servo to the left");
    //move servo to the left
    servo_1.write(0);  
    delay(300);
    // make another read on left side
    scan();
    left_distance = distance;

    //reset servo to 90
    servo_1.write(90);  
    delay(300);

    //comapre measurements and take the highest
    if (left_distance > right_distance) {
      Serial.println("Go ahead on the left side");
      moveLeft();
      delay(500);
      stopEngines();
      moveForward();
    }
    else {
      Serial.println("Go back on the right side");
      moveRight();
      delay(500);
      stopEngines();
      moveForward();
    }
  }
}

void scan () {
	unsigned int ping_time = sonar.ping();
	distance = ping_time/US_ROUNDTRIP_CM;
        // make sure we have a default distance
        if (distance > 399) {
          distance=399;
        }
	Serial.print("Object detected at : ");
	Serial.print(distance);
	Serial.println();
	delay(100);	
}

void stopEngines() {
  //make sure the motor is stopped
  motor(1,RELEASE,0);
  motor(2,RELEASE,0);
}

void moveBack() {
	//make sure the motor is stopped
	stopEngines();

	// 180 out of 255 backward
	motor(1,BACKWARD,180);
	motor(2,BACKWARD,180);
}

void moveForward() {
	//make sure the motor is stopped
	stopEngines();

	// forward full speed
	motor(1,FORWARD,255);
	motor(2,FORWARD,255);
}

void moveRight() {
	//make sure the motor is stopped
	stopEngines();

	//180 speed with the right motor
	motor(2,BACKWARD,0);
	motor(2,BACKWARD,180);
}

void moveLeft() {
	//make sure the motor is stopped
	stopEngines();

	//180 speed with the left motor
	motor(1,BACKWARD,180);
	motor(2,BACKWARD,0);
}

void motor(int nMotor, int command, int speed)
{
  int motorA, motorB;

  if (nMotor >= 1 && nMotor <= 4)
  {  
    switch (nMotor)
    {
    case 1:
      motorA   = MOTOR1_A;
      motorB   = MOTOR1_B;
      break;
    case 2:
      motorA   = MOTOR2_A;
      motorB   = MOTOR2_B;
      break;
    default:
      break;
    }

    switch (command)
    {
    case FORWARD:
      motor_output (motorA, HIGH, speed);
      motor_output (motorB, LOW, -1);     // -1: no PWM set
      break;
    case BACKWARD:
      motor_output (motorA, LOW, speed);
      motor_output (motorB, HIGH, -1);    // -1: no PWM set
      break;
    case BRAKE:
      // The AdaFruit library didn't implement a brake.
      // The L293D motor driver ic doesn't have a good
      // brake anyway.
      // It uses transistors inside, and not mosfets.
      // Some use a software break, by using a short
      // reverse voltage.
      // This brake will try to brake, by enabling 
      // the output and by pulling both outputs to ground.
      // But it isn't a good break.
      motor_output (motorA, LOW, 255); // 255: fully on.
      motor_output (motorB, LOW, -1);  // -1: no PWM set
      break;
    case RELEASE:
      motor_output (motorA, LOW, 0);  // 0: output floating.
      motor_output (motorB, LOW, -1); // -1: no PWM set
      break;
    default:
      break;
    }
  }
}


// ---------------------------------
// motor_output
//
// The function motor_ouput uses the motor driver to
// drive normal outputs like lights, relays, solenoids, 
// DC motors (but not in reverse).
//
// It is also used as an internal helper function 
// for the motor() function.
//
// The high_low variable should be set 'HIGH' 
// to drive lights, etc.
// It can be set 'LOW', to switch it off, 
// but also a 'speed' of 0 will switch it off.
//
// The 'speed' sets the PWM for 0...255, and is for 
// both pins of the motor output.
//   For example, if motor 3 side 'A' is used to for a
//   dimmed light at 50% (speed is 128), also the 
//   motor 3 side 'B' output will be dimmed for 50%.
// Set to 0 for completelty off (high impedance).
// Set to 255 for fully on.
// Special settings for the PWM speed:
//    Set to -1 for not setting the PWM at all.
//
void motor_output (int output, int high_low, int speed)
{
  int motorPWM;

  switch (output)
  {
  case MOTOR1_A:
  case MOTOR1_B:
    motorPWM = MOTOR1_PWM;
    break;
  case MOTOR2_A:
  case MOTOR2_B:
    motorPWM = MOTOR2_PWM;
    break;
  default:
    // Use speed as error flag, -3333 = invalid output.
    speed = -3333;
    break;
  }

  if (speed != -3333)
  {
    // Set the direction with the shift register 
    // on the MotorShield, even if the speed = -1.
    // In that case the direction will be set, but
    // not the PWM.
    shiftWrite(output, high_low);

    // set PWM only if it is valid
    if (speed >= 0 && speed <= 255)    
    {
      analogWrite(motorPWM, speed);
    }
  }
}


// ---------------------------------
// shiftWrite
//
// The parameters are just like digitalWrite().
//
// The output is the pin 0...7 (the pin behind 
// the shift register).
// The second parameter is HIGH or LOW.
//
// There is no initialization function.
// Initialization is automatically done at the first
// time it is used.
//
void shiftWrite(int output, int high_low)
{
  static int latch_copy;
  static int shift_register_initialized = false;

  // Do the initialization on the fly, 
  // at the first time it is used.
  if (!shift_register_initialized)
  {
    // Set pins for shift register to output
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);

    // Set pins for shift register to default value (low);
    digitalWrite(MOTORDATA, LOW);
    digitalWrite(MOTORLATCH, LOW);
    digitalWrite(MOTORCLK, LOW);
    // Enable the shift register, set Enable pin Low.
    digitalWrite(MOTORENABLE, LOW);

    // start with all outputs (of the shift register) low
    latch_copy = 0;

    shift_register_initialized = true;
  }

  // The defines HIGH and LOW are 1 and 0.
  // So this is valid.
  bitWrite(latch_copy, output, high_low);

  // Use the default Arduino 'shiftOut()' function to
  // shift the bits with the MOTORCLK as clock pulse.
  // The 74HC595 shiftregister wants the MSB first.
  // After that, generate a latch pulse with MOTORLATCH.
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, HIGH);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, LOW);
}
