#define IR_SENSOR_RIGHT 11
#define IR_SENSOR_LEFT 12
#define IR_SENSOR_MIDDLE 13
#define US_SENSOR_ABOVE 3
#define US_SENSOR_BELOW 2
#define SERVO 4
#define MOTOR_SPEED 180
// Line following from the end of SEP to EXIT only.
// No pickup, servo or ultrasonic code.
// Robotics arena sketch, split into separate parts:
//
//   PHASE_PICKUP       -> runPickup()      (your pickup code goes here)
//   PHASE_LINE_FOLLOW  -> runLineFollow()  (line following from the end of SEP to EXIT)
//   PHASE_DONE         -> motors stopped
//
// All hardware (motors, IR sensors, 2 ultrasonic sensors, servo) is set up once in setup().
// To test the line following on its own, set START_PHASE to PHASE_LINE_FOLLOW.

#include <Servo.h>

// =====================================================================
//  PINS
// =====================================================================
// IR line sensors
#define IR_SENSOR_RIGHT   11
#define IR_SENSOR_LEFT    12
#define IR_SENSOR_MIDDLE  13     // needs a third sensor wired here

// Ultrasonic sensors (HC-SR04 style: TRIG output, ECHO input)
#define US_ABOVE_TRIG     3
#define US_ABOVE_ECHO     A1      // pin 0 = RX: don't use Serial, and unplug this wire while uploading
#define US_BELOW_TRIG     2
#define US_BELOW_ECHO     A0

// Servo
#define SERVO_PIN         4

// Motors
int enableRightMotor = 6;
int rightMotorPin1   = 7;
int rightMotorPin2   = 8;

int enableLeftMotor = 5;
int leftMotorPin1   = 9;
int leftMotorPin2   = 10;

// =====================================================================
//  SETTINGS
// =====================================================================
#define MOTOR_SPEED       180
#define SOFT_TURN_SPEED   60     // inner wheel speed for gentle steering
#define LINE_COLOR        HIGH   // IR output over the black line (flip to LOW if inverted)

// Servo angles (adjust to your gripper)
#define SERVO_START_ANGLE 90
#define SERVO_OPEN_ANGLE  90     // TODO set for your gripper
#define SERVO_CLOSED_ANGLE 90    // TODO set for your gripper

// Line-following timing (measure your robot)
#define SPEED_CM_PER_S    25     // distance per second at MOTOR_SPEED
#define LOST_TRAVEL_CM    30     // blind distance before deciding the line has ended
#define SIDE_MEMORY_MS    120    // a side sensor this recent + lost line = corner
#define PIVOT_MAX_MS      1500   // give up searching for the line after this long

const unsigned long LOST_STOP_MS = (unsigned long)LOST_TRAVEL_CM * 1000UL / SPEED_CM_PER_S;

// =====================================================================
//  PHASES
// =====================================================================
enum Phase { PHASE_PICKUP, PHASE_LINE_FOLLOW, PHASE_DONE };

#define START_PHASE PHASE_PICKUP    // use PHASE_LINE_FOLLOW to test line following alone

Phase phase = START_PHASE;

Servo myServo;

// Line-following state
int  lastSide = 0;                // -1 line last seen on left, +1 on right
unsigned long lastSideMs = 0;
bool lost = false;
bool pivoting = false;
unsigned long lostSince = 0;

// =====================================================================
//  SETUP: initialise everything
// =====================================================================
void setup()
{
  // Leave TCCR0B untouched so millis() and pulseIn() timing stay correct.

  // Motors
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  // IR sensors
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  pinmode(IR_SENSOR_MIDDLE, INPUT);
  pinmode(US_SENSOR_ABOVE, INPUT);
  pinmode(US_SENSOR_BELOW, INPUT);
  pinmode(SERVO, OUTPUT);
  
  rotateMotor(0,0);   
}
  pinMode(IR_SENSOR_MIDDLE, INPUT);

  // Ultrasonic sensors
  pinMode(US_ABOVE_TRIG, OUTPUT);
  pinMode(US_ABOVE_ECHO, INPUT);
  pinMode(US_BELOW_TRIG, OUTPUT);
  pinMode(US_BELOW_ECHO, INPUT);

  // Servo
  myServo.attach(SERVO_PIN);
  myServo.write(SERVO_START_ANGLE);

  rotateMotor(0, 0);

  if (phase == PHASE_LINE_FOLLOW)
  {
    startLineFollow();
  }
}

// =====================================================================
//  MAIN LOOP: just runs whichever phase we are in
// =====================================================================
void loop()
{
  // The Sensors are place so that the Line Just fits between them
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int middle = digitalRead(IR_SENSOR_MIDDLE);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

  int above_ultrasonic_sensor = digitalRead(US_SENSOR_ABOVE);
  int below_ultrasonic_sensor = digitalRead(US_SENSOR_BELOW);

  //The Sensors are not seeing any black so we are sure the sensors are aligned on either side of the line and so the car is aligned to the line
  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
  {
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
  }
  //Only the Right Sensor Detects black so the car has swerved a bit too much to the left so we must turn right
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
  switch (phase)
  {
    case PHASE_PICKUP:
      runPickup();
      startLineFollow();             // pickup finished -> hand over to line following
      phase = PHASE_LINE_FOLLOW;
      break;

    case PHASE_LINE_FOLLOW:
      runLineFollow();
      break;

    case PHASE_DONE:
      rotateMotor(0, 0);
      break;
  }
}

// =====================================================================
//  PART 1: PICKUP
// =====================================================================
// Called once. Return when the cube is picked up and the robot is just past SEP,
// facing the start of the line. Available tools:
//   readDistanceCM(US_ABOVE_TRIG, US_ABOVE_ECHO)
//   readDistanceCM(US_BELOW_TRIG, US_BELOW_ECHO)
//   myServo.write(angle)
//   rotateMotor(rightSpeed, leftSpeed)
void runPickup()
{
  // TODO: write the pickup routine here.
  rotateMotor(0, 0);
}

// =====================================================================
//  PART 2: LINE FOLLOWING (from the end of SEP to EXIT)
// =====================================================================
void startLineFollow()
{
  lastSide = 0;
  lastSideMs = millis();
  lost = false;
  pivoting = false;
  lostSince = 0;
}

// One step of line following. Called repeatedly from loop().
void runLineFollow()
{
  bool right  = (digitalRead(IR_SENSOR_RIGHT)  == LINE_COLOR);
  bool middle = (digitalRead(IR_SENSOR_MIDDLE) == LINE_COLOR);
  bool left   = (digitalRead(IR_SENSOR_LEFT)   == LINE_COLOR);
  unsigned long now = millis();

  if (left || middle || right)
  {
    lost = false;

    // Remember which side the line is on (used when the line vanishes at a corner)
    if (right && !left)      { lastSide = +1; lastSideMs = now; }
    else if (left && !right) { lastSide = -1; lastSideMs = now; }

    if (left && right)
    {
      // Black on both sides: SEP block, diamond vertices, crossings -> straight
      rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
    }
    else if (middle)
    {
      // Middle on the line: go straight, ignore side branches
      rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
    }
    else if (right)
    {
      rotateMotor(SOFT_TURN_SPEED, MOTOR_SPEED);   // steer right
    }
    else
    {
      rotateMotor(MOTOR_SPEED, SOFT_TURN_SPEED);   // steer left
    }
  }
  else
  {
    // No sensor sees the line
    if (!lost)
    {
      lost = true;
      lostSince = now;
      // Side sensor saw the line just before it vanished -> corner, pivot toward it.
      // Otherwise we were centred -> gap or end of line, go straight.
      pivoting = (lastSide != 0) && (now - lastSideMs < SIDE_MEMORY_MS);
    }

    unsigned long lostFor = now - lostSince;

    if (pivoting)
    {
      if (lostFor > PIVOT_MAX_MS)
      {
        finishCourse();                            // could not find the line again
      }
      else if (lastSide > 0)
      {
        rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);    // pivot right
      }
      else
      {
        rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);    // pivot left
      }
    }
    else
    {
      if (lostFor > LOST_STOP_MS)
      {
        finishCourse();                            // line ended: we are in the EXIT box
      }
      else
      {
        rotateMotor(MOTOR_SPEED, MOTOR_SPEED);     // cross the gap
      }
    }
  }
}

void finishCourse()
{
  rotateMotor(0, 0);
  phase = PHASE_DONE;
}

// =====================================================================
//  HELPERS
// =====================================================================
// Distance in cm, or 0 if no echo came back within the timeout
long readDistanceCM(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000UL);
  return duration * 0.034 / 2;
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  }
  else
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  }
  else
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }

  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));
}
