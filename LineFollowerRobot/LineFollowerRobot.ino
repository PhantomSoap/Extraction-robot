#define IR_SENSOR_RIGHT 11
#define IR_SENSOR_LEFT 12
#define IR_SENSOR_MIDDLE 13
#define US_SENSOR_ABOVE 3
#define US_SENSOR_BELOW 2
#define SERVO 4
#define MOTOR_SPEED 180
// Line following from the end of SEP to EXIT only.
// No pickup, servo or ultrasonic code.
//
// Track features this logic is written for (see arena diagram):
//  - 90 degree right turn right after SEP, then long S-curves down the left side
//  - side branches and a dead-end diagonal (must be ignored, go straight)
//  - a diamond in the line (go straight through it)
//  - 10 cm gaps in the line (drive straight across them)
//  - the line simply ends at the EXIT box (stop inside the box)

#define IR_SENSOR_RIGHT   11
#define IR_SENSOR_LEFT    12
#define IR_SENSOR_MIDDLE  13

#define MOTOR_SPEED       180
#define SOFT_TURN_SPEED   60     // inner wheel speed for gentle corrections
#define LINE_COLOR        HIGH   // IR module output over the black line (flip to LOW if yours is inverted)

// ---------- Tuning (measure your robot!) ----------
#define SPEED_CM_PER_S    25     // how far the robot travels per second at MOTOR_SPEED. Time it over 1 m.
#define LOST_TRAVEL_CM    30     // drive this far with no line before deciding the line has ended (gap is 10 cm,
                                 // the rest carries the robot into the 40 cm deep EXIT box)
#define SIDE_MEMORY_MS    120    // if a side sensor saw the line this recently, a lost line means a corner
#define PIVOT_MAX_MS      1500   // give up searching after this long

const unsigned long LOST_STOP_MS = (unsigned long)LOST_TRAVEL_CM * 1000UL / SPEED_CM_PER_S;

// Right motor
int enableRightMotor = 6;
int rightMotorPin1   = 7;
int rightMotorPin2   = 8;

// Left motor
int enableLeftMotor = 5;
int leftMotorPin1   = 9;
int leftMotorPin2   = 10;

int  lastSide = 0;               // -1 = line was last seen on the left, +1 = on the right
unsigned long lastSideMs = 0;    // when a side sensor last saw the line
bool lost = false;
bool pivoting = false;
unsigned long lostSince = 0;

void setup()
{
  // Leave TCCR0B untouched so millis() keeps correct timing.
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  pinmode(IR_SENSOR_MIDDLE, INPUT);
  pinmode(US_SENSOR_ABOVE, INPUT);
  pinmode(US_SENSOR_BELOW, INPUT);
  pinmode(SERVO, OUTPUT);
  
  rotateMotor(0,0);   
}
  pinMode(IR_SENSOR_MIDDLE, INPUT);

  rotateMotor(0, 0);
}

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
  bool right  = (digitalRead(IR_SENSOR_RIGHT)  == LINE_COLOR);
  bool middle = (digitalRead(IR_SENSOR_MIDDLE) == LINE_COLOR);
  bool left   = (digitalRead(IR_SENSOR_LEFT)   == LINE_COLOR);
  unsigned long now = millis();

  if (left || middle || right)
  {
    lost = false;

    // Remember which side the line is on (used when the line disappears at a corner)
    if (right && !left)      { lastSide = +1; lastSideMs = now; }
    else if (left && !right) { lastSide = -1; lastSideMs = now; }

    if (left && right)
    {
      // Black on both sides: start area (SEP), diamond vertices, crossings -> straight
      rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
    }
    else if (middle)
    {
      // Middle on the line. Ignore side branches and keep going straight.
      rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
    }
    else if (right)
    {
      // Only right sensor sees the line: steer right (right wheel slower)
      rotateMotor(SOFT_TURN_SPEED, MOTOR_SPEED);
    }
    else
    {
      // Only left sensor sees the line: steer left (left wheel slower)
      rotateMotor(MOTOR_SPEED, SOFT_TURN_SPEED);
    }
  }
  else
  {
    // No sensor sees the line
    if (!lost)
    {
      lost = true;
      lostSince = now;
      // A side sensor saw the line just before it vanished -> sharp corner, pivot toward it.
      // Otherwise we were centred -> it is a gap (or the end of the line), go straight.
      pivoting = (lastSide != 0) && (now - lastSideMs < SIDE_MEMORY_MS);
    }

    unsigned long lostFor = now - lostSince;

    if (pivoting)
    {
      if (lostFor > PIVOT_MAX_MS)
      {
        stopForever();
      }
      else if (lastSide > 0)
      {
        rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);   // pivot right
      }
      else
      {
        rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);   // pivot left
      }
    }
    else
    {
      if (lostFor > LOST_STOP_MS)
      {
        stopForever();                            // line ended: we are in the EXIT box
      }
      else
      {
        rotateMotor(MOTOR_SPEED, MOTOR_SPEED);    // cross the gap
      }
    }
  }
}

void stopForever()
{
  rotateMotor(0, 0);
  while (true) { }
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
