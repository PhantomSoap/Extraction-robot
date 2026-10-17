// Speed calibration WITHOUT a USB cable attached while the robot drives.
//
// The result is saved in the Arduino's EEPROM, which keeps its contents when power is
// removed and even when you upload a new sketch. You read it back afterwards.
//
// HOW TO USE
//  Step 1  RUN_MODE = 1. Upload this sketch, then UNPLUG the USB cable.
//          Put the robot ~120 cm from a flat wall, facing it, and switch the battery on.
//          It waits 6 s, drives at the wall, stops 25 cm before it and saves the speed.
//          The built-in LED then blinks 3 times (success), or blinks rapidly (error).
//  Step 2  Set RUN_MODE = 0, plug in the USB cable, upload, and open Serial Monitor
//          at 115200 baud. It prints the saved speed. Repeat step 1 for more runs.
//
// Each run overwrites the previous one, so write the number down after each run.

#include <EEPROM.h>

#define RUN_MODE      1          // 1 = drive and measure, 0 = just print the saved result

#define MOTOR_SPEED   180        // must match the speed used in the line follower

#define US_TRIG       2
#define US_ECHO       A0         // pin 0 is not used here so Serial works in read mode

#define STOP_DIST_CM  25         // stop when the wall is this close
#define ACCEL_MS      400        // ignore the first part of the run while motors speed up
#define START_DELAY_MS 6000      // time to put the robot down after switching on

#define EEPROM_FLAG_ADDR   10    // 1 byte: 0xA5 means a valid result is stored
#define EEPROM_SPEED_ADDR  0     // 4 bytes: the float speed in cm/s
#define VALID_FLAG         0xA5

// Right motor
int enableRightMotor = 6;
int rightMotorPin1   = 7;
int rightMotorPin2   = 8;

// Left motor
int enableLeftMotor = 5;
int leftMotorPin1   = 9;
int leftMotorPin2   = 10;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

#if RUN_MODE
  runCalibration();
#else
  Serial.begin(115200);
  delay(300);
  if (EEPROM.read(EEPROM_FLAG_ADDR) == VALID_FLAG)
  {
    float speed;
    EEPROM.get(EEPROM_SPEED_ADDR, speed);
    Serial.print("Saved speed: ");
    Serial.print(speed, 1);
    Serial.println(" cm/s");
    Serial.print("Use in the line follower:  #define SPEED_CM_PER_S  ");
    Serial.println((int)(speed + 0.5));
  }
  else
  {
    Serial.println("No saved result yet. Run the test with RUN_MODE 1 first.");
  }
#endif
}

void loop()
{
  // nothing: everything happens in setup(). Press reset to repeat.
}

#if RUN_MODE
void runCalibration()
{
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);

  rotateMotor(0, 0);

  // Clear any old result so a failed run can't be mistaken for a good one
  EEPROM.write(EEPROM_FLAG_ADDR, 0);

  delay(START_DELAY_MS);

  float startDist = readDistanceMedian();
  if (startDist < STOP_DIST_CM + 60)
  {
    errorBlink();   // too close or no echo: need at least ~85 cm of run-up
  }

  unsigned long startMs = millis();
  rotateMotor(MOTOR_SPEED, MOTOR_SPEED);

  bool gotFirstPoint = false;
  unsigned long t1 = 0, t2 = 0;
  float d1 = 0, d2 = 0;

  while (true)
  {
    unsigned long now = millis();
    float d = readDistanceMedian();

    if (d <= 0)
    {
      rotateMotor(0, 0);
      errorBlink();  // lost the echo
    }

    if (!gotFirstPoint && (now - startMs) > ACCEL_MS)
    {
      t1 = now;
      d1 = d;
      gotFirstPoint = true;
    }

    if (d <= STOP_DIST_CM)
    {
      rotateMotor(0, 0);
      t2 = now;
      d2 = d;
      break;
    }
  }

  if (!gotFirstPoint || t2 <= t1 || d1 <= d2)
  {
    errorBlink();
  }

  float speed = (d1 - d2) / ((t2 - t1) / 1000.0);

  EEPROM.put(EEPROM_SPEED_ADDR, speed);
  EEPROM.write(EEPROM_FLAG_ADDR, VALID_FLAG);

  successBlink();
}

void successBlink()
{
  while (true)
  {
    for (int i = 0; i < 3; i++)
    {
      digitalWrite(LED_BUILTIN, HIGH); delay(300);
      digitalWrite(LED_BUILTIN, LOW);  delay(300);
    }
    delay(1500);
  }
}

void errorBlink()
{
  rotateMotor(0, 0);
  while (true)
  {
    digitalWrite(LED_BUILTIN, HIGH); delay(80);
    digitalWrite(LED_BUILTIN, LOW);  delay(80);
  }
}

// One distance reading in cm, or 0 if no echo came back
float readDistanceCM()
{
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);

  long duration = pulseIn(US_ECHO, HIGH, 30000UL);
  return duration * 0.034 / 2.0;
}

// Median of 3 readings, which removes random spikes
float readDistanceMedian()
{
  float a = readDistanceCM();
  float b = readDistanceCM();
  float c = readDistanceCM();

  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
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
#endif
