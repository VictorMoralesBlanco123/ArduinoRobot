#include <IRremote.h>
#include "Motor.h"
#include <Servo.h>
#include <AccelStepper.h>  // AccelStepper is already included via Motor.h, but included here for clarity if Motor.h is not visible.

// --- CONFIGURATION CONSTANTS ---
const int OBSTACLE_THRESHOLD_CM = 20;
const float MOVE_FORWARD_SCALE = 1.25f;
const float DRIVE_SPEED = 600.0f;
const unsigned long DELAY_AFTER_MOVE_MS = 100;

// Servo positions for scanning
const int SERVO_CENTER = 90;
const int SERVO_LEFT = 160;
const int SERVO_RIGHT = 20;


class Ultrasonic {
public:
  Ultrasonic(int trigPin, int echoPin) {
    setTrigPin(trigPin);
    setEchoPin(echoPin);

    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);

    digitalWrite(_trigPin, LOW);
  }

  long centimeters() {
    long duration = measureDuration();
    if (duration == 0 || duration > 30000) return 300;
    return (duration / 2) / 29.1;
  }

  void setTrigPin(int pin) {
    _trigPin = pin;
  }
  void setEchoPin(int pin) {
    _echoPin = pin;
  }
private:
  int _trigPin;
  int _echoPin;

  long measureDuration() {
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);

    return pulseIn(_echoPin, HIGH, 30000);  // 30ms timeout for a reading
  }
};

// MOTOR INITIALIZATION
Motor leftMotor = Motor(4, 5, 6, 7, false, 2.f);
Motor rightMotor = Motor(A0, A1, A2, A3, true, 2.f);  // Inverted right motor to ensure both move forward when given 'forward' command


// PIN & OBJECT DECLARATIONS
int servoPin = 9;
int ultrasonicTrigger = A4;
int ultrasonicEcho = A5;
int IRPin = 3;

Servo servo;
Ultrasonic sensor(ultrasonicTrigger, ultrasonicEcho);

enum CarState {
  INIT_SCAN,
  MOVING_FORWARD,
  DECIDING_TURN,
  TURNING_LEFT,
  TURNING_RIGHT,
  REVERSING
} state = CarState::INIT_SCAN;


void moveForwardInPlace();
void turnLeft90();
void turnRight90();
long scanDistance(int angle);
void driveMotors();
void runStateMachine();


void setup() {
  Serial.begin(9600);
  servo.attach(servoPin);
  servo.write(SERVO_CENTER);
  IrReceiver.begin(IRPin, true);

  leftMotor.setAcceleration(Motor::DEFAULT_ACCEL);
  rightMotor.setAcceleration(Motor::DEFAULT_ACCEL);
}

void loop() {
  driveMotors();

  if (!leftMotor.distanceToGo() && !rightMotor.distanceToGo()) {
    runStateMachine();
  }
}


void driveMotors() {
  leftMotor.run();
  rightMotor.run();
}


// MOVEMENT FUNCTIONS

void moveForwardInPlace() {
  leftMotor.forward(DRIVE_SPEED, MOVE_FORWARD_SCALE);
  rightMotor.forward(DRIVE_SPEED, MOVE_FORWARD_SCALE);

  state = CarState::MOVING_FORWARD;
}

void turnLeft90() {
  const float TURN_SCALE = 0.5f;

  leftMotor.reverse(DRIVE_SPEED, TURN_SCALE);
  rightMotor.forward(DRIVE_SPEED, TURN_SCALE);

  state = CarState::TURNING_LEFT;

  delay(DELAY_AFTER_MOVE_MS);
}

void turnRight90() {
  const float TURN_SCALE = 0.5f;

  leftMotor.forward(DRIVE_SPEED, TURN_SCALE);
  rightMotor.reverse(DRIVE_SPEED, TURN_SCALE);

  state = CarState::TURNING_RIGHT;

  delay(DELAY_AFTER_MOVE_MS);
}

// SENSOR FUNCTIONS

long scanDistance(int angle) {
  servo.write(angle);
  delay(300);

  long dist = sensor.centimeters();

  return dist;
}


// MAIN MAZE ALGORITHM LOGIC

void runStateMachine() {
  static unsigned long lastMoveCompleteTime = 0;
  unsigned long currentTime = millis();

  // Check if we just finished a movement/turn and need to pause for stability
  if (state != DECIDING_TURN && state != INIT_SCAN) {
    if (currentTime - lastMoveCompleteTime < DELAY_AFTER_MOVE_MS) {
      return;
    }

    state = CarState::DECIDING_TURN;
  }


  switch (state) {
    case INIT_SCAN:
    case DECIDING_TURN:
      {

        // Stop and center the servo for the forward check
        long frontDist = scanDistance(SERVO_Center);

        // Prioritizes moving forward if possible
        if (frontDist > OBSTACLE_THRESHOLD_CM) {
          moveForwardInPlace();
          lastMoveCompleteTime = millis();

        } else {
        
          long rightDist = scanDistance(SERVO_RIGHT);

          if (rightDist > OBSTACLE_THRESHOLD_CM) {
            turnRight90();
            lastMoveCompleteTime = millis();
          } else {
            long leftDist = scanDistance(SERVO_LEFT);

            if (leftDist > OBSTACLE_THRESHOLD_CM) {
              turnLeft90();
              lastMoveCompleteTime = millis();
            } else {
              Serial.println("DEAD END: Turning 180 degrees.");
              turnLeft90();
              lastMoveCompleteTime = millis();
            }
          }
        }
        break;
      }

    case MOVING_FORWARD:
    case TURNING_LEFT:
    case TURNING_RIGHT:
    case REVERSING:
      break;
  }
}
