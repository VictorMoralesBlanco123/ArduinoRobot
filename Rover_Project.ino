#include "Motor.h"
#include <Servo.h>
#include <IRremote.h>

// ------------ CONFIGURATION ------------
const int SERVO_PIN = 9;

const int TRIG_PIN  = A4;
const int ECHO_PIN  = A5;

const int IR_PIN    = 3;

const int OBSTACLE_THRESHOLD_CM = 20;   // wall distance
const float DRIVE_SPEED         = 600.f;

// Forward move and turn size (tune these on the real robot)
const float MOVE_FORWARD_SCALE  = 2.0f;   // bigger = longer forward segment
const float TURN_SCALE          = 0.75f;  // bigger = larger turn

// Servo scan angles
const int SERVO_CENTER = 90;
const int SERVO_LEFT   = 160;
const int SERVO_RIGHT  = 20;

// ------------ OBJECTS ------------
Motor leftMotor  = Motor(4, 5, 6, 7, false, 2.f);
Motor rightMotor = Motor(A0, A1, A2, A3, true,  2.f);

Servo servo;

// simple ultrasonic helper
long getDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30 ms timeout
  if (duration == 0) return 300;

  return (duration / 2) / 29.1;
}

long scanAt(int angle) {
  servo.write(angle);
  delay(250);
  return getDistanceCM();
}

// robot state
enum CarState { INIT_SCAN, MOVING, DECIDING_TURN };
CarState state = INIT_SCAN;

// global pause flag controlled by IR
bool paused = false;

// ------------ MOVEMENT HELPERS ------------
void stopMotors() {
  leftMotor.stop();
  rightMotor.stop();
}

void moveForward() {
  leftMotor.forward(DRIVE_SPEED, MOVE_FORWARD_SCALE);
  rightMotor.forward(DRIVE_SPEED, MOVE_FORWARD_SCALE);
  state = MOVING;
  Serial.println("AUTO: Forward");
}

void turnLeft() {
  leftMotor.reverse(DRIVE_SPEED, TURN_SCALE);
  rightMotor.forward(DRIVE_SPEED, TURN_SCALE);
  state = MOVING;
  Serial.println("AUTO: Turn left");
}

void turnRight() {
  leftMotor.forward(DRIVE_SPEED, TURN_SCALE);
  rightMotor.reverse(DRIVE_SPEED, TURN_SCALE);
  state = MOVING;
  Serial.println("AUTO: Turn right");
}

void goBackward() {
  leftMotor.reverse(DRIVE_SPEED, MOVE_FORWARD_SCALE * 0.7f);
  rightMotor.reverse(DRIVE_SPEED, MOVE_FORWARD_SCALE * 0.7f);
  state = MOVING;
  Serial.println("AUTO: Reverse (dead end)");
}

// ------------ SETUP ------------
void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  servo.attach(SERVO_PIN);
  servo.write(SERVO_CENTER);

  IrReceiver.begin(IR_PIN, true);   // just for pause/resume

  Serial.println("ROBOT READY (AUTO mode). Any IR button = pause/resume.");
}

// ------------ MAIN AUTO LOGIC ------------
void runAutoStateMachine() {
  // only called when not paused and current move finished
  servo.write(SERVO_CENTER);
  delay(150);

  long front = getDistanceCM();
  Serial.print("Front distance: ");
  Serial.println(front);

  if (front > OBSTACLE_THRESHOLD_CM) {
    moveForward();
    return;
  }

  Serial.println("Obstacle ahead – scanning left/right...");

  long right = scanAt(SERVO_RIGHT);
  long left  = scanAt(SERVO_LEFT);
  servo.write(SERVO_CENTER);

  if (right > OBSTACLE_THRESHOLD_CM) {
    turnRight();
  } else if (left > OBSTACLE_THRESHOLD_CM) {
    turnLeft();
  } else {
    // dead end
    goBackward();
  }
}

// ------------ LOOP ------------
void loop() {
  // keep steppers moving whenever they have a target
  leftMotor.run();
  rightMotor.run();

  // ----- IR pause/resume -----
  if (IrReceiver.decode()) {
    // any received IR signal toggles pause
    paused = !paused;

    if (paused) {
      Serial.println("IR: PAUSE – robot stopped.");
      stopMotors();
    } else {
      Serial.println("IR: RESUME – robot back to AUTO.");
      state = INIT_SCAN;  // force a new scan
    }

    IrReceiver.resume();
  }

  if (paused) {
    return;  // nothing else while paused
  }

  // If current motion finished, decide next action
  if (leftMotor.distanceToGo() == 0 && rightMotor.distanceToGo() == 0) {
    runAutoStateMachine();
  }
}
