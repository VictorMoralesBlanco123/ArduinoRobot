#include "Motor.h"
#include <Servo.h>
#include <IRremote.h>
#include <AccelStepper.h>

// -------- PIN ASSIGNMENTS -------------
const int SERVO_PIN = 9;
const int TRIG_PIN = A4;
const int ECHO_PIN = A5;
const int IR_PIN = 3;

// Motor setup 
Motor leftMotor = Motor(4, 5, 6, 7, false, 2.f);
Motor rightMotor = Motor(A0, A1, A2, A3, true, 2.f);

enum Dir { 
  Stop,
  Forward,
  TurnLeft,
  TurnRight 
};

Dir move = Dir::Stop;
bool moving = false;

int speed = 400; 

long lastSensorCheck = 0; 

float moveScale = 1.0;

void setup() {
  Serial.begin(9600);
  Serial.println("Robot initializing...");
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  randomSeed(analogRead(A7));
}

// Helper function to get distance in CM
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Timeout 5000 micros to prevent blocking
  long duration = pulseIn(ECHO_PIN, HIGH, 5000); 
  
  if (duration == 0) return 999; 
  return duration * 0.034 / 2;
}

void loop() {

  // ---------------------------------------------
  // 1. DECISION PHASE (Only happens when stopped)
  // ---------------------------------------------
  if (!moving) {
    
    long distance = getDistance();
    
    Serial.print("Dist: "); Serial.print(distance);
    
    Dir next;

    // --- OBSTACLE LOGIC (< 20cm) ---
    if (distance < 20) {
      Serial.println(" | OBSTACLE! Scanning...");
      
      // Use 0.2 scale (approx 1/5 turn) to scan instead of spin
      moveScale = 0.2; 
      
      // Randomly turn Left or Right to find a path
      next = Dir(random(2, 4)); 
    } 
    // --- PATH CLEAR LOGIC ---
    else {
      Serial.println(" | Path Clear.");
      
      // Use full scale (1.0) for normal movement
      moveScale = 1.0; 
      
      int choice = random(0, 10);
      if (choice < 8) next = Dir::Forward;
      else if (choice == 8) next = Dir::TurnLeft;
      else next = Dir::TurnRight;
    }

    if (next != move) delay(200);
    move = next;

    // Execute the setup for the motors
    switch (move) {
      case Dir::Stop:
        leftMotor.stop();
        rightMotor.stop();
        moving = false; 
        break;
      
      case Dir::Forward:
        Serial.println(" -> Moving Forward");
        leftMotor.forward(speed, moveScale);
        rightMotor.forward(speed, moveScale);
        moving = true;
        break;
      
      case Dir::TurnLeft:
        Serial.println(" -> Turning Left");
        leftMotor.reverse(speed, moveScale);
        rightMotor.forward(speed, moveScale);
        moving = true;
        break;
        
      case Dir::TurnRight:
        Serial.println(" -> Turning Right");
        leftMotor.forward(speed, moveScale);
        rightMotor.reverse(speed, moveScale);
        moving = true;
        break;
    }
  }

  // ---------------------------------------------
  // 2. EXECUTION PHASE (Motors are running)
  // ---------------------------------------------
  if (moving) {
    leftMotor.runSpeedToPosition();
    rightMotor.runSpeedToPosition();

    // --- SAFETY CHECK ---
    if (move == Dir::Forward && moveScale > 0.5 && (millis() - lastSensorCheck > 50)) {
      lastSensorCheck = millis();
      
      if (getDistance() < 15) {
        Serial.println("EMERGENCY STOP!");
        leftMotor.setCurrentPosition(leftMotor.currentPosition());
        rightMotor.setCurrentPosition(rightMotor.currentPosition());
        moving = false; 
      }
    }

    // Check if motors reached their target
    if (leftMotor.distanceToGo() == 0 && rightMotor.distanceToGo() == 0) {
      moving = false;
      delay(100); 
    }
  }
}
