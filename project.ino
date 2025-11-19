#include "Motor.h"
#include <Servo.h>

class Ultrasonic {
public:
  Ultrasonic(int trigPin, int echoPin) {
    setTrigPin(trigPin);
    setEchoPin(echoPin);

    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);

    digitalWrite(_trigPin, LOW);
  }
  long inches() {
    long duration = measureDuration();
    if (duration == 0) return 0;

    return ((duration / 2) / 74);
  }
  long centimeters() {
    long duration = measureDuration();
    if (duration == 0) return 0;

    return ((duration / 2) / 29.1);
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
    delayMicroseconds(5);

    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);

    return pulseIn(_echoPin, HIGH);
  }
};

Motor leftMotor = Motor(4, 5, 6, 7, false, 2.f);
Motor rightMotor = Motor(A0, A1, A2, A3, false .2.f);

int servoPin = 9;
int ultrasonicTrigger = A4;
int ultrasonicEcho = A5;
int IRPin = 3;

Servo servo;

enum Dir { Stop,
           Forward,
           TurnLeft,
           TurnRight }

Dir move = Dir::Stop;

int speed = 600;
Ultrasonic sensor(A4, A5);

volatile long distance;
unsigned long servoMoveStartTime = 0;
unsigned long servoTravelTimeEstimate = 1000;



void setup() {
  servo.attach(servoPin, 500, 2400);
  IrReceiver.begin(IRPin, true);
}

void loop() {
  if (IrReceiver.decode()) {
    switch (IrReceiver.decodedIRData.command) {
      case 0:  // Start-Stop
        break;
      case 1:  // Forward
        break;
      case 2:  // Backward
        break;
      case 3:  // Turn Left
        break;
      case 4;  // Turn Right
        break;
        default:
        break;
    }
  }
}
