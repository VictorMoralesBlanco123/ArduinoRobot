// Motor.h
// Prof. Allen
// FGCU Computing & Software Engineering
//
// Abstraction of a single 4-pin stepper motor such as the 28BYJ-48 using
// the AccelStepper library to support control of mutliple instances
// of this class.
//
// The motor can be inverted to invert directional control for motors installed in
// reverse or on the other side of a robot or car, and supports a gear ratio that is
// defaulted to 1.0. Forward and reverse movements make whole rotations by default, but
// can be scaled to make for fractional rotations such as quarter 0.25, or half 0.5, etc.
// Speed is expressed in steps-per-second and should be limited to 2000.
// --------------------------------------------------------------------------------

#include <AccelStepper.h>


class Motor : public AccelStepper {
  private:
    bool    _inverted;
    float   _ratio;
  public:

  static const float MAX_SPEED = 2000.f;
  static const float DEFAULT_SPEED = 600.f;
  static const float DEFAULT_ACCEL = 100.f;
  static const float FULL_STEP = 2038.f;

  Motor(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4, bool invert=false, float ratio=1.f) 
      : AccelStepper(AccelStepper::MotorInterfaceType::FULL4WIRE, in1, in3, in2, in4),
        _inverted{invert}, _ratio{ratio} {
    setMaxSpeed(MAX_SPEED);  
  }

  void forward(float speed, float scale=1.f) {
    setCurrentPosition(0);
    float target = FULL_STEP * _ratio;
    if (_inverted)
      target *= -1;
    target *= scale;
    move(target);
    setSpeed(speed);
  }

  void reverse(float speed, float scale=1.f) {
    setCurrentPosition(0);
    float target = -FULL_STEP * _ratio;
    if (_inverted)
      target *= -1;
    target *= scale;
    move(target);
    setSpeed(speed);
  }


}; // Motor

