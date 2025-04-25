/* PneuAccelStepper.h - Library that adds acceleration control to the existing PneumaticStepper library made by Vincent Groenhuis.
   This library allows handling pneumatic stepper motors in the same way as the existing library.
   A setpoint can be set and the stepper will ramp its speed up and down according to a fixed acceleration value,
   up to a maximum frequency.

   When changing the acceleration or the setpoint while the stepper is running, the stepper might overshoot and then go back to reach the setpoint.
   If this is not wanted, change the setpoint and acceleration only in standstill, or set the acceleration value such that no overshoot occurs.

   Parts of this code were copied and adopted from the AccelStepper library (https://www.airspayce.com/mikem/arduino/AccelStepper/index.html) made by Mike McCauley
*/



#ifndef PNEUACCEL_STEPPER_H
#define PNEUACCEL_STEPPER_H

#include "PneumaticStepper.h"

class PneuAccelStepper : public PneumaticStepper {
  public:
    PneuAccelStepper(int nCylinder, bool doubleActing, bool triState = false, int approachDirection = 0, CylinderStrategy cylinderStrategy = CylinderStrategy::ANY_ENGAGE, float frequency = 10, long position = 0, long setpoint = 0, int phaseNr = 0, bool running = true, float hysteresis = 0);

    // Returns a two-cylinder, double-acting stepper with default strategy
    static PneuAccelStepper TwoCylinderAccelStepper;
    // Returns a three-cylinder, single-acting stepper with default strategy
    static PneuAccelStepper ThreeCylinderAccelStepper;


    void setSetpointDouble(double setpoint) override;
    void setSetpoint(long setpoint) override;
    void setFrequency(float frequency)override;
    bool work()override;
    void workUntilNoChange()override;
    void setAcceleration(float acceleration);
    long distanceToGo();
    void printSpeed();
    void printn();
  private:
    unsigned long computeNewSpeed(bool nIncrease);
    typedef enum
    {
      DIRECTION_CCW = -1,  ///< Counter-Clockwise
      DIRECTION_CW  = 1   ///< Clockwise
    } Direction;
    /// The current motos speed in steps per second
    /// Positive is clockwise
    Direction      _direction = DIRECTION_CCW;
    float          _speed;         // Steps per second

    /// The acceleration to use to accelerate or decelerate the motor in steps
    /// per second per second. Must be > 0
    float          _acceleration = 1;

    /// The last step time in microseconds
    //unsigned long  _lastStepTime;

    /// The step counter for speed calculations
    long _n;

    /// Initial step size in microseconds
    float _c0;

    /// Last step size in microseconds
    float _cn;

    /// Min step size in microseconds based on maxSpeed
    float _cmin; // at max speed

    typedef enum {
      OVERSHOOT,
      JUMP_TO_ZERO,
      JUMP_TO_OPPOSITE
    } OvershootType;

    OvershootType overshootEnabled = OVERSHOOT;//JUMP_TO_OPPOSITE;
};


PneuAccelStepper PneuAccelStepper::TwoCylinderAccelStepper = PneuAccelStepper(2, true);
PneuAccelStepper PneuAccelStepper::ThreeCylinderAccelStepper = PneuAccelStepper(3, false);

PneuAccelStepper::PneuAccelStepper(int nCylinder, bool doubleActing, bool triState, int approachDirection, CylinderStrategy cylinderStrategy,
                                   float frequency, long position, long setpoint, int phaseNr, bool running, float hysteresis)
  : PneumaticStepper(nCylinder, doubleActing, triState, approachDirection, cylinderStrategy,
                     frequency, position, setpoint, phaseNr, running, hysteresis)
{
  _speed = 0.0;
  _n = 0;
  _c0 = 0.0;
  _cn = 0.0;
  _cmin = 1.0;
  _acceleration = 0.0;
  setAcceleration(1);       //Change acceleration to 1 (acceleration value should change, otherwise it doesn't work)
  setFrequency(_frequency); // set maximum frequency
}

// Sets a new acceleration value and recalculates where we are on the velocity ramp
void PneuAccelStepper::setAcceleration(float acceleration)
{
  //if (_speed == 0 && distanceToGo() == 0) {
  if (acceleration == 0.0)
    return;
  if (acceleration < 0.0)
    acceleration = -acceleration;
  if (_acceleration != acceleration)
  {
    long distanceTo = distanceToGo();
    long stepsToStop = ((_speed * _speed) / (2.0 * acceleration)); // Equation 16
    if (overshootEnabled != OVERSHOOT && stepsToStop > abs(distanceTo)) {
      _speed = min(sqrt((float)(2.0 * acceleration * (float)abs(distanceTo))), _frequency);
      if (distanceTo < 0)_speed = -_speed;
      _intervalUs =  _cn = abs(1000000.0 / _speed);
      _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
      /*_n = 0;
        _intervalUs = _c0;
        _speed = 0;
      */
    } else {
      // Recompute _n per Equation 17
      _n = _n * (_acceleration / acceleration);
      // New c0 per Equation 7, with correction per Equation 15
    }
    _c0 = 0.676 * sqrt(2.0 / acceleration) * 1000000.0; // Equation 15
    _acceleration = acceleration;
    //computeNewSpeed(false);
  }
}

//Sets the setpoint
void PneuAccelStepper::setSetpointDouble(double setpoint)
{
  PneumaticStepper::setSetpointDouble(setpoint);
  long distanceTo = distanceToGo();
  long stepsToStop = ((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
  if (overshootEnabled != OVERSHOOT && ((distanceTo > 0 && _direction == DIRECTION_CCW) || distanceTo < 0 && _direction == DIRECTION_CW)) {
    //Going in wrong direction, set speed to zero
    if (overshootEnabled == JUMP_TO_ZERO) {
      _n = 0;
      _intervalUs = _c0;
      _speed = 0;
    } else if (overshootEnabled == JUMP_TO_OPPOSITE) {
      _speed = min(sqrt((float)(2.0 * _acceleration * (float)abs(distanceTo))), _frequency);
      if (distanceTo < 0)_speed = -_speed;
      _intervalUs =  _cn = abs(1000000.0 / _speed);
      _direction = _direction == DIRECTION_CW ? DIRECTION_CCW : DIRECTION_CW;
      _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
    }

  } else if (overshootEnabled != OVERSHOOT && stepsToStop > abs(distanceTo)) {
    _speed = min(sqrt((float)(2.0 * _acceleration * (float)abs(distanceTo))), _frequency);
    if (distanceTo < 0)_speed = -_speed;
    _intervalUs =  _cn = abs(1000000.0 / _speed);
    _direction = _direction == DIRECTION_CW ? DIRECTION_CCW : DIRECTION_CW;
    _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
    /*_n = 0;
      _intervalUs = _c0;
      _speed = 0;
    */
  }

  if (_speed == 0) {
    _direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
  }
  /*
    Serial.println("Overwrote everything:");
    Serial.println(_speed);
    Serial.println(_intervalUs);
    Serial.println(_direction);
    Serial.println(_n);
    Serial.println(_cn);
  */
}

void PneuAccelStepper::setSetpoint(long setpoint)
{
  PneumaticStepper::setSetpoint(setpoint);
  long distanceTo = distanceToGo();
  long stepsToStop = ((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
  if (overshootEnabled != OVERSHOOT && ((distanceTo > 0 && _direction == DIRECTION_CCW) || distanceTo < 0 && _direction == DIRECTION_CW)) {
    //Going in wrong direction, set speed to zero
    if (overshootEnabled == JUMP_TO_ZERO) {
      _n = 0;
      _intervalUs = _c0;
      _speed = 0;
    } else if (overshootEnabled == JUMP_TO_OPPOSITE) {
      _speed = min(sqrt((float)(2.0 * _acceleration * (float)abs(distanceTo))), _frequency);
      if (distanceTo < 0)_speed = -_speed;
      _intervalUs =  _cn = abs(1000000.0 / _speed);
      _direction = _direction == DIRECTION_CW ? DIRECTION_CCW : DIRECTION_CW;
      _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
    }

  } else if (overshootEnabled != OVERSHOOT && stepsToStop > abs(distanceTo)) {
    _speed = min(sqrt((float)(2.0 * _acceleration * (float)abs(distanceTo))), _frequency);
    if (distanceTo < 0)_speed = -_speed;
    _intervalUs =  _cn = abs(1000000.0 / _speed);
    _direction = _direction == DIRECTION_CW ? DIRECTION_CCW : DIRECTION_CW;
    _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
    /*_n = 0;
      _intervalUs = _c0;
      _speed = 0;
    */
  }

  if (_speed == 0) {
    _direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
  }
  /*
    Serial.println("Overwrote everything:");
    Serial.println(_speed);
    Serial.println(_intervalUs);
    Serial.println(_direction);
    Serial.println(_n);
    Serial.println(_cn);
  */
}

void PneuAccelStepper::setFrequency(float frequency)
{
  _frequency = frequency;
  _cmin = 1000000 / frequency;
  // Recompute _n from current speed and adjust speed if accelerating or cruising
  if (_n > 0)
  {
    _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
    //computeNewSpeed(false);
  }
}


bool PneuAccelStepper::work()
{
  unsigned long timeUs = micros();
  bool doStep = false;

  // Do a step if: not at setpoint, running and enough time has elapsed
  if (_running) {
    // check if motor is at setpoint in correct direction
    bool atFinalSetpoint = true;
    if (_position != _setpoint) {
      atFinalSetpoint = false;
    } else {
      if (_approachDirection != 0) {
        if (_lastStepDir != _approachDirection) {
          // must do a step in -_approachDirection
          atFinalSetpoint = false;
        }
      } if (abs(_speed) >= 0.1) {  // Speed is non-zero, might have overshoot
        atFinalSetpoint = false;
      }
    }

    if (!atFinalSetpoint) {
      if (_frequency < 0) {
        doStep = true;
      } else if (_frequency > 0) {
        //_intervalUs = (unsigned long)(1000000.0 / _frequency);
        // Test if enough time has elapsed since last change
        unsigned long elapsedUs = timeUs - _lastChangeUs;
        if (elapsedUs >= _intervalUs) {
          doStep = true;
        }
      }
    } else {   // If we are
      //Smart
    }
  }

  if (doStep) {
    int step = 0;
    // Adjust _phaseNr and _position
    if (_direction == DIRECTION_CW)
    {
      // Clockwise
      step = 1;
    }
    else if (_direction == DIRECTION_CCW)
    {
      // Anticlockwise
      step = -1;
    } else {
      Serial.println("HELP (work function: step direction invalid)");
    }

    _phaseNr = (_phaseNr + step + 2 * _numCylinders) % (2 * _numCylinders);
    _position += step;

    _lastStepDir = step;

    // Skip adjusting _lastChangeUs if current phase is not consistent with cylinder strategy
    bool advanceClock = true;
    if (_cylinderStrategy == DOUBLE_ENGAGE_ONLY && (_phaseNr & 1) == 0) {
      advanceClock = false;
    }
    if (_cylinderStrategy == SINGLE_ENGAGE_ONLY && (_phaseNr & 1) == 1) {
      advanceClock = false;
    }
    if (advanceClock && _frequency > 0) {
      _lastChangeUs += _intervalUs;

      if (_lastChangeUs > _lastWorkUs + 2 * _intervalUs) {
        // _lastChangeUs is inconsistent, probably because the motor has not been operated for a while
        _lastChangeUs = _lastWorkUs + 2 * _intervalUs;
      }
      if (_lastChangeUs < _lastWorkUs) {
        // the previous work() should have executed the step, so the motor frequency was probably just changed. Or _lastChangeUs rolled over.
        _lastChangeUs = timeUs;
      } else if ((_lastChangeUs + 0.1 * _intervalUs) < timeUs) {
        // to avoid too small intervals (smaller than 90% of nominal) due to inconsistent calling of work()
        _lastChangeUs = timeUs;
      }

    }
  }

  PneumaticStepper::updateCylinderState();

  _changed |= doStep;
  _lastWorkUs = timeUs;
  if (doStep) {
    computeNewSpeed(true);
  }
  return doStep;
}

void PneuAccelStepper::printSpeed() {
  Serial.print("Speed: ");
  Serial.print(_speed);
}

void PneuAccelStepper::printn() {
  Serial.print("int: ");
  Serial.print(_intervalUs);
}
void PneuAccelStepper::workUntilNoChange()
{
  bool finished = false;
  while (!finished) {
    work();
    finished = !changed();
  }
}


unsigned long PneuAccelStepper::computeNewSpeed(bool nIncrease)
{
  long distanceTo = PneumaticStepper::getSetpoint() - PneumaticStepper::getPosition();

  float tStepsToStop = (((_speed * _speed) / (2.0 * _acceleration))); // Equation 16
  long stepsToStop;
  stepsToStop = (long)(tStepsToStop);
  //if (tStepsToStop < 0.55) { //
  //stepsToStop = 0;    // Makes sure that if we are on c0, so lowest speed, that we actually stop
  //}

  /*
    Serial.print("DistanceTo: ");
    Serial.println(distanceTo);
    Serial.print("StepsToStop: ");
    Serial.println(stepsToStop);
    Serial.print("Speed: ");
    Serial.println(_speed);
    Serial.print("Acceleration: ");
    Serial.println(_acceleration);
    Serial.print("_n: ");
    Serial.println(_n);
  */

  if (distanceTo == 0 && stepsToStop <= 1)
  {
    // We are at the target and its time to stop
    // TODO: What happens in case of extra step needed for approachDirection?
    if (_approachDirection != 0 && _direction != _approachDirection) {
      _n = 0;

    } else {
      _intervalUs = 0;
      _speed = 0.0;
      _n = 0;
      return _intervalUs;
    }
  }

  if (distanceTo > 0)
  {
    // We are anticlockwise from the target
    // Need to go clockwise from here, maybe decelerate now
    if (_n > 0)
    {
      // Currently accelerating, need to decel now? Or maybe going the wrong way?
      if ((stepsToStop >= distanceTo) || _direction == DIRECTION_CCW)
        _n = -stepsToStop; // Start deceleration
    }
    else if (_n < 0)
    {
      // Currently decelerating, need to accel again?
      if ((stepsToStop < distanceTo) && _direction == DIRECTION_CW)
        _n = -_n; // Start accceleration
    }
  }
  else if (distanceTo < 0)
  {
    // We are clockwise from the target
    // Need to go anticlockwise from here, maybe decelerate
    if (_n > 0)
    {
      // Currently accelerating, need to decel now? Or maybe going the wrong way?
      if ((stepsToStop >= -distanceTo) || _direction == DIRECTION_CW)
        _n = -stepsToStop; // Start deceleration
    }
    else if (_n < 0)
    {
      // Currently decelerating, need to accel again?
      if ((stepsToStop < -distanceTo) && _direction == DIRECTION_CCW)
        _n = -_n; // Start accceleration
    }
  }
  if ((_n > 0 && (abs(distanceTo) - stepsToStop - 1) == 0)) {  // If there are x steps left and x-1 stepsToStop, keep the same speed, so we deccelerate at the right time
    // Keep everything the same
  } else {
    // Need to accelerate or decelerate
    if (_n == 0)
    {
      // First step from stopped
      _cn = _c0;
      _direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
      if (_approachDirection != 0 && distanceTo == 0) _direction = (Direction) - _approachDirection; // Do extra step in wrong direction so the final step is in good direction
    }
    else
    {
      // Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
      _cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1)); // Equation 13
      _cn = max(_cn, _cmin);
    }
    _n++;
  }
  _intervalUs = _cn;
  _speed = 1000000.0 / _cn;
  if (_direction == DIRECTION_CCW)
    _speed = -_speed;

#if 0
  Serial.println(_speed);
  Serial.println(_acceleration);
  Serial.println(_cn);
  Serial.println(_c0);
  Serial.println(_n);
  Serial.println(_intervalUs);
  Serial.println(distanceTo);
  Serial.println(stepsToStop);
  Serial.println("-----");
#endif
  return _intervalUs;
}

long PneuAccelStepper::distanceToGo()
{
  return _setpoint - _position;
}





#endif
