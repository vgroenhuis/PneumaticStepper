#ifdef ARDUINO
#include <Arduino.h>
#else
#include <iostream>
#include <iomanip>
using namespace std;

// When testing the library, the millis() and micros() functions must be defined
// by the testing framework.
extern unsigned long millis();
extern unsigned long micros();

#endif


#include "PneuAccelStepper.h"

//PneuAccelStepper PneuAccelStepper::TwoCylinderAccelStepper = PneuAccelStepper(2, true);
//PneuAccelStepper PneuAccelStepper::ThreeCylinderAccelStepper = PneuAccelStepper(3, false);

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
  _direction = DIRECTION_CCW;
  overshootEnabled = OVERSHOOT;//JUMP_TO_ZERO;//OVERSHOOT;
  setAcceleration(10);       //Change acceleration to this value (acceleration value should change, otherwise it doesn't work)
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
      _speed = fmin(sqrt((float)(2.0 * acceleration * (float)abs(distanceTo))), _frequency);
      if (distanceTo < 0)_speed = -_speed;
      _cn = abs(1000000.0 / _speed);
      //if (_cn < 1000000.0 / _frequency)_cn = 1000000.0 / _frequency;
      _intervalUs = _cn;
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
    //if (_c0 < 1000000.0 / _frequency) _c0 = 1000000.0 / _frequency;
    _acceleration = acceleration;
    //computeNewSpeed(false);
  }
  //Serial.println("Set acceleration to ");
  //Serial.println(_acceleration);
  if (_intervalUs < 1000000.0 / _frequency) _intervalUs = 1000000.0 / _frequency;
}

//Sets the setpoint
void PneuAccelStepper::setSetpointDouble(double setpoint)
{
  //Serial.print("Set setpoint double:");
  //Serial.println(setpoint);
  PneumaticStepper::setSetpointDouble(setpoint);
  long distanceTo = distanceToGo();
  long stepsToStop = ((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
  if (overshootEnabled != OVERSHOOT && ((distanceTo > 0 && _direction == DIRECTION_CCW) || (distanceTo < 0 && _direction == DIRECTION_CW))) {
    //Going in wrong direction, set speed to zero
    if (overshootEnabled == JUMP_TO_ZERO) {
      _n = 0;
      _intervalUs = _c0;
      _speed = 0;
    } else if (overshootEnabled == JUMP_TO_OPPOSITE) {
      _speed = fmin(sqrt((float)(2.0 * _acceleration * (float)abs(distanceTo))), _frequency);
      if (distanceTo < 0)_speed = -_speed;
      _cn = abs(1000000.0 / _speed);
      //if (_cn < 1000000.0 / _frequency)_cn = 1000000.0 / _frequency;
      _intervalUs = _cn;
      _direction = _direction == DIRECTION_CW ? DIRECTION_CCW : DIRECTION_CW;
      _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
    }

  } else if (overshootEnabled != OVERSHOOT && stepsToStop > abs(distanceTo)) {
    _speed = fmin(sqrt((float)(2.0 * _acceleration * (float)abs(distanceTo))), _frequency);
    if (distanceTo < 0)_speed = -_speed;
    _cn = abs(1000000.0 / _speed);
    //if (_cn < 1000000.0 / _frequency)_cn = 1000000.0 / _frequency;
    _intervalUs = _cn;
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
  if (_intervalUs < 1000000.0 / _frequency) _intervalUs = 1000000.0 / _frequency;
}

void PneuAccelStepper::setSetpoint(long setpoint)
{
  //Serial.print("Set setpoint: ");
  //Serial.println(setpoint);

  PneumaticStepper::setSetpoint(setpoint);
  long distanceTo = distanceToGo();
  long stepsToStop = ((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
  if (overshootEnabled != OVERSHOOT && ((distanceTo > 0 && _direction == DIRECTION_CCW) || (distanceTo < 0 && _direction == DIRECTION_CW))) {
    //Going in wrong direction, set speed to zero
    if (overshootEnabled == JUMP_TO_ZERO) {
      _n = 0;
      _intervalUs = _c0;
      _speed = 0;
    } else if (overshootEnabled == JUMP_TO_OPPOSITE) {
      _speed = fmin(sqrt((float)(2.0 * _acceleration * (float)abs(distanceTo))), _frequency);
      if (distanceTo < 0)_speed = -_speed;
      _cn = abs(1000000.0 / _speed);
      //if (_cn < 1000000.0 / _frequency)_cn = 1000000.0 / _frequency;
      _intervalUs = _cn;
      _direction = _direction == DIRECTION_CW ? DIRECTION_CCW : DIRECTION_CW;
      _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
    }

  } else if (overshootEnabled != OVERSHOOT && stepsToStop > abs(distanceTo)) {
    //Serial.println("stepsToStop > abs(distanceTo)");
    _speed = fmin(sqrt((float)(2.0 * _acceleration * (float)abs(distanceTo))), _frequency);
    if (distanceTo < 0)_speed = -_speed;
    _cn = abs(1000000.0 / _speed);
    //if (_cn < 1000000.0 / _frequency)_cn = 1000000.0 / _frequency;
    _intervalUs = _cn;
    _direction = _direction == DIRECTION_CW ? DIRECTION_CCW : DIRECTION_CW;
    _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
    /*_n = 0;
      _intervalUs = _c0;
      _speed = 0;
    */
    if (_intervalUs<1000000.0 / _frequency) _intervalUs = 1000000.0 / _frequency;
  }

  if (_speed == 0) {
    _direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
  }
  
#if 0
    Serial.println("Overwrote everything:");
    Serial.println(_speed);
    Serial.println(_intervalUs);
    Serial.println(_direction);
    Serial.println(_n);
    Serial.println(_cn);
    Serial.println(_setpoint);
    Serial.println(_position);
#endif

}

// warning: this is called very frequently!
void PneuAccelStepper::setFrequency(float frequency)
{
  _frequency = frequency;
  _cmin = 1000000 / frequency;
  // Recompute _n from current speed and adjust speed if accelerating or cruising
#if 0
  if (_n > 0)
  {
    _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
    //computeNewSpeed(false);
  }
#endif
}


bool PneuAccelStepper::work()
{
  unsigned long timeUs = micros();
  bool doStep = false;

  bool atFinalSetpoint = true;
  // Do a step if: not at setpoint, running and enough time has elapsed
  if (_running) {
    // check if motor is at setpoint in correct direction
    if (_position != _setpoint) {
      atFinalSetpoint = false;
    } else {
      if (_approachDirection != 0) {
        if (_lastStepDir != _approachDirection) {
          // must do a step in -_approachDirection
          atFinalSetpoint = false;
        }
      } if (abs(_speed) >= 1) {
        // Speed is non-zero, might have overshoot
        long q = getStepsToStop();
        if (q != 0) {
          atFinalSetpoint = false;
        }
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
    } else {

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
  #ifdef ARDUINO
      Serial.println("HELP (work function: step direction invalid)");
  #else
      cout << "HELP (work function: step direction invalid)" << endl;
  #endif
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
        // the previous work() should have executed the step, so the motor frequency was probably just changed.
        // Or _lastChangeUs rolled over. Or the first step has been executed.
        _lastChangeUs = timeUs;
      } else if ((_lastChangeUs + 0.1 * _intervalUs) < timeUs) {
        // to avoid too small intervals (smaller than 90% of nominal) due to inconsistent calling of work()
        _lastChangeUs = timeUs;
      }
    }
  }
  if (atFinalSetpoint) {
    _n = 0;
    _speed = 0;
  }
  updateCylinderState();
  _changed |= doStep;
  _lastWorkUs = timeUs;
  if (doStep) {
    //Serial.println("doStep");
    computeNewSpeed(true);
  }
  return doStep;
}

/*
void PneuAccelStepper::printSpeed() {
  Serial.print("Speed: ");
  Serial.print(_speed);
}

void PneuAccelStepper::printn() {
  Serial.print("int: ");
  Serial.print(_intervalUs);
}
*/

void PneuAccelStepper::workUntilNoChange()
{
  bool finished = false;
  while (!finished) {
    work();
    finished = !changed();
  }
}

long PneuAccelStepper::getStepsToStop() const {
  float tStepsToStop = (((_speed * _speed) / (2.0 * _acceleration))); // Equation 16
  tStepsToStop = round(tStepsToStop);
  if (_speed < 0)
    tStepsToStop = -tStepsToStop;
  return (long)(tStepsToStop);
}

unsigned long PneuAccelStepper::computeNewSpeed(bool nIncrease)
{
  //Serial.println("Compute new speed called:");
  //Serial.print("_n=");
  //Serial.println(_n);
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
        _n = -_n; // Start acceleration
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
      _cn = _c0; // may be too small
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
  //if (_cn < 1000000.0 / _frequency)_cn = 1000000.0 / _frequency;
  _intervalUs = _cn;
  if (_intervalUs < 1000000.0 / _frequency) _intervalUs = 1000000.0 / _frequency;
  _speed = 1000000.0 / _intervalUs;
  if (_direction == DIRECTION_CCW)
    _speed = -_speed;

#if 0
  Serial.println("Compute new speed: ");
  Serial.println(_speed);
  Serial.println(_acceleration);
  Serial.println(_cn);
  Serial.println(_cmin);
  Serial.println(_c0);
  Serial.println(_n);
  Serial.println(_intervalUs);
  Serial.println(distanceTo);
  Serial.println(stepsToStop);
  Serial.println("-----");
#endif

  //Serial.println("Compute new speed finished:");
  //Serial.print("_n=");
  //Serial.println(_n);

  return _intervalUs;
}

long PneuAccelStepper::distanceToGo()
{
  return _setpoint - _position;
}

void PneuAccelStepper::printState() const {
#ifdef ARDUINO
	Serial.print("M-");
	Serial.print(_numCylinders);
	Serial.print(" tri=");
	Serial.print(_triState);
	Serial.print(" strat=");
	switch (_cylinderStrategy) {
	case SINGLE_ENGAGE_ONLY:
		Serial.print("single only");
		break;
	case SINGLE_ENGAGE_AT_POS:
		Serial.print("single at pos");
		break;
	case DOUBLE_ENGAGE_ONLY:
		Serial.print("double only");
		break;
	case DOUBLE_ENGAGE_AT_POS:
		Serial.print("double at pos");
		break;
	case ANY_ENGAGE:
		Serial.print("any");
		break;
	default:
		Serial.print("?");
	}
	Serial.print(" freq=");
	Serial.print(_frequency,2);
	Serial.print(" time=");
	Serial.print(micros()/1000000.0,3);
	Serial.print(" last=");
	Serial.print(_lastChangeUs/1000000.0,3);
	Serial.print(" pos=");
	Serial.print(_position);
	Serial.print(" set=");
	Serial.print(_setpoint);
	Serial.print(" phaseNr=");
	Serial.print(_phaseNr);
	Serial.print(" cyl=[");
	for (int i = 0; i < _numCylinders; i++) {
		Serial.print(_cylinderState[i]);
	}
	Serial.print("] err=");
	Serial.print(_errorCount);
  Serial.print(" speed=");
  Serial.print(_speed,2);
  Serial.print(" acc=");
  Serial.print(_acceleration,2);
  Serial.print(" n=");
  Serial.print(_n);
	Serial.println();
#else
	cout << "M-" << _numCylinders << " tri=" << _triState << " strat=";
	switch (_cylinderStrategy) {
	case SINGLE_ENGAGE_ONLY:
		cout << "single only";
		break;
	case SINGLE_ENGAGE_AT_POS:
		cout << "single at pos";
		break;
	case DOUBLE_ENGAGE_ONLY:
		cout << "double only";
		break;
	case DOUBLE_ENGAGE_AT_POS:
		cout << "double at pos";
		break;
	case ANY_ENGAGE:
		cout << "any";
		break;
	default:
		cout << "?";
	}

	cout << " freq=" << setprecision(2) << _frequency << " timeUs=" << micros() << " lastChangeUs=" << _lastChangeUs << " pos=" << _position << " set=" << _setpoint
		<< " phaseNr=" << _phaseNr << " cyl=[";
	for (int i = 0; i < _numCylinders; i++) {
		cout << (int)_cylinderState[i];
	}
	cout << "] err=" << _errorCount << endl;
#endif
}
