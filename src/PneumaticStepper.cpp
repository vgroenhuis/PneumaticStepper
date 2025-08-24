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

#include "PneumaticStepper.h"



PneumaticStepper PneumaticStepper::TwoCylinderStepper = PneumaticStepper(2, true);
PneumaticStepper PneumaticStepper::ThreeCylinderStepper = PneumaticStepper(3, false);


PneumaticStepper::PneumaticStepper(int nCylinder, bool doubleActing, bool triState, int approachDirection, CylinderStrategy cylinderStrategy, 
    float frequency, long position, long setpoint, int phaseNr, bool running, float hysteresis)
    : _intervalUs(1000000/frequency),
      _numCylinders(nCylinder),
      _doubleActing(doubleActing),
      _triState(triState),
      _approachDirection(approachDirection),
      _cylinderStrategy(cylinderStrategy),
      _frequency(frequency),
      _lastChangeUs(micros()),
      _lastWorkUs(0),
      _positionValid(true),
      _position(position),
      _setpoint(setpoint),
      _running(running),
      _phaseNr(phaseNr),
      _floating(false),
      _changed(true),
      _lastStepDir(0),
      _errorCount(0),
      _hysteresis(hysteresis)
{
    updateCylinderState();
}


bool PneumaticStepper::changed() {
	bool tmp = _changed;
	_changed = false;
	return tmp;
}

void PneumaticStepper::setCylinderStrategy(CylinderStrategy cylinderStrategy) { 
	_cylinderStrategy = cylinderStrategy;
	restrictSetpoint(); 
	updateCylinderState();
}

void PneumaticStepper::setSetpoint(long setpoint) {
	if (abs(setpoint - _position) < _hysteresis) {
		_setpoint = _position;
	}
	else
	{
		_setpoint = setpoint;
		restrictSetpoint();
	}
}

void PneumaticStepper::setSetpointDouble(double setpoint) {
	if (fabs(setpoint - _position) < _hysteresis) {
		_setpoint = _position;
	}
	else
	{
		_setpoint = roundf(setpoint);
		restrictSetpoint();
	}
}

void PneumaticStepper::restrictSetpoint() {
	long finalPhaseNr = (_phaseNr + (_setpoint - _position)) % (2 * _numCylinders);
	if (finalPhaseNr < 0) {
		finalPhaseNr += 2 * _numCylinders;
	}
	if (!_doubleActing) {
		switch(_cylinderStrategy) {
			case SINGLE_ENGAGE_ONLY:
				if ((finalPhaseNr & 1) == 1) {
					_setpoint ^= 1;
				}
				break;
			case DOUBLE_ENGAGE_ONLY:
				if ((finalPhaseNr & 1) == 0) {
					_setpoint ^= 1; // flip bit
				}
				break;
			default:
				break;
		}
	}
}
	
void PneumaticStepper::setFloating(bool floating) {
	_floating = floating;
	if (_floating) {
		_positionValid = false;
		_lastStepDir = 0;
	}
	_changed = true;
	updateCylinderState();
}


void PneumaticStepper::setPosition(long position) {
	if (_floating) {
		// warning: position should not be set in floating state! First resetFloating() and then set position.
		return;
	}
	_position = position;
	_positionValid = true;
	_lastStepDir = 0;
	restrictSetpoint();
	updateCylinderState();
}


// Must be called after changing _floating or _phaseNr
void PneumaticStepper::updateCylinderState() {
	if (_floating) {
		if (!_doubleActing) {
			// all cylinders down
			for (int i=0;i<_numCylinders;i++) {
				_cylinderState[i]=0;
			}
		} else if (_triState) {
			// all cylinders floating
			for (int i=0;i<_numCylinders;i++) {
				_cylinderState[i]=2;
			}
		} else {
			// we can't put it floating ourselves, so assume that system pressure is turned off. Just set all cylinders down.
			for (int i=0;i<_numCylinders;i++) {
				_cylinderState[i]=0;
			}
		}
	} else {
		// Not floating, so use _phaseNr
		if (_doubleActing) {
			if (_triState) {
				// Set unused cylinders floating
				for (int i = 0; i < _numCylinders; i++) {
					_cylinderState[i] = 2;
				}
				if (_phaseNr < _numCylinders) {
					_cylinderState[_phaseNr] = 1;
				} else {
					_cylinderState[_phaseNr - _numCylinders] = 0;
				}
			} else {
				// Double-acting motor: e.g. 2 cylinders: 00 01 11 10. 3 cylinders: 000 001 011 111 110 100.
				// 4 cylinders: 0000 0001 0011 0111 1111 1110 1100 1000.
				for (int i = 0; i < _numCylinders; i++) {
					_cylinderState[i] = (i < _phaseNr) && (_phaseNr <= _numCylinders + i);
				}
			}
		} else {
			// Single-acting motor: e.g. 4 cylinders: 0001 0011 0010 0110 0100 1100 1000 1001.
			for (int i=0;i<_numCylinders;i++) {
				// i=0: phaseNr=2n-1, 0, 1, 
				// i=1: phaseNr=1, 2, 3
				// i=2: phaseNr=3, 4, 5
				_cylinderState[i]=0;
				for (int j=-1;j<=1;j++) {
					if (_phaseNr==(2*i+j+2*_numCylinders)%(2*_numCylinders)) {
						_cylinderState[i]=1;
					}
				}
			}
		}
	}
}

bool PneumaticStepper::work() {
	unsigned long timeUs = micros();
	//unsigned long intervalUs;
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
			}
		}

		if (!atFinalSetpoint) {
			if (_frequency < 0) {
				doStep = true;
			} else if (_frequency > 0) {
				//intervalUs = (unsigned long)(1000000.0/_frequency);
				// Test if enough time has elapsed since last change
				unsigned long elapsedUs = timeUs-_lastChangeUs;
				if (elapsedUs >= _intervalUs) {
					doStep = true;
				}
			}
		}
	}
	
	if (doStep) {
		int step = 0;
		// Adjust _phaseNr and _position
		if (_setpoint<_position) {
			step = -1;
		} else if (_setpoint>_position) {
			step = 1;
		} else if (_lastStepDir != _approachDirection) {
			// is at setpoint, but not correct direction
			step = -_approachDirection;
		} else {
			// should not happen
			_errorCount++;
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
			/*
				In normal operation, _lastChangeUs is incremented by intervalUs and then approximately equal to timeUs
				_lastChangeUs may lag by at most MAX_JITTER*intervalUs
				If lag is larget then _lastChangeUs is set equal to timeUs.
			*/

			_lastChangeUs += _intervalUs;
			unsigned long lagUs = timeUs - _lastChangeUs;
			unsigned long maxLagUs = (unsigned long)(MAX_JITTER*_intervalUs);
			if (lagUs > maxLagUs) {
				_lastChangeUs = timeUs;
			}
		}
	}
	
	updateCylinderState();
	
	_changed |= doStep;
	_lastWorkUs = timeUs;
	return doStep;
}

void PneumaticStepper::workUntilNoChange() {
	bool finished = false;
	while (!finished) {
		work();
		finished = !changed();
	}
}

void PneumaticStepper::setPhaseNr(int phaseNr) {
	_phaseNr = phaseNr;
	_changed = true;
}

void PneumaticStepper::printState() const {
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
	Serial.print(micros());
	Serial.print(" lastChange=");
	Serial.print(_lastChangeUs);
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

void PneumaticStepper::resetLastChangeTime() {
	_lastChangeUs = micros();
}
