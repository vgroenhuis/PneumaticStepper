/*
	PneumaticStepper.h - Library for managing the cylinder states of various pneumatic stepper motors.
	A stepper motor can have N cylinders (N>=2), which are either single-acting or double-acting. In case of two cylinders these must be double-acting.
	A cylinder can be dual-state (up or down) or tri-state (up, floating (no pressure), down).
  
	A motor has a cylinder state. There are 2^N possible cylinder states (3^N in case of tri-state cylinders).
	The stepping frequency can be specified
	A motor has a current position and a setpoint position.
	The PneumaticStepper class does not make use of hardware timers. Instead, the work() function needs to be called periodically to update the state.
	The work() function returns true if the cylinder state has changed since the last call to work(). The caller is then responsible for driving the appropriate pins or servo objects to bring the software cylinder state to the physical cylinder.
  
	Copyright (c) 2019-2024 Vincent Groenhuis
	License: CC-BY-SA
*/
#ifndef PNEUMATIC_STEPPER_H
#define PNEUMATIC_STEPPER_H

constexpr int MAX_CYLINDERS = 8; // increase if using more cylinders;
constexpr float MAX_JITTER = 0.05f; // Timestamps may be off by at most this fraction of one period. This allows to keep a consistent frequency even when timing is slightly off.

class PneumaticStepper
{
  public:
	// CylinderStrategy:
	// ANY_ENGAGE: one or two cylinders may be engaged at all times, default strategy
	// SINGLE_ENGAGE_ONLY: exactly one cylinder is engaged at all times, the others are down (in single-acting cylinders) or floating (in double-acting cylinders)
	//    For single-acting motors this is enforced by ensuring that setpoint is at a single-engage phase and skipping the wait time at double-engage phase
	// SINGLE_ENGAGE_AT_POS: during transitions there may be two cylinders engaged, but at setpoints only one is engaged.
	//    For single-acting motors this is enforced by ensuring that setpoint is at single-engage phase.
	// DOUBLE_ENGAGE_ONLY: exactly two cylinders are engaged at all times, the others are down (in single-acting cylinders) or floating (in double-acting cylinders)
	//    For single-acting motors this is enforced by ensuring that setpoint is at a double-engage phase and skipping the wait time at single-engage phase
	// DOUBLE_ENGAGE_AT_POS: during transitions there may be one cylinders engaged, but at setpoints exactly two are engaged. Setpoint must be odd. This eliminates backlash in single-acting types at the cost of hysteresis
	//    For single-acting motors this is enforced by ensuring that setpoint is at a double-engage phase
	
	enum CylinderStrategy {ANY_ENGAGE, SINGLE_ENGAGE_ONLY, SINGLE_ENGAGE_AT_POS, DOUBLE_ENGAGE_ONLY, DOUBLE_ENGAGE_AT_POS};

	// Constructor
	// nCylinder: number of cylinders; doubleActing: true if each cylinder has two rows of teeth (phased 180 deg apart)
	// triState: when using 4/3-way or 5/3-way valves (e.g. servos) this allows a special, non-pressurized state in each cylinder which may be useful
	// in certain control strategies
	// approachDirection: 1 if setpoint must be approached in positive direction, 0 if it does not matter, -1 if in negative direction
	// frequency: in Hz. Negative value ignores stepping frequency and performs one step at every call of work().
	PneumaticStepper(int nCylinder, bool doubleActing, bool triState=false, int approachDirection=0, CylinderStrategy cylinderStrategy=CylinderStrategy::ANY_ENGAGE, float frequency=10, long position=0, long setpoint=0, int phaseNr=0, bool running=true, float hysteresis=0);

	// Returns a two-cylinder, double-acting stepper with default strategy
	static PneumaticStepper TwoCylinderStepper;
	// Returns a three-cylinder, single-acting stepper with default strategy
	static PneumaticStepper ThreeCylinderStepper;
	
	int getCylinderCount() const { return _n; }
	bool isDoubleActing() const { return _doubleActing; }
	bool isTriState() const { return _triState; }
	void setApproachDirection(int approachDirection) { _approachDirection = approachDirection; }
	int getApproachDirection() const { return _approachDirection; }
	void setCylinderStrategy(CylinderStrategy cylinderStrategy);
	CylinderStrategy getCylinderStrategy() const { return _cylinderStrategy; }
	void setFrequency(float frequency) { _frequency = frequency; }
	float getFrequency() const { return _frequency; }
	long getPosition() const { return _position; }
	long getSetpoint() const { return _setpoint; }
	void setSetpoint(long setpoint);
	void setSetpointDouble(double setpoint);
	void setHysteresis(float hysteresis) { _hysteresis = hysteresis; }
	bool isPositionValid() const { return _positionValid; }
	int getPhaseNr() const { return _phaseNr; }
	void setPhaseNr(int phaseNr);
	bool isFloating() const { return _floating; }
	// Returns difference between setpoint and current position
	long getStepsTodo() const { return _setpoint-_position; }
	int getLastStepDir() const { return _lastStepDir; }
	// Returns byte array with n elements indicating the cylinder states: 0=down, 1=up, 2=floating.
	const byte* getCylinderStates() const { return _cylinderState; }
	int getCylinderState(int i) { return _cylinderState[i]; }

	// Returns true if anything was changed since the last call to changed().
	bool changed();
	
	// If floating=true: sets cylinder state such that rack or gear can be back-driven. This is always possible in
	// single-acting motors, in tri-state double-acting motors and in motors where the system pressure can be switched off.
	// Setting it to floating invalidates the position.
	// If floating=false: turns floating off but the position is still invalid until the next clal to setPosition(...).
	void setFloating(bool floating);

	// Sets position and makes it valid but does not change the phase! Does nothing if motor is floating.
	void setPosition(long position);
	int getErrorCount() const { return _errorCount; }

	bool isRunning() const { return _running; }
	void run() { _running = true; }
	void pause() { _running = false; }

	// Resets last change time to current time, blocking changes to position for the next period
	void resetLastChangeTime() { _lastChangeUs = micros(); }

	// Performs logic, advancing the motor by one step towards the setpoint if enough time has elapsed since last change
	// Sets _changed to true if anything was changed.
	void work();

	// Repeatedly calls work() until changed() returns false
	void workUntilNoChange();

	// Prints representation to serial (or stdout if non-Arduino)
	void printState() const;
  private:
    int _n;
	bool _doubleActing;
	bool _triState;
	
	int _approachDirection;
	CylinderStrategy _cylinderStrategy;
	float _frequency;
	unsigned long _lastChangeUs; // timestamp of last change
	unsigned long _lastWorkUs; // timestamp of last work() routine
	bool _positionValid; // normally true, changes to false when floating and changes back to true when calibrating current position
	long _position;
	long _setpoint;
	bool _running; // if not running then it is paused
	
	/* 
	* There are 2N phases (but in single-acting motors with CylinderStrategy SINGLE_ENGAGE_ONLY and DOUBLE_ENGAGE_ONLY, half of the phases are skipped)
	* Single-acting motor: 3 cylinders: 001 011 010 110 100 101. 4 cylinders: 0001 0011 0010 0110 0100 1100 1000 1001. 5 cylinders: 00001 00011 etc.
	* Double-acting motor: 2 cylinders: 00 01 11 10. 3 cylinders: 000 001 011 111 110 100. 4 cylinders: 0000 0001 0011 0111 1111 1110 1100 1000. 5 cylinders: 00000	etc.
	*/
	int _phaseNr;
	bool _floating;
	// Is set when anything is changed, and reset when work() is called.
	bool _changed;
	// Direction of last step, can be -1, or 1 if it is known, and 0 if unknown.
	// Is used by work() to ensure that the setpoint is approached in the correct direction.
	int _lastStepDir;
	
	// The state of each cylinder, is linked to _phaseNr and _floating. 0=down, 1=up, 2=floating (only for tri-state cylinders).
	byte _cylinderState[MAX_CYLINDERS];

	int _errorCount;

	float _hysteresis; // Setting setpoint: if |newSetpoint-_position|<_hysteresis then _setpoint is set to _position, otherwise set to round(newSetpoint). Default 0.
private: // methods
	bool usesTiming() { return _frequency>0; }

	// Restricts _setpoint to a valid value, taking _cylinderStrategy into account. Sometimes only an odd or even setpoint is allowed.
	void restrictSetpoint();
	// Must be called after _floating and/or _phaseNr are changed.
	void updateCylinderState();
};

PneumaticStepper PneumaticStepper::TwoCylinderStepper = PneumaticStepper(2, true);
PneumaticStepper PneumaticStepper::ThreeCylinderStepper = PneumaticStepper(3, false);


PneumaticStepper::PneumaticStepper(int nCylinder, bool doubleActing, bool triState, int approachDirection, CylinderStrategy cylinderStrategy, 
	float frequency, long position, long setpoint, int phaseNr, bool running, float hysteresis)
	: _n(nCylinder), _doubleActing(doubleActing), _triState(triState), _approachDirection(approachDirection), _cylinderStrategy(cylinderStrategy), 
	_frequency(frequency), _position(position), _setpoint(setpoint), _lastChangeUs(micros()), _phaseNr(phaseNr), _running(running), 
	_floating(false), _positionValid(true), _changed(true), _lastStepDir(0), _errorCount(0), _hysteresis(hysteresis)
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
	if (abs(setpoint - _position) < _hysteresis) {
		_setpoint = _position;
	}
	else
	{
		_setpoint = roundf(setpoint);
		restrictSetpoint();
	}
}

void PneumaticStepper::restrictSetpoint() {
	long finalPhaseNr = (_phaseNr + (_setpoint - _position)) % (2 * _n);
	if (finalPhaseNr < 0) {
		finalPhaseNr += 2 * _n;
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
			for (int i=0;i<_n;i++) {
				_cylinderState[i]=0;
			}
		} else if (_triState) {
			// all cylinders floating
			for (int i=0;i<_n;i++) {
				_cylinderState[i]=2;
			}
		} else {
			// we can't put it floating ourselves, so assume that system pressure is turned off. Just set all cylinders down.
			for (int i=0;i<_n;i++) {
				_cylinderState[i]=0;
			}
		}
	} else {
		// Not floating, so use _phaseNr
		if (_doubleActing) {
			if (_triState) {
				// Set unused cylinders floating
				for (int i = 0; i < _n; i++) {
					_cylinderState[i] = 2;
				}
				if (_phaseNr < _n) {
					_cylinderState[_phaseNr] = 1;
				} else {
					_cylinderState[_phaseNr - _n] = 0;
				}
			} else {
				// Double-acting motor: e.g. 2 cylinders: 00 01 11 10. 3 cylinders: 000 001 011 111 110 100.
				// 4 cylinders: 0000 0001 0011 0111 1111 1110 1100 1000.
				for (int i = 0; i < _n; i++) {
					_cylinderState[i] = (i < _phaseNr) && (_phaseNr <= _n + i);
				}
			}
		} else {
			// Single-acting motor: e.g. 4 cylinders: 0001 0011 0010 0110 0100 1100 1000 1001.
			for (int i=0;i<_n;i++) {
				// i=0: phaseNr=2n-1, 0, 1, 
				// i=1: phaseNr=1, 2, 3
				// i=2: phaseNr=3, 4, 5
				_cylinderState[i]=0;
				for (int j=-1;j<=1;j++) {
					if (_phaseNr==(2*i+j+2*_n)%(2*_n)) {
						_cylinderState[i]=1;
					}
				}
			}
		}
	}
}

void PneumaticStepper::work() {
	unsigned long timeUs = micros();
	unsigned long intervalUs;
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
				intervalUs = (unsigned long)(1000000.0/_frequency);
				// Test if enough time has elapsed since last change
				unsigned long elapsedUs = timeUs-_lastChangeUs;
				if (elapsedUs >= intervalUs) {
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

		_phaseNr = (_phaseNr + step + 2 * _n) % (2 * _n);
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

			_lastChangeUs += intervalUs;
			unsigned long lagUs = timeUs - _lastChangeUs;
			unsigned long maxLagUs = (unsigned long)(MAX_JITTER*intervalUs);
			if (lagUs > maxLagUs) {
				_lastChangeUs = timeUs;
			}
		}
	}
	
	updateCylinderState();
	
	_changed |= doStep;
	_lastWorkUs = timeUs;
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
	Serial.print(_n);
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
	for (int i = 0; i < _n; i++) {
		Serial.print(_cylinderState[i]);
	}
	Serial.print("] err=");
	Serial.print(_errorCount);
	Serial.println();
#else
	cout << "M-" << _n << " tri=" << _triState << " strat=";
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
	for (int i = 0; i < _n; i++) {
		cout << (int)_cylinderState[i];
	}
	cout << "] err=" << _errorCount << endl;
#endif
}

#endif

