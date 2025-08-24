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

#include <stdint.h>
#include <math.h>


const int MAX_CYLINDERS = 8; // increase if using more cylinders;
const float MAX_JITTER = 0.05f; // Timestamps may be off by at most this fraction of one period. This allows to keep a consistent frequency even when timing is slightly off.

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
	PneumaticStepper(int nCylinder, bool doubleActing, bool triState=false, int approachDirection=0, CylinderStrategy cylinderStrategy=ANY_ENGAGE, float frequency=10, long position=0, long setpoint=0, int phaseNr=0, bool running=true, float hysteresis=0);

	// Returns a two-cylinder, double-acting stepper with default strategy
	static PneumaticStepper TwoCylinderStepper;
	// Returns a three-cylinder, single-acting stepper with default strategy
	static PneumaticStepper ThreeCylinderStepper;
	
	int getCylinderCount() const { return _numCylinders; }
	bool isDoubleActing() const { return _doubleActing; }
	bool isTriState() const { return _triState; }
	void setApproachDirection(int approachDirection) { _approachDirection = approachDirection; }
	int getApproachDirection() const { return _approachDirection; }
	void setCylinderStrategy(CylinderStrategy cylinderStrategy);
	CylinderStrategy getCylinderStrategy() const { return _cylinderStrategy; }
	virtual void setFrequency(float frequency) { _frequency = frequency; _intervalUs = 1000000 / frequency; }
	float getFrequency() const { return _frequency; }
	long getPosition() const { return _position; }
	long getSetpoint() const { return _setpoint; }
	virtual void setSetpoint(long setpoint);
	virtual void setSetpointDouble(double setpoint);
	void setHysteresis(float hysteresis) { _hysteresis = hysteresis; }
	bool isPositionValid() const { return _positionValid; }
	int getPhaseNr() const { return _phaseNr; }
	void setPhaseNr(int phaseNr);
	bool isFloating() const { return _floating; }
	// Returns difference between setpoint and current position
	long getStepsTodo() const { return _setpoint-_position; }
	int getLastStepDir() const { return _lastStepDir; }
	// Returns byte array with n elements indicating the cylinder states: 0=down, 1=up, 2=floating.
	const uint8_t* getCylinderStates() const { return _cylinderState; }
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
	void resetLastChangeTime();

	// Performs logic, advancing the motor by one step towards the setpoint if enough time has elapsed since last change
	// Sets _changed to true if anything was changed.
	virtual bool work();

	// Repeatedly calls work() until changed() returns false
	virtual void workUntilNoChange();

	// Prints representation to serial (or stdout if non-Arduino)
	void printState() const;
  protected:
  	unsigned long _intervalUs;
    int _numCylinders;
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
	uint8_t _cylinderState[MAX_CYLINDERS];

	int _errorCount;

	float _hysteresis; // Setting setpoint: if |newSetpoint-_position|<_hysteresis then _setpoint is set to _position, otherwise set to round(newSetpoint). Default 0.
protected: // methods
	bool usesTiming() { return _frequency>0; }

	// Restricts _setpoint to a valid value, taking _cylinderStrategy into account. Sometimes only an odd or even setpoint is allowed.
	void restrictSetpoint();
	// Must be called after _floating and/or _phaseNr are changed.
	void updateCylinderState();
};

#endif

