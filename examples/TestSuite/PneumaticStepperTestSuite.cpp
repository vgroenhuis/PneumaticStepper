/*
 * Test protocol for PneumaticStepper.h and ServoValve.h
 * Some tests are done automatically; other tests require manual inspection for correctness.
 * 
*/

#include <iostream>
#include <assert.h>
#include <iomanip>
#include <stdio.h>
#include <stdint.h>
#define byte uint8_t

using namespace std;

// returns milliseconds since start of program
long _millis = 0;

unsigned long millis() {
	return _millis;
}

void waitMillis(unsigned long d) {
	_millis += d;
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#include <PneumaticStepper.h> // this must be included after 
#include <ServoValve.h>




// Validates phase for n=2 stepper motor
void validateCylinderState(const byte* state, byte c0, byte c1) {
	assert(state[0]==c0);
	assert(state[1]==c1);
}

// Validates phase for n=3 stepper motors
void validateCylinderState(const byte* state, byte c0, byte c1, byte c2) {
	assert(state[0]==c0);
	assert(state[1]==c1);
	assert(state[2]==c2);
}

// Validates phase for n=4 stepper motors
void validateCylinderState(const byte* state, byte c0, byte c1, byte c2, byte c3) {
	assert(state[0]==c0);
	assert(state[1]==c1);
	assert(state[2]==c2);
	assert(state[3]==c3);
}


void testBasics() {
	cout << "Testing PneumaticStepper basics..." << endl;
	PneumaticStepper s1 = PneumaticStepper::ThreeCylinderStepper; // Constructor with default parameters
	assert(s1.getCylinderCount()==3);
	assert(!s1.isDoubleActing());
	assert(!s1.isTriState());
	assert(s1.getApproachDirection()==0);
	assert(abs(s1.getFrequency()-10)<1e-8);
	assert(s1.getPosition()==0);
	assert(s1.getSetpoint()==0);
	assert(s1.isPositionValid());
	assert(s1.getPhaseNr()==0);
	assert(!s1.isFloating());
	assert(s1.changed());
	assert(!s1.changed());
	assert(s1.getLastStepDir()==0);
	assert(s1.isRunning());
	validateCylinderState(s1.getCylinderStates(),1,0,0);

	// Test floating
	s1.setFloating(true);
	assert(s1.isFloating());
	assert(!s1.isPositionValid());
	
	s1.setFloating(false);
	assert(!s1.isFloating());
	assert(!s1.isPositionValid());
	s1.setPosition(0);
	assert(!s1.isFloating());
	assert(s1.isPositionValid());
	assert(s1.getStepsTodo()==0);
	
	s1.setSetpoint(2);
	assert(s1.getStepsTodo()==2);
	
	s1.setFrequency(-1);
	s1.pause();
	s1.work();
	assert(s1.getPosition() == 0);
	s1.run();
	s1.work();
	assert(s1.getPosition()==1);
	s1.pause();
	s1.work();
	assert(s1.getPosition() == 1);
	s1.run();
	s1.work();
	s1.work();
	assert(s1.getPosition() == 2);

	PneumaticStepper myMotor[2] = { PneumaticStepper::TwoCylinderStepper, PneumaticStepper::ThreeCylinderStepper };
}

void testTiming() {
	cout << "Testing timing..." << endl;
	PneumaticStepper s1 = PneumaticStepper::TwoCylinderStepper; // 10 Hz
	s1.resetLastChangeTime();
	s1.setSetpoint(-10);
	s1.printState();
	waitMillis(50);

	s1.work();
	assert(s1.getPosition() == 0);
	waitMillis(100);
	s1.work();
	assert(s1.getPosition() == -1);
	s1.work();
	assert(s1.getPosition() == -1);
	waitMillis(100);
	s1.work();
	assert(s1.getPosition() == -2);
	s1.work();
	assert(s1.getPosition() == -2);
	waitMillis(10);
	s1.work();
	assert(s1.getPosition() == -2);

	PneumaticStepper s2 = PneumaticStepper::ThreeCylinderStepper;
	s2.setCylinderStrategy(PneumaticStepper::SINGLE_ENGAGE_ONLY);
	s2.resetLastChangeTime();
	s2.setSetpoint(10);
	waitMillis(50);
	s2.workUntilNoChange();
	assert(s2.getPosition() == 0);
	waitMillis(100);
	s2.workUntilNoChange();
	assert(s2.getPosition() == 2);

	// 
	PneumaticStepper s3 = PneumaticStepper::TwoCylinderStepper;
	s3.resetLastChangeTime();
	s3.setSetpoint(0);
	s3.setFrequency(1); // 1 Hz
	waitMillis(1900);
	s3.setSetpoint(10);
	s3.work();
	assert(s3.getPosition() == 1); // lastChangeTime is 1 s, current time 1.9 s
	waitMillis(200); // current time 2.1 s
	s3.work();
	assert(s3.getPosition() == 1); // in v1.0.6 this fails because _lastChangeMillis was 1 s
	waitMillis(700); // current time 2.8 s
	s3.work();
	assert(s3.getPosition() == 1);
	waitMillis(200); // current time 3.0 s
	s3.work();
	assert(s3.getPosition() == 2);
	//
}

void testCylinderStrategy(){
	cout << "Testing cylinder strategies..." << endl;
	// three-cylinder single-acting, single-engage only, ignore frequency
	PneumaticStepper s1(3, false, false, 0, PneumaticStepper::SINGLE_ENGAGE_ONLY, -1, 0, 0, 0, true);
	assert(s1.getCylinderStrategy()==PneumaticStepper::SINGLE_ENGAGE_ONLY);
	s1.setSetpoint(3);
	assert(s1.getSetpoint() == 2); // actual setpoint is even
	validateCylinderState(s1.getCylinderStates(), 1, 0, 0);
	s1.work();
	validateCylinderState(s1.getCylinderStates(), 1, 1, 0);
	s1.work();
	validateCylinderState(s1.getCylinderStates(), 0, 1, 0);
	s1.work();
	validateCylinderState(s1.getCylinderStates(), 0, 1, 0);
	s1.work();
	validateCylinderState(s1.getCylinderStates(), 0, 1, 0);
	assert(s1.getCylinderState(0) == 0);
	assert(s1.getCylinderState(1) == 1);
	assert(s1.getCylinderState(2) == 0);

	assert(s1.getPosition() == 2);
	// tricky: redefine position
	s1.setPosition(1);
	assert(s1.getPosition() == 1);
	assert(s1.getPhaseNr() == 2);
	validateCylinderState(s1.getCylinderStates(), 0, 1, 0);
	s1.setSetpoint(3);
	assert(s1.getSetpoint() == 3);
}

// Performs ten steps while printing state before each step
void playMotor(PneumaticStepper& stepper) {
	//char buf[100];
	for (int i = 0; i < 20; i++) {
		//stepper.toString(buf, 100);
		stepper.printState();
		//cout << buf << endl;
		waitMillis(3);
		stepper.work();
	}
}

void testOperation() {
	cout << "testOperation: requires manual inspection of motor state prints" << endl;
	cout << "s1: three-cylinder, double-acting stepper" << endl;
	PneumaticStepper s1(3, true);
	s1.setFrequency(100);
	s1.setSetpoint(5);
	playMotor(s1);
	cout << "s2: default three-cylinder single-acting stepper, single-engage strategy" << endl;
	PneumaticStepper s2 = PneumaticStepper::ThreeCylinderStepper;
	s2.setFrequency(100);
	s2.setSetpoint(8);
	s2.setCylinderStrategy(PneumaticStepper::SINGLE_ENGAGE_ONLY);
	playMotor(s2);
	cout << "s3: three-cylinder double-acting, single-engage only" << endl;
	PneumaticStepper s3(3, true, true, 0, PneumaticStepper::SINGLE_ENGAGE_ONLY, 100, 0, 0, 0, true);
	s3.setSetpoint(10);
	playMotor(s3);
	cout << "s4: two-cylinder double-acting" << endl;
	PneumaticStepper s4 = PneumaticStepper::TwoCylinderStepper;
	s4.setFrequency(100);
	s4.setSetpoint(-3);
	playMotor(s4);
}

void testApproachDirection() {
	cout << "testApproachDirection" << endl;
	PneumaticStepper s1 = PneumaticStepper::TwoCylinderStepper;// (2, true);
	s1.setApproachDirection(-1);
	s1.setFrequency(-1);
	s1.setSetpoint(1);
	assert(s1.getPosition() == 0);
	s1.work();
	assert(s1.getPosition() == 1);
	s1.work();
	assert(s1.getPosition() == 2);
	s1.work();
	assert(s1.getPosition() == 1);
	s1.work();
	assert(s1.getPosition() == 1);
	s1.work();
	assert(s1.getPosition() == 1);
	s1.setApproachDirection(1);
	s1.work();
	assert(s1.getPosition() == 0);
	s1.work();
	assert(s1.getPosition() == 1);
	s1.work();
	assert(s1.getPosition() == 1);
}

void testHysteresis() {
	cout << "Testing hysteresis..." << endl;
	PneumaticStepper s1 = PneumaticStepper::TwoCylinderStepper;
	s1.setFrequency(-1);
	s1.setHysteresis(3);
	s1.setSetpointDouble(2.1);
	assert(s1.getSetpoint() == 0);
	s1.setSetpointDouble(3.2);
	assert(s1.getSetpoint() == 3);
	s1.work();
	assert(s1.getPosition() == 1);
	s1.work();
	assert(s1.getPosition() == 2);
	s1.setSetpointDouble(4);
	assert(s1.getSetpoint() == 2);
	s1.work();
	assert(s1.getPosition() == 2);
	s1.setHysteresis(0.7);
	s1.setSetpointDouble(2.8);
	assert(s1.getSetpoint() == 3);
	s1.work();
	s1.work();
	assert(s1.getPosition() == 3);
	s1.setSetpointDouble(2.4);
	assert(s1.getSetpoint() == 3);
	s1.setSetpointDouble(2.1);
	assert(s1.getSetpoint() == 2);
}

void testServoValve() {
	cout << "Testing ServoValve..." << endl;
	ServoValve v1(90);
	assert(v1.changed());
	assert(!v1.changed());
	v1.setLogicalState(0);
	v1.work();
	assert(v1.changed());
	assert(v1.getGoalAngle() == 45);
	assert(v1.getSetAngle() == 43);
	assert(v1.isMoving());
	v1.work();
	assert(!v1.changed());
	assert(v1.getSetAngle() == 43);
	waitMillis(300);
	v1.setLogicalState(0);
	assert(v1.getSetAngle() == 43);
	v1.setLogicalState(0);
	v1.work();
	assert(!v1.changed());
	assert(v1.getSetAngle() == 43);
	assert(v1.isMoving());
	waitMillis(300);
	v1.work();
	assert(v1.changed());
	assert(!v1.changed());
	v1.setLogicalState(0);
	assert(v1.getSetAngle() == 45);
	v1.setLogicalState(0);
	assert(!v1.isMoving());
	assert(v1.getSetAngle() == 45);
	v1.setLogicalState(1);
	assert(v1.changed());
	v1.work();
	assert(v1.getSetAngle() == 137);
	v1.setLogicalState(1);
	v1.work();
	assert(!v1.changed());
	assert(v1.getSetAngle() == 137);
	assert(v1.getSetAngle() == 137);
	v1.setLogicalState(2);
	assert(v1.changed());
	assert(v1.getSetAngle() == 88);
	waitMillis(300);
	v1.work();
	assert(!v1.changed());
	assert(v1.getSetAngle() == 88);
	waitMillis(300);
	v1.work();
	assert(v1.changed());
	assert(v1.getSetAngle() == 90);

	ServoValve v2(60);
	v2.setLogicalState(0);
	assert(v2.getSetAngle() == 13);
	v2.setLogicalState(2);
	assert(v2.getSetAngle() == 62);
	v2.setLogicalState(1);
	assert(v2.getSetAngle() == 107);
	v2.setLogicalState(2);
	assert(v2.getGoalAngle() == 60);
	assert(v2.getSetAngle() == 58);
	waitMillis(600);
	v2.work();
	assert(v2.getSetAngle() == 60);

	ServoValve v3(90, 70, 110, 0, 1800, 3, 200, 0);
	assert(v3.getSetAngle() == 70);
	assert(v3.getPulse() == 700);
	v3.setLogicalState(1);
	assert(v3.getSetAngle()==113);
	waitMillis(100);
	v3.work();
	assert(v3.getSetAngle()==113);
	assert(v3.getPulse() == 1130);
	waitMillis(150);
	v3.work();
	assert(v3.getSetAngle()==110);
}

int main() {
	testBasics();
	testTiming();
	testCylinderStrategy();
	testOperation();
	testApproachDirection();
	testHysteresis();
	testServoValve();
	return 0;
}

