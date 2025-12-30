#include "test_pneumatic_stepper.h"

#include <unity.h>

#include "test_common.h"
#include "test_PneuAccel.h"

#include "PneumaticStepper.h"
#include "ServoValve.h"


void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

// Validates phase for n=2 stepper motor
void validateCylinderState(const uint8_t* state, uint8_t c0, uint8_t c1) {
	TEST_ASSERT_TRUE(state[0]==c0);
	TEST_ASSERT_TRUE(state[1]==c1);
}

// Validates phase for n=3 stepper motors
void validateCylinderState(const uint8_t* state, uint8_t c0, uint8_t c1, uint8_t c2) {
	TEST_ASSERT_TRUE(state[0]==c0);
	TEST_ASSERT_TRUE(state[1]==c1);
	TEST_ASSERT_TRUE(state[2]==c2);
}

// Validates phase for n=4 stepper motors
void validateCylinderState(const uint8_t* state, uint8_t c0, uint8_t c1, uint8_t c2, uint8_t c3) {
	TEST_ASSERT_TRUE(state[0]==c0);
	TEST_ASSERT_TRUE(state[1]==c1);
	TEST_ASSERT_TRUE(state[2]==c2);
	TEST_ASSERT_TRUE(state[3]==c3);
}


void testBasics() {
	cout << "Testing PneumaticStepper basics..." << endl;
	PneumaticStepper s1 = PneumaticStepper::makeThreeCylinderStepper(); // Constructor with default parameters
	TEST_ASSERT_TRUE(s1.getCylinderCount()==3);
	TEST_ASSERT_TRUE(!s1.isDoubleActing());
	TEST_ASSERT_TRUE(!s1.isTriState());
	TEST_ASSERT_TRUE(s1.getApproachDirection()==0);
	TEST_ASSERT_TRUE(abs(s1.getFrequency()-10)<1e-8);
	TEST_ASSERT_TRUE(s1.getPosition()==0);
	TEST_ASSERT_TRUE(s1.getSetpoint()==0);
	TEST_ASSERT_TRUE(s1.isPositionValid());
	TEST_ASSERT_TRUE(s1.getPhaseNr()==0);
	TEST_ASSERT_TRUE(!s1.isFloating());
	TEST_ASSERT_TRUE(s1.changed());
	TEST_ASSERT_TRUE(!s1.changed());
	TEST_ASSERT_TRUE(s1.getLastStepDir()==0);
	TEST_ASSERT_TRUE(s1.isRunning());
	validateCylinderState(s1.getCylinderStates(),1,0,0);

	// Test floating
	s1.setFloating(true);
	TEST_ASSERT_TRUE(s1.isFloating());
	TEST_ASSERT_TRUE(!s1.isPositionValid());
	
	s1.setFloating(false);
	TEST_ASSERT_TRUE(!s1.isFloating());
	TEST_ASSERT_TRUE(!s1.isPositionValid());
	s1.setPosition(0);
	TEST_ASSERT_TRUE(!s1.isFloating());
	TEST_ASSERT_TRUE(s1.isPositionValid());
	TEST_ASSERT_TRUE(s1.getStepsTodo()==0);
	
	s1.setSetpoint(2);
	TEST_ASSERT_TRUE(s1.getStepsTodo()==2);
	
	s1.setFrequency(-1);
	s1.pause();
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == 0);
	s1.run();
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition()==1);
	s1.pause();
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == 1);
	s1.run();
	s1.work();
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == 2);

	// test array of motors
	PneumaticStepper myMotor[2] = { PneumaticStepper::makeTwoCylinderStepper(), PneumaticStepper::makeThreeCylinderStepper() };

	// test copy constructor
	s1 = PneumaticStepper::makeTwoCylinderStepper();
}

void testTiming() {
	cout << "Testing timing..." << endl;
	PneumaticStepper s1 = PneumaticStepper::makeTwoCylinderStepper(); // 10 Hz
	s1.resetLastChangeTime();
	s1.setSetpoint(-10);
	s1.printState();
	waitMillis(50);

	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == 0);
	waitMillis(100);
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == -1);
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == -1);
	waitMillis(100);
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == -2);
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == -2);
	waitMillis(10);
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == -2);

	PneumaticStepper s2 = PneumaticStepper::makeThreeCylinderStepper();
	s2.setCylinderStrategy(PneumaticStepper::SINGLE_ENGAGE_ONLY);
	s2.resetLastChangeTime();
	s2.setSetpoint(10);
	waitMillis(50);
	s2.workUntilNoChange();
	TEST_ASSERT_TRUE(s2.getPosition() == 0);
	waitMillis(100);
	s2.workUntilNoChange();
	TEST_ASSERT_TRUE(s2.getPosition() == 2);

	// 
	PneumaticStepper s3 = PneumaticStepper::makeTwoCylinderStepper();
	s3.resetLastChangeTime();
	s3.setSetpoint(0);
	s3.setFrequency(1); // 1 Hz
	waitMillis(1900);
	s3.setSetpoint(10);
	s3.work();
	TEST_ASSERT_TRUE(s3.getPosition() == 1); // lastChangeTime is 1 s, current time 1.9 s
	waitMillis(200); // current time 2.1 s
	s3.work();
	TEST_ASSERT_TRUE(s3.getPosition() == 1); // in v1.0.6 this fails because _lastChangeMillis was 1 s
	waitMillis(700); // current time 2.8 s
	s3.work();
	TEST_ASSERT_TRUE(s3.getPosition() == 1);
	waitMillis(200); // current time 3.0 s
	s3.work();
	TEST_ASSERT_TRUE(s3.getPosition() == 2);
	//
}

void testCylinderStrategy(){
	cout << "Testing cylinder strategies..." << endl;
	// three-cylinder single-acting, single-engage only, ignore frequency
	PneumaticStepper s1(3, false, false, 0, PneumaticStepper::SINGLE_ENGAGE_ONLY, -1, 0, 0, 0, true);
	TEST_ASSERT_TRUE(s1.getCylinderStrategy()==PneumaticStepper::SINGLE_ENGAGE_ONLY);
	s1.setSetpoint(3);
	TEST_ASSERT_TRUE(s1.getSetpoint() == 2); // actual setpoint is even
	validateCylinderState(s1.getCylinderStates(), 1, 0, 0);
	s1.work();
	validateCylinderState(s1.getCylinderStates(), 1, 1, 0);
	s1.work();
	validateCylinderState(s1.getCylinderStates(), 0, 1, 0);
	s1.work();
	validateCylinderState(s1.getCylinderStates(), 0, 1, 0);
	s1.work();
	validateCylinderState(s1.getCylinderStates(), 0, 1, 0);
	TEST_ASSERT_TRUE(s1.getCylinderState(0) == 0);
	TEST_ASSERT_TRUE(s1.getCylinderState(1) == 1);
	TEST_ASSERT_TRUE(s1.getCylinderState(2) == 0);

	TEST_ASSERT_TRUE(s1.getPosition() == 2);
	// tricky: redefine position
	s1.setPosition(1);
	TEST_ASSERT_TRUE(s1.getPosition() == 1);
	TEST_ASSERT_TRUE(s1.getPhaseNr() == 2);
	validateCylinderState(s1.getCylinderStates(), 0, 1, 0);
	s1.setSetpoint(3);
	TEST_ASSERT_TRUE(s1.getSetpoint() == 3);
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
	PneumaticStepper s2 = PneumaticStepper::makeThreeCylinderStepper();
	s2.setFrequency(100);
	s2.setSetpoint(8);
	s2.setCylinderStrategy(PneumaticStepper::SINGLE_ENGAGE_ONLY);
	playMotor(s2);
	cout << "s3: three-cylinder double-acting, single-engage only" << endl;
	PneumaticStepper s3(3, true, true, 0, PneumaticStepper::SINGLE_ENGAGE_ONLY, 100, 0, 0, 0, true);
	s3.setSetpoint(10);
	playMotor(s3);
	cout << "s4: two-cylinder double-acting" << endl;
	PneumaticStepper s4 = PneumaticStepper::makeTwoCylinderStepper();
	s4.setFrequency(100);
	s4.setSetpoint(-3);
	playMotor(s4);
}

void testApproachDirection() {
	cout << "testApproachDirection" << endl;
	PneumaticStepper s1 = PneumaticStepper::makeTwoCylinderStepper();// (2, true);
	s1.setApproachDirection(-1);
	s1.setFrequency(-1);
	s1.setSetpoint(1);
	TEST_ASSERT_TRUE(s1.getPosition() == 0);
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == 1);
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == 2);
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == 1);
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == 1);
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == 1);
	s1.setApproachDirection(1);
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == 0);
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == 1);
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == 1);
}

void testHysteresis() {
	cout << "Testing hysteresis..." << endl;
	PneumaticStepper s1 = PneumaticStepper::makeTwoCylinderStepper();
	s1.setFrequency(-1);
	s1.setHysteresis(3);
	s1.setSetpointDouble(2.1);
	TEST_ASSERT_TRUE(s1.getSetpoint() == 0);
	s1.setSetpointDouble(3.2);
	TEST_ASSERT_TRUE(s1.getSetpoint() == 3);
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == 1);
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == 2);
	s1.setSetpointDouble(4);
	TEST_ASSERT_TRUE(s1.getSetpoint() == 2); // not changed because within hysteresis
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == 2);
	s1.setHysteresis(0.7);
	s1.setSetpointDouble(2.8);
	cout << "Position: " << s1.getPosition() << " Setpoint: " << s1.getSetpoint() << endl;
	TEST_ASSERT_TRUE(s1.getSetpoint() == 3);
	s1.work();
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == 3);
	s1.setSetpointDouble(2.4);
	TEST_ASSERT_TRUE(s1.getSetpoint() == 3);
	s1.setSetpointDouble(2.1);
	TEST_ASSERT_TRUE(s1.getSetpoint() == 2);
}

void testServoValve() {
	cout << "Testing ServoValve..." << endl;
	ServoValve v1(90);
	TEST_ASSERT_TRUE(v1.changed());
	TEST_ASSERT_TRUE(!v1.changed());
	v1.setLogicalState(0);
	v1.work();
	TEST_ASSERT_TRUE(v1.changed());
	TEST_ASSERT_TRUE(v1.getGoalAngle() == 45);
	TEST_ASSERT_TRUE(v1.getSetAngle() == 43);
	TEST_ASSERT_TRUE(v1.isMoving());
	v1.work();
	TEST_ASSERT_TRUE(!v1.changed());
	TEST_ASSERT_TRUE(v1.getSetAngle() == 43);
	waitMillis(300);
	v1.setLogicalState(0);
	TEST_ASSERT_TRUE(v1.getSetAngle() == 43);
	v1.setLogicalState(0);
	v1.work();
	TEST_ASSERT_TRUE(!v1.changed());
	TEST_ASSERT_TRUE(v1.getSetAngle() == 43);
	TEST_ASSERT_TRUE(v1.isMoving());
	waitMillis(300);
	v1.work();
	TEST_ASSERT_TRUE(v1.changed());
	TEST_ASSERT_TRUE(!v1.changed());
	v1.setLogicalState(0);
	TEST_ASSERT_TRUE(v1.getSetAngle() == 45);
	v1.setLogicalState(0);
	TEST_ASSERT_TRUE(!v1.isMoving());
	TEST_ASSERT_TRUE(v1.getSetAngle() == 45);
	v1.setLogicalState(1);
	TEST_ASSERT_TRUE(v1.changed());
	v1.work();
	TEST_ASSERT_TRUE(v1.getSetAngle() == 137);
	v1.setLogicalState(1);
	v1.work();
	TEST_ASSERT_TRUE(!v1.changed());
	TEST_ASSERT_TRUE(v1.getSetAngle() == 137);
	TEST_ASSERT_TRUE(v1.getSetAngle() == 137);
	v1.setLogicalState(2);
	TEST_ASSERT_TRUE(v1.changed());
	TEST_ASSERT_TRUE(v1.getSetAngle() == 88);
	waitMillis(300);
	v1.work();
	TEST_ASSERT_TRUE(!v1.changed());
	TEST_ASSERT_TRUE(v1.getSetAngle() == 88);
	waitMillis(300);
	v1.work();
	TEST_ASSERT_TRUE(v1.changed());
	TEST_ASSERT_TRUE(v1.getSetAngle() == 90);

	ServoValve v2(60);
	v2.setLogicalState(0);
	TEST_ASSERT_TRUE(v2.getSetAngle() == 13);
	v2.setLogicalState(2);
	TEST_ASSERT_TRUE(v2.getSetAngle() == 62);
	v2.setLogicalState(1);
	TEST_ASSERT_TRUE(v2.getSetAngle() == 107);
	v2.setLogicalState(2);
	TEST_ASSERT_TRUE(v2.getGoalAngle() == 60);
	TEST_ASSERT_TRUE(v2.getSetAngle() == 58);
	waitMillis(600);
	v2.work();
	TEST_ASSERT_TRUE(v2.getSetAngle() == 60);

	ServoValve v3(90, 70, 110, 0, 1800, 3, 200, 0);
	TEST_ASSERT_TRUE(v3.getSetAngle() == 70);
	TEST_ASSERT_TRUE(v3.getPulse() == 700);
	v3.setLogicalState(1);
	TEST_ASSERT_TRUE(v3.getSetAngle()==113);
	waitMillis(100);
	v3.work();
	TEST_ASSERT_TRUE(v3.getSetAngle()==113);
	TEST_ASSERT_TRUE(v3.getPulse() == 1130);
	waitMillis(150);
	v3.work();
	TEST_ASSERT_TRUE(v3.getSetAngle()==110);
}

void testSetPhase() {
	cout << "Testing SetPhase..." << endl;
	PneumaticStepper s = PneumaticStepper::makeTwoCylinderStepper();
	waitMillis(1000);
	s.work(); // 00
	TEST_ASSERT_TRUE(!s.getCylinderState(0));
	TEST_ASSERT_TRUE(!s.getCylinderState(1));

	s.setPhaseNr(1);
	waitMillis(1000);
	s.work(); // 01
	TEST_ASSERT_TRUE(s.getPosition() == 0);
	TEST_ASSERT_TRUE(s.getPhaseNr() == 1);
	TEST_ASSERT_TRUE(s.getCylinderState(0));
	TEST_ASSERT_TRUE(!s.getCylinderState(1));

	s.setPosition(2);
	s.setSetpoint(2);
	waitMillis(1000);
	s.work(); // 01
	TEST_ASSERT_TRUE(s.getPosition() == 2);
	TEST_ASSERT_TRUE(s.getPhaseNr() == 1);
	TEST_ASSERT_TRUE(s.getCylinderState(0));
	TEST_ASSERT_TRUE(!s.getCylinderState(1));

	s.setSetpoint(3);
	waitMillis(1000);
	s.work(); // 11
	TEST_ASSERT_TRUE(s.getPosition() == 3);
	TEST_ASSERT_TRUE(s.getPhaseNr() == 2);
	TEST_ASSERT_TRUE(s.getCylinderState(0));
	TEST_ASSERT_TRUE(s.getCylinderState(1));

	s.setSetpoint(1);
	waitMillis(1000);
	s.work(); // 01
	TEST_ASSERT_TRUE(s.getPosition() == 2);
	TEST_ASSERT_TRUE(s.getPhaseNr() == 1);
	TEST_ASSERT_TRUE(s.getCylinderState(0));
	TEST_ASSERT_TRUE(!s.getCylinderState(1));
	waitMillis(1000);
	s.work(); // 00
	TEST_ASSERT_TRUE(s.getPosition() == 1);
	TEST_ASSERT_TRUE(s.getPhaseNr() == 0);
	TEST_ASSERT_TRUE(!s.getCylinderState(0));
	TEST_ASSERT_TRUE(!s.getCylinderState(1));
	waitMillis(1000);
	s.work();
	waitMillis(1000);
	s.work();
	TEST_ASSERT_TRUE(!s.getCylinderState(0));
	TEST_ASSERT_TRUE(!s.getCylinderState(1));
}



void testPneumaticStepper() {

    RUN_TEST(testBasics);
	RUN_TEST(testTiming);
	RUN_TEST(testCylinderStrategy);
	RUN_TEST(testOperation);
	RUN_TEST(testApproachDirection);
	RUN_TEST(testHysteresis);
	RUN_TEST(testServoValve);
	RUN_TEST(testSetPhase);

}

