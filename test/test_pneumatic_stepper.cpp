#include "test_pneumatic_stepper.h"

#include <unity.h>

#include "test_common.h"

#include "PneumaticStepper.h"
#include "ServoValve.h"

#include <array>

void setUp(void)
{
	// set stuff up here
}

void tearDown(void)
{
	// clean stuff up here
}

// Validates phase for n=2 stepper motor
void validateCylinderState(std::vector<uint8_t> state, std::vector<uint8_t> test)
{
	TEST_ASSERT_EQUAL_UINT8_ARRAY(state.data(), test.data(), state.size());
	/*
		for (int i=0; i<test.size(); i++) {
			TEST_ASSERT_EQUAL(state[i],test[i]);
		}
	*/
}

void testBasics()
{
	bool changed;

	cout << "Testing PneumaticStepper basics..." << endl;
	PneumaticStepper s1 = PneumaticStepper::makeThreeCylinderStepper(); // Constructor with default parameters
	TEST_ASSERT_EQUAL(3, s1.getCylinderCount());
	TEST_ASSERT_TRUE(!s1.isDoubleActing());
	TEST_ASSERT_TRUE(!s1.isTriState());
	TEST_ASSERT_EQUAL(0, s1.getApproachDirection());
	TEST_ASSERT_EQUAL_FLOAT(0, abs(s1.getMaxVelocity() - 10));
	TEST_ASSERT_EQUAL(0, s1.getPosition());
	TEST_ASSERT_EQUAL(0, s1.getSetpointPosition());
	TEST_ASSERT_TRUE(s1.isPositionValid());
	TEST_ASSERT_EQUAL(0, s1.getPhaseNr());
	TEST_ASSERT_TRUE(!s1.isFloating());
	TEST_ASSERT_TRUE(s1.testResetRoundedPositionChanged());
	TEST_ASSERT_TRUE(!s1.testResetRoundedPositionChanged());
	TEST_ASSERT_EQUAL(0, s1.getLastStepDir());
	TEST_ASSERT_TRUE(s1.isRunning());
	validateCylinderState(s1.getCylinderStates(), {1, 0, 0});

	// Test floating
	s1.setFloating(true);
	TEST_ASSERT_TRUE(s1.isFloating());
	TEST_ASSERT_FALSE(s1.isPositionValid());

	s1.setFloating(false);
	TEST_ASSERT_FALSE(s1.isFloating());
	TEST_ASSERT_FALSE(s1.isPositionValid());
	s1.setPosition(0);
	TEST_ASSERT_FALSE(s1.isFloating());
	TEST_ASSERT_TRUE(s1.isPositionValid());
	TEST_ASSERT_TRUE(s1.getPositionError() == 0);
	s1.setSetpointPosition(2);
	TEST_ASSERT_TRUE(s1.getPositionError() == 2);
	TEST_ASSERT_TRUE(s1.testResetRoundedPositionChanged());

	s1.setMaxVelocity(1000); // steps/sec
	s1.printState();
	printf("Pausing...\n");
	s1.pause();
	s1.printState();
	printf("Stepping motor while paused:\n");
	changed = s1.workUntilRoundedPositionChanged();
	TEST_ASSERT_FALSE(changed); // motor is paused, so velocity stays zero
	TEST_ASSERT_EQUAL(0, s1.getPosition());
	s1.run();
	changed = s1.workUntilRoundedPositionChanged(); // advances by one step (rounded)
	TEST_ASSERT_TRUE(changed);
	TEST_ASSERT_TRUE(s1.testResetRoundedPositionChanged());
	TEST_ASSERT_EQUAL(1, s1.getRoundedPosition());
	s1.pause(); // it still has some velocity, so position will continue to increase, but drop to zero sufficiently quickly
	changed = s1.workUntilRoundedPositionChanged();
	TEST_ASSERT_FALSE(changed);
	s1.printState();
	TEST_ASSERT_EQUAL(1, s1.getRoundedPosition()); // is 0 (pos=0.496)
	s1.run();
	s1.workUntilRoundedPositionChanged();
	TEST_ASSERT_EQUAL(2, s1.getRoundedPosition());
	// test array of motors
	PneumaticStepper myMotor[2] = {PneumaticStepper::makeTwoCylinderStepper(), PneumaticStepper::makeThreeCylinderStepper()};

	// test copy constructor
	s1 = PneumaticStepper::makeTwoCylinderStepper();
}

void testTiming()
{
	cout << "Testing timing..." << endl;
	PneumaticStepper s1 = PneumaticStepper::makeTwoCylinderStepper(); // 10 Hz
	s1.resetLastChangeTime();
	s1.setSetpointPosition(-10);
	s1.printState();
	waitMillis(50);

	s1.work();
	TEST_ASSERT_EQUAL(0, s1.getRoundedPosition());
	waitMillis(100);
	s1.work();
	TEST_ASSERT_EQUAL(-1, s1.getRoundedPosition());
	s1.work();
	TEST_ASSERT_EQUAL(-1, s1.getRoundedPosition());
	waitMillis(100);
	s1.work();
	TEST_ASSERT_EQUAL(-2, s1.getRoundedPosition());
	s1.work();
	TEST_ASSERT_EQUAL(-2, s1.getRoundedPosition());
	waitMillis(10);
	s1.work();
	TEST_ASSERT_EQUAL(-2, s1.getRoundedPosition());

	PneumaticStepper s2 = PneumaticStepper::makeThreeCylinderStepper();
	s2.setCylinderStrategy(PneumaticStepper::CylinderStrategy::SINGLE_ENGAGE_ONLY);
	s2.resetLastChangeTime();
	s2.setSetpointPosition(10);
	waitMillis(50);
	// s2 has position 0, setpoint 10
	s2.workUntilRoundedPositionChanged();
	s2.testResetRoundedPositionChanged();
	TEST_ASSERT_EQUAL(0, s2.getRoundedPosition());
	s2.workUntilRoundedPositionChanged();
	s2.testResetRoundedPositionChanged();
	TEST_ASSERT_EQUAL(1, s2.getRoundedPosition());
	s2.workUntilRoundedPositionChanged();
	s2.testResetRoundedPositionChanged();
	TEST_ASSERT_EQUAL(2, s2.getRoundedPosition());

	PneumaticStepper s3 = PneumaticStepper::makeTwoCylinderStepper();
	s3.resetLastChangeTime();
	s3.setSetpointPosition(0);
	s3.setMaxVelocity(1); // max 1 steps/sec
	waitMillis(1900);
	s3.setSetpointPosition(10);
	s3.testResetRoundedPositionChanged();
	s3.work();
	s3.testResetRoundedPositionChanged();
	TEST_ASSERT_EQUAL(0, s3.getRoundedPosition());
	s3.workUntilRoundedPositionChanged();
	s3.testResetRoundedPositionChanged();
	TEST_ASSERT_EQUAL(1, s3.getRoundedPosition());
	waitMillis(200);
	s3.work();
	TEST_ASSERT_EQUAL(1, s3.getRoundedPosition());
	waitMillis(700);
	s3.work();
	TEST_ASSERT_EQUAL(1, s3.getRoundedPosition());
	waitMillis(200);
	s3.work();
	TEST_ASSERT_EQUAL(2, s3.getRoundedPosition());
	//
}

void testCylinderStrategy()
{
	cout << "Testing cylinder strategies..." << endl;
	// three-cylinder single-acting, single-engage only, ignore frequency
	PneumaticStepper s1(3, false, false, 0, PneumaticStepper::CylinderStrategy::SINGLE_ENGAGE_ONLY, -1, 0, 0, 0, true);
	TEST_ASSERT_EQUAL(s1.getCylinderStrategy(), PneumaticStepper::CylinderStrategy::SINGLE_ENGAGE_ONLY);
	s1.setSetpointPosition(3);
	TEST_ASSERT_EQUAL(s1.getSetpointPosition(), 2); // actual setpoint is even

	uint8_t piii[][3] = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}, {1, 0, 1}};
	TEST_ASSERT_EQUAL_UINT8_ARRAY(piii[0], s1.getCylinderStates().data(), 3);
	TEST_ASSERT_EQUAL(0, s1.getPosition());
	s1.testResetRoundedPositionChanged();
	s1.workUntilRoundedPositionChanged();
	TEST_ASSERT_EQUAL(1, s1.getPosition());
	TEST_ASSERT_EQUAL_UINT8_ARRAY(piii[0], s1.getCylinderStates().data(), 3);
	s1.testResetRoundedPositionChanged();
	s1.workUntilRoundedPositionChanged();
	TEST_ASSERT_EQUAL(2, s1.getPosition());
	TEST_ASSERT_EQUAL_UINT8_ARRAY(piii[2], s1.getCylinderStates().data(), 3);
	s1.testResetRoundedPositionChanged();
	s1.workUntilRoundedPositionChanged();
	TEST_ASSERT_EQUAL(2, s1.getPosition()); // failed
	TEST_ASSERT_EQUAL_UINT8_ARRAY(piii[2], s1.getCylinderStates().data(), 3);
	s1.testResetRoundedPositionChanged();
	s1.workUntilRoundedPositionChanged();
	TEST_ASSERT_EQUAL_UINT8_ARRAY(piii[2], s1.getCylinderStates().data(), 3);

	TEST_ASSERT_EQUAL(0, s1.getCylinderState(0));
	TEST_ASSERT_EQUAL(1, s1.getCylinderState(1));
	TEST_ASSERT_EQUAL(0, s1.getCylinderState(2));

	TEST_ASSERT_TRUE(s1.getPosition() == 2);
	// tricky: redefine position
	s1.setPosition(1);
	TEST_ASSERT_EQUAL(1, s1.getPosition());
	TEST_ASSERT_EQUAL(2, s1.getPhaseNr());
	validateCylinderState(s1.getCylinderStates(), {0, 1, 0});
	s1.setSetpointPosition(3);
	TEST_ASSERT_EQUAL(3, s1.getSetpointPosition());
}

// Performs ten steps while printing state before each step
void playMotor(PneumaticStepper &stepper)
{
	for (int i = 0; i < 10; i++)
	{
		stepper.testResetRoundedPositionChanged();
		stepper.workUntilRoundedPositionChanged();
#ifndef PNEU_DEBUG
//		stepper.printState();
#endif
		// waitMillis(10);
	}
}

void testOperation()
{
	cout << "testOperation: requires manual inspection of motor state prints" << endl;
	cout << "s1: three-cylinder, double-acting stepper" << endl;
	PneumaticStepper s1(3, true);
	s1.setMaxVelocity(100);
	s1.setAcceleration(1000);
	s1.setSetpointPosition(5);
	playMotor(s1);
	cout << "s2: default three-cylinder single-acting stepper, single-engage strategy" << endl;
	PneumaticStepper s2 = PneumaticStepper::makeThreeCylinderStepper();
	s2.setMaxVelocity(100);
	s2.setAcceleration(1000);
	s2.setSetpointPosition(8);
	s2.setCylinderStrategy(PneumaticStepper::CylinderStrategy::SINGLE_ENGAGE_ONLY);
	playMotor(s2);
	cout << "s3: three-cylinder double-acting, single-engage only" << endl;
	PneumaticStepper s3(3, true, true, 0, PneumaticStepper::CylinderStrategy::SINGLE_ENGAGE_ONLY, 100, 0, 0, 0, true);
	s3.setSetpointPosition(10);
	playMotor(s3);
	cout << "s4: two-cylinder double-acting" << endl;
	PneumaticStepper s4 = PneumaticStepper::makeTwoCylinderStepper();
	s4.setMaxVelocity(100);
	s4.setAcceleration(1000);
	s4.setSetpointPosition(-3);
	playMotor(s4);
}

void testApproachDirection()
{
	cout << "testApproachDirection" << endl;
	PneumaticStepper s1 = PneumaticStepper::makeTwoCylinderStepper(); // (2, true);
	s1.setApproachDirection(-1);
	s1.setMaxVelocity(-1);
	s1.setSetpointPosition(1);
	TEST_ASSERT_EQUAL(0, s1.getRoundedPosition());
	s1.work();
	TEST_ASSERT_EQUAL(1, s1.getRoundedPosition());
	s1.work();
	TEST_ASSERT_EQUAL(2, s1.getRoundedPosition());
	s1.work();
	TEST_ASSERT_EQUAL(1, s1.getRoundedPosition());
	s1.work();
	TEST_ASSERT_EQUAL(1, s1.getRoundedPosition());
	s1.work();
	TEST_ASSERT_EQUAL(1, s1.getRoundedPosition());
	s1.setApproachDirection(1);
	s1.work();
	TEST_ASSERT_EQUAL(0, s1.getRoundedPosition());
	s1.work();
	TEST_ASSERT_EQUAL(1, s1.getRoundedPosition());
	s1.work();
	TEST_ASSERT_EQUAL(1, s1.getRoundedPosition());
}

/*
void testHysteresis() {
	cout << "Testing hysteresis..." << endl;
	PneumaticStepper s1 = PneumaticStepper::makeTwoCylinderStepper();
	s1.setMaxVelocity(1000);
	s1.setSetpointPosition(2.1);
	TEST_ASSERT_TRUE(s1.getSetpointPosition() == 0);
	s1.setSetpointPosition(3.2);
	TEST_ASSERT_TRUE(s1.getSetpointPosition() == 3);
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == 1);
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == 2);
	s1.setSetpointPosition(4);
	TEST_ASSERT_TRUE(s1.getSetpoint() == 2); // not changed because within hysteresis
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == 2);
	s1.setHysteresis(0.7);
	s1.setSetpointPosition(2.8);
	cout << "Position: " << s1.getPosition() << " Setpoint: " << s1.getSetpointPosition() << endl;
	TEST_ASSERT_TRUE(s1.getSetpointPosition() == 3);
	s1.work();
	s1.work();
	TEST_ASSERT_TRUE(s1.getPosition() == 3);
	s1.setSetpointPosition(2.4);
	TEST_ASSERT_TRUE(s1.getSetpointPosition() == 3);
	s1.setSetpointPosition(2.1);
	TEST_ASSERT_TRUE(s1.getSetpointPosition() == 2);
}*/

void testServoValve()
{
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
	TEST_ASSERT_TRUE(v3.getSetAngle() == 113);
	waitMillis(100);
	v3.work();
	TEST_ASSERT_TRUE(v3.getSetAngle() == 113);
	TEST_ASSERT_TRUE(v3.getPulse() == 1130);
	waitMillis(150);
	v3.work();
	TEST_ASSERT_TRUE(v3.getSetAngle() == 110);
}

void testSetPhase()
{
	cout << "Testing SetPhase..." << endl;
	PneumaticStepper s = PneumaticStepper::makeTwoCylinderStepper();
	s.setMaxVelocity(100);
	s.setAcceleration(1000);
	waitMillis(1000);
	s.work(); // 00
	TEST_ASSERT_FALSE(s.getCylinderState(0));
	TEST_ASSERT_FALSE(s.getCylinderState(1));

	s.setPhaseNr(1);
	waitMillis(1000);
	s.work(); // 01
	TEST_ASSERT_EQUAL(0, s.getRoundedPosition());
	TEST_ASSERT_EQUAL(1, s.getPhaseNr());
	TEST_ASSERT_TRUE(s.getCylinderState(0));
	TEST_ASSERT_FALSE(s.getCylinderState(1));

	s.setPosition(2);
	s.setSetpointPosition(2);
	waitMillis(1000);
	s.work(); // 01
	TEST_ASSERT_EQUAL(2, s.getRoundedPosition());
	TEST_ASSERT_EQUAL(1, s.getPhaseNr());
	TEST_ASSERT_TRUE(s.getCylinderState(0));
	TEST_ASSERT_FALSE(s.getCylinderState(1));

	s.setSetpointPosition(3);
	s.testResetRoundedPositionChanged();
	s.workUntilRoundedPositionChanged();
	s.printState();
	TEST_ASSERT_EQUAL(3, s.getRoundedPosition());
	TEST_ASSERT_EQUAL(2, s.getPhaseNr());
	TEST_ASSERT_TRUE(s.getCylinderState(0));
	TEST_ASSERT_TRUE(s.getCylinderState(1));

	s.setSetpointPosition(1);
	s.testResetRoundedPositionChanged();
	s.workUntilRoundedPositionChanged();
	TEST_ASSERT_EQUAL(2, s.getRoundedPosition());
	TEST_ASSERT_EQUAL(1, s.getPhaseNr());
	TEST_ASSERT_TRUE(s.getCylinderState(0));
	TEST_ASSERT_FALSE(s.getCylinderState(1));
	s.testResetRoundedPositionChanged();
	s.workUntilRoundedPositionChanged();
	TEST_ASSERT_EQUAL(1, s.getRoundedPosition());
	TEST_ASSERT_EQUAL(0, s.getPhaseNr());
	TEST_ASSERT_FALSE(s.getCylinderState(0));
	TEST_ASSERT_FALSE(s.getCylinderState(1));
	s.testResetRoundedPositionChanged();
	s.workUntilRoundedPositionChanged();
	s.testResetRoundedPositionChanged();
	s.workUntilRoundedPositionChanged();
	TEST_ASSERT_TRUE(!s.getCylinderState(0));
	TEST_ASSERT_TRUE(!s.getCylinderState(1));
}


void testAccelerationSimpleProfile() {
    cout << "\033[34mrunMotorSimpleProfile\033[0m" << endl;
    resetClock();
    
    PneumaticStepper stepper = PneumaticStepper::makeTwoCylinderStepper();

	stepper.setAcceleration(10); // in steps/s^2
    stepper.setMaxVelocity(10); // in Hz
    stepper.setSetpointPosition(200); // in steps


    unsigned long previousStepMillis = millis();

    stepper.printState("Start");

    while (/*stepper.getPosition() != stepper.getSetpointPosition() && */millis()<100000) {
		stepper.testResetRoundedPositionChanged();
        stepper.work();
        bool change = stepper.testResetRoundedPositionChanged();
        if (change) {
            float dt = (millis() - previousStepMillis) / 1000.0;
            //cout << "t=" << fixed << setprecision(4) << millis()/1000.0 << " f=" << (1.0/dt) << " position=" << stepper.getPosition() << endl;
            stepper.printState("Step");
            previousStepMillis = millis();
			float effectiveVelocity = 1.0/dt;

			if (effectiveVelocity > stepper.getMaxVelocity()+0.1) {
				cout << "\033[31mError: effective velocity " << effectiveVelocity << " exceeds max velocity " << stepper.getMaxVelocity() << "\033[0m" << endl;
			}
            TEST_ASSERT_LESS_OR_EQUAL(stepper.getMaxVelocity()+0.1, effectiveVelocity);
        }
        waitMillis(1);
    }

    TEST_ASSERT_EQUAL(stepper.getRoundedPosition(), stepper.getSetpointPosition());
    TEST_ASSERT_EQUAL_FLOAT(stepper.getPosition(), stepper.getSetpointPosition());
}

void testAccelerationChangeSetpointMidway() {
    cout << "\033[34mrunMotorChangeSetpointMidway\033[0m" << endl;

    resetClock();
    PneumaticStepper stepper = PneumaticStepper::makeTwoCylinderStepper();
    stepper.setPosition(0);
    stepper.setAcceleration(20); // in steps/s^2
    stepper.setMaxVelocity(10); // in steps/s
    stepper.setSetpointPosition(200); // in steps


    unsigned long previousStepMillis = millis();

    // simulate for 10 seconds
    while (millis()<100000) {
		stepper.testResetRoundedPositionChanged();
        stepper.work();
        bool change = stepper.testResetRoundedPositionChanged();
        if (change) {
            float dt = (millis() - previousStepMillis) / 1000.0; // the time interval from previous step to current step
            cout << "t=" << fixed << setprecision(4) << millis()/1000.0 << " f=" << (1.0/dt) << " position=" << stepper.getPosition() << endl;
            previousStepMillis = millis();
            float effectiveVelocity = 1.0/dt;
			stepper.printState("Step");
            TEST_ASSERT_LESS_OR_EQUAL(stepper.getMaxVelocity()+0.1, effectiveVelocity);
        }
        if (stepper.getPosition() == 100 && stepper.getSetpointPosition() == 200) {
            stepper.setSetpointPosition(-100);
            cout << "Changed setpoint to -100 at t=" << millis()/1000.0 << endl;
        }
        waitMillis(1);
    }

    TEST_ASSERT_EQUAL(stepper.getRoundedPosition(), stepper.getSetpointPosition());
}

void testVelocityControl()
{
	cout << "\033[34mtestVelocityControl\033[0m" << endl;

	resetClock();
	PneumaticStepper stepper = PneumaticStepper::makeTwoCylinderStepper();
	TEST_ASSERT_EQUAL(PneumaticStepper::Controlstrategy::POSITION_CONTROL, stepper.getControlStrategy());
	stepper.setControlStrategy(PneumaticStepper::Controlstrategy::VELOCITY_CONTROL);
	TEST_ASSERT_EQUAL(PneumaticStepper::Controlstrategy::VELOCITY_CONTROL, stepper.getControlStrategy());
	stepper.setControlStrategy(PneumaticStepper::Controlstrategy::POSITION_CONTROL);
	TEST_ASSERT_EQUAL(PneumaticStepper::Controlstrategy::POSITION_CONTROL, stepper.getControlStrategy());
	stepper.setPosition(0);
	stepper.setAcceleration(10000); // in steps/s^2
	stepper.setMaxVelocity(20); // in steps/s
	stepper.setSetpointVelocity(15); // in steps/s
	TEST_ASSERT_EQUAL(PneumaticStepper::Controlstrategy::VELOCITY_CONTROL, stepper.getControlStrategy());

	while (millis() < 1000)
	{
		stepper.work();
		waitMillis(1);
	}
	TEST_ASSERT_FLOAT_WITHIN(0.5, 15, stepper.getPosition());
	stepper.setSetpointPosition(5);
	TEST_ASSERT_EQUAL(PneumaticStepper::Controlstrategy::POSITION_CONTROL, stepper.getControlStrategy());

	while (millis() < 2000)
	{
		stepper.work();
		waitMillis(1);
		stepper.printState("Down to 5");
		//cout << "t=" << millis() << " pos=" << stepper.getPosition() << endl;
	}
	TEST_ASSERT_FLOAT_WITHIN(0.01, 5, stepper.getPosition());
}


void testPneumaticStepper()
{
	RUN_TEST(testBasics);
	RUN_TEST(testTiming);
	RUN_TEST(testCylinderStrategy);
	RUN_TEST(testOperation);
	RUN_TEST(testApproachDirection);
	RUN_TEST(testServoValve);
	RUN_TEST(testSetPhase);
    RUN_TEST(testAccelerationSimpleProfile);
    RUN_TEST(testAccelerationChangeSetpointMidway);
	RUN_TEST(testVelocityControl);
}
