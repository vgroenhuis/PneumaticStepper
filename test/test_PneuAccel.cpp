#include "test_PneuAccel.h"

#include <unity.h>
#include <iostream>
#include <iomanip>
#include <math.h>
using namespace std;

#include "test_common.h"

#include "PneuAccelStepper.h"



void runMotorSimpleProfile() {
    cout << "\033[34mrunMotorSimpleProfile\033[0m" << endl;
    resetClock();
    
    PneuAccelStepper stepper = PneuAccelStepper::makeTwoCylinderAccelStepper();
    //stepper.setPosition(0);
    stepper.setAcceleration(10); // in steps/s^2
    stepper.setFrequency(10); // in Hz
    stepper.setSetpoint(200); // in steps


    unsigned long previousStepMillis = millis();

    //cout << "t=" << fixed << setprecision(4) << millis()/1000.0 << " position=" << stepper.getPosition() << endl;

    stepper.printState();

    while (/*stepper.getPosition() != stepper.getSetpoint() && */millis()<100000) {
        bool change = stepper.work();
        if (change) {
            float dt = (millis() - previousStepMillis) / 1000.0;
            //cout << "t=" << fixed << setprecision(4) << millis()/1000.0 << " f=" << (1.0/dt) << " position=" << stepper.getPosition() << endl;
            stepper.printState();
            previousStepMillis = millis();
            TEST_ASSERT_TRUE(1.0/dt <= stepper.getFrequency()+0.1);
        }
        waitMillis(1);
    }

    TEST_ASSERT_TRUE(stepper.getPosition() == stepper.getSetpoint());
}

void runMotorChangeSetpointMidway() {
    cout << "\033[34mrunMotorChangeSetpointMidway\033[0m" << endl;

    resetClock();
    PneuAccelStepper stepper = PneuAccelStepper::makeTwoCylinderAccelStepper();
    stepper.setPosition(0);
    stepper.setAcceleration(20); // in steps/s^2
    stepper.setFrequency(10); // in Hz
    stepper.setSetpoint(200); // in steps


    unsigned long previousStepMillis = millis();

    while (millis()<100000) {
        bool change = stepper.work();
        if (change) {
            float dt = (millis() - previousStepMillis) / 1000.0;
            cout << "t=" << fixed << setprecision(4) << millis()/1000.0 << " f=" << (1.0/dt) << " position=" << stepper.getPosition() << endl;
            previousStepMillis = millis();
            TEST_ASSERT_TRUE(1.0/dt <= stepper.getFrequency()+0.1);
        }
        if (stepper.getPosition() == 100 && stepper.getSetpoint() == 200) {
            stepper.setSetpoint(-100);
            cout << "Changed setpoint to -100 at t=" << millis()/1000.0 << endl;
        }
        waitMillis(1);
    }

    TEST_ASSERT_TRUE(stepper.getPosition() == stepper.getSetpoint());
}


void runPneuAccelTests() {
    RUN_TEST(runMotorSimpleProfile);
    RUN_TEST(runMotorChangeSetpointMidway);
}
