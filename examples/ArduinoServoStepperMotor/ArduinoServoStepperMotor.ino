#include <PneumaticStepper.h>
#include <Servo.h>

// Pneumatic three-cylinder stepper motor controlled by RC servo connected to Arduino output pins 2, 3 and 4
// by Vincent Groenhuis
// Servo-controlled valves on Thingiverse: [TODO]
// Frequency is 2 Hz, setpoint at 1000

// Created May 2019
// Not tested after last major changes!

PneumaticStepper stepper(3, false, true, 0, PneumaticStepper::SINGLE_ENGAGE_ONLY, 2, 0, 1000, 0, true);

Servo myServo[3];
int servoAngles[3][3] = {{45,135,90},{45,135,90},{45,135,90}}; // angles for low, hi and central positions for each servo

void setup()
{
	myServo[0].attach(2,544,2400); // default: 0 deg = 544 us, 180 deg = 2400 us
	myServo[1].attach(3,544,2400);
	myServo[2].attach(4,544,2400);
}

void loop()
{
	stepper.work();
	if (stepper.changed()) {
		for (int i=0;i<3;i++) {
      myServo[i].write(servoAngles[i][stepper.getCylinderState(i)]);
		}
    stepper.printState();
	}
}
