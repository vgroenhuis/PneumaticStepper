/*
 * Pneumatic three-cylinder stepper motor controlled by RC servo valves connected to Arduino output pins 2, 3 and 4
 * By Vincent Groenhuis
 * Created May 2019
 * Video: https://youtu.be/523QuGY_VwU (using Adafruit servo shield)
 * Servo-controlled valves on Thingiverse: https://www.thingiverse.com/thing:3655215 so that you can make low-cost 4/2-way pneumatic valves with analog flow control.
 * In this example the steppering frequency is 2 Hz and setpoint position is +100
 * If you prefer to use a servo shield, look at the AdafruitServoShield_PneumaticStepperMotor example instead.
 * 
 */

#include <PneumaticStepper.h>
#include <Servo.h>

// Create pneumatic stepper motor with three single-acting cylinders of which one engages the rack at any given time. Stepping frequency 2 Hz, setpoint position +100 steps
PneumaticStepper stepper(3, false, true, 0, PneumaticStepper::SINGLE_ENGAGE_ONLY, 2, 0, 100, 0, true);

// Three RC servos are used as valves
Servo myServo[3];
int servoAngles[3][3] = {{45,135,90},{45,135,90},{45,135,90}}; // angles for low, hi and central positions for each servo

void setup()
{
	myServo[0].attach(2,544,2400); // Servo attached to Arduino pins 2, 3 and 4. Timing: default: 0 deg = 544 us, 180 deg = 2400 us
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
