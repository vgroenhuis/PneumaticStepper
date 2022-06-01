/*
 * This minimal example controls a two-cylinder stepper motor using pneumatic valves which are controlled by Arduino pins 2 and 3.
 * A transistor (e.g. TIP120) can be used to switch a high-voltage (e.g. 24V) valve using a low-voltage logic signal (3.3V or 5V).
 * In this example the motor is configured to move 100 steps in forward direction, at a stepping frequency of two steps per second.
 */

#include "PneumaticStepper.h"

PneumaticStepper motor = PneumaticStepper::TwoCylinderStepper;

void setup() {
  // put your setup code here, to run once:
  motor.setFrequency(2); // steps per second
  motor.setSetpoint(100); // 100 steps

  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);  
}

void loop() {
  // put your main code here, to run repeatedly:
  motor.work();
  digitalWrite(2,motor.getCylinderState(0));
  digitalWrite(3,motor.getCylinderState(1));
}
