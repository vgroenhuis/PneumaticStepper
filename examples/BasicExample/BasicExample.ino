/*
 * This minimal example controls a two-cylinder stepper motor using pneumatic valves which are controlled by Arduino pins 2 and 3.
 * A transistor (e.g. TIP120) can be used to switch a high-voltage (e.g. 24V) valve using Arduino's low-voltage logic signals (3.3V or 5V).
 * In this example the motor is configured to move 100 steps in forward direction, at a stepping frequency of two steps per second.
 * Example suitable stepper motors are the T-63, T-49 etc.
 * A two-cylinder stepper motor has two cylinders which are controlled in quadrature fashion. So the successive cylinder states are 00, 01, 11, 10, 00, 01, 11, 10 etc.
 * To use a three-cylinder stepper motor, use PneumaticStepper::ThreeCylinderStepper, or the more generic PneumaticStepper constructor.
 */

#include "PneumaticStepper.h"

PneumaticStepper motor = PneumaticStepper::TwoCylinderStepper; // This variable models a two-cylinder stepper motor and keeps track of position, stepping frequency and other attributes

void setup() {
  motor.setFrequency(2); // Set stepping frequency to two steps per second
  motor.setSetpoint(100); // Set setpoint position to +100 steps

  pinMode(2,OUTPUT); // Use Arduino digital pins 2 and 3 for controlling the valves
  pinMode(3,OUTPUT);  
}

void loop() {
  motor.work(); // This function takes care of proper motor control and cylinder states
  digitalWrite(2,motor.getCylinderState(0)); // Copy motor's cylinder state to output pin
  digitalWrite(3,motor.getCylinderState(1));
}
