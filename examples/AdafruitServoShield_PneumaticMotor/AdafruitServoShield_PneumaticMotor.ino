/* 
 * Pneumatic stepper motor control using Adafruit servo shield
 * Video: https://youtu.be/523QuGY_VwU
 * Example uses three valves, connected at ports #0, #1, #2 of the servo shield
 * Analog A0: frequency (0.1..10 Hz)
 * Analog A1: setpoint position (0..127)
 * Digital 3: if pressed then motor is floating (all valves down). If pressed during boot: different control strategy is used.
 * Serial communication is at 115200 baud
 */

#include <Adafruit_PWMServoDriver.h>
#include <PneumaticStepper.h>
#include <ServoValve.h>

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();
PneumaticStepper stepper = PneumaticStepper::ThreeCylinderStepper; // change to TwoCylinderStepper for Bourke engine or T-63 like motors

#define VALVE_COUNT 3
ServoValve valves[] = {
  ServoValve(77,32,122,140,536), // Neutral angle, low angle, high angle, pulsemin, pulsemax (calibrate using e.g. AdafruitServoShield_ServoCalibration)
  ServoValve(77,32,122,140,536),
  ServoValve(79,34,124,140,536)
};

void setup() {
  Serial.begin(115200);
  Serial.println("Pneumatic stepper motor control with Adafruit Servo Shield");
  servoDriver.begin(); 
  servoDriver.setPWMFreq(60);
  pinMode(2,INPUT_PULLUP);
  if (digitalRead(2)==LOW) {
    stepper.setCylinderStrategy(PneumaticStepper::SINGLE_ENGAGE_ONLY);
  } else {
    stepper.setCylinderStrategy(PneumaticStepper::ANY_ENGAGE);
  }
  delay(10);
}

void loop() {
  // get frequency setting from analog 0
  int analog0 = analogRead(A0);
  float freq = 0.1 * exp(log(100)*analog0/1023.0); // range 0.1..10
  stepper.setFrequency(freq);

  // check if button is pressed, in that case the motor is floating (all valves down)
  bool floating = (digitalRead(2)==LOW);
  if (floating != stepper.isFloating()) {
    stepper.setFloating(floating);
    if (!floating) {
      stepper.setPosition(stepper.getSetpoint());
    }
  }

  // get setpoint setting from analog 1 (0..1023), dividing it by eight to (0..127)
  int analog1 = analogRead(A1);
  float setpointValueF = analog1 / 8.0;
  // round it towards current position to avoid jitter in case analogRead oscillates between neighbouring values
  int setpointValue = stepper.getPosition() + (int)(setpointValueF-stepper.getPosition());
  stepper.setSetpoint(setpointValue);
  
  stepper.work();
  bool stepperChanged = stepper.changed();
  if (stepperChanged) {
    stepper.printState();
  }
  for (int valveNr=0;valveNr<VALVE_COUNT;valveNr++) {
    if (stepperChanged) {
      valves[valveNr].setLogicalState(stepper.getCylinderState(valveNr));
    }
    valves[valveNr].work();
    if (valves[valveNr].changed()) {
      servoDriver.setPWM(valveNr,0,valves[valveNr].getPulse());
      valves[valveNr].printState(valveNr);
    }
  }
}
