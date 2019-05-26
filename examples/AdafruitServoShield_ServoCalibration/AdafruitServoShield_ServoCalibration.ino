// Calibration of servo in Adafruit servo shield
// Connect a potmeter to A0 and a servo to the first port of the servo shield
// First calibrate the min and max pulse lengths such that the servo travel is 180 deg while respecting the hard endstops. These values can be used for all servos of this brand/type.
// Next, calibrate the low/high/neutral angles for each individual servo and use these values in the ServoValve constructor.
// Vincent Groenhuis
// May, 2019
//
// Analog A0: angle (0..180)

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  120 // this is the 'minimum' pulse length count (out of 4096), default 150 (~0.6 ms)
#define SERVOMAX  520 // this is the 'maximum' pulse length count (out of 4096), default 600 (~2.4 ms)

void setup() {
  Serial.begin(115200);
  Serial.println("Calibration of servo on Adafruit Servo Shield. Use potmeter to A0 and servo to port #0");
  pwm.begin(); 
  pwm.setPWMFreq(60);
  delay(10);
}

void loop() {
  int potmeter = analogRead(A0);
  int angle = map(potmeter, 0, 1023, 0, 180);
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(0, 0, pulse);

  Serial.print("angle=");
  Serial.print(angle);
  Serial.print(" pulse=");
  Serial.println(pulse);
  delay(100);
}
