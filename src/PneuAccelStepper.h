/* PneuAccelStepper.h - Library that adds acceleration control to the existing PneumaticStepper library made by Vincent Groenhuis.
   This library allows handling pneumatic stepper motors in the same way as the existing library.
   A setpoint can be set and the stepper will ramp its speed up and down according to a fixed acceleration value,
   up to a maximum frequency.

   When changing the acceleration or the setpoint while the stepper is running, the stepper might overshoot and then go back to reach the setpoint.
   If this is not wanted, change the setpoint and acceleration only in standstill, or set the acceleration value such that no overshoot occurs.

   Parts of this code were copied and adopted from the AccelStepper library (https://www.airspayce.com/mikem/arduino/AccelStepper/index.html) made by Mike McCauley

   By Matthias Bac; modified by Vincent Groenhuis
*/

#ifndef PNEUACCEL_STEPPER_H
#define PNEUACCEL_STEPPER_H

#include "PneumaticStepper.h"

class PneuAccelStepper : public PneumaticStepper {
  public:
    PneuAccelStepper(int nCylinder, bool doubleActing, bool triState = false, int approachDirection = 0, CylinderStrategy cylinderStrategy = ANY_ENGAGE, float frequency = 10, long position = 0, long setpoint = 0, int phaseNr = 0, bool running = true, float hysteresis = 0);

    // Returns a two-cylinder, double-acting stepper with default strategy
    //static PneuAccelStepper TwoCylinderAccelStepper;
    // Returns a three-cylinder, single-acting stepper with default strategy
    //static PneuAccelStepper ThreeCylinderAccelStepper;

    static PneuAccelStepper makeTwoCylinderAccelStepper() { return PneuAccelStepper(2, true); }
    static PneuAccelStepper makeThreeCylinderAccelStepper() { return PneuAccelStepper(3, false); }

    void setSetpointDouble(double setpoint); // override;
    void setSetpoint(long setpoint); // override;
    void setFrequency(float frequency); // override;
    bool work(); // override;
    void workUntilNoChange(); // override;
    void setAcceleration(float acceleration);
    float getAcceleration() const { return _acceleration; }
    long distanceToGo();
    void printState() const;
    long getStepsToStop() const;
    //void printSpeed();
    //void printn();
  private:
    unsigned long computeNewSpeed(bool nIncrease);
    typedef enum
    {
      DIRECTION_CCW = -1,  ///< Counter-Clockwise
      DIRECTION_CW  = 1   ///< Clockwise
    } Direction;
    /// The current motos speed in steps per second
    /// Positive is clockwise
    Direction      _direction;
    float          _speed;         // Steps per second

    /// The acceleration to use to accelerate or decelerate the motor in steps
    /// per second per second. Must be > 0
    float          _acceleration;

    /// The last step time in microseconds
    //unsigned long  _lastStepTime;

    /// The step counter for speed calculations
    long _n;

    /// Initial step size in microseconds based on acceleration only (not on frequency)
    float _c0;

    /// Last step size in microseconds
    float _cn;

    /// Min step size in microseconds based on maxSpeed
    float _cmin; // at max speed

    typedef enum {
      OVERSHOOT,
      JUMP_TO_ZERO,
      JUMP_TO_OPPOSITE
    } OvershootType;

    OvershootType overshootEnabled;//JUMP_TO_OPPOSITE;
};






#endif
