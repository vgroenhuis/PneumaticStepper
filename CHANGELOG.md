# Changelog
PneumaticStepper library

## [2.1.0] - 2026-02-05
Implemented dedicated velocity control. Previously this was mimicked by setting the setpoint position to a very high (or very low) value, but this was not the most elegant solution. Now the velocity control has been implemented.

## [2.0.0] - 2026-01-04
Major overhaul of PneumaticStepper library.
PneuAccelStepper has been removed and its functionality integrated in PneumaticStepper.
Algorithms have been rewritten, resulting in better behaviour when parameters (velocity, setpoint) change during motion.
Frequency has been renamed to velocity.
Several other breaking changes. Test scrips have been adapted accordingly.
Example "WiFiDashboardPneumaticStepper" added.

## [1.1.0] - 2025-12-30
Added fields in library.json
Added test runners

## [1.0.15] - 2025-08-25
One example fixed, one new example created, three examples renamed to .txt as those have to be repaired first.

## [1.0.14] - 2025-08-24
Library now managed in PlatformIO.
Changelog file created. For earlier changes, see github commit messages.
