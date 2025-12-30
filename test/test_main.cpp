#include <unity.h>
#include "test_pneumatic_stepper.h"
#include "test_PneuAccel.h"

int main(int argc, char **argv) {
    UNITY_BEGIN();

    testPneumaticStepper();
  	runPneuAccelTests();

    UNITY_END();
}