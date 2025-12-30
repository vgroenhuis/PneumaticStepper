#pragma once

#include <iostream>
#include <iomanip>
using namespace std;

extern long _millis;
extern long _micros;

extern unsigned long millis();
extern unsigned long micros();
extern void resetClock();

extern void waitMillis(unsigned long d);

extern long map(long x, long in_min, long in_max, long out_min, long out_max);