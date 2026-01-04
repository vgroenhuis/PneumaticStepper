#include "test_common.h"

long _millis = 0;
long _micros = 0;

unsigned long millis() {
	return _millis;
}

unsigned long micros() {
	return _micros;
}

void resetClock() {
	_millis = 0;
	_micros = 0;
}

void delay(unsigned long d) {
	_millis += d;
	_micros += 1000 * d;
}

void waitMillis(unsigned long d) {
	_millis += d;
	_micros += 1000 * d;
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}