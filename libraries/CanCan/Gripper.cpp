#include "Gripper.h"
#include <Arduino.h>
#include "gsr_lcd.h"

CGripper Gripper;

CGripper::CGripper() {
	_pin = -1;
	_open = 88;
	_closed = 36;
	_speed = 1;
	_period = 20;
	
	_lastUpdate = 0;
	
	_target = _open;
	_current = _open;
}

CGripper::~CGripper() {
	_servo.detach();
}

void CGripper::setPin(int pin) {
	if (_servo.attached()) {
		_servo.detach();
	}
	
	_servo.attach(pin);
	_pin = pin;
}

void CGripper::setOpen(int val) {
	_open = val;
}

void CGripper::setClosed(int val) {
	_closed = val;
}

void CGripper::setPeriod(unsigned long ms) {
	_period = ms;
}

void CGripper::setSpeed(int speed) {
	_speed = speed;
}

void CGripper::open() {
	_target = _open;
}

void CGripper::close() {
	_target = _closed;
}

void CGripper::moveTo(int val) {
	_target = val;
}

bool CGripper::done() {
	return (_target == _current);
}

void CGripper::update() {
	unsigned long curMillis = millis();
	
	if (_lastUpdate + _period <= curMillis) {
		_lastUpdate = curMillis;
		
		if (_current != _target) {
			if (_current < _target) {
				_current += _speed;
				if (_current > _target) _current = _target;
			} else {
				_current -= _speed;
				if (_current < _target) _current = _target;
			}
			
			_servo.write(_current);
		}
	}
}

void CGripper::quickMove() {
	_current = _target;
	_servo.write(_current);
}
