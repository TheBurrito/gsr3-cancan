#include "Path.h"

Path::Path() {
	_head = 0;
	_tail = 0;
}

Path::~Path() {
}

void Path::pushFront(const Point& p) {
	--_head;
	if (_head < 0) {
		_head = PATH_LENGTH - 1;
		if (_head == _tail) {
			--_tail;
			if (_tail < 0) {
				_tail = PATH_LENGTH - 1;
			}
		}
	}
	
	_points[_head] = p;
}

void Path::pushBack(const Point& p) {
	_tail = (_tail + 1) % PATH_LENGTH;
	if (_tail == _head) {
	}
}

const Point& Path::operator[](int i) {
	return _points[(i + _head) % PATH_LENGTH];
}

void Path::removeFront() {
	if (_head == _tail) {
		return;
	}
	
	_head = (_head + 1) % PATH_LENGTH;
}

void Path::removeBack() {
	if (_head == _tail) {
		return;
	}
	
	--_tail;
}

int Path::size() {
	int s = _tail - _head;
	if (s < 0) s += PATH_LENGTH;
	return s;
}
