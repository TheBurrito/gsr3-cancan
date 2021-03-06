#ifndef CircularBuffer_H_
#define CircularBuffer_H_

#include <stdint.h>
#include <stdlib.h>

template <typename T>
class CircularBuffer {
public:
	CircularBuffer();
	CircularBuffer(int size);
	~CircularBuffer();
	
	void reset();
	
	bool insertFront(const T& b);
	bool pushBack(const T& b);
	bool pullBack(T &b);
	bool pullFront(T &b);
	
	bool removeFront();
	
	const T& operator[](int i);
	
	int size();
	bool empty();
	
private:
	void initBuffer(int size);
	void freeBuffer();
	
	T *_buf;
	int _head, _tail, _size;
};

template <typename T>
CircularBuffer<T>::CircularBuffer() {
	initBuffer(100);
}

template <typename T>
CircularBuffer<T>::CircularBuffer(int size) {
	initBuffer(size);
}

template <typename T>
void CircularBuffer<T>::initBuffer(int size) {
    _buf = (T *)malloc(size * sizeof(T));
    _head = 0;
    _tail = 0;
    _size = size;
}

template <typename T>
CircularBuffer<T>::~CircularBuffer() {
    free(_buf);
}

template <typename T>
void CircularBuffer<T>::reset() {
    _head = 0;
    _tail = 0;
}

template <typename T>
const T& CircularBuffer<T>::operator[](int i) {
	return _buf[(i + _tail) % _size];
}

template <typename T>
bool CircularBuffer<T>::insertFront(const T& b) {
	int t = _tail - 1;
	if (t < 0) t += _size;
	bool loss = false;
	
	if (t == _head) {
		loss = true;
		--_head;
		if (_head < 0) _head += _size;
	}
	
	_buf[t] = b;
	
	return loss;
}

template <typename T>
bool CircularBuffer<T>::empty() {
	return (_head == _tail);
}

template <typename T>
int CircularBuffer<T>::size() {
	int s = _head - _tail;
	if (s < 0) s += _size;
	return s;
}

template <typename T>
bool CircularBuffer<T>::pushBack(const T& b) {
    _buf[_head++] = b;
    
    if (_head == _size) {
        _head = 0;
    }
    
    if (_head == _tail) {
        ++_tail;
        if (_tail == _size) {
            _tail = 0;
        }
        
        return true;
    }
    
    return false;
}

template <typename T>
bool CircularBuffer<T>::removeFront() {
    if (_head == (_tail + 1) % _size) {
        return false;
    }
    
    ++_head;    
    if (_head == _size) {
        _head = 0;
    }
    
    return true;
}

template <typename T>
bool CircularBuffer<T>::pullBack(T &b) {
    if (_head == _tail) {
        return false;
    }
    
    b = _buf[_tail++];
    
    if (_tail == _size) {
        _tail = 0;
    }
    
    return true;
}

#endif //CircularBuffer_H_
