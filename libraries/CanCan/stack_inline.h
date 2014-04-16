template <typename T>
Stack<T>::Stack() {
	_pos = 0;
}

template <typename T>
Stack<T>::~Stack() {
}

template <typename T>
bool Stack<T>::push(const T& val) {
	if (_pos < STACK_CAPACITY) {
		_vals[_pos++] = val;
		return true;
	}
	
	return false;
}

template <typename T>
bool Stack<T>::pop(T& val) {
	if (_pos > 0) {
		val = _vals[--_pos];
		return true;
	}
	
	return false;
}

template <typename T>
bool Stack<T>::peek(T& val) const {
	if (_pos > 0) {
		val = _vals[_pos - 1];
		return true;
	}
	
	return false;
}

template <typename T>
void Stack<T>::clear() {
	_pos = 0;
}

template <typename T>
bool Stack<T>::empty() const {
	return (_pos == 0);
}

template <typename T>
int Stack<T>::size() const {
	return (_pos);
}

template <typename T>
bool Stack<T>::full() const {
	return (_pos == STACK_CAPACITY);
}
