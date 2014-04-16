#ifndef stack_h_
#define stack_h_

#ifndef STACK_CAPACITY
#define STACK_CAPACITY 64
#endif

template <typename T>
class Stack {
public:
	Stack();
	~Stack();
	
	bool push(const T& val);
	bool pop(T& val);
	void clear();
	
	bool peek(T& val) const;
	
	int size() const;
	
	bool full() const;
	bool empty() const;
	
private:
	int _pos;
	
	T _vals[STACK_CAPACITY];
};

#include "stack_inline.h"	

#endif //stack_h_
