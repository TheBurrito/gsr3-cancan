#ifndef Path_h_
#define Path_h_

#include "common.h"

#ifndef PATH_LENGTH
#define PATH_LENGTH 100
#endif

class Path {
public:
	Path();
	~Path();
	
	void pushFront(const Point& p);
	void pushBack(const Point& p);
	
	const Point& operator[](int i);
	
	void removeFront();
	void removeBack();
	
	int size();
	
private:
	
	Point _points[PATH_LENGTH];
	int _head;
	int _tail;
};

#endif //Path_h_
