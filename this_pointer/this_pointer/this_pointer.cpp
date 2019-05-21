#include <iostream>
using namespace std;
//https://blankspace-dev.tistory.com/151
class exClass
{
private:
	int num;
public:
	exClass() {}
	exClass(int num)
	{
		this->num = 200;
		num = 105;
	}
	~exClass(){}
};

exClass::exClass()
{
	num = 3;
}