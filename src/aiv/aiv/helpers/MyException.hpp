#include <exception>
using namespace std;

class MyException: public exception
{
private:
	string _msg;

public:
	MyException(string msg): exception() { _msg = msg; };
	virtual const char* what() const throw()
	{
		return _msg.c_str();
	}
};

// cmake:sourcegroup=Helpers