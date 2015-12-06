#ifndef TIME_MANAGER_H
#define TIME_MANAGER_H
#include <sys\timeb.h> 
#include <Windows.h>

/*
	Utility class to keep track of system time 
*/
class TimeManager
{
private:
	double sCurrentTime; //current time in seconds
public:
	TimeManager()
	{
		update();
	}

	void update()
	{
		sCurrentTime = GetTickCount();

		sCurrentTime /= 1000.0f;
		int i = 0;
	}

	double getCurrentTime()
	{
		return sCurrentTime;
	}

	static float getCurrentSystemTime()
	{
		return (double)GetTickCount() / 1000;
	}

};
#endif