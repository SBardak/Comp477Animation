#ifndef TIME_MANAGER_H
#define TIME_MANAGER_H
#include <sys\timeb.h> 

/*
	TODO: Doesnt work, sometimes updated time is less than last time??
	Utility class to keep track of system time 
*/
class TimeManager
{
private:
	timeb currentTime;
	float sCurrentTime; //current time in seconds
public:
	TimeManager()
	{
		update();
	}

	void update()
	{
		ftime(&currentTime);
		sCurrentTime = ((currentTime.time / 1000.0) + currentTime.millitm) / 1000.0;
	}

	float getCurrentTime()
	{
		return sCurrentTime;
	}

	static float getCurrentSystemTime()
	{
		timeb currSysTime;
		ftime(&currSysTime);
		return ((currSysTime.time / 1000.0) + currSysTime.millitm) / 1000.0;
	}

};
#endif