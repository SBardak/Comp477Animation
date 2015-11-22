/*
	* Interpolation class
*/
#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include <vector>

#include "quaternion.h"

class Interpolation
{
public:
	void MatrixLerp(float start[16], float end[16], float percent, float* rotation)
	{
		for (int i = 0; i < 10; ++i)
			if (i % 4 == 3) continue;
			else
			rotation[i] = (1 - percent) * start[i] + percent * end[i];
	}

	void EulerLerp(float start[3], float end[3], float percent, float* rotation)
	{
		for (int i = 0; i < 3; ++i)
			rotation[i] = (1 - percent) * start[i] + percent * end[i];
	}

	void QuaternionLerp(Quaternion start, Quaternion end, float percent, Quaternion* rotation)
	{
		*rotation = (1 - percent) * start + percent * end;
	}

	void QuaternionSLERP(Quaternion start, Quaternion end, float percent, Quaternion* rotation)
	{
		float sum = start.w * end.w +
			start.x * end.x +
			start.y * end.y +
			start.z * end.z;

		if (abs(sum) >= 1.0)
		{
			*rotation = start;
			return;
		}

		float angle = acos(sum),
			denom = sin(angle);

		if (abs(denom) < 0.0001)
			return QuaternionLerp(start, end, percent, rotation);

		float
			nl = sin((1 - percent) * angle) / denom,
			nr = sin(percent * angle) / denom;

		*rotation = (nl * start + nr * end);
	}
};
#endif
