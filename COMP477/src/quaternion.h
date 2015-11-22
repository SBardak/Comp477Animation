/*
* Quaternion
*/
#ifndef QUATERNION_H
#define QUATERNION_H

#include <vector>
#include "simpleMath.h"

class Quaternion
{
private:

public:
	float x = 0, y = 0, z = 0, w = 1;

	Quaternion()
	{
		x = y = z = 0;
		w = 1;
	}

	/* Matrix */
	void FromMatrix(float mat[16]);
	void FromMatrix2(float mat[16]);

	void ToMatrix(float* arr);
	/* Matrix */

	/* Euler */
	void FromEuler(float mat[3]);
	void ToEuler(float* arr);
	/* Euler */

	/* Angle/Axis */
	void ToAxisAngle(float* angle, Vec3* axis);

	Quaternion& operator+=(const Quaternion& rhs) 
	{                           
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
		w += rhs.w;
		return *this; // return the result by reference
	}
	Quaternion& operator*=(const Quaternion& rhs);
	Quaternion& operator*=(float rhs)
	{
		x *= rhs;
		y *= rhs;
		z *= rhs;
		w *= rhs;
		return *this; // return the result by reference
	}
	Quaternion& operator=(Quaternion rhs)
	{
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		w = rhs.w;
		return *this; // return the result by reference
	}

	Quaternion Conjugation();
	float Norm();
	Quaternion& Normalize();
	Quaternion Inverse();
};
std::ostream & operator<< (std::ostream &out, Quaternion const &t);

Quaternion operator*(Quaternion lhs, float t);
Quaternion operator*(float t, Quaternion lhs);
Quaternion operator+(Quaternion lhs, Quaternion rhs);



#endif
