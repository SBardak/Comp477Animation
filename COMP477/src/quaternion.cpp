#include "quaternion.h"
#include <iomanip>
#include "simpleMath.h"

#define _USE_MATH_DEFINES
#include <math.h>

Quaternion& Quaternion::operator *= (const Quaternion& rhs)
{
	float tw = w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z;
	float tx = w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y;
	float ty = w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x;
	float tz = w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w;
	w = tw;
	x = tx;
	y = ty;
	z = tz;
	return *this; // return the result by reference
}

Quaternion operator*(Quaternion lhs, float t)
{
	return lhs *= t;
}
Quaternion operator*(float t, Quaternion lhs)
{
	return lhs *= t;
}

Quaternion operator+(Quaternion lhs, Quaternion rhs)
{
	return lhs += rhs;
}
std::ostream & operator<< (std::ostream &out, Quaternion const &t)
{
	std::setprecision(5);
	out << t.w << " "
		<< t.x << " "
		<< t.y << " "
		<< t.z;
	return out;
}



Quaternion Quaternion::Conjugation()
{
	Quaternion c;
	c = *this;
	c *= -1;
	c.w *= -1;
	return c;
}
float Quaternion::Norm()
{
	return sqrt(
		w * w +
		x * x +
		y * y +
		z * z
		);
}
Quaternion& Quaternion::Normalize()
{
	*this *= (1 / Norm());
	return *this;
}
Quaternion Quaternion::Inverse()
{
	float normSq = (1 / Norm());
	normSq *= normSq;
	return Conjugation() * (normSq);
}





void Quaternion::FromMatrix(float mat[16])
{
	/*
	0 1 2 3
	4 5 6 7
	8 9 a b
	c d e f

	OpenGL?
	0 4 8 c
	1 5 9 d
	2 6 a e
	3 7 b f
	*/
	float t, s, u;
	
	t = sqrt(mat[0] + mat[5] + mat[10]);
	if (t > 0)
	{
		s = sqrt(t + 1) * 2;
		w = s * 0.25;

		x = (mat[6] - mat[9]) / s;
		y = (mat[8] - mat[2]) / s;
		z = (mat[1] - mat[4]) / s;
	}
	// find which diag value has largest value
	else if (mat[0] > mat[5] && mat[0] > mat[10])
	{
		// largest is 0
		s = sqrt(1 + mat[0] - mat[5] - mat[10]) * 2;

		w = (mat[6] - mat[9]) / s;
		x = 0.25 * s;
		y = (mat[4] + mat[1]) / s;
		z = (mat[8] + mat[2]) / s;
	}
	else if (mat[5] > mat[10])
	{
		// largest is 1
		s = sqrt(1 - mat[0] + mat[5] - mat[10]) * 2;

		w = (mat[8] - mat[2]) / s;
		x = (mat[4] + mat[1]) / s;
		y = 0.25 * s;
		z = (mat[9] + mat[6]) / s;
	}
	else
	{
		// largest is 2
		s = sqrt(1 - mat[0] - mat[5] + mat[10]) * 2;

		w = (mat[1] - mat[4]) / s;
		x = (mat[8] + mat[2]) / s;
		y = (mat[9] + mat[6]) / s;
		z = 0.25 * s;
	}

	//Unit
	u = sqrt(x * x + y * y + z * z + w * w);
	x /= u;
	y /= u;
	z /= u;
	w /= u;
}


void Quaternion::FromMatrix2(float mat[16])
{
	//float t, s, u;

	//t = sqrt(mat[0] + mat[5] + mat[10]);
	//if (t > 0)
	//{
	//	s = 0.5 / sqrt(t + 1);
	//	w = 0.25 / s;

	//	x = ( mat[9] - mat[6] ) * s;
	//	y = ( mat[2] - mat[8] ) * s;
	//	z = ( mat[4] - mat[1] ) * s;
	//}
	//// find which diag value has largest value
	//else if (mat[0] > mat[5] && mat[0] > mat[10])
	//{
	//	// largest is 0
	//	s = sqrt(1 + mat[0] - mat[5] - mat[10]) * 2;

	//	w = (mat[9] - mat[6]) / s;
	//	x = 0.25 * s;
	//	y = (mat[4] + mat[1]) / s;
	//	z = (mat[2] + mat[8]) / s;
	//}
	//else if (mat[5] > mat[10])
	//{
	//	// largest is 1
	//	s = sqrt(1 - mat[0] + mat[5] - mat[10]) * 2;

	//	w = (mat[2] - mat[8]) / s;
	//	x = (mat[4] + mat[1]) / s;
	//	y = 0.25 * s;
	//	z = (mat[9] + mat[6]) / s;
	//}
	//else
	//{
	//	// largest is 2
	//	s = sqrt(1 - mat[0] - mat[5] + mat[10]) * 2;

	//	w = (mat[4] - mat[1]) / s;
	//	x = (mat[2] + mat[8]) / s;
	//	y = (mat[9] + mat[6]) / s;
	//	z = 0.25 * s;
	//}

	////Unit
	//u = sqrt(x * x + y * y + z * z + w * w);
	//x /= u;
	//y /= u;
	//z /= u;
	//w /= u;
}

void Quaternion::ToMatrix(float* arr)
{
	float
		xx = x * x,
		xy = x * y,
		xz = x * z,
		xw = x * w,

		yy = y * y,
		yz = y * z,
		yw = y * w,

		zz = z * z,
		zw = z * w,

		ww = w * w;

	//arr[0] = 1 -	2 * (yy + zz);
	//arr[1] =		2 * (xy + zw);
	//arr[2] =		2 * (xz + yw);

	//arr[4] =		2 * (xy + zw);
	//arr[5] = 1 -	2 * (xx + zz);
	//arr[6] =		2 * (yz - xw);

	//arr[8] =		2 * (xz - yw);
	//arr[9] =		2 * (yz + xw);
	//arr[10] = 1 -	2 * (xx + yy);

	arr[0] = 1 -	2 * (yy + zz);
	arr[1] =		2 * (xy + zw);
	arr[2] =		2 * (xz - yw);

	arr[4] =		2 * (xy - zw);
	arr[5] = 1 -	2 * (xx + zz);
	arr[6] =		2 * (yz + xw);

	arr[8] =		2 * (xz + yw);
	arr[9] =		2 * (yz - xw);
	arr[10] = 1 -	2 * (xx + yy);

	arr[3] = arr[7] = arr[11] = arr[12] = arr[13] = arr[14] = 0;
	arr[15] = 1;
}

void Quaternion::FromEuler(float mat[3])
{
	//Values in rads

	float c1 = cos(mat[0] / 2), 
		c2 = cos(mat[1] / 2), 
		c3 = cos(mat[2] / 2), 
		s1 = sin(mat[0] / 2), 
		s2 = sin(mat[1] / 2),
		s3 = sin(mat[2] / 2);

	w = c1 * c2 * c3 - s1 * s2 * s3;
	x = c1 * c2 * s3 + s1 * s2 * c3;
	y = s1 * c2 * c3 + c1 * s2 * s3;
	z = c1 * s2 * c3 - s1 * c2 * s3;
}

void Quaternion::ToEuler(float* arr)
{
	//Values in rads

	Quaternion q;
	q = *this;
	q.Normalize();

	float t = q.x * q.y + q.z * q.w;
	if (t > 0.499) {
		arr[0] = 2 * atan2(q.x, q.w);
		arr[1] = M_PI / 2;
		arr[2] = 0;
		return;
	}
	if (t < -0.499)
	{
		arr[0] = -2 * atan2(q.x, q.w);
		arr[1] = -M_PI / 2;
		arr[2] = 0;
		return;
	}

	float sqx = q.x * q.x;
	float sqy = q.y * q.y;
	float sqz = q.z * q.z;
	arr[0] = atan2(2 * q.y * q.w - 2 * q.x * q.z, 1 - 2 * sqy - 2 * sqz);
	arr[1] = asin(2 * t);
	arr[2] = atan2(2 * q.x * q.w - 2 * q.y * q.z, 1 - 2 * sqx - 2 * sqz);
}

void Quaternion::ToAxisAngle(float* angle, Vec3* axis)
{
	if (w > 1) Normalize();
	*angle = 2 * acos(w);
	float denom = sqrt(1 - w * w);
	if (denom < 0.0001)
	{
		axis->x = 1; 
		axis->y = 0;
		axis->z = 0;
	}
	else
	{
		axis->x = x / denom;
		axis->y = y / denom;
		axis->z = z / denom;
	}
}