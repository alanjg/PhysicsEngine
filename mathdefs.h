#ifndef MATHDEFS_H
#define MATHDEFS_H

namespace physics {

	const float PI = 3.1415926535897932384626433832795f;
	const double DOUBLE_PI = 3.1415926535897932384626433832795f;
	const float E = 2.71828182845904523536f;
	const double DOUBLE_E = 2.71828182845904523536;

	const float FLOAT_TOLERANCE = 1e-4f;
	const double DOUBLE_TOLERANCE = 1e-8;

	
	inline float Deg2Rad(float deg)
	{
		return deg*PI/180.0f;
	}
	inline double Deg2Rad(double deg)
	{
		return deg*DOUBLE_PI/180.0;
	}
	inline float Rad2Deg(float rad)
	{
		return rad*180.0f/PI;
	}
	inline double Rad2Deg(double rad)
	{
		return rad*180.0/DOUBLE_PI;
	}
	
	bool IsEqual(float lhs,float rhs,float tolerance = FLOAT_TOLERANCE);
	bool IsEqual(double lhs,double rhs,double tolerance = DOUBLE_TOLERANCE);
	bool IsEqual(double lhs,float rhs,double tolerance = DOUBLE_TOLERANCE);
	bool IsEqual(float lhs,double rhs,double tolerance = DOUBLE_TOLERANCE);
	bool IsZero(float lhs,float tolerance = FLOAT_TOLERANCE);
	bool IsZero(double lhs,double tolerance = DOUBLE_TOLERANCE);
	
}

#endif
