#include "mathdefs.h"
#include <cmath>

namespace physics {

bool IsEqual(float lhs,float rhs,float tolerance)
{
	return std::fabsf(lhs - rhs) < tolerance;
}

bool IsEqual(double lhs,double rhs,double tolerance)
{
	return std::fabs(lhs - rhs) < tolerance;
}

bool IsEqual(double lhs,float rhs,double tolerance)
{
	return IsEqual(lhs,double(rhs),tolerance);
}

bool IsEqual(float lhs,double rhs,double tolerance)
{
	return IsEqual(double(lhs),rhs,tolerance);
}

bool IsZero(float lhs,float tolerance)
{
	return std::fabsf(lhs) < tolerance;
}

bool IsZero(double lhs,double tolerance)
{
	return std::fabs(lhs) < tolerance;
}

}
