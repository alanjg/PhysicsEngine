#include "Vector4.h"
#include <ostream>

namespace physics
{
	std::ostream& operator<<(std::ostream& out,const Vector4f& v)
	{
		return out<<"("<<v[0]<<","<<v[1]<<","<<v[2]<<","<<v[3]<<")";
	}
	
	Vector4f operator*(const Vector4f& lhs, float rhs)
	{		
		return Vector4f(lhs) *= rhs;
	}
	Vector4f operator*(float lhs, const Vector4f& rhs)
	{
		return Vector4f(rhs) *= lhs;
	}
	std::ostream& operator<<(std::ostream& out,const Vector4d& v)
	{
		return out<<"("<<v[0]<<","<<v[1]<<","<<v[2]<<","<<v[3]<<")";
	}
	
	Vector4d operator*(const Vector4d& lhs, double rhs)
	{		
		return Vector4d(lhs) *= rhs;
	}
	Vector4d operator*(double lhs, const Vector4d& rhs)
	{
		return Vector4d(rhs) *= lhs;
	}
}