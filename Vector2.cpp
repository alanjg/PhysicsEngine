#include "Vector2.h"
#include <ostream>

namespace physics
{
	std::ostream& operator<<(std::ostream& out,const Vector2f& v)
	{
		return out<<"("<<v[0]<<","<<v[1]<<")";
	}
	
	Vector2f operator*(const Vector2f& lhs, float rhs)
	{		
		return Vector2f(lhs) *= rhs;
	}
	Vector2f operator*(float lhs, const Vector2f& rhs)
	{
		return Vector2f(rhs) *= lhs;
	}

	
	std::ostream& operator<<(std::ostream& out,const Vector2d& v)
	{
		return out<<"("<<v[0]<<","<<v[1]<<")";
	}

	Vector2d operator*(const Vector2d& lhs, double rhs)
	{		
		return Vector2d(lhs) *= rhs;
	}
	Vector2d operator*(double lhs, const Vector2d& rhs)
	{
		return Vector2d(rhs) *= lhs;
	}
}