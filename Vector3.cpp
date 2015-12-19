#include "Vector3.h"
#include <ostream>

namespace physics
{
	std::ostream& operator<<(std::ostream& out,const Vector3f& v)
	{
		return out<<"("<<v[0]<<","<<v[1]<<","<<v[2]<<")";
	}
	
	Vector3f operator*(const Vector3f& lhs, float rhs)
	{		
		return Vector3f(lhs) *= rhs;
	}
	Vector3f operator*(float lhs, const Vector3f& rhs)
	{
		return Vector3f(rhs) *= lhs;
	}

	
	std::ostream& operator<<(std::ostream& out,const Vector3d& v)
	{
		return out<<"("<<v[0]<<","<<v[1]<<","<<v[2]<<")";
	}

	Vector3d operator*(const Vector3d& lhs, double rhs)
	{		
		return Vector3d(lhs) *= rhs;
	}
	Vector3d operator*(double lhs, const Vector3d& rhs)
	{
		return Vector3d(rhs) *= lhs;
	}
}